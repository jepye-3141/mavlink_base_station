#include "nonBlockingCLI.h"
#include "printing.h"
#include <rc/mavlink_udp.h>
#include <rc/math/quaternion.h>
#include <rc/time.h>
#include "trajectory.h"

#define TAKEOFF_PATTERN -1
#define LANDING_PATTERN -2

void hello() {
    printf("got message, sent from callback");
}

int main() 
{
    mav_cleanup();
    printf("hello!\n");
    uint8_t id = (uint8_t)7;

    path_cleanup_all();
    path_init();

    if (path_load_from_file("guided_drone_waypoints.cfg", 0) == -1) {
        printf("Failed to initialize path");
        return -1;
    }
    
    if (mav_init(id, 1, "192.168.8.1", RC_MAV_DEFAULT_UDP_PORT, RC_MAV_DEFAULT_CONNECTION_TIMEOUT_US) == -1) {
        printf("Failed to initialize mavlink1");
        return -1;
    }
    

    msg_t command_packets[NUM_DRONES];
    // Pattern defaults to first pattern
    int pattern = -100;
    float current_x = 0;
    float current_y = 0;
    float current_z = 0;
    const int scope = SCOPE_PATTERNN_ONLY;
    int step = 0;
    command_packets[0] = MAVLINK_COMMAND_INITIALIZER;    

    // printKeybindings();

    while (true) {

        // Copy the up-to-date state to other command packets 
        //     if they need to be copied to
        if (scope != SCOPE_PATTERNN_ONLY) {
            updateState(command_packets[0].x, command_packets[0].y, 
                    command_packets[0].z, command_packets[0].rpy[0], 
                    command_packets[0].rpy[1], command_packets[0].rpy[2], 
                    pattern, scope);
            for (int i = 0; i < NUM_DRONES; i++) {
                command_packets[i].x = command_packets[0].x;
                command_packets[i].y = command_packets[0].y;
                command_packets[i].z = command_packets[0].z;
                command_packets[i].rpy[0] = command_packets[0].rpy[0];
                command_packets[i].rpy[1] = command_packets[0].rpy[1];
                command_packets[i].rpy[2] = command_packets[0].rpy[2];
            }
            send_new_series(command_packets);
            rc_usleep(MSG_RATE);
        }
        //otherwise, we are on pattern-only control, so we need to do path updates
        else {
                printf("\nCurrent step: %i\n", step);
                int prev_pattern = pattern;
                
                updateState(command_packets[0].x, command_packets[0].y, 
                    command_packets[0].z, command_packets[0].rpy[0], 
                    command_packets[0].rpy[1], command_packets[0].rpy[2], 
                    pattern, scope);

                current_x = command_packets[0].x;
                current_y = command_packets[0].y;
                current_z = command_packets[0].z;

                if (prev_pattern != pattern && pattern != 0 && prev_pattern != 0) {
                    step = 0;
                    printf("change traj, reset step and pause!\n");
                }

                if (pattern == 0 || pattern == -100) { // pause
                    for (int k = 0; k < NUM_DRONES; k++) {
                        command_packets[k].x_dot = 0;
                        command_packets[k].y_dot = 0;
                        command_packets[k].z_dot = 0;
                    }
                    step--;
                }
                else if (pattern == -1) { // takeoff
                    if (prev_pattern != pattern && pattern != 0 && prev_pattern != 0) {
                        takeoff_gen();
                    }
                    for (int k = 0; k < NUM_DRONES; k++) {
                        command_packets[k].x = path[TAKEOFF_POS].waypoints[step].x[k];
                        command_packets[k].y = path[TAKEOFF_POS].waypoints[step].y[k];
                        command_packets[k].z = path[TAKEOFF_POS].waypoints[step].z[k];
                        command_packets[k].x_dot = path[TAKEOFF_POS].waypoints[step].x_dot[k];
                        command_packets[k].y_dot = path[TAKEOFF_POS].waypoints[step].y_dot[k];
                        command_packets[k].z_dot = path[TAKEOFF_POS].waypoints[step].z_dot[k];
                        command_packets[k].rpy[TAKEOFF_POS] = 0;
                        command_packets[k].rpy[TAKEOFF_POS] = 0;
                        command_packets[k].rpy[TAKEOFF_POS] = 0;
                    } 
                }
                else if (pattern == -2) { // landing
                    if (prev_pattern != pattern && pattern != 0 && prev_pattern != 0) {
                        landing_gen(current_x, current_y, current_z);
                    }
                
                    for (int k = 0; k < NUM_DRONES; k++) {
                        command_packets[k].x = path[LANDING_POS].waypoints[step].x[k];
                        command_packets[k].y = path[LANDING_POS].waypoints[step].y[k];
                        command_packets[k].z = path[LANDING_POS].waypoints[step].z[k];
                        command_packets[k].x_dot = path[LANDING_POS].waypoints[step].x_dot[k];
                        command_packets[k].y_dot = path[LANDING_POS].waypoints[step].y_dot[k];
                        command_packets[k].z_dot = path[LANDING_POS].waypoints[step].z_dot[k];
                        command_packets[k].rpy[LANDING_POS] = 0;
                        command_packets[k].rpy[LANDING_POS] = 0;
                        command_packets[k].rpy[LANDING_POS] = 0;
                    } 
                }
                else { // unique
                    if (pattern > NUM_TRAJ && pattern != TAKEOFF_PATTERN && pattern != LANDING_PATTERN) {
                        printf("Invalid pattern, reverting to previous pattern");
                        pattern = prev_pattern;
                    }
                    for (int k = 0; k < NUM_DRONES; k++) {
                        command_packets[k].x = path[pattern - 1].waypoints[step].x[k];
                        command_packets[k].y = path[pattern - 1].waypoints[step].y[k];
                        command_packets[k].z = path[pattern - 1].waypoints[step].z[k];
                        command_packets[k].x_dot = path[pattern - 1].waypoints[step].x_dot[k];
                        command_packets[k].y_dot = path[pattern - 1].waypoints[step].y_dot[k];
                        command_packets[k].z_dot = path[pattern - 1].waypoints[step].z_dot[k];
                        command_packets[k].rpy[pattern - 1] = 0;
                        command_packets[k].rpy[pattern - 1] = 0;
                        command_packets[k].rpy[pattern - 1] = 0;
                    }
                }
                send_new_series(command_packets);
                if (pattern != 0) {
                    if (pattern != TAKEOFF_PATTERN && pattern != LANDING_PATTERN) {
                        if ((int)path[pattern - 1].len == step + 1) {
                            step--;
                            printf("end of traj, reset and pause!\n");
                        }
                    }
                    else if (pattern == TAKEOFF_PATTERN) {
                        if ((int)path[TAKEOFF_POS].len == step + 1) {
                            step--;
                            printf("end of traj, reset and pause!\n");
                        }
                    }
                    else if (pattern == LANDING_PATTERN) {
                        if ((int)path[LANDING_POS].len == step + 1) {
                            step--;
                            printf("end of traj, reset and pause!\n");
                        }
                    }
                    
                }
                rc_usleep(MSG_RATE);
                printf("\n%i\n", pattern);
        }
        step++;
    }

        // printf("%f | %f | %f | %f | %f | %f\n", command_packets[0].x, command_packets[0].y, command_packets[0].z, command_packets[0].rpy[0], 
        //            command_packets[0].rpy[1], command_packets[0].rpy[2]);

    mav_cleanup();

    return 0;
}