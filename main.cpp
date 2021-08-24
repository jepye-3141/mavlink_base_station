#include "nonBlockingCLI.h"
#include "printing.h"
#include <rc/mavlink_udp.h>
#include <rc/math/quaternion.h>
#include <rc/time.h>
#include "trajectory.h"

#define TAKEOFF_PATTERN -1
#define LANDING_PATTERN -2
#define PAUSE_PATTERN 0
#define PAUSE_ON_STARTUP_PATTERN -100


void hello() {
    printf("got message, sent from callback");
}

int main(int argc, char* argv[]) 
{
    mav_cleanup();
    uint8_t id = (uint8_t)7;

    path_cleanup_all();
    path_init();

    int c;
    char* settings_file_path;
    while ((c = getopt(argc, argv, "s:h")) != -1) {
        switch (c)
        {
            // settings
            case 's':
                settings_file_path = optarg;
                printf("User specified settings file:\n%s\n", settings_file_path);
                break;
            case 'h':
                printf("\n");
                printf(" Options\n");
                printf(" -s\t\t<settings file> Specify settings file to use\n");
                printf(" -h\t\tPrint this help message\n");
                printf("\n");
                printf("Some example settings files are included with the\n");
                printf("source code. You must specify the location of one of these\n");
                printf("files or ideally the location of your own settings file.\n");
                printf("\n");

            default:
                break;
        }
    }

    if (path_load_from_file("guided_drone_waypoints.cfg", 0) == -1) {
        printf("Failed to initialize path");
        return -1;
    }
    
    // Hi John from the future, you're probably are wondering why in god's name you can't make multiple listener threads.
    // Don't worry. John from the past has you covered. The problem is the ports: you have to run rav_init with different port
    //    numbers, or else the addresses will be the same and the computer will become mightily confused. 
    // So, don't panic, change the ports.
    if (mav_init(id, 1, "192.168.8.1", RC_MAV_DEFAULT_UDP_PORT, RC_MAV_DEFAULT_CONNECTION_TIMEOUT_US) == -1) {
        printf("Failed to initialize mavlink1");
        return -1;
    }
    
    // if (mav_init(id, 2, "192.168.1.202", RC_MAV_DEFAULT_UDP_PORT + 1000, RC_MAV_DEFAULT_CONNECTION_TIMEOUT_US) == -1) {
    //     printf("Failed to initialize mavlink1");
    //     return -1;
    // }

    // if (mav_init(id, 3, "192.168.1.203", RC_MAV_DEFAULT_UDP_PORT + 2000, RC_MAV_DEFAULT_CONNECTION_TIMEOUT_US) == -1) {
    //     printf("Failed to initialize mavlink1");
    //     return -1;
    // }

    // if (mav_init(id, 4, "192.168.1.204", RC_MAV_DEFAULT_UDP_PORT + 3000, RC_MAV_DEFAULT_CONNECTION_TIMEOUT_US) == -1) {
    //     printf("Failed to initialize mavlink1");
    //     return -1;
    // }

    // if (mav_init(id, 5, "192.168.1.205", RC_MAV_DEFAULT_UDP_PORT + 4000, RC_MAV_DEFAULT_CONNECTION_TIMEOUT_US) == -1) {
    //     printf("Failed to initialize mavlink1");
    //     return -1;
    // }
    

    msg_t command_packets[NUM_DRONES];
    // Pattern defaults to first pattern
    int pattern = -100;
    float current_x[NUM_DRONES];
    float current_y[NUM_DRONES];
    float current_z[NUM_DRONES];
    float offset_x[NUM_DRONES];
    float offset_y[NUM_DRONES];
    float offset_z[NUM_DRONES];

    for (int i = 0; i < NUM_DRONES; i++) {
        current_x[i] = 0.0;
        current_y[i] = 0.0;
        current_z[i] = 0.0;
        offset_x[i] = 0.0;
        offset_y[i] = 0.0;
        offset_z[i] = 0.0;
    }

    const int scope = SCOPE_PATTERNN_ONLY;
    int step = 0;
    command_packets[0] = {1.075, 1.075, 0.0, 0.0, 0.0, 0.0, {0.0, 0.0, 0.0}};
    command_packets[1] = {-1.075, 1.075, 0.0, 0.0, 0.0, 0.0, {0.0, 0.0, 0.0}};
    command_packets[2] = {-1.075, -1.075, 0.0, 0.0, 0.0, 0.0, {0.0, 0.0, 0.0}};
    command_packets[3] = {1.075, -1.075, 0.0, 0.0, 0.0, 0.0, {0.0, 0.0, 0.0}};
    command_packets[4] = {1.075, 0.0, 0.0, 0.0, 0.0, 0.0, {0.0, 0.0, 0.0}};

    // printKeybindings();

    while (true) {

        float dx = 0.0;
        float dy = 0.0;
        float dz = 0.0;
        float dr = 0.0;
        float dp = 0.0;
        float dyaw = 0.0;
        int prev_pattern = pattern;
        updateState(dx, dy, dz, dr, dp, dyaw, pattern, scope);

        // Copy the up-to-date state to other command packets 
        //     if they need to be copied to
        if (scope != SCOPE_PATTERNN_ONLY) {
            for (int i = 0; i < NUM_DRONES; i++) {
                    command_packets[i].x += dx;
                    command_packets[i].y += dy;
                    command_packets[i].z += dz;
                    command_packets[i].rpy[0] += dr;
                    command_packets[i].rpy[1] += dp;
                    command_packets[i].rpy[2] += dyaw;
                }
        }
        //otherwise, we are on pattern-only control, so we need to do path updates
        else {
                printf("\nCurrent step: %i\n", step);

                for (int i = 0; i < NUM_DRONES; i++) {
                    current_x[i] = command_packets[i].x;
                    current_y[i] = command_packets[i].y;
                    current_z[i] = command_packets[i].z;
                }

                switch (pattern) {
                    case PAUSE_ON_STARTUP_PATTERN:
                    case PAUSE_PATTERN:
                        for (int k = 0; k < NUM_DRONES; k++) {
                            command_packets[k].x_dot = 0;
                            command_packets[k].y_dot = 0;
                            command_packets[k].z_dot = 0;
                        }
                        step--;
                        break;
                    case TAKEOFF_PATTERN:
                        printf("Previous pattern: %i New pattern: %i\n", prev_pattern, pattern);
                        if (prev_pattern != pattern && pattern != PAUSE_PATTERN && prev_pattern != PAUSE_PATTERN) {
                            printf("got here\n");
                            if (NUM_DRONES < 5) {
                                takeoff_gen(current_x, current_y);
                            }
                            else if (NUM_DRONES == 5) {
                                takeoff_5_gen(current_x, current_y);
                            }
                            
                            step = 0;
                            
                            // got rid of dynamic offsets 
                            // offset_x = current_x - path[TAKEOFF_POS].waypoints[0].x[0];
                            // offset_y = current_y - path[TAKEOFF_POS].waypoints[0].y[0];
                            // offset_z = current_z - path[TAKEOFF_POS].waypoints[0].z[0];
                            printf("change traj, reset step!\n");
                        }
                        for (int k = 0; k < NUM_DRONES; k++) {
                            // printf("command packet %i: %f, %f, %f\n", step, path[TAKEOFF_POS].waypoints[step].x[k], path[TAKEOFF_POS].waypoints[step].y[k], path[TAKEOFF_POS].waypoints[step].z[k]);
                            command_packets[k].x = path[TAKEOFF_POS].waypoints[step].x[k];
                            command_packets[k].y = path[TAKEOFF_POS].waypoints[step].y[k];
                            command_packets[k].z = path[TAKEOFF_POS].waypoints[step].z[k];
                            command_packets[k].x_dot = path[TAKEOFF_POS].waypoints[step].x_dot[k];
                            command_packets[k].y_dot = path[TAKEOFF_POS].waypoints[step].y_dot[k];
                            command_packets[k].z_dot = path[TAKEOFF_POS].waypoints[step].z_dot[k];
                            command_packets[k].rpy[0] = 0;
                            command_packets[k].rpy[1] = 0;
                            command_packets[k].rpy[2] = 0;
                        }
                        break;
                    case LANDING_PATTERN:
                        if (prev_pattern != pattern && pattern != PAUSE_PATTERN && prev_pattern != PAUSE_PATTERN) {
                            if (NUM_DRONES < 5) {
                                landing_gen(current_x, current_y, current_z[0]);
                            }
                            else if (NUM_DRONES == 5) {
                                landing_5_gen(current_x, current_y, current_z[0]);
                            }
                            
                            step = 0;

                            for (int i = 0; i < NUM_DRONES; i++) {
                                offset_x[i] = current_x[i];
                                offset_y[i] = current_y[i];
                                offset_z[i] = current_z[i];
                            }

                            // got rid of dynamic offsets 
                            // offset_x = current_x - path[LANDING_POS].waypoints[0].x[0];
                            // offset_y = current_y - path[LANDING_POS].waypoints[0].y[0];
                            // offset_z = current_z - path[LANDING_POS].waypoints[0].z[0];
                            printf("change traj, reset step and record offset!\n");
                        }
                    
                        for (int k = 0; k < NUM_DRONES; k++) {
                            command_packets[k].x = path[LANDING_POS].waypoints[step].x[k];
                            command_packets[k].y = path[LANDING_POS].waypoints[step].y[k];
                            command_packets[k].z = path[LANDING_POS].waypoints[step].z[k];
                            command_packets[k].x_dot = path[LANDING_POS].waypoints[step].x_dot[k];
                            command_packets[k].y_dot = path[LANDING_POS].waypoints[step].y_dot[k];
                            command_packets[k].z_dot = path[LANDING_POS].waypoints[step].z_dot[k];
                            command_packets[k].rpy[0] = 0;
                            command_packets[k].rpy[1] = 0;
                            command_packets[k].rpy[2] = 0;
                        }
                        break;
                    default:
                        if (pattern > NUM_TRAJ && pattern != TAKEOFF_PATTERN && pattern != LANDING_PATTERN) {
                            printf("Invalid pattern, reverting to previous pattern");
                            pattern = prev_pattern;
                        }
                        if (prev_pattern != pattern && pattern != PAUSE_PATTERN && prev_pattern != PAUSE_PATTERN) {
                            for (int i = 0; i < NUM_DRONES; i++) {
                                offset_x[i] = current_x[i];
                                offset_y[i] = current_y[i];
                                offset_z[i] = current_z[i];
                            }

                            step = 0;
                            // got rid of dynamic offsets 
                            // offset_x = current_x - path[pattern - 1].waypoints[0].x[0];
                            // offset_y = current_y - path[pattern - 1].waypoints[0].y[0];
                            // offset_z = current_z - path[pattern - 1].waypoints[0].z[0];
                            printf("change traj, reset step and record offset!\n");
                        }

                        if (step == 0) {
                            test_trajectory(current_x, current_y, OP_ALTITUDE, 1);
                        }

                        for (int i = 0; i < NUM_DRONES; i++) {
                            if (abs((offset_x[i]) - path[pattern - 1].waypoints[0].x[i]) > 0.05 || abs(offset_y[i] - path[pattern - 1].waypoints[0].y[i]) > 0.05) {
                                printf("\nError: starting waypoint of trajectory does not align with end waypoint of previous trajectory.\n");
                                step = 0;
                                break;
                            }
                            if (RESPECT_TRAJECTORY_Z == 0) {
                                if (abs((offset_z[i]) - path[pattern - 1].waypoints[0].z[i]) > 0.05) {
                                    printf("\nError: starting waypoint of trajectory does not align with end waypoint of previous trajectory.\n");
                                    step = 0;
                                    break;
                                }
                            }
                        }

                        for (int k = 0; k < NUM_DRONES; k++) {
                            // got rid of dynamic offsets 
                            // command_packets[k].x = path[pattern - 1].waypoints[step].x[k] + offset_x;
                            // command_packets[k].y = path[pattern - 1].waypoints[step].y[k] + offset_y;
                            command_packets[k].x = path[pattern - 1].waypoints[step].x[k];
                            command_packets[k].y = path[pattern - 1].waypoints[step].y[k];
                            command_packets[k].x_dot = path[pattern - 1].waypoints[step].x_dot[k];
                            command_packets[k].y_dot = path[pattern - 1].waypoints[step].y_dot[k];
                            if (RESPECT_TRAJECTORY_Z == 1 && pattern == 2) {
                                command_packets[k].z = path[pattern - 1].waypoints[step].z[k];
                                command_packets[k].z_dot = path[pattern - 1].waypoints[step].z_dot[k];
                            }
                            else {
                                command_packets[k].z = offset_z[k];
                                command_packets[k].z_dot = 0;
                            }
                            command_packets[k].rpy[0] = 0;
                            command_packets[k].rpy[1] = 0;
                            if (path[pattern - 1].waypoints[step].yaw_flag == 1) {
                                // printf("enabling yaw\n");
                                // printf("yaw received to pos %f vel %f\n", path[pattern - 1].waypoints[step].yaw[0], path[pattern - 1].waypoints[step].yaw_dot[0]);

                                command_packets[k].yaw_flag = 1;
                                command_packets[k].rpy[2] = path[pattern - 1].waypoints[step].yaw[0];
                                command_packets[k].rpy_dot[2] = path[pattern - 1].waypoints[step].yaw_dot[0];
                                // printf("yaw set to pos %f vel %f\n", command_packets[k].rpy[2], command_packets[k].rpy_dot[2]);
                            }
                            else {
                                command_packets[k].rpy[2] = 0;
                                command_packets[k].rpy_dot[2] = 0;
                            }
                            
                        }
                        break;
                }
                
                switch (pattern) {
                    case TAKEOFF_PATTERN:
                        if ((int)path[TAKEOFF_POS].len == step + 1) {
                            in_progress_flag = false;
                            for (int k = 0; k < NUM_DRONES; k++) {
                                command_packets[k].x_dot = 0;
                                command_packets[k].y_dot = 0;
                                command_packets[k].z_dot = 0;
                            }
                            step--;
                            printf("safe mode off: end of traj, reset and pause!\n");
                        }
                    case LANDING_PATTERN:
                        if ((int)path[LANDING_POS].len == step + 1) {
                            in_progress_flag = false;
                            for (int k = 0; k < NUM_DRONES; k++) {
                                command_packets[k].x_dot = 0;
                                command_packets[k].y_dot = 0;
                                command_packets[k].z_dot = 0;
                            }
                            step--;
                            printf("safe mode off: end of traj, reset and pause!\n");
                        }
                    default:
                        if ((int)path[pattern - 1].len == step + 1) {
                            in_progress_flag = false;
                            for (int k = 0; k < NUM_DRONES; k++) {
                                command_packets[k].x_dot = 0;
                                command_packets[k].y_dot = 0;
                                command_packets[k].z_dot = 0;
                            }
                            step--;
                            printf("end of traj, reset and pause!\n");
                        }
                }
                    
        }

        send_new_series(command_packets);
        rc_usleep(MSG_RATE);
        printf("\n%i\n", pattern);
        step++;
    }
        

        // printf("%f | %f | %f | %f | %f | %f\n", command_packets[0].x, command_packets[0].y, command_packets[0].z, command_packets[0].rpy[0], 
        //            command_packets[0].rpy[1], command_packets[0].rpy[2]);

    mav_cleanup();

    return 0;
}