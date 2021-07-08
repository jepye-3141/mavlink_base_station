#include "nonBlockingCLI.h"
#include "printing.h"
#include <rc/mavlink_udp.h>
#include <rc/math/quaternion.h>
#include <rc/time.h>
#include "trajectory.h"

void hello() {
    printf("got message, sent from callback");
}

int main() 
{
    mav_cleanup();
    printf("hello!\n");
    uint8_t id = (uint8_t)7;

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
    int pattern = 0;
    const int scope = SCOPE_PATTERNN_ONLY;
    

    // printKeybindings();

    while (true) {
        updateState(command_packets[0].x, command_packets[0].y, 
                    command_packets[0].z, command_packets[0].rpy[0], 
                    command_packets[0].rpy[1], command_packets[0].rpy[2], 
                    pattern, scope);

        // Copy the up-to-date state to other command packets 
        //     if they need to be copied to
        if (scope != SCOPE_PATTERNN_ONLY) {
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
            for (int i = 0; i < (int)path[0].len; i++) {
                for (int k = 0; k < NUM_DRONES; k++) {
                    command_packets[k].x = path[0].waypoints[i].x[k];
                    command_packets[k].y = path[0].waypoints[i].y[k];
                    command_packets[k].z = path[0].waypoints[i].z[k];
                    command_packets[k].x_dot = path[0].waypoints[i].x_dot[k];
                    command_packets[k].y_dot = path[0].waypoints[i].y_dot[k];
                    command_packets[k].z_dot = path[0].waypoints[i].z_dot[k];
                    command_packets[k].rpy[0] = 0;
                    command_packets[k].rpy[1] = 0;
                    command_packets[k].rpy[2] = 0;
                } 
                send_new_series(command_packets);
                rc_usleep(MSG_RATE);
            }
        }

        // printf("%f | %f | %f | %f | %f | %f\n", command_packets[0].x, command_packets[0].y, command_packets[0].z, command_packets[0].rpy[0], 
        //            command_packets[0].rpy[1], command_packets[0].rpy[2]);

    }

    mav_cleanup();

    return 0;
}