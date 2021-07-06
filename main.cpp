#include "nonBlockingCLI.h"
#include "printing.h"
#include <rc/mavlink_udp.h>
#include <rc/math/quaternion.h>
#include <rc/time.h>
#include "trajectory.h"

struct kb_input
{
    float x;
    float y;
    float z;
    float rpy[3];
};

void hello() {
    printf("got message, sent from callback");
}

int main() 
{
    mav_cleanup();
    printf("hello!\n");
    uint8_t id = (uint8_t)7;

    if (path_load_from_file("guided_drone_waypoints.cfg") == -1) {
        printf("Failed to initialize path");
        return -1;
    }
    
    if (mav_init(id, 1, "192.168.8.1", RC_MAV_DEFAULT_UDP_PORT, RC_MAV_DEFAULT_CONNECTION_TIMEOUT_US) == -1) {
        printf("Failed to initialize mavlink1");
        return -1;
    }

    

    msg_t command_packets[NUM_DRONES];
    

    // printKeybindings();

    // while (true) {
    //     updateState(command_packets[0].x, command_packets[0].y, 
    //                 command_packets[0].z, command_packets[0].rpy[0], 
    //                 command_packets[0].rpy[1], command_packets[0].rpy[2]);
        
    //     updateState(command_packets[1].x, command_packets[1].y, 
    //                 command_packets[1].z, command_packets[1].rpy[0], 
    //                 command_packets[1].rpy[1], command_packets[1].rpy[2]);

    //     send_new_series(command_packets);

    //     // printf("%f | %f | %f | %f | %f | %f\n", command_packets[0].x, command_packets[0].y, command_packets[0].z, command_packets[0].rpy[0], 
    //     //            command_packets[0].rpy[1], command_packets[0].rpy[2]);
    //     rc_usleep(MSG_RATE);
    // }

    for (int i = 0; i < (int)path.len; i++) {
        command_packets[0].x = path.waypoints[i].x[0];
        command_packets[0].y = path.waypoints[i].y[0];
        command_packets[0].z = path.waypoints[i].z[0];
        command_packets[0].rpy[0] = path.waypoints[i].r[0];
        command_packets[0].rpy[1] = path.waypoints[i].p[0];
        command_packets[0].rpy[2] = path.waypoints[i].yaw[0];

        send_new_series(command_packets);
        rc_usleep(MSG_RATE);
        printf("\n%i\n",i);
    }

    mav_cleanup();

    return 0;
}