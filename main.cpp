#include "nonBlockingCLI.h"
#include "printing.h"
#include <rc/mavlink_udp.h>
#include <rc/math/quaternion.h>
#include <rc/time.h>
#include "neue_mavlink_prot.h"

struct kb_input
{
    float x;
    float y;
    float z;
    float rpy[3];
};

int main() 
{
    printf("hello!");
    uint8_t id = (uint8_t)7;
    
    if (mav_init(id, 1, "127.0.0.1", RC_MAV_DEFAULT_UDP_PORT, RC_MAV_DEFAULT_CONNECTION_TIMEOUT_US) == -1) {
        printf("Failed to initialize mavlink");
        return -1;
    }

    msg_t command_packets[1];
    command_packets[0] = {0, 0, 0, {0, 0, 0}};

    printKeybindings();

    while (true) {
        updateState(command_packets[0].x, command_packets[0].y, 
                    command_packets[0].z, command_packets[0].rpy[0], 
                    command_packets[0].rpy[1], command_packets[0].rpy[2]);
        
        send_new_series(command_packets);

        printf("%f | %f | %f | %f | %f | %f\n", command_packets[0].x, command_packets[0].y, command_packets[0].z, command_packets[0].rpy[0], 
                    command_packets[0].rpy[1], command_packets[0].rpy[2]);
        rc_usleep(MSG_RATE);
    }

    mav_cleanup();

    return 0;
}