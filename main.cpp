#include "nonBlockingCLI.h"
#include "printing.h"
#include <rc/mavlink_udp.h>
#include <rc/math/quaternion.h>
#include <rc/time.h>

struct msg
{
    float x;
    float y;
    float z;
    double rpy[3];
};

int i = 0;

int main() {
    printf("hello!");
    uint8_t id = (uint8_t)7;
    if (rc_mav_init(id, "192.168.2.203", RC_MAV_DEFAULT_UDP_PORT, RC_MAV_DEFAULT_CONNECTION_TIMEOUT_US) == -1) {
        printf("Failed to initialize mavlink");
    }
    rc_mav_set_dest_ip("192.168.2.203");

    msg command_packet = {0, 0, 0, 0, 0, 0};

    printKeybindings();

    while (true) {
        updateState(command_packet.x, command_packet.y, 
                    command_packet.z, command_packet.rpy[0], 
                    command_packet.rpy[1], command_packet.rpy[2]);
        double quat[4];
        rc_quaternion_from_tb_array(quat, command_packet.rpy);
        rc_mav_send_att_pos_mocap((float*)(quat), command_packet.x, 
                                   command_packet.y, command_packet.z);
        printf("%f | %f | %f\n", command_packet.x, command_packet.y, command_packet.z);
        rc_usleep(50000);
    }


}