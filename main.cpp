#include "nonBlockingCLI.h"
#include "printing.h"
#include <rc/mavlink_udp.h>
#include <rc/math/quaternion.h>

struct msg
{
    float x;
    float y;
    float z;
    double rpy[3];
};


int main(int argc, char** argv) {

    if (rc_mav_init(7, "192.168.8.1", 145551, 2000000) == -1) {
        printf("Failed to initialize mavlink");
    }

    msg command_packet = {0, 0, 0, 0, 0, 0};

    while (true) {
        updateState(command_packet.x, command_packet.y, 
                    command_packet.z, command_packet.rpy[0], 
                    command_packet.rpy[1], command_packet.rpy[2]);
        double quat[4];
        rc_quaternion_from_tb_array(quat, command_packet.rpy);
        rc_mav_send_att_pos_mocap((float*)(quat), command_packet.x, 
                                   command_packet.y, command_packet.z);
    }

    printKeybindings();

}