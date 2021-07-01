#include <rc/mavlink_udp.h>
#include <rc/math/quaternion.h>
#include <rc/time.h>
#include <netinet/in.h>
#define NUM_DRONES 1
#define MSG_RATE 50000

struct address_node
{
    int id;
    int port;
    struct sockaddr_in address;
};

struct msg_t
{
    float x;
    float y;
    float z;
    float rpy[3];
};

int mav_init(uint8_t sysid, int dest_id, const char* dest_ip, uint16_t port, uint64_t connection_timeout_us);

int mav_set_callback_all(void (*func)(void));

int send_new_series(struct msg_t new_message[NUM_DRONES]);

int mav_cleanup();