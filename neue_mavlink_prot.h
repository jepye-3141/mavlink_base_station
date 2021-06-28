#include <rc/mavlink_udp.h>
#include <rc/math/quaternion.h>
#include <rc/time.h>

int mav_init(uint8_t sysid, int dest_id, const char* dest_ip, uint16_t port, uint64_t connection_timeout_us);

int add_msg_to_series(mavlink_message_t msg, int dest_id);

int mav_cleanup();