#include "neue_mavlink_prot.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h> // for close()
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <arpa/inet.h>   // Sockets & networking include <sys/types.h> include <sys/socket.h> include <unistd.h> include
#include <string.h>

#include <rc/pthread.h>
#include <rc/mavlink_udp.h>

#define BUFFER_LENGTH			512 // common networking buffer size
#define MAX_UNIQUE_MSG_TYPES		256
#define MAX_PENDING_CONNECTIONS		32
#define LOCALHOST_IP			"127.0.0.1"
#define CONNECTION_TIMEOUT_US_MIN	200000

#define NUM_DRONES 5


// connection stuff
static int init_flag = 0;
static int sock_fd;
static int current_port;
static struct sockaddr_in my_address ;
static uint8_t system_id;
struct timeval rcv_timeo;

// callbacks
static void (*callbacks[MAX_UNIQUE_MSG_TYPES])(void);
static void (*callback_all)(void); // called when any packet arrives
static void (*connection_lost_callback)(void);

// flags and info populated by the listening thread
static int received_flag[MAX_UNIQUE_MSG_TYPES];
static int new_msg_flag[MAX_UNIQUE_MSG_TYPES];
static uint64_t connection_timeout_us_current;
static uint64_t us_of_last_msg[MAX_UNIQUE_MSG_TYPES];
static uint64_t us_of_last_msg_any;
static int msg_id_of_last_msg;
static uint8_t sys_id_of_last_msg;
static mavlink_message_t messages[MAX_UNIQUE_MSG_TYPES];
rc_mav_connection_state_t connection_state;

// thread startup and shutdown flags
static pthread_t listener_thread;
static int shutdown_flag = 0;
static int listening_flag = 0;
//static int listening_init_flag=0;

bool first_init = true;
typedef struct address_node
{
    int id;
    struct sockaddr_in* address;
};

struct address_node destinations[NUM_DRONES];
// TODO: multiple message - single drone
mavlink_message_t msg_series[NUM_DRONES];
static pthread_t drone_threads[NUM_DRONES];
static int shutdown_flag = 0;
int drones_init = 0;
int thread_id[NUM_DRONES];

// private local function declarations;
static uint64_t __us_since_boot();
static int __address_init(struct sockaddr_in* address, const char* dest_ip, uint16_t port);
int __get_msg_common_checks(int msg_id);

static void* __listen_thread_func(__attribute__((unused)) void* ptr, struct sockaddr_in my_address)
{
	int i;
	uint64_t time;
	ssize_t num_bytes_rcvd;
	uint8_t buf[BUFFER_LENGTH];
	socklen_t addr_len = sizeof(my_address);
	mavlink_message_t msg;
	mavlink_status_t parse_status;

	#ifdef DEBUG
	printf("beginning of __listen_thread_func thread\n");
	#endif

	// parse packets as they come in until listening flag set to 0
	listening_flag=1;
	while (shutdown_flag==0){
		memset(buf, 0, BUFFER_LENGTH);
		num_bytes_rcvd = recvfrom(sock_fd, buf, BUFFER_LENGTH, 0, (struct sockaddr *) &my_address, &addr_len);

		// check for timeout
		if(num_bytes_rcvd <= 0){
			if (errno == EAGAIN || errno == EWOULDBLOCK){
				// check last message time > MESSAGE_TIMEOUT then throw warning no heartbeat rcvd
				if((__us_since_boot()-us_of_last_msg_any) > connection_timeout_us_current){
					if(connection_state==MAV_CONNECTION_ACTIVE && connection_lost_callback!=NULL){
						connection_state = MAV_CONNECTION_LOST;
						connection_lost_callback();
					}
				}
				continue;
			}
		}
		// do mavlink's silly byte-wise parsing method
		for(i=0; i<num_bytes_rcvd; ++i){
			// parse on channel 0 (MAVLINK_COMM_0)
			if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &parse_status)){
				#ifdef DEBUG
				printf("\nReceived packet: SYSID: %d, MSG ID: %d\n", msg.sysid, msg.msgid);
				#endif
				// update timestamps and received flag
				time = __us_since_boot();
				us_of_last_msg[msg.msgid]=time;
				us_of_last_msg_any = time;
				received_flag[msg.msgid] = 1;
				new_msg_flag[msg.msgid] = 1;
				sys_id_of_last_msg=msg.sysid;
				msg_id_of_last_msg=msg.msgid;
				connection_state = MAV_CONNECTION_ACTIVE;

				// save local copy of message
				messages[msg.msgid]=msg;

				// run the generic callback
				if(callback_all!=NULL) callback_all();

				// run the msg-specific callback
				if(callbacks[msg.msgid]!=NULL) callbacks[msg.msgid]();
			}
		}
	}

	#ifdef DEBUG
	printf("exiting __listen_thread_func thread\n");
	#endif

	return 0;
}

static void* __transmit_thread_func(void* arg) {
    int identity = *(int*)arg;

    while (shutdown_flag == 0) {
        if(drones_init == NUM_DRONES && (identity == destinations[identity].id)){
            uint8_t buf[BUFFER_LENGTH];
            int msg_len, bytes_sent;

            if(init_flag == 0){
                fprintf(stderr, "ERROR: in rc_mav_send_msg, socket not initialized\n");
                return -1;
            }

            memset(buf, 0, BUFFER_LENGTH);
            msg_len = mavlink_msg_to_send_buffer(buf, &(msg_series[identity]));
            if(msg_len < 0){
                fprintf(stderr, "ERROR: in rc_mav_send_msg, unable to pack message for sending\n");
                return -1;
            }
            bytes_sent = sendto(sock_fd, buf, msg_len, 0, (struct sockaddr *) &(destinations[identity].address),
                                    sizeof (destinations[identity].address));
            if(bytes_sent != msg_len){
                perror("ERROR in rc_mav_send_msg failed to write to UDP socket");
                return -1;
            }
        }
    }
}

int mav_init(uint8_t sysid, int dest_id, const char* dest_ip, uint16_t port, uint64_t connection_timeout_us) 
{
    int i;

    if(init_flag == 0){
        for(int k = 0; k < NUM_DRONES; k++) {
            thread_id[k] = k;
        }
    }

    if(dest_ip==NULL){
		fprintf(stderr, "ERROR: in rc_mav_init received NULL dest_ip string\n");
		return -1;
	}
	if(connection_timeout_us<CONNECTION_TIMEOUT_US_MIN){
		fprintf(stderr,"ERROR in rc_mav_init, connection_timeout_us must be >%d\n", CONNECTION_TIMEOUT_US_MIN);
		return -1;
	}

    // set destination address
	/////////////////NEW///////////////////
    destinations[dest_id - 1].id = dest_id;
    if(__address_init(&(destinations[dest_id - 1].address), dest_ip, port) != 0){
        fprintf(stderr, "ERROR: in rc_mav_init: couldn't set local address\n");
		return -1;
    }
    if(rc_pthread_create(&(drone_threads[dest_id - 1]), __transmit_thread_func, (void*)&(thread_id[dest_id - 1]), SCHED_OTHER, 0) < 0){
        fprintf(stderr,"ERROR: in rc_mav_init, couldn't start transmit thread\n");
        return -1;
    }



	// first-time initialization
	if(init_flag==0){
	
        // set the connection state as waiting early
        // this will be change by listening thread
        connection_state=MAV_CONNECTION_WAITING;
        connection_timeout_us_current = connection_timeout_us;
        // save port globally for other functions to use
        current_port = port;

        // set all the callback pointers to something sane
        callback_all = NULL;
        connection_lost_callback = NULL;
        for(i=0;i<MAX_UNIQUE_MSG_TYPES;i++) callbacks[i] = NULL;

        // set up all global variables to default values
        for(i=0;i<MAX_UNIQUE_MSG_TYPES;i++){
            received_flag[i]=0;
            new_msg_flag[i]=0;
            us_of_last_msg[i]=UINT64_MAX;
        }
        memset(&messages,i,sizeof(mavlink_message_t));
        us_of_last_msg_any=UINT64_MAX;
        msg_id_of_last_msg=-1;

        // open socket for UDP packets
        if((sock_fd=socket(AF_INET, SOCK_DGRAM, 0)) < 0){
            perror("ERROR: in rc_mav_init: ");
            return -1;
        }    

        // fill out rest of sockaddr_in struct
        if(__address_init(&my_address, 0, current_port) != 0){
            fprintf(stderr, "ERROR: in rc_mav_init: couldn't set local address\n");
            return -1;
        }
        // socket timeout should be half the connection timeout detection window
        rcv_timeo.tv_sec = (connection_timeout_us/2)/1000000;
        rcv_timeo.tv_usec = (connection_timeout_us/2)%1000000;
        if(setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, (struct timeval *)&rcv_timeo, sizeof (struct timeval)) < 0){
            perror("ERROR: in rc_mav_init: ");
            return -1;
        }

        // bind address to listening port
        if(bind(sock_fd, (struct sockaddr *) &my_address, sizeof my_address) < 0){
            perror("ERROR: in rc_mav_init: ");
            return -1;
        }

        // signal initialization finished
        init_flag=1;
        system_id=sysid;

        // spawn listener thread
        if(rc_pthread_create(&listener_thread, __listen_thread_func, NULL, SCHED_OTHER, 0) < 0){
            fprintf(stderr,"ERROR: in rc_mav_init, couldn't start listening thread\n");
            return -1;
        }
    }

    drones_init++;
    return 0;
}

void add_msg_to_series(mavlink_message_t msg, int dest_id)
{
    msg_series[dest_id - 1] = msg;
}

int mav_cleanup(void)
{
    int ret = 0;
	if(init_flag==0 || listening_flag==0){
		fprintf(stderr, "WARNING, trying to cleanup mavlink listener when it's not running\n");
		return -1;
	}
	shutdown_flag=1;
	listening_flag=0;

	// wait for thread to join
	ret=rc_pthread_timed_join(listener_thread, NULL, 1.5);
	if(ret==1) fprintf(stderr,"WARNING in rc_mav_cleanup, joining thread timed out\n");
    while(drones_init > 0) {
        ret=rc_pthread_timed_join(drone_threads[drones_init - 1], NULL, 1.5);
	    if(ret==1) fprintf(stderr,"WARNING in rc_mav_cleanup, joining thread timed out\n");
        drones_init--;
    }

	close(sock_fd);
	init_flag=0;
	return ret;
}
