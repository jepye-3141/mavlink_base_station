#include <fcntl.h>  // for F_OK
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>  // for access()
#include <stddef.h>
#include "mavlink_prot.h"

#define MAX_DRONES 5
typedef struct waypoint_t
{
    double x[MAX_DRONES];
    double y[MAX_DRONES];
    double z[MAX_DRONES];
    double r[MAX_DRONES];
    double p[MAX_DRONES];
    double yaw[MAX_DRONES];
    double t;                 ///< time
    int flag;  ///< flag to specify state transitions, etc. (included to "future proof")

} waypoint_t;

typedef struct path_t
{
    waypoint_t* waypoints;  ///< pointer to head of the waypoint array
    size_t len;             ///< length of the path (number of waypoints)

    int initialized;  ///< 1 if initialized, 0 if uninitialized
} path_t;

/**
 * @brief       Initial values for path_t
 */
#define PATH_INITIALIZER                              \
    {                                                 \
        .waypoints = NULL, .len = 0, .initialized = 0 \
    }

extern path_t path;

int path_load_from_file(const char* file_path);

void path_cleanup();

#define TIME_TRANSITION_FLAG 0
#define POS_TRANSITION_FLAG 1