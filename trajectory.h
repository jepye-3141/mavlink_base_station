#include <fcntl.h>  // for F_OK
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>  // for access()
#include <stddef.h>

typedef struct waypoint_t
{
    double x, y, z;           ///< position
    double xd, yd, zd;        ///< velocity
    double roll, pitch, yaw;  ///< angles
    double p, q, r;           ///< angular rates
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

#define TIME_TRANSITION_FLAG 0
#define POS_TRANSITION_FLAG 1