#include <rc/math/filter.h>
#include <rc/mpu.h>

#define NUM_UNIQUE_TRAJ 1
#define NUM_DYNAMIC_TRAJ 2
#define NUM_TRAJ NUM_UNIQUE_TRAJ+NUM_DYNAMIC_TRAJ
#define TAKEOFF_POS NUM_TRAJ-2
#define LANDING_POS NUM_TRAJ-1
#define MAX_DRONES 5 // because wp files are generate w/ 5 drones 
#define VERT_OFFSET -0.0
#define OP_ALTITUDE -3.22
#define TENSION_ALTITUDE -1.52
#define X_OFFSET 1.075
#define NUM_DRONES 1
#define MSG_RATE 50000
#define MAVLINK_COMMAND_INITIALIZER {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, {0.0, 0.0, 0.0}}

typedef struct settings_t
{
    int num_waypoints_per_path[NUM_UNIQUE_TRAJ];
    int *x_waypoints[NUM_UNIQUE_TRAJ];
    int *y_waypoints[NUM_UNIQUE_TRAJ];
    int *z_waypoints[NUM_UNIQUE_TRAJ];

} settings_t;