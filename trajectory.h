#include <fcntl.h>  // for F_OK
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>  // for access()
#include <stddef.h>
#include "math_utils.h"
#include "mavlink_prot.h"

/**
 * @brief       The type of waypoint which paths comprise of
 */
typedef struct waypoint_t
{
    double x[MAX_DRONES];
    double y[MAX_DRONES];
    double z[MAX_DRONES];
    double x_dot[MAX_DRONES];
    double y_dot[MAX_DRONES];
    double z_dot[MAX_DRONES];
    // double r[MAX_DRONES];
    // double p[MAX_DRONES];
    double yaw[MAX_DRONES];
    double yaw_dot[MAX_DRONES];
    double t;                 ///< time
    int flag;  ///< flag to specify state transitions, etc. (included to "future proof")
    int yaw_flag;

} waypoint_t;

typedef struct path_t
{
    waypoint_t* waypoints;  ///< pointer to head of the waypoint array
    size_t len;             ///< length of the path (number of waypoints)

    int initialized;  ///< 1 if initialized, 0 if uninitialized
} path_t;

typedef struct quintic_spline_1d_t
{
    float c0, c1, c2, c3, c4, c5;
} quintic_spline_1d_t;

/**
 * @brief       Initial values for path_t
 */
#define PATH_INITIALIZER                              \
    {                                                 \
        .waypoints = NULL, .len = 0, .initialized = 0 \
    }

extern path_t path[NUM_TRAJ + NUM_DYNAMIC_TRAJ];

int path_init();

int path_load_from_file(const char* file_path, int pos);

void path_cleanup_all();

void path_cleanup(int pos);

/**
 * @brief       Dynamically generate takeoff spline trajectory
 */
void takeoff_gen(float *current_x, float *current_y);

/**
 * @brief       Dynamically generate landing spline trajectory
 */
void landing_gen(float *current_x, float *current_y, float current_z);

void takeoff_5_gen(float *current_x, float *current_y);

void landing_5_gen(float *current_x, float *current_y, float current_z);

/**
 * @brief       Plan a path based on realsense payload landing command
 * 
 * @return      0 on success, -1 on failure
 */
int path_plan_from_rsp_cmd();

/**
 * @brief       Compute quintic spline coefficients for simple 1d path.
 * 
 * Starting and ending velocity and acceleration are 0.
 * 
 * @return      quintic_spline_1d_t with proper coefficients
 */
quintic_spline_1d_t make_1d_quintic_spline(float dx, float dt);

/**
 * @brief       Compute position along spline based on time
 * 
 * @return      1d position along spline
 */
float compute_spline_position(quintic_spline_1d_t* the_spline, float t);

/**
 * @brief       Compute velocity along spline based on time
 * 
 * @return      1d position along spline
 */
float compute_spline_velocity(quintic_spline_1d_t* the_spline, float t);

void test_trajectory(float *current_x, float *current_y, float current_z, int pos);

#define TIME_TRANSITION_FLAG 0
#define POS_TRANSITION_FLAG 1