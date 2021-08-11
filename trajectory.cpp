/**
 * @file waypoint.c
 **/

#include <fcntl.h>  // for F_OK
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>  // for access()

#include "trajectory.h"

path_t path[NUM_TRAJ + NUM_DYNAMIC_TRAJ];

/*********************************
 * Functions for internal use only
 *
 */

/**
 * @brief   Count the number of lines in a file, indicates number of waypoints
 *
 * @return  Number of lines in the file
 */
static int __count_file_lines(const char* file_path);

/**
 * @brief   Read all of the waypoints from a file into the path variable
 *
 * @return  0 on success, -1 on failure
 */
static int __read_waypoints(FILE* fd, int pos);

static void __dynamic_z_change(float *current_x, float *current_y, float current_z, float target_z, int pos);

static void __dynamic_pos_change(path_t &dynamic_path, float *current_x, float *current_y, float *current_z, float *target_x, float *target_y, float *target_z, double dt);

static void __waypoint_trajectory(float **x_waypoints, float **y_waypoints, float **z_waypoints, double *dt, int num_waypoints, int pos );

static void __yaw_spline(float *yaw_waypoints, double *dt, int num_waypoints, int pos);

/**
 * ********************************
 */

int path_init() {
    // Clear any previously stored path, set init to 0
    path_cleanup_all();

    for (int i = 0; i < NUM_TRAJ; i++) {
        path[i] = PATH_INITIALIZER;
    }

    return 0;
}

int path_load_from_file(const char* file_path, int pos)
{
    // Check for valid file
    if (access(file_path, F_OK) != 0)
    {
        fprintf(stderr, "ERROR: waypoint file missing\n");
        return -1;
    }

    if (pos >= NUM_UNIQUE_TRAJ || pos < 0) {
        fprintf(stderr, "ERROR: position specified beyond allowed path array indices");
        return -1;
    }

    // Count number of waypoints contained in file
    path[pos].len = __count_file_lines(file_path);

    // Open file for waypoint reading
    FILE* fd = fopen(file_path, "r");

    // Read path size and allocate waypoint memory
    path[pos].waypoints = (waypoint_t*)malloc(sizeof(waypoint_t) * path[pos].len);
    if (path[pos].waypoints == NULL)
    {
        fprintf(stderr, "ERROR: failed allocating memory for path\n");
        return -1;
    }

    // Read waypoints from file
    if (__read_waypoints(fd, pos) < 0)
    {
        path_cleanup(pos); //Added to prevent potential memory leak
        fprintf(stderr, "ERROR: failed reading waypoint file\n");
        return -1;
    }

    fclose(fd);

    path[pos].initialized = 1;
    return 0;
}

void path_cleanup_all() {
    for (int i = 0; i < NUM_TRAJ; i++) {
        path_cleanup(i);
    }
}

void path_cleanup(int pos)
{
    free(path[pos].waypoints);
    path[pos].waypoints = NULL;
    path[pos].len = 0;
    path[pos].initialized = 0;
    
}

static int __count_file_lines(const char* file_path)
{
    FILE* fd = fopen(file_path, "r");
    int c = 0;
    size_t count = 0;

    c = getc(fd);
    while (c != EOF)
    {
        if (c == '\n' || c == EOF)
        {
            ++count;
        }
        c = getc(fd);
    }

    fclose(fd);
    return count;
}

static int __read_waypoints(FILE* fd, int pos)
{
    int rcount = 0;
    int waypoint_num = 0;

    while (rcount > -1)
    {
        rcount = 0;

        // Read formated file line (30 doubles)
        for (int i = 0; i < MAX_DRONES; i++) {
            rcount += fscanf(fd, "%lf %lf %lf %lf %lf %lf",
                &path[pos].waypoints[waypoint_num].x[i], &path[pos].waypoints[waypoint_num].y[i], 
                &path[pos].waypoints[waypoint_num].z[i], &path[pos].waypoints[waypoint_num].x_dot[i], 
                &path[pos].waypoints[waypoint_num].y_dot[i], &path[pos].waypoints[waypoint_num].z_dot[i]);
            // if (i == 0 && (path[pos].waypoints[waypoint_num])) {

            // }
        }
                    
        printf("I got %i on %i\n", rcount, waypoint_num);

        // If not end of file, but an invalid read (waypoints have 31 values)
        if (rcount > -1 && rcount != 30)
        {
            fprintf(stderr, "ERROR: invalid waypoint read from line: %i\n",
                waypoint_num + 1);  // lines 1 indexed, waypoints zero indexed
            printf("I got %i\n", rcount);
            return -1;
        }

        // Increment line number for next iteration
        ++waypoint_num;
    }
    return 0;
}

void takeoff_gen(float *current_x, float *current_y) {
    __dynamic_z_change(current_x, current_y, 0.0, OP_ALTITUDE, TAKEOFF_POS);
}

void landing_gen(float *current_x, float *current_y, float current_z) {
    __dynamic_z_change(current_x, current_y, current_z, 0, LANDING_POS);
}

void takeoff_5_gen(float *current_x, float *current_y) {
    // printf("x waypoint 1: %f, %f, %f, %f, %f\n", current_x[0], current_x[1], current_x[2], current_x[3], current_x[4]);
    float wp_x_2[] = {current_x[0], current_x[1], current_x[2], current_x[3], ((float)(current_x[4]) - (X_OFFSET))};
    float wp_x_3[] = {current_x[0], current_x[1], current_x[2], current_x[3], ((float)(current_x[4]) - (X_OFFSET))};

    float wp_z_1[] = {0.0, 0.0, 0.0, 0.0, 0.0};
    float wp_z_2[] = {TENSION_ALTITUDE, TENSION_ALTITUDE, TENSION_ALTITUDE, TENSION_ALTITUDE, TENSION_ALTITUDE};
    float wp_z_3[] = {OP_ALTITUDE, OP_ALTITUDE, OP_ALTITUDE, OP_ALTITUDE, OP_ALTITUDE};

    float *wp_x[] = {current_x, wp_x_2, wp_x_3};
    float *wp_y[] = {current_y, current_y, current_y};
    float *wp_z[] = {wp_z_1, wp_z_2, wp_z_3};

    double dt[] = {15.0, 15.0};
    // printf("x waypoint 2: %f, %f, %f, %f, %f\n", wp_x[1][0], wp_x[1][1], wp_x[1][2], wp_x[1][3], wp_x[1][4]);
    __waypoint_trajectory(wp_x, wp_y, wp_z, dt, 3, TAKEOFF_POS);
}

void landing_5_gen(float *current_x, float *current_y, float current_z) {
    float wp_x_2[] = {current_x[0], current_x[1], current_x[2], current_x[3], ((float)(current_x[4]) + (X_OFFSET))};
    float wp_x_3[] = {current_x[0], current_x[1], current_x[2], current_x[3], ((float)(current_x[4]) + (X_OFFSET))};

    float wp_z_1[] = {OP_ALTITUDE, OP_ALTITUDE, OP_ALTITUDE, OP_ALTITUDE, OP_ALTITUDE};
    float wp_z_2[] = {TENSION_ALTITUDE, TENSION_ALTITUDE, TENSION_ALTITUDE, TENSION_ALTITUDE, TENSION_ALTITUDE};
    float wp_z_3[] = {0.0, 0.0, 0.0, 0.0, 0.0};

    float *wp_x[] = {current_x, wp_x_2, wp_x_3};
    float *wp_y[] = {current_y, current_y, current_y};
    float *wp_z[] = {wp_z_1, wp_z_2, wp_z_3};
    
    double dt[] = {15.0, 15.0};
    // printf("x waypoint 2: %f, %f, %f, %f, %f\n", wp_x[1][0], wp_x[1][1], wp_x[1][2], wp_x[1][3], wp_x[1][4]);
    __waypoint_trajectory(wp_x, wp_y, wp_z, dt, 3, LANDING_POS);
}

void test_trajectory(float *current_x, float *current_y, float current_z, int pos) {
    float wp_x_1[] = {current_x[0]};
    float wp_x_2[] = {current_x[0]};
    float wp_x_r1[] = {current_x[0]};

    float wp_x_3[] = {current_x[0] + 1.8};
    float wp_x_r2[] = {current_x[0] + 1.8};

    float wp_x_4[] = {current_x[0] + 1.8};
    float wp_x_r3[] = {current_x[0] + 1.8};

    float wp_x_5[] = {current_x[0] + 2.3};
    float wp_x_6[] = {current_x[0] + 2.3};
    float wp_x_7[] = {current_x[0] + 2.3};
    float wp_x_8[] = {current_x[0] + 3.3};
    float wp_x_r4[] = {current_x[0] + 3.3};

    float wp_x_9[] = {current_x[0] + 3.3};
    float wp_x_10[] = {current_x[0] + 3.3};
    float wp_x_11[] = {current_x[0] + 3.3};
    float wp_x_12[] = {current_x[0] + 3.3};
    float wp_x_r5[] = {current_x[0] + 3.3};

    float wp_x_13[] = {current_x[0] + 2.3};
    float wp_x_14[] = {current_x[0] + 2.3};
    float wp_x_15[] = {current_x[0] + 2.3};
    float wp_x_16[] = {current_x[0] + 1.8};
    float wp_x_r6[] = {current_x[0] + 1.8};

    float wp_x_17[] = {current_x[0] + 1.8};
    float wp_x_r7[] = {current_x[0] + 1.8};

    float wp_x_18[] = {current_x[0]};
    float wp_x_r8[] = {current_x[0]};

    float wp_x_19[] = {current_x[0]};
////////////////////////////////////////////////////////////////
    float wp_y_1[] = {current_y[0]};
    float wp_y_2[] = {current_y[0] + 3.4};
    float wp_y_r1[] = {current_y[0] + 3.4};

    float wp_y_3[] = {current_y[0] + 3.4};
    float wp_y_r2[] = {current_y[0] + 3.4};

    float wp_y_4[] = {current_y[0] + 1.8};
    float wp_y_r3[] = {current_y[0] + 1.8};

    float wp_y_5[] = {current_y[0] + 1.8};
    float wp_y_6[] = {current_y[0] + 1.8};
    float wp_y_7[] = {current_y[0] + 1.8};
    float wp_y_8[] = {current_y[0] + 1.8};
    float wp_y_r4[] = {current_y[0] + 1.8};

    float wp_y_9[] = {current_y[0] + 0.8};
    float wp_y_10[] = {current_y[0] + 0.8};
    float wp_y_11[] = {current_y[0] + 0.8};
    float wp_y_12[] = {current_y[0] - 0.2};
    float wp_y_r5[] = {current_y[0] - 0.2};

    float wp_y_13[] = {current_y[0] - 0.2};
    float wp_y_14[] = {current_y[0] - 0.2};
    float wp_y_15[] = {current_y[0] - 0.2};
    float wp_y_16[] = {current_y[0] - 0.2};
    float wp_y_r6[] = {current_y[0] - 0.2};

    float wp_y_17[] = {current_y[0] - 0.8};
    float wp_y_r7[] = {current_y[0] - 0.8};

    float wp_y_18[] = {current_y[0] - 0.8};
    float wp_y_r8[] = {current_y[0] - 0.8};

    float wp_y_19[] = {current_y[0]};
///////////////////////////////////////////////////////////////
    float wp_z_1[] = {current_z};
    float wp_z_2[] = {current_z};
    float wp_z_r1[] = {current_z};

    float wp_z_3[] = {current_z};
    float wp_z_r2[] = {current_z};

    float wp_z_4[] = {current_z};
    float wp_z_r3[] = {current_z};

    float wp_z_5[] = {current_z};
    float wp_z_6[] = {current_z - 0.8};
    float wp_z_7[] = {current_z};
    float wp_z_8[] = {current_z};
    float wp_z_r4[] = {current_z};

    float wp_z_9[] = {current_z};
    float wp_z_10[] = {current_z - 0.8};
    float wp_z_11[] = {current_z};
    float wp_z_12[] = {current_z};
    float wp_z_r5[] = {current_z};

    float wp_z_13[] = {current_z};
    float wp_z_14[] = {current_z - 0.8};
    float wp_z_15[] = {current_z};
    float wp_z_16[] = {current_z};
    float wp_z_r6[] = {current_z};

    float wp_z_17[] = {current_z};
    float wp_z_r7[] = {current_z};

    float wp_z_18[] = {current_z};
    float wp_z_r8[] = {current_z};

    float wp_z_19[] = {current_z};
///////////////////////////////////////////////////////////////
    float wp_yaw_1 = 0;
    float wp_yaw_2 = 0;

    float wp_yaw_r1 = -M_PI_2;
    float wp_yaw_3 = -M_PI_2;

    float wp_yaw_r2 = -M_PI;
    float wp_yaw_4 = -M_PI;

    float wp_yaw_r3 = -M_PI_2;
    float wp_yaw_5 = -M_PI_2;
    float wp_yaw_6 = -M_PI_2;
    float wp_yaw_7 = -M_PI_2;
    float wp_yaw_8 = -M_PI_2;

    float wp_yaw_r4 = -M_PI;
    float wp_yaw_9 = -M_PI;
    float wp_yaw_10 = -M_PI;
    float wp_yaw_11 = -M_PI;
    float wp_yaw_12 = -M_PI;

    float wp_yaw_r5 = M_PI_2;
    float wp_yaw_13 = M_PI_2;
    float wp_yaw_14 = M_PI_2;
    float wp_yaw_15 = M_PI_2;
    float wp_yaw_16 = M_PI_2;

    float wp_yaw_r6 = M_PI;
    float wp_yaw_17 = M_PI;

    float wp_yaw_r7 = M_PI_2;
    float wp_yaw_18 = M_PI_2;

    float wp_yaw_r8 = 0;
    float wp_yaw_19 = 0;
/////////////////////////////////////////////////////////////////

    float *wp_x[] = {wp_x_1, wp_x_2, wp_x_r1, wp_x_3, wp_x_r2, wp_x_4, wp_x_r3, wp_x_5, wp_x_6, wp_x_7, wp_x_8, wp_x_r4, wp_x_9, wp_x_10, wp_x_11, wp_x_12, wp_x_r5, wp_x_13, wp_x_14, wp_x_15, wp_x_16, wp_x_r6, wp_x_17, wp_x_r7, wp_x_18, wp_x_r8, wp_x_19};
    float *wp_y[] = {wp_y_1, wp_y_2, wp_y_r1, wp_y_3, wp_y_r2, wp_y_4, wp_y_r3, wp_y_5, wp_y_6, wp_y_7, wp_y_8, wp_y_r4, wp_y_9, wp_y_10, wp_y_11, wp_y_12, wp_y_r5, wp_y_13, wp_y_14, wp_y_15, wp_y_16, wp_y_r6, wp_y_17, wp_y_r7, wp_y_18, wp_y_r8, wp_y_19};
    float *wp_z[] = {wp_z_1, wp_z_2, wp_z_r1, wp_z_3, wp_z_r2, wp_z_4, wp_z_r3, wp_z_5, wp_z_6, wp_z_7, wp_z_8, wp_z_r4, wp_z_9, wp_z_10, wp_z_11, wp_z_12, wp_z_r5, wp_z_13, wp_z_14, wp_z_15, wp_z_16, wp_z_r6, wp_z_17, wp_z_r7, wp_z_18, wp_z_r8, wp_z_19};
    float wp_yaw[] = {wp_yaw_1, wp_yaw_2, wp_yaw_r1, wp_yaw_3, wp_yaw_r2, wp_yaw_4, wp_yaw_r3, wp_yaw_5, wp_yaw_6, wp_yaw_7, wp_yaw_8, wp_yaw_r4, wp_yaw_9, wp_yaw_10, wp_yaw_11, wp_yaw_12, wp_yaw_r5, wp_yaw_13, wp_yaw_14, wp_yaw_15, wp_yaw_16, wp_yaw_r6, wp_yaw_17, wp_yaw_r7, wp_yaw_18, wp_yaw_r8, wp_yaw_19};

    double dt[] = {17, 2, 9, 2, 8, 2, 3, 6, 6, 5, 2, 5, 6, 6, 5, 2, 5, 6, 6, 3, 2, 3, 2, 9, 2, 4};

    __waypoint_trajectory(wp_x, wp_y, wp_z, dt, 24, pos);
    __yaw_spline(wp_yaw, dt, 24, pos);
}

static void __dynamic_z_change(float *current_x, float *current_y, float current_z, float target_z, int pos) {
    // current z position
    rc_vector_t z1[NUM_DRONES];
    for (int i = 0; i < NUM_DRONES; i++) {
        z1[i] = RC_VECTOR_INITIALIZER;
        rc_vector_alloc(&z1[i], 3);
        z1[i].d[0] = (double)current_x[i];
        z1[i].d[1] = (double)current_y[i];
        z1[i].d[2] = (double)current_z;
    }
    
    double t1 = 0.0;

    // final z position
    rc_vector_t z2[NUM_DRONES];
    for (int i = 0; i < NUM_DRONES; i++) {
        z2[i] = RC_VECTOR_INITIALIZER;
        rc_vector_alloc(&z2[i], 3);
        z2[i].d[0] = (double)current_x[i];
        z2[i].d[1] = (double)current_y[i];
        z2[i].d[2] = (double)target_z;
    }
    double t2 = 15.0;

    int num_pts = 250;
    
    path_cleanup(pos);
    path[pos].len = num_pts;
    path[pos].waypoints = (waypoint_t*)malloc(sizeof(waypoint_t) * path[pos].len);
    if (path[pos].waypoints == NULL)
    {
        fprintf(stderr, "ERROR: failed allocating memory for path\n");
        // return -1;
    }

    // Start at t=0
    double t_curr = 0;
    double s_curr = 0;
    double v_curr = 0;
    double z_curr = 0;
    double z_dot_curr = 0;

    // Setup Segment #1
    double dx[NUM_DRONES];
    double dy[NUM_DRONES];
    double dz[NUM_DRONES];
    double d_len[NUM_DRONES];
    quintic_spline_1d_t q_spline_1[NUM_DRONES];
    for (int i = 0; i < NUM_DRONES; i++) {
        dx[i] = z2[i].d[0] - z1[i].d[0];
        dy[i] = z2[i].d[1] - z1[i].d[1];
        dz[i] = z2[i].d[2] - z1[i].d[2];
        d_len[i] = sqrt(dx[i]*dx[i] + dy[i]*dy[i] + dz[i]*dz[i]);
        q_spline_1[i] = make_1d_quintic_spline(d_len[i], t2 - t1);
    }

    // Run through Segment #1
    for (int k = 0; k < NUM_DRONES; k++) {
        for (int i=0; i < num_pts; i++) 
        {
            // 1) Get 1d position
            t_curr = ( ((double) i) / ((double) (num_pts-1))) * (t2 - t1);
            s_curr = compute_spline_position(&q_spline_1[k], t_curr);
            v_curr = compute_spline_velocity(&q_spline_1[k], t_curr);

            // 2) Convert to 3d position
            if (d_len[k] > 0) 
            {
                z_curr = (s_curr / d_len[k]) * dz[k]  + z1[k].d[2];
                z_dot_curr = (dz[k] / abs(dz[k])) * v_curr;
            }   
            else
            {
                // If d_len is 0, avoid NaNs
                z_curr = z1[k].d[2];
            }

            

            // 3) Write to the path
            path[pos].waypoints[i].t = t_curr + t1;
            path[pos].waypoints[i].x[k] = current_x[k];
            path[pos].waypoints[i].y[k] = current_y[k];
            path[pos].waypoints[i].z[k] = z_curr;
            path[pos].waypoints[i].x_dot[k] = 0;
            path[pos].waypoints[i].y_dot[k] = 0;
            path[pos].waypoints[i].z_dot[k] = z_dot_curr;
            path[pos].waypoints[i].flag = 0;
        }

    }

    path[pos].initialized = 1;
    // return 0;
}

static void __yaw_spline(float *yaw_waypoints, double *dt, int num_waypoints, int pos) {

    int prev_pts = 0;
    for (int i = 0; i < num_waypoints - 1; i++) {
        float starting_yaw = yaw_waypoints[i];
        float target_yaw = yaw_waypoints[i + 1];
        int num_pts = dt[i] * 20;

        double t_curr = 0;
        double s_curr = 0;
        double v_curr = 0;
        double yaw_curr = 0;
        double yaw_dot_curr = 0;
        double dyaw = 0;
        double d_len = 0;

        quintic_spline_1d_t q_spline_1;
        dyaw = target_yaw - starting_yaw;
        d_len = sqrt(dyaw * dyaw);
        q_spline_1 = make_1d_quintic_spline(d_len, dt[i]);

        printf("starting at  %i\n", prev_pts);
        for (int k = prev_pts; k < num_pts + prev_pts; k++) {
            t_curr = ( ((double) k - prev_pts) / ((double) (num_pts-1))) * (dt[i]);
            s_curr = compute_spline_position(&q_spline_1, t_curr);
            v_curr = compute_spline_velocity(&q_spline_1, t_curr);

            printf("d_len yaw: %f\n", d_len);

            if (d_len > 0)
            {
                yaw_curr = (s_curr / d_len) * dyaw + starting_yaw;
                if (abs(dyaw) > 0) {
                    yaw_dot_curr = (dyaw / abs(dyaw)) * v_curr;
                }
                else {
                    yaw_dot_curr = 0.0;
                }
            }
            else {
                yaw_curr = starting_yaw;
                yaw_dot_curr = 0.0;
            }

            path[pos].waypoints[k].t = t_curr;
            for (int j = 0; j < NUM_DRONES; j++) {
                path[pos].waypoints[k].yaw[j] = yaw_curr;
                path[pos].waypoints[k].yaw_dot[j] = yaw_dot_curr;
            }
            path[pos].waypoints[k].yaw_flag = 1;
            printf("yaw waypoint %i: pos %f vel %f\n", k, path[pos].waypoints[k].yaw[0], path[pos].waypoints[k].yaw_dot[0]);            
        }
        prev_pts += num_pts;

    }
}

static void __waypoint_trajectory(float **x_waypoints, float **y_waypoints, float **z_waypoints, double *dt, int num_waypoints, int pos ) {
    path_t path_fragments[num_waypoints - 1];
    for (int i = 0; i < (num_waypoints - 1); i++) {
        // free(path_fragments[i].waypoints);
        path_fragments[i].waypoints = NULL;
        path_fragments[i].len = 0;
        path_fragments[i].initialized = 0;
        path_fragments[i] = PATH_INITIALIZER;
        __dynamic_pos_change(path_fragments[i], x_waypoints[i], y_waypoints[i], z_waypoints[i], x_waypoints[i + 1], y_waypoints[i + 1], z_waypoints[i + 1], dt[i]);
        for (int j = 0; j < (int)path_fragments[i].len; j++) {
            printf("path %i 1 waypoint %i: %f, %f, %f\n", i, j, path_fragments[i].waypoints[j].x[0], path_fragments[i].waypoints[j].y[0], path_fragments[i].waypoints[j].z[0]);
        }
    }
    
    int size = 0;
    for (int i = 0; i < (num_waypoints - 1); i++) {
        size += path_fragments[i].len;
    }
    printf("my size is %i\n", size);
    
    path_cleanup(pos);
    path[pos].len = size;
    path[pos].waypoints = (waypoint_t*)malloc(sizeof(waypoint_t) * path[pos].len);
    if (path[pos].waypoints == NULL)
    {
        fprintf(stderr, "ERROR: failed allocating memory for path\n");
        // return -1;
    }

    int acc = 0;
    for (int i = 0; i < (num_waypoints - 1); i++) {
        for (int k = 0; k < (int)path_fragments[i].len; k++) {
            if (acc == 0) {
                path[pos].waypoints[k + acc].t = path_fragments[i].waypoints[k].t;
            }
            else {
                path[pos].waypoints[k + acc].t = path[pos].waypoints[acc - 1].t + path_fragments[i].waypoints[k].t;
            }
            for (int p = 0; p < NUM_DRONES; p++) {
                // printf("drone %i on path %i fed waypoint %f, %f, %f\n", p, i, path_fragments[i].waypoints[k].x[p], path_fragments[i].waypoints[k].y[p], path_fragments[i].waypoints[k].z[p]);
                path[pos].waypoints[k + acc].x[p] = path_fragments[i].waypoints[k].x[p];
                path[pos].waypoints[k + acc].y[p] = path_fragments[i].waypoints[k].y[p];
                path[pos].waypoints[k + acc].z[p] = path_fragments[i].waypoints[k].z[p];
                path[pos].waypoints[k + acc].x_dot[p] = path_fragments[i].waypoints[k].x_dot[p];
                path[pos].waypoints[k + acc].y_dot[p] = path_fragments[i].waypoints[k].y_dot[p];
                path[pos].waypoints[k + acc].z_dot[p] = path_fragments[i].waypoints[k].z_dot[p];
                // printf("drone %i on path %i fed waypoint %f, %f, %f\n", p, i, path[pos].waypoints[k].x[p], path_fragments[i].waypoints[k].y[p], path_fragments[i].waypoints[k].z[p]);
            }
            path[pos].waypoints[i].flag = 0;
        }
        acc += path_fragments[i].len;
    }
}

static void __dynamic_pos_change(path_t &dynamic_path, float *current_x, float *current_y, float *current_z, float *target_x, float *target_y, float *target_z, double dt) {
    
    // 1.1 Initialize current drone positions vectors
    rc_vector_t x1[NUM_DRONES];
    for (int i = 0; i < NUM_DRONES; i++) {
        x1[i] = RC_VECTOR_INITIALIZER;
        rc_vector_alloc(&x1[i], 3);

        x1[i].d[0] = current_x[i];
        x1[i].d[1] = current_y[i];
        x1[i].d[2] = current_z[i];
        // printf("waypoint for %i: %f, %f, %f\n", i, x1[i].d[0], x1[i].d[1], x1[i].d[2]);
    }

    // 1.2 Intiialize target drone positions vectors
    rc_vector_t x2[NUM_DRONES];
    for (int i = 0; i < NUM_DRONES; i++) {
        x2[i] = RC_VECTOR_INITIALIZER;
        rc_vector_alloc(&x2[i], 3);

        x2[i].d[0] = target_x[i];
        x2[i].d[1] = target_y[i];
        x2[i].d[2] = target_z[i];
    }
    // x2[4].d[2] += VERT_OFFSET;
    
    // 2 Normalize number of points in spline to message send rate
    int num_pts = dt * 20; 

    dynamic_path.len = num_pts;
    dynamic_path.waypoints = (waypoint_t*)malloc(sizeof(waypoint_t) * dynamic_path.len);
    if (dynamic_path.waypoints == NULL)
    {
        fprintf(stderr, "ERROR: failed allocating memory for path\n");
        // return -1;
    }

    // 3 Declare the variables we use in the spline process
    double t_curr = 0;
    double s_curr = 0;
    double v_curr = 0;
    double x_curr = 0;
    double x_dot_curr = 0;
    double y_curr = 0;
    double y_dot_curr = 0;
    double z_curr = 0;
    double z_dot_curr = 0;

    double dx[NUM_DRONES];
    double dy[NUM_DRONES];
    double dz[NUM_DRONES];
    double d_len[NUM_DRONES];

    // 4 Create the spline based on the deltas
    quintic_spline_1d_t q_spline_1[NUM_DRONES];
    for (int i = 0; i < NUM_DRONES; i++) {
        dx[i] = x2[i].d[0] - x1[i].d[0];
        dy[i] = x2[i].d[1] - x1[i].d[1];
        dz[i] = x2[i].d[2] - x1[i].d[2];
        d_len[i] = sqrt(dx[i]*dx[i] + dy[i]*dy[i] + dz[i]*dz[i]);
        q_spline_1[i] = make_1d_quintic_spline(d_len[i], dt);
    }

    // 5 Project the spline position and velocity onto x, y, and z to obtain waypoints
    for (int k = 0; k < NUM_DRONES; k++) {
        for (int i = 0; i < num_pts; i++) {
            t_curr = ( ((double) i) / ((double) (num_pts-1))) * (dt);
            s_curr = compute_spline_position(&q_spline_1[k], t_curr);
            v_curr = compute_spline_velocity(&q_spline_1[k], t_curr);

             if (d_len[k] > 0) 
            {
                x_curr = (s_curr / d_len[k]) * dx[k] + x1[k].d[0];
                if (abs(dx[k]) > 0) {
                    x_dot_curr = (dx[k] / abs(dx[k])) * v_curr;
                }
                else {
                    x_dot_curr = 0.0;
                }               
                y_curr = (s_curr / d_len[k]) * dy[k]  + x1[k].d[1];
                if (abs(dy[k]) > 0) {
                    y_dot_curr = (dy[k] / abs(dy[k])) * v_curr;
                }
                else {
                    y_dot_curr = 0.0;
                }
                z_curr = (s_curr / d_len[k]) * dz[k]  + x1[k].d[2];
                if (abs(dz[k]) > 0) {
                    z_dot_curr = (dz[k] / abs(dz[k])) * v_curr;
                }
                else {
                    z_dot_curr = 0.0;
                }
            }   
            else
            {
                x_curr = x1[k].d[0];
                y_curr = x1[k].d[1];
                z_curr = x1[k].d[2];
                x_dot_curr = 0;
                y_dot_curr = 0;
                z_dot_curr = 0;
            }

            // 6 Write the waypoints
            dynamic_path.waypoints[i].t = t_curr;
            dynamic_path.waypoints[i].x[k] = x_curr;
            dynamic_path.waypoints[i].y[k] = y_curr;
            dynamic_path.waypoints[i].z[k] = z_curr;
            dynamic_path.waypoints[i].x_dot[k] = x_dot_curr;
            dynamic_path.waypoints[i].y_dot[k] = y_dot_curr;
            dynamic_path.waypoints[i].z_dot[k] = z_dot_curr;
            dynamic_path.waypoints[i].flag = 0;
            // printf("waypoint for %i: %f, %f, %f\n", k, dynamic_path.waypoints[i].x[k], dynamic_path.waypoints[i].y[k], dynamic_path.waypoints[i].z[k]);
        }
    }

}

quintic_spline_1d_t make_1d_quintic_spline(float dx, float dt)
{
    quintic_spline_1d_t simple_quintic_spline;

    simple_quintic_spline.c0 = 0;
    simple_quintic_spline.c1 = 0;
    simple_quintic_spline.c2 = 0;
    simple_quintic_spline.c3 =  10 * dx / pow(dt,3);
    simple_quintic_spline.c4 = -15 * dx / pow(dt,4);
    simple_quintic_spline.c5 =   6 * dx / pow(dt,5);

    return simple_quintic_spline;
}

float compute_spline_position(quintic_spline_1d_t* the_spline, float t)
{
    return the_spline->c0 
         + the_spline->c3 * pow(t,3)
         + the_spline->c4 * pow(t,4)
         + the_spline->c5 * pow(t,5);
}

float compute_spline_velocity(quintic_spline_1d_t* the_spline, float t)
{
    return 0
         + 3*(the_spline->c3) * pow(t,2)
         + 4*(the_spline->c4) * pow(t,3)
         + 5*(the_spline->c5) * pow(t,4);
}