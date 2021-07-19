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
    // current position
    rc_vector_t x1[NUM_DRONES];
    for (int i = 0; i < NUM_DRONES; i++) {
        x1[i] = RC_VECTOR_INITIALIZER;
        rc_vector_alloc(&x1[i], 3);
        x1[i].d[0] = (double)current_x[i];
        x1[i].d[1] = (double)current_y[i];
        x1[i].d[2] = 0.0;
    }
    
    double t1 = 0.0;
    /////////////////NEEDS REFINEMENT/////////////////
    double target_z = TENSION_ALTITUDE;
    /////////////////////////////////////////////////

    // 2nd z position
    printf("starting 2nd z pos\n");
    rc_vector_t x2[NUM_DRONES];
    for (int i = 0; i < NUM_DRONES; i++) {
        x2[i] = RC_VECTOR_INITIALIZER;
        rc_vector_alloc(&x2[i], 3);
        x2[i].d[0] = (double)current_x[i];
        x2[i].d[1] = (double)current_y[i];
        x2[i].d[2] = (double)target_z;
    }
    if (NUM_DRONES == 5) {
        x2[4].d[0] = (double)current_x[4];
        x2[4].d[1] = (double)current_y[4];
        x2[4].d[2] = (double)target_z;
    }

    double t2 = 5.0;

    int num_pts_1 = 100;
    int num_pts_2 = 100;
    int num_pts_3 = 100;
    int num_pts_4 = 100;
    
    path_cleanup(TAKEOFF_POS);
    path[TAKEOFF_POS].len = num_pts_1 + num_pts_2 + num_pts_3 + num_pts_4;
    path[TAKEOFF_POS].waypoints = (waypoint_t*)malloc(sizeof(waypoint_t) * path[TAKEOFF_POS].len);
    if (path[TAKEOFF_POS].waypoints == NULL)
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
    printf("starting first spline\n");
    for (int i = 0; i < NUM_DRONES; i++) {
        dx[i] = x2[i].d[0] - x1[i].d[0];
        dy[i] = x2[i].d[1] - x1[i].d[1];
        dz[i] = x2[i].d[2] - x1[i].d[2];
        d_len[i] = sqrt(dx[i]*dx[i] + dy[i]*dy[i] + dz[i]*dz[i]);
        q_spline_1[i] = make_1d_quintic_spline(d_len[i], t2 - t1);
    }

    // Run through Segment #1
    for (int k = 0; k < NUM_DRONES; k++) {
        for (int i=0; i < num_pts_1; i++) 
        {
            // 1) Get 1d position
            t_curr = ( ((double) i) / ((double) (num_pts_1-1))) * (t2 - t1);
            s_curr = compute_spline_position(&q_spline_1[k], t_curr);
            v_curr = compute_spline_velocity(&q_spline_1[k], t_curr);

            // 2) Convert to 3d position
            if (d_len[k] > 0) 
            {
                z_curr = (s_curr / d_len[k]) * dz[k]  + x1[k].d[2];
                z_dot_curr = (dz[k] / abs(dz[k])) * v_curr;
            }   
            else
            {
                // If d_len is 0, avoid NaNs
                z_curr = x1[k].d[2];
            }
            

            // 3) Write to the path
            path[TAKEOFF_POS].waypoints[i].t = t_curr + t1;
            path[TAKEOFF_POS].waypoints[i].x[k] = current_x[k];
            path[TAKEOFF_POS].waypoints[i].y[k] = current_y[k];
            path[TAKEOFF_POS].waypoints[i].z[k] = z_curr;
            path[TAKEOFF_POS].waypoints[i].x_dot[k] = 0;
            path[TAKEOFF_POS].waypoints[i].y_dot[k] = 0;
            path[TAKEOFF_POS].waypoints[i].z_dot[k] = z_dot_curr;
            path[TAKEOFF_POS].waypoints[i].flag = 0;
        }
        printf("spline %i done\n", k);

    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Perform y translation to reach true tensioning point
    rc_vector_t x3[NUM_DRONES];
    for (int i = 0; i < NUM_DRONES; i++) {
        x3[i] = RC_VECTOR_INITIALIZER;
        rc_vector_alloc(&x3[i], 3);
        x3[i].d[0] = (double)current_x[i];
        x3[i].d[1] = (double)current_y[i];
        x3[i].d[2] = (double)target_z;
    }
    if (NUM_DRONES == 5) {
        x3[4].d[0] = (double)0.0;
        x3[4].d[1] = (double)0.0;
        x3[4].d[2] = (double)target_z;
    }
    double t3 = 10.0;

    // Start at t=0
    t_curr = 0;
    s_curr = 0;
    v_curr = 0;
    z_curr = target_z;
    z_dot_curr = 0;

    // Setup Segment #2
    quintic_spline_1d_t q_spline_2[NUM_DRONES];
    for (int i = 0; i < NUM_DRONES; i++) {
        dx[i] = x3[i].d[0] - x2[i].d[0];
        dy[i] = x3[i].d[1] - x2[i].d[1];
        dz[i] = x3[i].d[2] - x2[i].d[2];
        d_len[i] = sqrt(dx[i]*dx[i] + dy[i]*dy[i] + dz[i]*dz[i]);
        q_spline_2[i] = make_1d_quintic_spline(d_len[i], t3 - t2);
    }

    // Run through Segment #2
    for (int k = 0; k < NUM_DRONES; k++) {
        for (int i=num_pts_1; i < (num_pts_2+num_pts_1); i++) 
        {
            // 1) Get 1d position
            t_curr = ( ((double) (i - num_pts_1)) / ((double) (num_pts_2-1))) * (t3 - t2);
            s_curr = compute_spline_position(&q_spline_2[k], t_curr);
            v_curr = compute_spline_velocity(&q_spline_2[k], t_curr);

            // 2) Convert to 3d position
            if (d_len[k] > 0) 
            {
                z_curr = (s_curr / d_len[k]) * dz[k]  + x2[k].d[2];
                z_dot_curr = (dz[k] / abs(dz[k])) * v_curr;
            }   
            else
            {
                // If d_len is 0, avoid NaNs
                z_curr = x2[k].d[2];
            }
            

            // 3) Write to the path
            path[TAKEOFF_POS].waypoints[i].t = t_curr + t2;
            path[TAKEOFF_POS].waypoints[i].x[k] = current_x[k];
            path[TAKEOFF_POS].waypoints[i].y[k] = current_y[k];
            path[TAKEOFF_POS].waypoints[i].z[k] = z_curr;
            path[TAKEOFF_POS].waypoints[i].x_dot[k] = 0;
            path[TAKEOFF_POS].waypoints[i].y_dot[k] = 0;
            path[TAKEOFF_POS].waypoints[i].z_dot[k] = 0;
            path[TAKEOFF_POS].waypoints[i].flag = 0;
        }
        printf("spline %i done\n", k);
    }
    for (int i=num_pts_1; i < (num_pts_2+num_pts_1); i++) 
    {
        int k = 4;
        double y_curr;
        double y_dot_curr;
        // 1) Get 1d position
        t_curr = ( ((double) (i - ((num_pts_1))) / ((double) (num_pts_2-1))) * (t3 - t2));
        s_curr = compute_spline_position(&q_spline_2[k], t_curr);
        v_curr = compute_spline_velocity(&q_spline_2[k], t_curr);

        // 2) Convert to 3d position
        if (d_len > 0) 
        {
            y_curr = (s_curr / d_len[k]) * dy[k]  + x2[k].d[1];
            y_dot_curr = (dy[k] / abs(dy[k])) * v_curr;
        }   
        else
        {
            // If d_len is 0, avoid NaNs
            y_curr = x2[k].d[1];
        }
        

        // 3) Write to the path
        path[TAKEOFF_POS].waypoints[i].y[k] = y_curr;
        path[TAKEOFF_POS].waypoints[i].y_dot[k] = y_dot_curr;
    }
    printf("extra spline\n");


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 3rd z position
    rc_vector_t x4[NUM_DRONES];
    for (int i = 0; i < NUM_DRONES; i++) {
        x4[i] = RC_VECTOR_INITIALIZER;
        rc_vector_alloc(&x4[i], 3);
        x4[i].d[0] = (double)current_x[i];
        x4[i].d[1] = (double)current_y[i];
        x4[i].d[2] = (double)target_z;
    }
    if (NUM_DRONES == 5) {
        x4[4].d[0] = (double)0.0;
        x4[4].d[1] = (double)0.0;
        x4[4].d[2] = (double)target_z + VERT_OFFSET;
    }
    double t4 = 15.0;

    // Start at t=0
    t_curr = 0;
    s_curr = 0;
    v_curr = 0;
    z_curr = 0;
    z_dot_curr = 0;

    printf("starting last but one spline setup\n");
    // Setup Segment #2
    quintic_spline_1d_t q_spline_3[NUM_DRONES];
    for (int i = 0; i < NUM_DRONES; i++) {
        dx[i] = x4[i].d[0] - x3[i].d[0];
        dy[i] = x4[i].d[1] - x3[i].d[1];
        dz[i] = x4[i].d[2] - x3[i].d[2];
        d_len[i] = sqrt(dx[i]*dx[i] + dy[i]*dy[i] + dz[i]*dz[i]);
        q_spline_3[i] = make_1d_quintic_spline(d_len[i], t4 - t3);
    }

    printf("last spline set up\n");

    // Run through Segment #2
    for (int k = 0; k < NUM_DRONES; k++) {
        for (int i=(num_pts_1 + num_pts_2); i < (num_pts_1 + num_pts_2 + num_pts_3); i++) 
        {
            // 1) Get 1d position
            t_curr = ( ((double) (i - ((num_pts_1 + num_pts_2))) / ((double) (num_pts_3-1))) * (t4 - t3));
            s_curr = compute_spline_position(&q_spline_3[k], t_curr);
            v_curr = compute_spline_velocity(&q_spline_3[k], t_curr);

            // 2) Convert to 3d position
            if (d_len > 0) 
            {
                z_curr = (s_curr / d_len[k]) * dz[k]  + x3[k].d[2];
                z_dot_curr = (dz[k] / abs(dz[k])) * v_curr;
            }   
            else
            {
                // If d_len is 0, avoid NaNs
                z_curr = x3[k].d[2];
            }
            

            // 3) Write to the path
            path[TAKEOFF_POS].waypoints[i].t = t_curr + t2;
            if (k != 4) {
                path[TAKEOFF_POS].waypoints[i].x[k] = x4[k].d[0];
                path[TAKEOFF_POS].waypoints[i].y[k] = x4[k].d[1];
                path[TAKEOFF_POS].waypoints[i].z[k] = x4[k].d[2];
                path[TAKEOFF_POS].waypoints[i].z_dot[k] = 0;
            }
            else {
                path[TAKEOFF_POS].waypoints[i].x[k] = x4[k].d[0];
                path[TAKEOFF_POS].waypoints[i].y[k] = x4[k].d[1];
                path[TAKEOFF_POS].waypoints[i].z[k] = z_curr;
                path[TAKEOFF_POS].waypoints[i].z_dot[k] = z_dot_curr;
            }
            
            path[TAKEOFF_POS].waypoints[i].x_dot[k] = 0;
            path[TAKEOFF_POS].waypoints[i].y_dot[k] = 0;
            path[TAKEOFF_POS].waypoints[i].flag = 0;
        }

    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 4rd z position
    rc_vector_t x5[NUM_DRONES];
    double target_z_2 = OP_ALTITUDE;
    for (int i = 0; i < NUM_DRONES; i++) {
        x5[i] = RC_VECTOR_INITIALIZER;
        rc_vector_alloc(&x5[i], 3);
        x5[i].d[0] = (double)current_x[i];
        x5[i].d[1] = (double)current_y[i];
        x5[i].d[2] = (double)target_z_2;
    }
    if (NUM_DRONES == 5) {
        x5[4].d[0] = (double)0.0;
        x5[4].d[1] = (double)0.0;
        x5[4].d[2] = (double)target_z_2 + VERT_OFFSET;
    }
    double t5 = 20.0;

    // Start at t=0
    t_curr = 0;
    s_curr = 0;
    v_curr = 0;
    z_curr = 0;
    z_dot_curr = 0;

    printf("starting last spline setup\n");
    // Setup Segment #2
    quintic_spline_1d_t q_spline_4[NUM_DRONES];
    for (int i = 0; i < NUM_DRONES; i++) {
        dx[i] = x5[i].d[0] - x4[i].d[0];
        dy[i] = x5[i].d[1] - x4[i].d[1];
        dz[i] = x5[i].d[2] - x4[i].d[2];
        d_len[i] = sqrt(dx[i]*dx[i] + dy[i]*dy[i] + dz[i]*dz[i]);
        q_spline_4[i] = make_1d_quintic_spline(d_len[i], t5 - t4);
    }

    printf("last spline set up\n");

    // Run through Segment #2
    for (int k = 0; k < NUM_DRONES; k++) {
        for (int i=(num_pts_1 + num_pts_2 + num_pts_3); i < (num_pts_1 + num_pts_2 + num_pts_3 + num_pts_4); i++) 
        {
            // 1) Get 1d position
            t_curr = ( ((double) (i - ((num_pts_1 + num_pts_2 + num_pts_3))) / ((double) (num_pts_4-1))) * (t5 - t4));
            s_curr = compute_spline_position(&q_spline_4[k], t_curr);
            v_curr = compute_spline_velocity(&q_spline_4[k], t_curr);

            // 2) Convert to 3d position
            if (d_len > 0) 
            {
                z_curr = (s_curr / d_len[k]) * dz[k]  + x4[k].d[2];
                z_dot_curr = (dz[k] / abs(dz[k])) * v_curr;
            }   
            else
            {
                // If d_len is 0, avoid NaNs
                z_curr = x4[k].d[2];
            }
            

            // 3) Write to the path
            path[TAKEOFF_POS].waypoints[i].t = t_curr + t2;
            path[TAKEOFF_POS].waypoints[i].x[k] = x5[k].d[0];
            path[TAKEOFF_POS].waypoints[i].y[k] = x5[k].d[1];
            path[TAKEOFF_POS].waypoints[i].z[k] = z_curr;
            path[TAKEOFF_POS].waypoints[i].x_dot[k] = 0;
            path[TAKEOFF_POS].waypoints[i].y_dot[k] = 0;
            path[TAKEOFF_POS].waypoints[i].z_dot[k] = z_dot_curr;
            path[TAKEOFF_POS].waypoints[i].flag = 0;
        }

    }

    path[TAKEOFF_POS].initialized = 1;
    // return 0;
}

void landing_5_gen(float *current_x, float *current_y, float current_z) {
    // current position
    rc_vector_t x1[NUM_DRONES];
    for (int i = 0; i < NUM_DRONES; i++) {
        x1[i] = RC_VECTOR_INITIALIZER;
        rc_vector_alloc(&x1[i], 3);
        x1[i].d[0] = (double)current_x[i];
        x1[i].d[1] = (double)current_y[i];
        x1[i].d[2] = OP_ALTITUDE;
    }
    if (NUM_DRONES == 5) {
        x1[NUM_DRONES - 1].d[2] = OP_ALTITUDE + VERT_OFFSET;
    }
    
    double t1 = 0.0;
    /////////////////NEEDS REFINEMENT/////////////////
    double target_z = TENSION_ALTITUDE;
    /////////////////////////////////////////////////

    // 2nd z position
    printf("starting 2nd z pos\n");
    rc_vector_t x2[NUM_DRONES];
    for (int i = 0; i < NUM_DRONES; i++) {
        x2[i] = RC_VECTOR_INITIALIZER;
        rc_vector_alloc(&x2[i], 3);
        x2[i].d[0] = (double)current_x[i];
        x2[i].d[1] = (double)current_y[i];
        x2[i].d[2] = (double)target_z;
    }
    if (NUM_DRONES == 5) {
        x2[4].d[0] = (double)current_x[4];
        x2[4].d[1] = (double)current_y[4];
        x2[4].d[2] = (double)target_z;
        printf("\n x2 at %f %f %f\n", x2[4].d[0], x2[4].d[1], x2[4].d[2]);
    }

    double t2 = 5.0;

    int num_pts_1 = 100;
    int num_pts_2 = 100;
    int num_pts_3 = 100;
    
    path_cleanup(LANDING_POS);
    path[LANDING_POS].len = num_pts_1 + num_pts_2 + num_pts_3;
    path[LANDING_POS].waypoints = (waypoint_t*)malloc(sizeof(waypoint_t) * path[LANDING_POS].len);
    if (path[LANDING_POS].waypoints == NULL)
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
    printf("starting first spline\n");
    for (int i = 0; i < NUM_DRONES; i++) {
        dx[i] = x2[i].d[0] - x1[i].d[0];
        dy[i] = x2[i].d[1] - x1[i].d[1];
        dz[i] = x2[i].d[2] - x1[i].d[2];
        d_len[i] = sqrt(dx[i]*dx[i] + dy[i]*dy[i] + dz[i]*dz[i]);
        q_spline_1[i] = make_1d_quintic_spline(d_len[i], t2 - t1);
    }

    // Run through Segment #1
    for (int k = 0; k < NUM_DRONES; k++) {
        for (int i=0; i < num_pts_1; i++) 
        {
            // 1) Get 1d position
            t_curr = ( ((double) i) / ((double) (num_pts_1-1))) * (t2 - t1);
            s_curr = compute_spline_position(&q_spline_1[k], t_curr);
            v_curr = compute_spline_velocity(&q_spline_1[k], t_curr);

            // 2) Convert to 3d position
            if (d_len[k] > 0) 
            {
                z_curr = (s_curr / d_len[k]) * dz[k]  + x1[k].d[2];
                z_dot_curr = (dz[k] / abs(dz[k])) * v_curr;
            }   
            else
            {
                // If d_len is 0, avoid NaNs
                z_curr = x1[k].d[2];
            }
            

            // 3) Write to the path
            path[LANDING_POS].waypoints[i].t = t_curr + t1;
            path[LANDING_POS].waypoints[i].x[k] = current_x[k];
            path[LANDING_POS].waypoints[i].y[k] = current_y[k];
            path[LANDING_POS].waypoints[i].z[k] = z_curr;
            path[LANDING_POS].waypoints[i].x_dot[k] = 0;
            path[LANDING_POS].waypoints[i].y_dot[k] = 0;
            path[LANDING_POS].waypoints[i].z_dot[k] = z_dot_curr;
            path[LANDING_POS].waypoints[i].flag = 0;
        }
        printf("spline %i done\n", k);

    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    rc_vector_t x3[NUM_DRONES];
    for (int i = 0; i < NUM_DRONES; i++) {
        x3[i] = RC_VECTOR_INITIALIZER;
        rc_vector_alloc(&x3[i], 3);
        x3[i].d[0] = (double)current_x[i];
        x3[i].d[1] = (double)current_y[i];
        x3[i].d[2] = (double)target_z;
    }
    if (NUM_DRONES == 5) {
        x3[4].d[0] = (double)current_x[4];
        x3[4].d[1] = ((double)current_y[4]) + 2.0;
        x3[4].d[2] = (double)target_z;
    }
    printf("\n x3 at %f %f %f\n", x3[4].d[0], x3[4].d[1], x3[4].d[2]);
    double t3 = 10.0;


    // Start at t=0
    t_curr = 0;
    s_curr = 0;
    v_curr = 0;
    z_curr = 0;
    z_dot_curr = 0;

    // Setup Segment #2
    quintic_spline_1d_t q_spline_2[NUM_DRONES];
    for (int i = 0; i < NUM_DRONES; i++) {
        dx[i] = x3[i].d[0] - x2[i].d[0];
        dy[i] = x3[i].d[1] - x2[i].d[1];
        dz[i] = x3[i].d[2] - x2[i].d[2];
        d_len[i] = sqrt(dx[i]*dx[i] + dy[i]*dy[i] + dz[i]*dz[i]);
        q_spline_2[i] = make_1d_quintic_spline(d_len[i], t3 - t2);
    }

    // Run through Segment #2
    for (int k = 0; k < NUM_DRONES; k++) {
        for (int i=num_pts_1; i < (num_pts_2+num_pts_1); i++) 
        {
            // 1) Get 1d position
            t_curr = ( ((double) (i - num_pts_1)) / ((double) (num_pts_2-1))) * (t3 - t2);
            s_curr = compute_spline_position(&q_spline_2[k], t_curr);
            v_curr = compute_spline_velocity(&q_spline_2[k], t_curr);

            // 2) Convert to 3d position
            if (d_len[k] > 0) 
            {
                z_curr = (s_curr / d_len[k]) * dz[k]  + x2[k].d[2];
                z_dot_curr = (dz[k] / abs(dz[k])) * v_curr;
            }   
            else
            {
                // If d_len is 0, avoid NaNs
                z_curr = x2[k].d[2];
            }
            

            // 3) Write to the path
            path[LANDING_POS].waypoints[i].t = t_curr + t2;
            path[LANDING_POS].waypoints[i].x[k] = current_x[k];
            path[LANDING_POS].waypoints[i].y[k] = current_y[k];
            path[LANDING_POS].waypoints[i].z[k] = z_curr;
            path[LANDING_POS].waypoints[i].x_dot[k] = 0;
            path[LANDING_POS].waypoints[i].y_dot[k] = 0;
            path[LANDING_POS].waypoints[i].z_dot[k] = 0;
            path[LANDING_POS].waypoints[i].flag = 0;
        }
        printf("spline %i done\n", k);
    }
    for (int i=num_pts_1; i < (num_pts_2+num_pts_1); i++) 
    {
        
        int k = 4;
        double y_curr;
        double y_dot_curr;
        // 1) Get 1d position
        t_curr = ( ((double) (i - ((num_pts_1))) / ((double) (num_pts_2-1))) * (t3 - t2));
        s_curr = compute_spline_position(&q_spline_2[k], t_curr);
        v_curr = compute_spline_velocity(&q_spline_2[k], t_curr);

        // 2) Convert to 3d position
        // if (d_len > 0) 
        // {
            y_curr = (s_curr / d_len[k]) * dy[k]  + x2[k].d[1];
            y_dot_curr = (dy[k] / abs(dy[k])) * v_curr;
        // }   
        // else
        // {
        //     // If d_len is 0, avoid NaNs
        //     y_curr = x2[k].d[1];
        // }
        

        // 3) Write to the path
        path[LANDING_POS].waypoints[i].y[k] = y_curr;
        path[LANDING_POS].waypoints[i].y_dot[k] = y_dot_curr;
    }
    printf("extra spline\n");


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // 3rd z position
    rc_vector_t x4[NUM_DRONES];
    for (int i = 0; i < NUM_DRONES; i++) {
        x4[i] = RC_VECTOR_INITIALIZER;
        rc_vector_alloc(&x4[i], 3);
        x4[i].d[0] = (double)current_x[i];
        x4[i].d[1] = (double)current_y[i];
        x4[i].d[2] = 0.0;
    }
    if (NUM_DRONES == 5) {
        x4[4].d[0] = 0.0;
        x4[4].d[1] = 2.0;
        x4[4].d[2] = 0.0;
    }
    double t4 = 15.0;

    // Start at t=0
    t_curr = 0;
    s_curr = 0;
    v_curr = 0;
    z_curr = 0;
    z_dot_curr = 0;

    printf("starting last but one spline setup\n");
    // Setup Segment #2
    quintic_spline_1d_t q_spline_3[NUM_DRONES];
    for (int i = 0; i < NUM_DRONES; i++) {
        dx[i] = x4[i].d[0] - x3[i].d[0];
        dy[i] = x4[i].d[1] - x3[i].d[1];
        dz[i] = x4[i].d[2] - x3[i].d[2];
        d_len[i] = sqrt(dx[i]*dx[i] + dy[i]*dy[i] + dz[i]*dz[i]);
        q_spline_3[i] = make_1d_quintic_spline(d_len[i], t4 - t3);
    }

    printf("last spline set up\n");

    // Run through Segment #2
    for (int k = 0; k < NUM_DRONES; k++) {
        for (int i=(num_pts_1 + num_pts_2); i < (num_pts_1 + num_pts_2 + num_pts_3); i++) 
        {
            // 1) Get 1d position
            t_curr = ( ((double) (i - ((num_pts_1 + num_pts_2))) / ((double) (num_pts_3-1))) * (t4 - t3));
            s_curr = compute_spline_position(&q_spline_3[k], t_curr);
            v_curr = compute_spline_velocity(&q_spline_3[k], t_curr);

            // 2) Convert to 3d position
            if (d_len > 0) 
            {
                z_curr = (s_curr / d_len[k]) * dz[k]  + x3[k].d[2];
                z_dot_curr = (dz[k] / abs(dz[k])) * v_curr;
            }   
            else
            {
                // If d_len is 0, avoid NaNs
                z_curr = x3[k].d[2];
            }
            

            // 3) Write to the path
            path[LANDING_POS].waypoints[i].t = t_curr + t2;
            if (k != 4) {
                path[LANDING_POS].waypoints[i].x[k] = x3[k].d[0];
                path[LANDING_POS].waypoints[i].y[k] = x3[k].d[1];
                path[LANDING_POS].waypoints[i].z[k] = z_curr;
                path[LANDING_POS].waypoints[i].z_dot[k] = z_dot_curr;
            }
            else {
                path[LANDING_POS].waypoints[i].x[k] = x3[k].d[0];
                path[LANDING_POS].waypoints[i].y[k] = x3[k].d[1];
                path[LANDING_POS].waypoints[i].z[k] = z_curr;
                path[LANDING_POS].waypoints[i].z_dot[k] = z_dot_curr;
            }
            
            path[LANDING_POS].waypoints[i].x_dot[k] = 0;
            path[LANDING_POS].waypoints[i].y_dot[k] = 0;
            path[LANDING_POS].waypoints[i].flag = 0;
        }

    }

    path[LANDING_POS].initialized = 1;
    // return 0;

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
    double t2 = 5.0;

    int num_pts = 100;
    
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
            if (d_len > 0) 
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