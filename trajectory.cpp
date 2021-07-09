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

static void __dynamic_z_change(float current_x, float current_y, float current_z, float target_z, int pos);
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

void takeoff_gen(float current_z) {
    __dynamic_z_change(0, 0, current_z, -4.5, TAKEOFF_POS);
}

void landing_gen(float current_x, float current_y, float current_z) {
    __dynamic_z_change(current_x, current_y, current_z, 0, LANDING_POS);
}

static void __dynamic_z_change(float current_x, float current_y, float current_z, float target_z, int pos) {
    // current z position
    rc_vector_t z1 = RC_VECTOR_INITIALIZER;
    rc_vector_alloc(&z1, 3);
    z1.d[0] = (double)current_x;
    z1.d[1] = (double)current_y;
    z1.d[2] = (double)current_z;
    double t1 = 0.0;

    // final z position
    rc_vector_t z2 = RC_VECTOR_INITIALIZER;
    rc_vector_alloc(&z2, 3);
    z2.d[0] = (double)current_x;
    z2.d[1] = (double)current_y;
    z2.d[2] = (double)target_z;
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
    double dx = z2.d[0] - z1.d[0];
    double dy = z2.d[1] - z1.d[1];
    double dz = z2.d[2] - z1.d[2];
    double d_len = sqrt(dx*dx + dy*dy + dz*dz);
    quintic_spline_1d_t q_spline_1  = make_1d_quintic_spline(d_len, t2 - t1);

    // Run through Segment #1
    for (int i=0; i < num_pts; i++) 
    {
        // 1) Get 1d position
        t_curr = ( ((double) i) / ((double) (num_pts-1))) * (t2 - t1);
        s_curr = compute_spline_position(&q_spline_1, t_curr);
        v_curr = compute_spline_velocity(&q_spline_1, t_curr);

        // 2) Convert to 3d position
        if (d_len > 0) 
        {
            z_curr = (s_curr / d_len) * dz  + z1.d[2];
            z_dot_curr = (dz / abs(dz)) * v_curr;
        }   
        else
        {
            // If d_len is 0, avoid NaNs
            z_curr = z1.d[2];
        }
        

        // 3) Write to the path
        for (int k = 0; k < NUM_DRONES; k++) {
            path[pos].waypoints[i].t = t_curr + t1;
            path[pos].waypoints[i].x[k] = current_x;
            path[pos].waypoints[i].y[k] = current_y;
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