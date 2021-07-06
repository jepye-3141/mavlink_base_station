/**
 * @file waypoint.c
 **/

#include <fcntl.h>  // for F_OK
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>  // for access()

#include "trajectory.h"

path_t path = PATH_INITIALIZER;

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
static int __read_waypoints(FILE* fd);
/**
 * ********************************
 */

int path_load_from_file(const char* file_path)
{
    // Clear any previously stored path, set init to 0
    path_cleanup();

    // Check for valid file
    if (access(file_path, F_OK) != 0)
    {
        fprintf(stderr, "ERROR: waypoint file missing\n");
        return -1;
    }

    // Count number of waypoints contained in file
    path.len = __count_file_lines(file_path);

    // Open file for waypoint reading
    FILE* fd = fopen(file_path, "r");

    // Read path size and allocate waypoint memory
    path.waypoints = (waypoint_t*)malloc(sizeof(waypoint_t) * path.len);
    if (path.waypoints == NULL)
    {
        fprintf(stderr, "ERROR: failed allocating memory for path\n");
        return -1;
    }

    // Read waypoints from file
    if (__read_waypoints(fd) < 0)
    {
        path_cleanup(); //Added to prevent potential memory leak
        fprintf(stderr, "ERROR: failed reading waypoint file\n");
        return -1;
    }

    fclose(fd);

    path.initialized = 1;
    return 0;
}

void path_cleanup()
{
    free(path.waypoints);
    path.waypoints = NULL;
    path.len = 0;
    path.initialized = 0;
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

static int __read_waypoints(FILE* fd)
{
    int rcount = 0;
    int waypoint_num = 0;

    while (rcount > -1)
    {
        rcount = 0;

        // Read formated file line (31 doubles)
        for (int i = 0; i < MAX_DRONES; i++) {
            rcount += fscanf(fd, "%lf %lf %lf %lf %lf %lf",
                &path.waypoints[waypoint_num].x[i], &path.waypoints[waypoint_num].y[i], 
                &path.waypoints[waypoint_num].z[i], &path.waypoints[waypoint_num].r[i], 
                &path.waypoints[waypoint_num].p[i], &path.waypoints[waypoint_num].yaw[i]);
        }
                    
        printf("I got %i on %i\n", rcount, waypoint_num);

        // If not end of file, but an invalid read (waypoints have 31 values)
        // if (rcount != EOF && rcount != 31)
        // {
        //     fprintf(stderr, "ERROR: invalid waypoint read from line: %i\n",
        //         waypoint_num + 1);  // lines 1 indexed, waypoints zero indexed
        //     printf("I got %i\n", rcount);
        //     return -1;
        // }

        // Increment line number for next iteration
        ++waypoint_num;
    }
    return 0;
}