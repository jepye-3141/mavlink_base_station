## Using the Mavlink Base Station

The mavlink base station is broken up into 3 essential modules - the keyboard input module (nonBlockingCLI), the path generation module (trajectory.cpp), and the main module, which decides which input type to use. 

#### Changing input method

Currently, to change between keyboard input and path input is a compile-time change. In main.cpp, change ```scope``` to ```SCOPE_MANUAL_ONLY``` for manual keyboard control or ```SCOPE_PATTERN_ONLY``` for autonomous path control.

#### Adding new trajectories

There are 2 base trajectoies - takeoff and landing. You can then add other user-created trajectories, or dynamic trajectories. In settings.h, change ```NUM_DYNAMIC_TRAJECTORIES``` to the number of user-generated trajectories that you want included. Currently, to add a new trajectory is also a compile-time change. Simply enumerate a 2-d array of waypoints for x, y, and z (2-d because each element of the outer array are the waypoints for each drone, the number of which are specified under ```NUM_DRONES``` in settings.h) as shown in the test_trajectory function in trajectory.cpp. Then, call ```__waypoint_trajectory``` on those new waypoint arrays, an array of dts, and the position in the path array you want (pos 0 will bind to key 1, 1 to 2, and so forth).

### Setting up main.cpp

In main.cpp you will notice that there are commented-out mav_init functions. Currently, in order to initialize mavlink links for multiple drones, uncommen the corresponding number of mav_init statements. Make sure the ports are different, and the ip addresses are correct.

### Setting up the drone

Make sure that the drone's listening port is the same as the base station's port. TO ensure this, ake sure that int he settings file loaded, drone_id is set to the corresponding drone_id that it is initialized with on the base station through mav_init.