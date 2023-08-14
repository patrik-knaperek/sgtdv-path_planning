# **PathPlanning package**

___

© **SGT Driverless**

**Authors:** Juraj Krasňanský, Samuel Mazúr, Patrik Knaperek

**Objective:** Trajectory planning based on map data. 
___

The implementation consists of 2 algorithms. For **reactive navigation** (first lap, unknown map), **Lagrange interpolating polynomial** method is used. The same amount of cones on both side of track is asumed, which isn't however guaranteed by the rules. For **global navigation** (known map), **RRT\*** finds the shortest path through the track, considering given constraints.

### Related packages
* [`slam_sim_interface`](../simulation_interface/slam_sim_interface/README.md)

### Referencies
* [S. MAZÚR: Path Planning for Autonomous Formula Student Vehicle. (Bachelor's thesis)](https://drive.google.com/file/d/17erZrSe4Bqdqr1wfQmzG4VMhgwLZWuhS/view?usp=drive_link)

## Compilation
```sh
$ cd ${SGT_ROOT}/ros_implementation
$ catkin build path_planning -DCMAKE_BUILD_TYPE=Release
```

### Compilation configuration
* [`SGT_Macros.h`](../SGT_Macros.h)
	* `SGT_VISUALIZE` : publish intermediate calculations on visualizable topics
		- `/pathplanning/visualize/interpolated_cones [visualization_msgs/MarkerArray]` - Track boundaries and interpolated cones
		- `/pathplanning/visualize/rrt [visualization_msgs/MarkerArray]` - RRT nodes and trajectory
* [`PathPlanning.h`](./include/PathPlanning.h)
	* `NODE_STEP_SIZE` : [m]; distance between parent and child node
	* `RRTSTAR_NEIGHBOR_RADIUS` : [m]; radius for searching neighbor nodes
	* `CAR_WIDTH` : [m]; minimum distance from the track boundary
	* `MAX_ITER` : maximum number of valid algorithm iterations (only nodes in track count)
	* `MAX_ANGLE` : [rad]; maximum angle between parent and child node


## Launch
```sh
$ cd ${SGT_ROOT}/ros_implementation
$ source ./devel/setup.bash
```
* standalone
```sh
$ roslaunch path_planning_bp path_planning.launch
```
* on rosbag data (`.bag` file has to be located in the `/bags` folder)
```sh
$ roslaunch path_planning_bp path_planning_rosbag.launch bag_name:=YOUR_BAG_FILE
```
* alongside FSSIM (the `slam_sim_interface` package has to be built first)
```sh
$ roslaunch path_planning path_planning_sim.launch
```
