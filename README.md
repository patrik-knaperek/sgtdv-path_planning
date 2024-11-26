# **PathPlanning package**

___

© **SGT Driverless**

**Authors:** Juraj Krasňanský, Samuel Mazúr, Patrik Knaperek

**Objective:** Trajectory planning based on map data. 
___

The implementation consists of 2 algorithms. For **reactive navigation** (first lap, unknown map), **Lagrange interpolating polynomial** method is used. The same amount of cones on both side of track is asumed, which isn't however guaranteed by the rules. For **global navigation** (known map), **RRT\*** finds the shortest path through the track, considering given constraints.

*Note: If only global navigation is being tested, set the `full_map_` initial value to `true`.*
```cpp
PathPlanning::PathPlanning(ros::NodeHandle& handle) :
  ...
  full_map_(false),
  ...
{
	...
}
```

### Related packages
* [`slam_si`](../simulation_interface/slam_si/README.md)

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
		- `/path_planning/visualize/track_boundaries [visualization_msgs/MarkerArray]` - Track boundaries and interpolated cones
		- `/path_planning/visualize/rrt [visualization_msgs/MarkerArray]` - RRT nodes and trajectory
	* `SGT_DEBUG_STATE` : publish debug state topic for `debug_visualization` processing
* [`path_planning.yaml`](./params/path_planning_rc.yaml)
  * `ref_speed` : reference speed sended to the `path_tracking` node
    * `slow` : reactive navigation
    * `fast` : global navigation
  * `rrt_conf`
	* `node_step_size` : [m]; distance between parent and child node
	* `car_width` : [m]; minimum distance from the track boundary
	* `max_iter` : maximum number of valid algorithm iterations (only nodes in track count)
	* `max_angle` : [rad]; maximum angle between parent and child node


## Launch
```sh
$ cd ${SGT_ROOT}/ros_implementation
$ source ./devel/setup.bash
```
* standalone
```sh
$ roslaunch path_planning path_planning.launch
```
* on rosbag data (`.bag` file has to be located in the `/bags` folder)
```sh
$ roslaunch path_planning path_planning_rosbag.launch bag_name:=YOUR_BAG_FILE
```
* alongside FSSIM (the `slam_si` package has to be built first)
```sh
$ roslaunch path_planning path_planning_sim.launch
```
* with RC car config
```sh
$ roslaunch path_planning path_planning_rc.launch
```
