# **path_planning package**

___

© **SGT Driverless**

**Authors:** Juraj Krasňanský, Samuel Mazúr, Patrik Knaperek

**Objective:** Trajectory planning based on map data. 
___

## Overview

The implementation consists of 2 algorithms:
* **reactive navigation** (first lap, unknown map) - **Lagrange interpolating polynomial** method is used. The same amount of cones on both side of track is asumed, which isn't however guaranteed by the rules.
* **global navigation** (known map) - **RRT\*** algorithm finds the shortest path through the track, considering given constraints.

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

### Referencies
* [S. MAZÚR: Path Planning for Autonomous Formula Student Vehicle. (Bachelor's thesis)](https://drive.google.com/file/d/17erZrSe4Bqdqr1wfQmzG4VMhgwLZWuhS/view?usp=drive_link)


### ROS Interface

**Subscribed topics**
* `/slam/map`[[`sgtdv_msgs/ConeArr`](../sgtdv_msgs/msg/ConeArr.msg)] : landmark coordinates from SLAM state vector in `map` frame
* `/slam/pose` [[`sgtdv_msgs/CarPose`](../sgtdv_msgs/msg/CarPose.msg)] : car pose from SLAM state vector in `map` frame
* `/slam/loop_closure` [[`std_msgs/Empty`](/opt/ros/noetic/share/std_msgs/msg/Empty.msg)] : signalization from SLAM allowing to switch from local to global navigation

**Published topics**
* `/path_planning/trajectory` [[`sgtdv_msgs/Point2DArr`](../sgtdv_msgs/msg/Point2DArr.msg)] : path 2D coordinates in `map` frame

*If `SGT_VISUALIZE` macro enabled*
* `/path_planning/visualize/track_boundaries` [[`visualization_msgs/MarkerArray`](/opt/ros/noetic/share/visualization_msgs/msg/MarkerArray.msg)] : estimated track boundaries visualization (cones, interpolated cones, trajectory start cones, trajectory finish cones)
* `/path_planning/visualize/rrt` [[`visualization_msgs/MarkerArray`](/opt/ros/noetic/share/visualization_msgs/msg/MarkerArray.msg)] : RRT* data visualization (nodes, trajectory)

*If `SGT_DEBUG_STATE` macro enabled*
* `/path_planning/debug_state` [[`sgtdv_msgs/DebugState](../sgtdv_msgs/msg/DebugState.msg)] : node lifecycle information (active/inactive, number of trajectory points)

**Service clients**
* `/path_tracking/set_speed` [[`sgtdv_msgs/FLoat32Srv`](../sgtdv_msgs/srv/Float32Srv.srv)] : set reference speed for `path_tracking` node

**Parameters**
* `/ref_speed/slow` : [m/s] reference speed for reactive navigation
* `/ref_speed/fast` : [m/s] reference speed for global navigation
* `/rrt_conf/node_step_size` : [m] distance between parent and child node
* `/rrt_conf/car_width` : [m] minimum distance from the track boundary
* `/rrt_conf/max_iter` : maximum number of valid algorithm iterations (only nodes in track count)
* `/rrt_conf/max_angle` : [rad] maximum angle between parent and child node

### Related packages
* [`slam`](../slam/README.md) : `/slam/map`, `/slam/pose` and `/slam/loop_closure` publisher
* [`mapper`](../mapper/README.md) : temporary substitution for `slam` package
* [`slam_si`](../simulation_interface/slam_si/README.md) : (FSSIM setup) `/slam/map`, `/slam/pose` and `/slam/loop_closure` publisher
* [`path_tracking`](../path_tracking/README.md) : `/path_planning/trajectory` subscriber and `/path_tracking/set_speed` server

## Compilation
* standalone
```sh
$ catkin build path_planning -DCMAKE_BUILD_TYPE=Release
```
* FSSIM setup
```sh
  $ source ${SGT_ROOT}/scripts/build_sim.sh
```
* RC car setup
```sh
  $ source ${SGT_ROOT}/scripts/build_rc.sh
```

### Compilation configuration
* [`SGT_Macros.h`](../SGT_Macros.h)
	* `SGT_VISUALIZE` : publish intermediate calculations on visualizable topics
	* `SGT_DEBUG_STATE` : publish node lifecycle information
  
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
* FSSIM config (the `slam_si` package has to be built first) (check [FSSIM testing](../../doc/FSSIM_testing.md) manual for more info)
```sh
$ roslaunch path_planning path_planning_sim.launch
```
* RC car config
```sh
$ roslaunch path_planning path_planning_rc.launch
```

### Launch configuration
* [`path_planning_rc.yaml`](./params/path_planning_rc.yaml) : RC car setup
* [`path_planning_sim.yaml`](./params/path_planning_sim.yaml) : FSSIM setup

### RViz visualization
* FSSIM setup
```sh
  $ roslaunch data_visualization data_visualization_sim.launch
```
* RC car setup
```sh
  $ roslaunch data_visualization data_visualization_rc.launch
```

## Diagrams and flowcharts
