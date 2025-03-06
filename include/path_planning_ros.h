/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Samuel Mazúr, Patrik Knaperek
/*****************************************************/

#pragma once

/* C++ */
#include <iostream>

/* ROS */
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/MarkerArray.h>

/* SGT */
#include "path_planning.h"
//#include "../include/path_planning_disciplines.h"
#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/Float32Srv.h>

class PathPlanningROS
{
public:
  explicit PathPlanningROS(ros::NodeHandle& handle);
  ~PathPlanningROS() = default;

private:
  void loadParams(const ros::NodeHandle& nh);

  void update();
  
  void mapCallback(const sgtdv_msgs::ConeArr::ConstPtr &msg);
  void poseCallback(const sgtdv_msgs::CarPose::ConstPtr &msg);
  void loopClosureCallback(const std_msgs::Empty::ConstPtr &msg);

  bool setSpeedCallback(sgtdv_msgs::Float32Srv::Request &req, sgtdv_msgs::Float32Srv::Response &res);
  
  //void setDiscipline(Discipline discipline);

#ifdef SGT_VISUALIZATION
  void initPathBoundariesMarkers();
  void initRRTPointsMarker();
  void visualizeInterpolatedCones();
  void visualizeRRTPoints();
  void deleteMarkers(visualization_msgs::MarkerArray& marker_array,
                    const ros::Publisher& publisher) const;
#endif /* SGT_VISUALIZATION */

  ros::Publisher trajectory_pub_;
  ros::ServiceServer set_speed_server_;

#ifdef SGT_VISUALIZATION
  ros::Publisher boundaries_vis_pub_;
  ros::Publisher rrt_vis_pub_;
  visualization_msgs::Marker left_cones_interpolated_marker_, right_cones_interpolated_marker_,
                              left_cones_marker_, right_cones_marker_,
                              start_marker_, finish_marker_,
                              rrt_nodes_marker_, rrt_trajectory_marker_;
#endif /* SGT_VISUALIZATION */

#ifdef SGT_DEBUG_STATE
  ros::Publisher vis_debug_publisher_;
#endif

  ros::Subscriber map_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber loop_close_sub_;
  
  PathPlanning path_planning_obj_;   
  sgtdv_msgs::PathPlanningMsg path_planning_msg_;

  bool map_received_;
  bool pose_received_;
};