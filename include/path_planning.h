/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Samuel Mazur, Patrik Knaperek
/*****************************************************/

#pragma once

/* C++ */
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <vector>
#include <map>
#include <chrono>

// #include "opencv2/core/core.hpp"

/* ROS */
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

/* SGT */
#include "messages.h"
//#include "path_planning_disciplines.h"
#include "../include/rrt_star.h"
#include <sgtdv_msgs/PathPlanningMsg.h>
#include <sgtdv_msgs/Cone.h>
#include <sgtdv_msgs/Point2D.h>
#include <sgtdv_msgs/Point2DArr.h>
#include <sgtdv_msgs/Float32Srv.h>
#include <sgtdv_msgs/DebugState.h>
#include "../../SGT_Utils.h"
#include "../../SGT_Macros.h"

constexpr float BEZIER_RESOLUTION = 0.125;

class PathPlanning
{
public:
  struct Params
  {
    float ref_speed_slow;
    float ref_speed_fast;
  };
  
public:
  PathPlanning(ros::NodeHandle& handle);
  ~PathPlanning() = default;

  void update(const PathPlanningMsg &msg);  
  void yellowOnLeft(bool value);
  void fullMap() { full_map_ = true; };
  //void setDiscipline(Discipline discipline);

private:
  bool rrtRun();
  void sortCones(const PathPlanningMsg &msg);
  std::vector<Eigen::Vector2f> linearInterpolation(std::vector<Eigen::Vector2f> points) const;
  sgtdv_msgs::Point2DArr findMiddlePoints();

#ifdef SGT_VISUALIZATION
  void initPathBoundariesMarkers();
  void initRRTPointsMarker();
  void visualizeInterpolatedCones();
  void visualizeRRTPoints();
  void deleteMarkers(visualization_msgs::MarkerArray& marker_array,
                    const ros::Publisher& publisher) const;
#endif /* SGT_VISUALIZATION */
    
  ros::Publisher trajectory_pub_;
  ros::ServiceClient set_speed_client_;

#ifdef SGT_VISUALIZATION
  ros::Publisher boundaries_vis_pub_;
  ros::Publisher rrt_vis_pub_;
#endif /* SGT_VISUALIZATION */

#ifdef SGT_DEBUG_STATE
  ros::Publisher vis_debug_publisher_;
#endif
  
  RRTStar rrt_star_obj_;
  Params params_;
	
  float timer_avg_;
  int timer_avg_count_;

  sgtdv_msgs::Float32Srv set_speed_msg_;
  bool is_yellow_on_left_;
  bool once_;
  bool full_map_;

  std::vector<Eigen::Vector2f> left_cones_, left_cones_interpolated_, right_cones_, 
                              right_cones_interpolated_, middle_line_points_;
  //PathPlanningDiscipline *path_planning_discipline_obj = nullptr;

#ifdef SGT_VISUALIZATION
  visualization_msgs::Marker left_cones_interpolated_marker_, right_cones_interpolated_marker_,
                              left_cones_marker_, right_cones_marker_,
                              start_marker_, finish_marker_,
                              rrt_nodes_marker_, rrt_trajectory_marker_;
#endif /* SGT_VISUALIZATION */
};