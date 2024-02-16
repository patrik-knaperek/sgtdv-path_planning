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
  PathPlanning(const ros::NodeHandle& handle);
  ~PathPlanning() = default;

  void setPublisher(const ros::Publisher &trajectory_pub
                #ifdef SGT_VISUALIZATION
                  , const ros::Publisher &path_planning_vis_pub
                  , const ros::Publisher &interpolated_cones_pub
                #endif /* SGT_VISUALIZATION */
                  );
  void SetServiceClient(const ros::ServiceClient &set_speed_client)
  {
    set_speed_client_ = set_speed_client;
  };
  void update(const PathPlanningMsg &msg);  
  void yellowOnLeft(bool value);
  void fullMap() { full_map_ = true; };
  //void setDiscipline(Discipline discipline);

#ifdef SGT_DEBUG_STATE
  void setVisDebugPublisher(const ros::Publisher& publisher) { vis_debug_publisher_ = publisher; };
#endif

private:
  bool rrtRun();
  void sortCones(const PathPlanningMsg &msg);
  std::vector<Eigen::Vector2f> linearInterpolation(std::vector<Eigen::Vector2f> points) const;
  sgtdv_msgs::Point2DArr findMiddlePoints();

#ifdef SGT_VISUALIZATION
  void visualizeInterpolatedCones();
  void visualizeRRTPoints();
#endif /* SGT_VISUALIZATION */
    
  RRTStar rrt_star_obj_;
  Params params_;
	
  float timer_avg_;
  int timer_avg_count_;

  ros::Publisher trajectory_pub_;
  ros::ServiceClient set_speed_client_;
  sgtdv_msgs::Float32Srv set_speed_msg_;
  bool is_yellow_on_left_;
  bool once_;
  bool full_map_;

  std::vector<Eigen::Vector2f> left_cones_, left_cones_interpolated_,  right_cones_, 
                              right_cones_interpolated_, middle_line_points_;
  //PathPlanningDiscipline *path_planning_discipline_obj = nullptr;

#ifdef SGT_VISUALIZATION
  ros::Publisher interpolated_cones_pub_, path_planning_vis_pub_;
#endif /* SGT_VISUALIZATION */
#ifdef SGT_DEBUG_STATE
  ros::Publisher vis_debug_publisher_;
#endif
};