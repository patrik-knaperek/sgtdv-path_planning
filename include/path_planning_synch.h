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

/* SGT */
#include "../include/path_planning.h"
//#include "../include/path_planning_disciplines.h"
#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/CarPose.h>
#include "../include/messages.h"


class PathPlanningSynch
{
public:
  PathPlanningSynch(ros::NodeHandle& handle);
  ~PathPlanningSynch() = default;

  void update();
  void updateMap(const sgtdv_msgs::ConeArr::ConstPtr &msg);
  void updatePose(const sgtdv_msgs::CarPose::ConstPtr &msg);
  void loopClosureCallback(const std_msgs::Empty::ConstPtr &msg);
  void yellowOnLeft(bool value);
  //void setDiscipline(Discipline discipline);

private:
  ros::Subscriber map_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber loop_close_sub_;

  PathPlanning path_planning_obj_;   
  PathPlanningMsg path_planning_msg_;

  bool map_received_;
  bool pose_received_;
};