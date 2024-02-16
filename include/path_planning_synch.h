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
  PathPlanningSynch(const ros::NodeHandle& handle);
  ~PathPlanningSynch() = default;

  void setPublisher(const ros::Publisher &trajectory_pub
                #ifdef SGT_VISUALIZATION
                  , const ros::Publisher &trajectory_vis_pub
                  , const ros::Publisher &interpolated_cones_pub
                #endif /* SGT_VISUALIZATION */
                  );
  void setServiceClient(const ros::ServiceClient &set_speed_client)
  {
    path_planning_obj_.SetServiceClient(set_speed_client);
  };

  void update();
  void updateMap(const sgtdv_msgs::ConeArr::ConstPtr &msg);
  void updatePose(const sgtdv_msgs::CarPose::ConstPtr &msg);
  void loopClosureCallback(const std_msgs::Empty::ConstPtr &msg);
  void yellowOnLeft(bool value);
  //void setDiscipline(Discipline discipline);

#ifdef SGT_DEBUG_STATE
  void setVisDebugPublisher(ros::Publisher publisher) { path_planning_obj_.setVisDebugPublisher(publisher); };
#endif

private:
  PathPlanning path_planning_obj_;   
  PathPlanningMsg path_planning_msg_;
  bool map_received_;
  bool pose_received_;
};