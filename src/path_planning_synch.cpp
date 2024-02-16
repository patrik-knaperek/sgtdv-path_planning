/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Samuel Mazur, Patrik Knaperek
/*****************************************************/

#include "path_planning_synch.h"

PathPlanningSynch::PathPlanningSynch(ros::NodeHandle& nh) :
  /* ROS interface init */
  map_sub_(nh.subscribe("slam/map", 1, &PathPlanningSynch::updateMap, this)),
  pose_sub_(nh.subscribe("slam/pose", 1, &PathPlanningSynch::updatePose, this)),
  loop_close_sub_(nh.subscribe("slam/loop_closure", 1, &PathPlanningSynch::loopClosureCallback, this)),
  
  path_planning_obj_(nh),
  
  map_received_(false),
  pose_received_(false)
{
}

/**
 * @brief Main function in class.
 * @param incoming_ros_msg
 */
void PathPlanningSynch::update()
{
  if(pose_received_ && map_received_)
  {
    map_received_ = false;
    pose_received_ = false;
    path_planning_obj_.update(path_planning_msg_);
  }
  else
  {
    //ROS_ERROR("PathPlanningSynch - Do: PathPlanning message not ready\n");
  }    
}

/**
 * @brief Read SLAM map message.
 * @param slam_map_msg
 */
void PathPlanningSynch::updateMap(const sgtdv_msgs::ConeArr::ConstPtr &msg)
{
  if(!msg->cones.empty())
  {
    path_planning_msg_.cone_map = msg;
    map_received_ = true;
    update();
  }
  else
    map_received_ = false;
}

void PathPlanningSynch::loopClosureCallback(const std_msgs::Empty::ConstPtr &msg)
{
  path_planning_obj_.fullMap();
}

/**
 * @brief Read car position message.
 * @param car_pose_msg
 */
void PathPlanningSynch::updatePose(const sgtdv_msgs::CarPose::ConstPtr &msg)
{
  path_planning_msg_.car_pose = msg;
  pose_received_ = true;
  update();
}

/*void PathPlanningSynch::SetDiscipline(Discipline discipline)
{
  m_pathPlanning.SetDiscipline(discipline);
}*/
