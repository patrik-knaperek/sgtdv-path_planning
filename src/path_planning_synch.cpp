/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Samuel Mazur, Patrik Knaperek
/*****************************************************/

#include "../include/path_planning_synch.h"

PathPlanningSynch::PathPlanningSynch(const ros::NodeHandle& handle):
  path_planning_obj_(handle),
  map_received_(false),
  pose_received_(false)
{

}

/**
 * @brief Seting ROS publishers.
 * @param trajectory_pub
 * @param interpolated_cones_pub
 */
void PathPlanningSynch::setPublisher(const ros::Publisher &trajectoryPub
                              #ifdef SGT_VISUALIZATION
                                  , const ros::Publisher &trajectoryVisPub
                                  , const ros::Publisher &interpolatedConesPub
                              #endif /* SGT_VISUALIZATION */
                                  )
{
  path_planning_obj_.setPublisher(trajectoryPub
                          #ifdef SGT_VISUALIZATION
                              , trajectoryVisPub
                              , interpolatedConesPub
                          #endif /* SGT_VISUALIZATION */
                              );
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

/**
 * @brief Swap color of cones in arrays.
 * @param is_yellow_on_left
 */
void PathPlanningSynch::yellowOnLeft(bool value)
{
  path_planning_obj_.yellowOnLeft(value);
}

/*void PathPlanningSynch::SetDiscipline(Discipline discipline)
{
  m_pathPlanning.SetDiscipline(discipline);
}*/