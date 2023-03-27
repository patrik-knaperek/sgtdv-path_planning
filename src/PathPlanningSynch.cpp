/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Samuel Mazur
/*****************************************************/


#include "../include/PathPlanningSynch.h"

PathPlanningSynch::PathPlanningSynch()
    : m_mapReceived(false)
    , m_poseReceived(false)
{

}

/**
 * @brief Seting ROS publishers.
 * @param trajectoryPub
 * @param interpolatedConesPub
 */
//TODO: replace multiple arguments with single one
void PathPlanningSynch::SetPublisher(const ros::Publisher &trajectoryPub, const ros::Publisher &trajectoryVisPub, const ros::Publisher &interpolatedConesPub)
{
    m_pathPlanning.SetPublisher(trajectoryPub, trajectoryVisPub, interpolatedConesPub);
}

/**
 * @brief Main function in class.
 * @param incomingROSMsg
 */
void PathPlanningSynch::Do()
{
    if (m_poseReceived && m_mapReceived)
    {
        m_mapReceived = false;
        m_poseReceived = false;
        ROS_INFO("Starting trajectory planning");
        m_pathPlanning.Do(m_pathPlanningMsg);
    }
    else
    {
        //ROS_ERROR("PathPlanningSynch - Do: PathPlanning message not ready\n");
    }    
}

/**
 * @brief Read SLAM map message.
 * @param SLAMMapMsg
 */
void PathPlanningSynch::UpdateMap(const sgtdv_msgs::ConeArr::ConstPtr &msg)
{
    ROS_INFO("Map received");
    m_pathPlanningMsg.coneMap = msg;
    m_mapReceived = true;
    Do();
}

/**
 * @brief Read car position message.
 * @param carPoseMsg
 */
void PathPlanningSynch::UpdatePose(const sgtdv_msgs::CarPose::ConstPtr &msg)
{
    ROS_INFO_ONCE("Pose received");
    m_pathPlanningMsg.carPose = msg;
    m_poseReceived = true;
    Do();
}

/**
 * @brief Swap color of cones in arrays.
 * @param isYellowOnLeft
 */
void PathPlanningSynch::YellowOnLeft(bool value)
{
    m_pathPlanning.YellowOnLeft(value);
}

/*void PathPlanningSynch::SetDiscipline(Discipline discipline)
{
    m_pathPlanning.SetDiscipline(discipline);
}*/