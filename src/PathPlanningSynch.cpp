/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Samuel Mazur
/*****************************************************/


#include "../include/PathPlanningSynch.h"

PathPlanningSynch::PathPlanningSynch() :
    m_poseReceived(false)
{

}

/**
 * @brief Seting ROS publishers.
 * @param trajectoryPub
 * @param interpolatedConesPub
 */
//TODO: replace multiple arguments with single one
void PathPlanningSynch::SetPublisher(const ros::Publisher &trajectoryPub, const ros::Publisher &interpolatedConesPub)
{
    m_pathPlanning.SetPublisher(trajectoryPub, interpolatedConesPub);
}

/**
 * @brief Main function in class.
 * @param incomingROSMsg
 */
void PathPlanningSynch::Do(const sgtdv_msgs::ConeArr::ConstPtr &msg)
{
    if (m_poseReceived)
    {
        m_poseReceived = false;
        m_pathPlanningMsg.coneMap = msg;

        m_pathPlanning.Do(m_pathPlanningMsg);
    }
    else
    {
        std::cerr << "PathPlanningSynch - Do: Map received before pose\n";
    }    
}

/**
 * @brief Read car position message.
 * @param carPoseMsg
 */
void PathPlanningSynch::UpdatePose(const sgtdv_msgs::CarPose::ConstPtr &msg)
{
    m_pathPlanningMsg.carPose = msg;
    m_poseReceived = true;
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