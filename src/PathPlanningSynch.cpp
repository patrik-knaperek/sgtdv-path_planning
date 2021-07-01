/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#include "../include/PathPlanningSynch.h"

PathPlanningSynch::PathPlanningSynch()
{
    m_poseReceived = false;
}

PathPlanningSynch::~PathPlanningSynch()
{

}

void PathPlanningSynch::SetPublisher(ros::Publisher publisher)
{
    m_pathPlanning.SetPublisher(publisher);
}

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

void PathPlanningSynch::UpdatePose(const sgtdv_msgs::CarPose::ConstPtr &msg)
{
    m_pathPlanningMsg.carPose = msg;
    m_poseReceived = true;
}

void PathPlanningSynch::YellowOnLeft(bool value)
{
    m_pathPlanning.YellowOnLeft(value);
}

void PathPlanningSynch::SetDiscipline(Discipline discipline)
{
    m_pathPlanning.SetDiscipline(discipline);
}