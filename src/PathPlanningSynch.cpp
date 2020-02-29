/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#include "../include/PathPlanningSynch.h"

PathPlanningSynch::PathPlanningSynch()
{
    m_poseReceived = false;
    m_pathPlanning = nullptr;
}

PathPlanningSynch::~PathPlanningSynch()
{

}

void PathPlanningSynch::SetPublisher(ros::Publisher publisher)
{
    m_pathPlanning->SetPublisher(publisher);
}

void PathPlanningSynch::Do(const sgtdv_msgs::ConeArr::ConstPtr &msg)
{
    if (m_poseReceived)
    {
        m_poseReceived = false;
        m_pathPlanningMsg.coneMap = msg;

        m_pathPlanning->Do(m_pathPlanningMsg);
    }
    else
    {
        std::cerr << "PathPlanningSynch - Do: Map received before pose\n";
    }    
}

void PathPlanningSynch::UpdatePose(const sgtdv_msgs::CarState::ConstPtr &msg)
{
    m_pathPlanningMsg.carState = msg;
    m_poseReceived = true;
}

void PathPlanningSynch::YellowOnLeft(bool value)
{
    m_pathPlanning->YellowOnLeft(value);
}

void PathPlanningSynch::SetDiscipline(PathPlanningDiscipline discipline)
{
    switch (discipline)
    {
        case UNKNOWN_TRACK: m_pathPlanning = new UnknownTrack; break;
        case SKIDPAD: m_pathPlanning = new Skidpad; break;
        default: m_pathPlanning = nullptr;
    }

    if (!m_pathPlanning) ros::shutdown();
}