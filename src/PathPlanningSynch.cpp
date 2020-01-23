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

void PathPlanningSynch::Do(const sgtdv_msgs::ColoredConeArr::ConstPtr &msg)
{
    if (m_poseReceived)
    {
        sgtdv_msgs::PathPlanningMsgPtr pathPlanningMsg( new sgtdv_msgs::PathPlanningMsg );

        m_poseReceived = false;
        pathPlanningMsg->carState = *m_lastPose;
        pathPlanningMsg->coneMap = *m_lastMap;

        m_pathPlanning.Do(pathPlanningMsg);
    }
    else
    {
        std::cerr << "PathPlanningSynch - Do: Map received before pose\n";
    }    
}

void PathPlanningSynch::UpdatePose(const sgtdv_msgs::CarState::ConstPtr &msg)
{
    m_lastPose = msg;
    m_poseReceived = true;
}