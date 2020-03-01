/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#include "../include/PathPlanning.h"

PathPlanning::PathPlanning()
{

}

PathPlanning::~PathPlanning()
{

}

void PathPlanning::SetPublisher(ros::Publisher publisher)
{
    m_publisher = publisher;
}

void PathPlanning::SetDiscipline(Discipline discipline)
{
    switch (discipline)
    {
        case UNKNOWN_TRACK: m_pathPlanningDiscipline = new UnknownTrack; break;
        case SKIDPAD: m_pathPlanningDiscipline = new Skidpad; break;
        default: m_pathPlanningDiscipline = nullptr;
    }

    if (!m_pathPlanningDiscipline) ros::shutdown();
}

void PathPlanning::YellowOnLeft(bool value)
{
    m_pathPlanningDiscipline->YellowOnLeft(value);
}

void PathPlanning::Do(const PathPlanningMsg &msg)
{
    m_publisher.publish(m_pathPlanningDiscipline->Do(msg));
}

