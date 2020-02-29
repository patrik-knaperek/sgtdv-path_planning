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

void PathPlanning::YellowOnLeft(bool value)
{
    m_isYellowOnLeft = value;
}


