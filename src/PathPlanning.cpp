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

void PathPlanning::Do(const sgtdv_msgs::PathPlanningMsgPtr &msg)
{
    //TODO function body

    sgtdv_msgs::PathTrackingMsgPtr pathTrackingMsg( new sgtdv_msgs::PathTrackingMsg );

    /*Set values*/

    //pathTrackingMsg->carState =
    //pathTrackingMsg->trajectory =
    //pathTrackingMsg->speed = 
    //pathTrackingMsg->yawRate = 

    m_publisher.publish(pathTrackingMsg);
}