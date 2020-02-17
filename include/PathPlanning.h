#pragma once

#include <ros/ros.h>
#include <sgtdv_msgs/PathPlanningMsg.h>
#include <sgtdv_msgs/PathTrackingMsg.h>
#include "../include/Messages.h"

class PathPlanning
{
public:
    PathPlanning();
    ~PathPlanning();

    void Do(const PathPlanningMsg &msg);
    void SetPublisher(ros::Publisher publisher);

private:
    ros::Publisher m_publisher;
};