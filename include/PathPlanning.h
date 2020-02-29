#pragma once

#include "../include/Messages.h"
#include "opencv2/core/core.hpp"
#include <ros/ros.h>


class PathPlanning
{
public:
    void SetPublisher(ros::Publisher publisher);
    virtual void Do(const PathPlanningMsg &msg) = 0;  
    void YellowOnLeft(bool value);

protected:
    PathPlanning();
    ~PathPlanning();

    ros::Publisher m_publisher;
    bool m_isYellowOnLeft;
};