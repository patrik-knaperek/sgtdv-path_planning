#pragma once

#include <ros/ros.h>
#include "../include/PathPlanning.h"
#include <sgtdv_msgs/ColoredConeArr.h>
#include <sgtdv_msgs/CarState.h>
#include <iostream>
#include "../include/Messages.h"


class PathPlanningSynch
{
public:
    PathPlanningSynch();
    ~PathPlanningSynch();

    void SetPublisher(ros::Publisher publisher);
    void Do(const sgtdv_msgs::ColoredConeArr::ConstPtr &msg);
    void UpdatePose(const sgtdv_msgs::CarState::ConstPtr &msg);

private:
    PathPlanning m_pathPlanning;   
    PathPlanningMsg m_pathPlanningMsg;
    bool m_poseReceived;
};