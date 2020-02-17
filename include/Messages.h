#pragma once

#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/CarState.h>


struct PathPlanningMsg
{
    sgtdv_msgs::CarState::ConstPtr carState;
    sgtdv_msgs::ConeArr::ConstPtr coneMap;
};