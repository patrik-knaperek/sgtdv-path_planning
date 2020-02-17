#pragma once

#include <sgtdv_msgs/ColoredConeArr.h>
#include <sgtdv_msgs/CarState.h>


struct PathPlanningMsg
{
    sgtdv_msgs::CarState::ConstPtr carState;
    sgtdv_msgs::ColoredConeArr::ConstPtr coneMap;
};