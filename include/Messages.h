/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#pragma once

#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/CarPose.h>


struct PathPlanningMsg
{
    sgtdv_msgs::CarPose::ConstPtr carPose;
    sgtdv_msgs::ConeArr::ConstPtr coneMap;
};