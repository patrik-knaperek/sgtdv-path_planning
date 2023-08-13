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

struct Params
{
    float ref_speed_slow;
    float ref_speed_fast;
};

struct RRTconf
{
    float car_width;
    float node_step_size;
    float neighbor_radius;
    float max_angle;
};