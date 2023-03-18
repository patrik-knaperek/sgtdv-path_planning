/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Samuel Mazúr
/*****************************************************/

#pragma once

// C++ 
#include <iostream>

// ROS
#include <ros/ros.h>

// SGT
#include "../include/PathPlanning.h"
//#include "../include/PathPlanningDisciplines.h"
#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/CarPose.h>
#include "../include/Messages.h"


class PathPlanningSynch
{
public:
    PathPlanningSynch();
    ~PathPlanningSynch() = default;

    void SetPublisher(const ros::Publisher &trajectoryPub, const ros::Publisher &interpolatedConesPub);
    void Do(const sgtdv_msgs::ConeArr::ConstPtr &msg);
    void UpdatePose(const sgtdv_msgs::CarPose::ConstPtr &msg);
    void YellowOnLeft(bool value);
    //void SetDiscipline(Discipline discipline);

private:
    PathPlanning m_pathPlanning;   
    PathPlanningMsg m_pathPlanningMsg;
    bool m_poseReceived;
};