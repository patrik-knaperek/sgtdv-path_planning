/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Samuel Mazúr, Patrik Knaperek
/*****************************************************/

#pragma once

// C++ 
#include <iostream>

// ROS
#include <ros/ros.h>
#include <std_msgs/Empty.h>

// SGT
#include "../include/PathPlanning.h"
//#include "../include/PathPlanningDisciplines.h"
#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/CarPose.h>
#include "../include/Messages.h"


class PathPlanningSynch
{
public:
    PathPlanningSynch(const ros::NodeHandle& handle);
    ~PathPlanningSynch() = default;

    void SetPublisher(const ros::Publisher &trajectoryPub
                    , const ros::Publisher &trajectoryVisPub
                    , const ros::Publisher &interpolatedConesPub
                    );
    void SetServiceClient(const ros::ServiceClient &setSpeedClient)
    {
        m_pathPlanning.SetServiceClient(setSpeedClient);
    };

    void Do();
    void UpdateMap(const sgtdv_msgs::ConeArr::ConstPtr &msg);
    void UpdatePose(const sgtdv_msgs::CarPose::ConstPtr &msg);
    void LoopClosureCallback(const std_msgs::Empty::ConstPtr &msg);
    void YellowOnLeft(bool value);
    //void SetDiscipline(Discipline discipline);

private:
    PathPlanning m_pathPlanning;   
    PathPlanningMsg m_pathPlanningMsg;
    bool m_mapReceived;
    bool m_poseReceived;
};