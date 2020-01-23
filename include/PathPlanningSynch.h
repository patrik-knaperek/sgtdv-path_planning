#include <ros/ros.h>
#include "../include/PathPlanning.h"
#include <sgtdv_msgs/ColoredConeArr.h>
#include <sgtdv_msgs/CarState.h>
#include <sgtdv_msgs/PathPlanningMsg.h>
#include <iostream>

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
    sgtdv_msgs::CarState::ConstPtr m_lastPose;
    sgtdv_msgs::ColoredConeArr::ConstPtr m_lastMap;
    bool m_poseReceived;
};