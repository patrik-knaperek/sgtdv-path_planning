/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#include <ros/ros.h>
#include "../include/PathPlanning.h"
#include "../include/PathPlanningDisciplines.h"
#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/CarState.h>
#include <iostream>
#include "../include/Messages.h"


enum PathPlanningDiscipline
{
    UNKNOWN_TRACK = 0,
    SKIDPAD
};

class PathPlanningSynch
{
public:
    PathPlanningSynch();
    ~PathPlanningSynch();

    void SetPublisher(ros::Publisher publisher);
    void Do(const sgtdv_msgs::ConeArr::ConstPtr &msg);
    void UpdatePose(const sgtdv_msgs::CarState::ConstPtr &msg);
    void YellowOnLeft(bool value);
    void SetDiscipline(PathPlanningDiscipline discipline);

private:
    PathPlanning *m_pathPlanning;   
    PathPlanningMsg m_pathPlanningMsg;
    bool m_poseReceived;
};