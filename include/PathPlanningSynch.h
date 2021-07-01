/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#include <ros/ros.h>
#include "../include/PathPlanning.h"
#include "../include/PathPlanningDisciplines.h"
#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/CarPose.h>
#include <iostream>
#include "../include/Messages.h"


class PathPlanningSynch
{
public:
    PathPlanningSynch();
    ~PathPlanningSynch();

    void SetPublisher(ros::Publisher publisher);
    void Do(const sgtdv_msgs::ConeArr::ConstPtr &msg);
    void UpdatePose(const sgtdv_msgs::CarPose::ConstPtr &msg);
    void YellowOnLeft(bool value);
    void SetDiscipline(Discipline discipline);

private:
    PathPlanning m_pathPlanning;   
    PathPlanningMsg m_pathPlanningMsg;
    bool m_poseReceived;
};