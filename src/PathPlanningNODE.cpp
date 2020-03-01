/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#include <ros/ros.h>
#include "../include/PathPlanningSynch.h"
#include <sgtdv_msgs/Point2DArr.h>
#include "../include/PathPlanningDisciplines.h"

int main (int argc, char** argv)
{
    PathPlanningSynch synchObj;

    ros::init(argc, argv, "pathPlanning");
    ros::NodeHandle handle;

    ros::Publisher publisher = handle.advertise<sgtdv_msgs::Point2DArr>("pathplanning_trajectory", 1);

    synchObj.SetPublisher(publisher);

    ros::Subscriber mapSub = handle.subscribe("slam_map", 1, &PathPlanningSynch::Do, &synchObj);
    ros::Subscriber poseSub = handle.subscribe("slam_pose", 1, &PathPlanningSynch::UpdatePose, &synchObj);    

    if (/*arg from launchfile*/true)
    {
        synchObj.SetDiscipline(UNKNOWN_TRACK);
    }   
    else
    {
        synchObj.SetDiscipline(SKIDPAD);
    }

    //TODO: Set yellow cones side (left or right)
    //synchObj.YellowOnLeft(true/false);

    ros::spin();

    return 0;
}