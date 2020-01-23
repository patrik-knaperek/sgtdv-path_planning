#include <ros/ros.h>
#include "../include/PathPlanningSynch.h"
#include <sgtdv_msgs/PathTrackingMsg.h>

int main (int argc, char** argv)
{
    PathPlanningSynch synchObj;

    ros::init(argc, argv, "pathPlanning");
    ros::NodeHandle handle;

    ros::Publisher publisher = handle.advertise<sgtdv_msgs::PathTrackingMsg>("pathplanning_trajectory", 1);

    synchObj.SetPublisher(publisher);    

    ros::Subscriber mapSub = handle.subscribe("slam_map", 1, &PathPlanningSynch::Do, &synchObj);
    ros::Subscriber poseSub = handle.subscribe("slam_pose", 1, &PathPlanningSynch::UpdatePose, &synchObj);
    
    ros::spin();

    return 0;
}