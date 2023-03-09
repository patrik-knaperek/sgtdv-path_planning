/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#include "../include/PathPlanningSynch.h"
//#include "../include/PathPlanningDisciplines.h"

int main (int argc, char** argv)
{
    PathPlanningSynch synchObj;

    ros::init(argc, argv, "pathPlanning");
    ros::NodeHandle handle;

    ros::Publisher publisherTrajectory = handle.advertise<sgtdv_msgs::Point2DArr>("pathplanning_trajectory", 1);
    ros::Publisher publisherTrajectoryVisualize = handle.advertise<visualization_msgs::MarkerArray>("pathplanning_trajectory_visualize", 1);
    ros::Publisher publisherInterpolatedCones = handle.advertise<visualization_msgs::MarkerArray>("pathplanning_interpolated_cones", 1);
    
    synchObj.SetPublisher(publisherTrajectory
                        , publisherTrajectoryVisualize
                        , publisherInterpolatedCones
                        );

    ros::Subscriber mapSub = handle.subscribe("slam_map", 1, &PathPlanningSynch::UpdateMap, &synchObj);
    ros::Subscriber poseSub = handle.subscribe("slam_pose", 1, &PathPlanningSynch::UpdatePose, &synchObj);
    ros::Subscriber loopCloseSub = handle.subscribe("slam_loop_closure", 1, &PathPlanningSynch::LoopClosureCallback, &synchObj);

    //if (/*arg from launchfile*/true)
    //{
    //    synchObj.SetDiscipline(UNKNOWN_TRACK);
    //}   
    //else
    //{
    //    synchObj.SetDiscipline(SKIDPAD);
    //}

    //TODO: Set yellow cones side (left or right)
    //synchObj.YellowOnLeft(true/false);

    ros::spin();

    return 0;
}