/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#include "../include/PathPlanningSynch.h"
//#include "../include/PathPlanningDisciplines.h"

int main (int argc, char** argv)
{
    ros::init(argc, argv, "pathPlanning");
    ros::NodeHandle handle;

    PathPlanningSynch synchObj(handle);

    ros::Publisher publisherTrajectory = handle.advertise<sgtdv_msgs::Point2DArr>("pathplanning_trajectory", 1);
    ros::ServiceClient setSpeedClient = handle.serviceClient<sgtdv_msgs::Float32Srv>("pathTracking/set_speed");

#ifdef SGT_VISUALIZATION
    ros::Publisher publisherPathPlanningVisualize = handle.advertise<visualization_msgs::MarkerArray>("pathplanning/visualize/rrt", 1);
    ros::Publisher publisherInterpolatedCones = handle.advertise<visualization_msgs::MarkerArray>("pathplanning/visualize/interpolated_cones", 1);
#endif /* SGT_VISUALIZATION */
    
    synchObj.SetPublisher(publisherTrajectory
                    #ifdef SGT_VISUALIZATION
                        , publisherPathPlanningVisualize
                        , publisherInterpolatedCones
                    #endif /* SGT_VISUALIZATION */
                        );

    ros::Subscriber mapSub = handle.subscribe("slam/map", 1, &PathPlanningSynch::UpdateMap, &synchObj);
    ros::Subscriber poseSub = handle.subscribe("slam/pose", 1, &PathPlanningSynch::UpdatePose, &synchObj);
    ros::Subscriber loopCloseSub = handle.subscribe("slam/loop_closure", 1, &PathPlanningSynch::LoopClosureCallback, &synchObj);

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