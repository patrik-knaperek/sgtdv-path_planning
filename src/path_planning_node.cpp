/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský
/*****************************************************/


#include "../include/path_planning_synch.h"
//#include "../include/path_planning_disciplines.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_planning");
  ros::NodeHandle handle;

  PathPlanningSynch synch_obj(handle);

  ros::Publisher publisher_trajectory = handle.advertise<sgtdv_msgs::Point2DArr>("pathplanning_trajectory", 1);
  ros::ServiceClient set_speed_client = handle.serviceClient<sgtdv_msgs::Float32Srv>("pathTracking/set_speed");

#ifdef SGT_DEBUG_STATE
  ros::Publisher pathplanning_vis_debug_pub = handle.advertise<sgtdv_msgs::DebugState>("pathplanning_debug_state", 10);
  synch_obj.setVisDebugPublisher(pathplanning_vis_debug_pub);
#endif

#ifdef SGT_VISUALIZATION
  ros::Publisher rrt_vis_pub = handle.advertise<visualization_msgs::MarkerArray>("pathplanning/visualize/rrt", 1);
  ros::Publisher interpolated_cones_vis_pub = handle.advertise<visualization_msgs::MarkerArray>("pathplanning/visualize/interpolated_cones", 1);
#endif /* SGT_VISUALIZATION */
  
  synch_obj.setPublisher(publisher_trajectory
                  #ifdef SGT_VISUALIZATION
                      , rrt_vis_pub
                      , interpolated_cones_vis_pub
                  #endif /* SGT_VISUALIZATION */
                      );

  ros::Subscriber map_sub = handle.subscribe("slam/map", 1, &PathPlanningSynch::updateMap, &synch_obj);
  ros::Subscriber pose_sub = handle.subscribe("slam/pose", 1, &PathPlanningSynch::updatePose, &synch_obj);
  ros::Subscriber loop_close_sub = handle.subscribe("slam/loop_closure", 1, &PathPlanningSynch::loopClosureCallback, &synch_obj);

  //if(/*arg from launchfile*/true)
  //{
  //    synch_obj.setDiscipline(UNKNOWN_TRACK);
  //}   
  //else
  //{
  //    synch_obj.setDiscipline(SKIDPAD);
  //}

  //TODO: Set yellow cones side (left or right)
  //synch_obj.yellowOnLeft(true/false);

  ros::spin();

  return 0;
}