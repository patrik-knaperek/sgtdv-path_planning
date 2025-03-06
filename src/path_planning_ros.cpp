/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Samuel Mazur, Patrik Knaperek
/*****************************************************/

/* SGT-DV */
#include <sgtdv_msgs/DebugState.h>
#include <sgtdv_msgs/Float32Srv.h>

/* Header */
#include "path_planning_ros.h"

PathPlanningROS::PathPlanningROS(ros::NodeHandle& nh) :
  /* ROS interface init */
  trajectory_pub_(nh.advertise<sgtdv_msgs::Trajectory>("path_planning/trajectory", 1)),
  set_speed_server_(nh.advertiseService("path_planning/set_speed", &PathPlanningROS::setSpeedCallback, this)),
#ifdef SGT_VISUALIZATION
  boundaries_vis_pub_(nh.advertise<visualization_msgs::MarkerArray>("path_planning/visualize/track_boundaries", 1)),
  rrt_vis_pub_(nh.advertise<visualization_msgs::MarkerArray>("path_planning/visualize/rrt", 1)),
#endif /* SGT_VISUALIZATION */
#ifdef SGT_DEBUG_STATE
  vis_debug_publisher_(nh.advertise<sgtdv_msgs::DebugState>("path_planning/debug_state", 10)),
#endif

  map_sub_(nh.subscribe("slam/map", 1, &PathPlanningROS::mapCallback, this)),
  pose_sub_(nh.subscribe("slam/pose", 1, &PathPlanningROS::poseCallback, this)),
  loop_close_sub_(nh.subscribe("slam/loop_closure", 1, &PathPlanningROS::loopClosureCallback, this)),
    
  map_received_(false),
  pose_received_(false)
{
  loadParams(nh);

#ifdef SGT_VISUALIZATION
  initPathBoundariesMarkers();
  initRRTPointsMarker();
#endif /* SGT_VISUALIZATION */
}

/// @brief Load paramteres from server.
/// @param nh ROS node handle
void PathPlanningROS::loadParams(const ros::NodeHandle& nh)
{
  PathPlanning::Params params;

  Utils::loadParam(nh, "/ref_speed/slow", &params.ref_speed_slow);
  Utils::loadParam(nh, "/ref_speed/fast", &params.ref_speed_fast);

  RRTStar::RRTconf rrt_conf;
  Utils::loadParam(nh, "/rrt_conf/car_width", &rrt_conf.car_width);
  Utils::loadParam(nh, "/rrt_conf/node_step_size", &rrt_conf.node_step_size);
  rrt_conf.neighbor_radius = 5 * rrt_conf.node_step_size;
  Utils::loadParam(nh, "/rrt_conf/max_angle", &rrt_conf.max_angle);
  Utils::loadParam(nh, "/rrt_conf/max_iter", &rrt_conf.max_iter);

  path_planning_obj_.setParams(params, rrt_conf);
}

/**
 * @brief Main function in class.
 * @param incoming_ros_msg
 */
void PathPlanningROS::update()
{
  if(pose_received_ && map_received_)
  {
    map_received_ = false;
    pose_received_ = false;

  #ifdef SGT_DEBUG_STATE
    sgtdv_msgs::DebugState state;
    state.stamp = ros::Time::now();
    state.working_state = 1;
    vis_debug_publisher_.publish(state);
  #endif // SGT_DEBUG_STATE

    const auto trajectory = path_planning_obj_.update(path_planning_msg_);

    if(trajectory.path.points.size() > 0)
    { 
      trajectory_pub_.publish(trajectory);

    #ifdef SGT_VISUALIZATION
      visualizeInterpolatedCones();
      visualizeRRTPoints();
    #endif /* SGT_VISUALIZATION */
    }
    
  #ifdef SGT_DEBUG_STATE
    state.stamp = ros::Time::now();
    state.working_state = 0;
    state.num_of_cones = trajectory.path.points.size();
    vis_debug_publisher_.publish(state);
  #endif // SGT_DEBUG_STATE
  }
  else
  {
    // ROS_DEBUG("PathPlanningSynch - Do: PathPlanning message not ready\n");
  }    
}

/**
 * @brief Read SLAM map message.
 * @param slam_map_msg
 */
void PathPlanningROS::mapCallback(const sgtdv_msgs::ConeArr::ConstPtr &msg)
{
  if(!msg->cones.empty())
  {
    path_planning_msg_.cone_map = *msg;
    map_received_ = true;
    update();
  }
  else
    map_received_ = false;
}

void PathPlanningROS::loopClosureCallback(const std_msgs::Empty::ConstPtr &msg)
{
  path_planning_obj_.fullMap();
}

/**
 * @brief Read car position message.
 * @param car_pose_msg
 */
void PathPlanningROS::poseCallback(const sgtdv_msgs::CarPose::ConstPtr &msg)
{
  path_planning_msg_.car_pose = *msg;
  pose_received_ = true;
  update();
}

bool PathPlanningROS::setSpeedCallback(sgtdv_msgs::Float32Srv::Request &req, sgtdv_msgs::Float32Srv::Response &res)
{
  path_planning_obj_.setRefSpeed(req.data);

  ROS_INFO_STREAM("Setting reference speed manually: " << req.data);
  return true;
}

/*void PathPlanningSynch::SetDiscipline(Discipline discipline)
{
  m_pathPlanning.SetDiscipline(discipline);
}*/

#ifdef SGT_VISUALIZATION
void PathPlanningROS::initPathBoundariesMarkers()
{
  left_cones_interpolated_marker_.type = visualization_msgs::Marker::POINTS;
  left_cones_interpolated_marker_.header.frame_id = "map";
  left_cones_interpolated_marker_.id = 0;
  left_cones_interpolated_marker_.ns = "left_interpolated";
  left_cones_interpolated_marker_.scale.x = 0.2;
  left_cones_interpolated_marker_.scale.y = 0.2;
  left_cones_interpolated_marker_.color.b = 1.0f;
  left_cones_interpolated_marker_.color.a = 1.0;

  right_cones_interpolated_marker_.type = visualization_msgs::Marker::POINTS;
  right_cones_interpolated_marker_.header.frame_id = "map";
  right_cones_interpolated_marker_.id = 1;
  right_cones_interpolated_marker_.ns = "right_interpolated";
  right_cones_interpolated_marker_.scale.x = 0.2;
  right_cones_interpolated_marker_.scale.y = 0.2;
  right_cones_interpolated_marker_.color.r = 0.8f;
  right_cones_interpolated_marker_.color.g = 0.8f;
  right_cones_interpolated_marker_.color.a = 1.0;

  left_cones_marker_.type = visualization_msgs::Marker::POINTS;
  left_cones_marker_.header.frame_id = "map";
  left_cones_marker_.id = 2;
  left_cones_marker_.ns = "left_cones";
  left_cones_marker_.scale.x = 0.4;
  left_cones_marker_.scale.y = 0.4;
  left_cones_marker_.color.b = 1.0f;
  left_cones_marker_.color.a = 1.0;
  left_cones_marker_.pose.orientation.w = 1.0;

  right_cones_marker_.type = visualization_msgs::Marker::POINTS;
  right_cones_marker_.header.frame_id = "map";
  right_cones_marker_.id = 3;
  right_cones_marker_.ns = "right_cones";
  right_cones_marker_.scale.x = 0.4;
  right_cones_marker_.scale.y = 0.4;
  right_cones_marker_.color.r = 0.8f;
  right_cones_marker_.color.g = 0.8f;
  right_cones_marker_.color.a = 1.0;
  right_cones_marker_.pose.orientation.w = 1.0;

  start_marker_.type = visualization_msgs::Marker::POINTS;
  start_marker_.header.frame_id = "map";
  start_marker_.id = 4;
  start_marker_.ns = "start";
  start_marker_.scale.x = 0.5;
  start_marker_.scale.y = 0.5;
  start_marker_.color.r = 0.7f;
  start_marker_.color.a = 1.0;
  
  finish_marker_.type = visualization_msgs::Marker::POINTS;
  finish_marker_.header.frame_id = "map";
  finish_marker_.id = 5;
  finish_marker_.ns = "finish";
  finish_marker_.scale.x = 0.5;
  finish_marker_.scale.y = 0.5;
  finish_marker_.color.r = 0.0f;
  finish_marker_.color.g = 0.7f;
  finish_marker_.color.a = 1.0;
}

void PathPlanningROS::initRRTPointsMarker()
{
  rrt_nodes_marker_.type = visualization_msgs::Marker::POINTS;
  rrt_nodes_marker_.action = visualization_msgs::Marker::ADD;
  rrt_nodes_marker_.header.frame_id = "map";
  rrt_nodes_marker_.id = 0;
  rrt_nodes_marker_.ns = "RRT nodes";
  rrt_nodes_marker_.scale.x = 0.15;
  rrt_nodes_marker_.scale.y = 0.15;
  rrt_nodes_marker_.color.g = 1.0f;
  rrt_nodes_marker_.color.a = 1.0;
  rrt_nodes_marker_.pose.orientation.w = 1.0;
  rrt_nodes_marker_.pose.position.z = -0.2;

  rrt_trajectory_marker_.type = visualization_msgs::Marker::LINE_STRIP;
  rrt_trajectory_marker_.action = visualization_msgs::Marker::ADD;
  rrt_trajectory_marker_.header.frame_id = "map";
  rrt_trajectory_marker_.id = 1;
  rrt_trajectory_marker_.ns = "RRT trajectory";
  rrt_trajectory_marker_.scale.x = 0.2;
  rrt_trajectory_marker_.scale.y = 0.2;
  rrt_trajectory_marker_.color.g = 1.0f;
  rrt_trajectory_marker_.color.a = 1.0;
  rrt_trajectory_marker_.pose.orientation.w = 1.0;
  rrt_trajectory_marker_.pose.position.z = -0.2;
}

/**
 * @brief Publishing message for sorted and interpolated cone data.
 * @return
 */
void PathPlanningROS::visualizeInterpolatedCones()
{	
  static visualization_msgs::MarkerArray marker_arr;

  const auto cones_interpolated = path_planning_obj_.getConesInterpolated();
  const auto cones = path_planning_obj_.getCones();
  
  /* reinit markers */
  deleteMarkers(marker_arr, boundaries_vis_pub_);

  left_cones_interpolated_marker_.points.clear();
  left_cones_interpolated_marker_.points.reserve(cones_interpolated.first.size());
  
  right_cones_interpolated_marker_.points.clear();
  right_cones_interpolated_marker_.points.reserve(cones_interpolated.second.size());
  
  left_cones_marker_.points.clear();
  left_cones_marker_.points.reserve(cones.first.size());
  
  right_cones_marker_.points.clear();
  right_cones_marker_.points.reserve(cones.second.size());

  start_marker_.points.clear();
  start_marker_.points.reserve(2);

  finish_marker_.points.clear();
  finish_marker_.points.reserve(2);
  
  marker_arr.markers.reserve(6);

  geometry_msgs::Point temp;

  /* update interpolated markers */
  for(const auto &cone : cones_interpolated.first)
  {
    temp.x = cone(0);
    temp.y = cone(1);
    
    left_cones_interpolated_marker_.points.push_back(temp);
  }

  for(const auto &cone : cones_interpolated.second)
  {
    temp.x = cone(0);
    temp.y = cone(1);

    right_cones_interpolated_marker_.points.push_back(temp);
  }

  marker_arr.markers.push_back(right_cones_interpolated_marker_);
  marker_arr.markers.push_back(left_cones_interpolated_marker_);

  /* update cone markers */
  for(const auto &cone : cones.first)
  {
    temp.x = cone(0);
    temp.y = cone(1);

    left_cones_marker_.points.push_back(temp);
  }

  for(const auto &cone : cones.second)
  {
    temp.x = cone(0);
    temp.y = cone(1);

    right_cones_marker_.points.push_back(temp);
  }

  marker_arr.markers.push_back(right_cones_marker_);
  marker_arr.markers.push_back(left_cones_marker_);

  /* update start and finish markers */
  temp.x = cones.second[0](0);
  temp.y = cones.second[0](1);
  start_marker_.points.push_back(temp);
  temp.x = cones.first[0](0);
  temp.y = cones.first[0](1);
  start_marker_.points.push_back(temp);
  marker_arr.markers.push_back(start_marker_);

  temp.x = cones.second[cones.second.size()-1](0);
  temp.y = cones.second[cones.second.size()-1](1);
  finish_marker_.points.push_back(temp);
  temp.x = cones.first[cones.first.size()-1](0);
  temp.y = cones.first[cones.first.size()-1](1);
  finish_marker_.points.push_back(temp);
  marker_arr.markers.push_back(finish_marker_);

  boundaries_vis_pub_.publish(marker_arr);
}

/**
 * @brief Publishing message for RRT data.
 * @return
 */
void PathPlanningROS::visualizeRRTPoints()
{
  static visualization_msgs::MarkerArray rrt_markers;
  const auto rrt_path = path_planning_obj_.getRRTPath();
  const auto rrt_nodes = path_planning_obj_.getRRTNodes();
  
  /* reinit markers */
  deleteMarkers(rrt_markers, rrt_vis_pub_);
  rrt_nodes_marker_.points.clear();
  rrt_nodes_marker_.points.reserve(rrt_nodes.size());
  rrt_trajectory_marker_.points.clear();
  rrt_trajectory_marker_.points.reserve(rrt_path.points.size());

  geometry_msgs::Point point_vis;

  /* update RRT nodes markers */
  for(const auto &node : rrt_nodes)
  {
    point_vis.x = node->position(0);
    point_vis.y = node->position(1);
    rrt_nodes_marker_.points.push_back(point_vis);	
  }

  rrt_markers.markers.push_back(rrt_nodes_marker_);
 
  /* update RRT trajectory markers */
  for(const auto &path_it : rrt_path.points)
  {
    point_vis.x = path_it.x;
    point_vis.y = path_it.y;
    rrt_trajectory_marker_.points.push_back(point_vis);
  }

  rrt_markers.markers.push_back(rrt_trajectory_marker_);
  rrt_vis_pub_.publish(rrt_markers);
}

void PathPlanningROS::deleteMarkers(visualization_msgs::MarkerArray& marker_array,
                                const ros::Publisher& publisher) const
{
  marker_array.markers.clear();
  visualization_msgs::Marker marker;

  marker.id = 0;
  marker.action = marker.DELETEALL;
  marker_array.markers.emplace_back(marker);

  publisher.publish(marker_array);
}
#endif /* SGT_VISUALIZATION */
