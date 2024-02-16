/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Samuel Mazur, Patrik Knaperek
/*****************************************************/


#include "../include/path_planning.h"

PathPlanning::PathPlanning(ros::NodeHandle& handle) :
  /* ROS interface init */
  trajectory_pub_(handle.advertise<sgtdv_msgs::Point2DArr>("pathplanning_trajectory", 1)),
  set_speed_client_(handle.serviceClient<sgtdv_msgs::Float32Srv>("pathTracking/set_speed")),
#ifdef SGT_VISUALIZATION
  interpolated_cones_pub_(handle.advertise<visualization_msgs::MarkerArray>("pathplanning/visualize/interpolated_cones", 1)),
  rrt_vis_pub_(handle.advertise<visualization_msgs::MarkerArray>("pathplanning/visualize/rrt", 1)),
#endif /* SGT_VISUALIZATION */
#ifdef SGT_DEBUG_STATE
  vis_debug_publisher_(handle.advertise<sgtdv_msgs::DebugState>("pathplanning_debug_state", 10)),
#endif

  rrt_star_obj_(handle),
  is_yellow_on_left_(true),
  once_(true),
  full_map_(false)
{
  Utils::loadParam(handle, "/ref_speed/slow", &params_.ref_speed_slow);
  Utils::loadParam(handle, "/ref_speed/fast", &params_.ref_speed_fast);
}

/*void PathPlanning::SetDiscipline(Discipline discipline)
{
  switch(discipline)
  {
    case UNKNOWN_TRACK: path_planning_discipline_obj = new UnknownTrack; break;
    case SKIDPAD: path_planning_discipline_obj = new Skidpad; break;
    default: path_planning_discipline_obj = nullptr;
  }

  if(!path_planning_discipline_obj) ros::shutdown();
}*/

/**
 * @brief Swap color of cones in arrays.
 * @param is_yellow_on_left
 */
void PathPlanning::yellowOnLeft(bool value)
{
  //m_pathPlanningDiscipline->YellowOnLeft(value);
  is_yellow_on_left_ = value;
}

/**
 * @brief Main function in class.
 * @param incoming_ros_msg
 */
void PathPlanning::update(const PathPlanningMsg &msg)
{
#ifdef SGT_DEBUG_STATE
  sgtdv_msgs::DebugState state;
  state.stamp = ros::Time::now();
  state.working_state = 1;
  vis_debug_publisher_.publish(state);
#endif // SGT_DEBUG_STATE

//m_publisher.publish(m_pathPlanningDiscipline->Do(msg));

  sortCones(msg);
  if(!left_cones_.size() || !right_cones_.size())
  {
    ROS_WARN("Invalid map obtained. Cannot distinguish track borders.");
    return;
  }
  left_cones_interpolated_ = linearInterpolation(left_cones_);
  right_cones_interpolated_ = linearInterpolation(right_cones_);

#ifdef SGT_VISUALIZATION
  visualizeInterpolatedCones();
#endif /* SGT_VISUALIZATION */

  bool rrt_completed(false);
  if(full_map_)
  {
    rrt_completed = rrtRun();
  }

  sgtdv_msgs::Point2DArr trajectory;
  if(rrt_completed)
  {
    trajectory = rrt_star_obj_.getPath();
    set_speed_msg_.request.data = params_.ref_speed_fast;
  }
  else
  {
    trajectory = findMiddlePoints();
    set_speed_msg_.request.data = params_.ref_speed_slow;
  }

  // when used with path_tracking
  if(!ros::service::call("pathTracking/set_speed", set_speed_msg_))
  {
    ROS_ERROR("Service \"pathTracking/set_speed\" failed");
  }
  trajectory_pub_.publish(trajectory);

#ifdef SGT_VISUALIZATION
  visualizeRRTPoints();
#endif /* SGT_VISUALIZATION */
#ifdef SGT_DEBUG_STATE
  state.stamp = ros::Time::now();
  state.working_state = 0;
  state.num_of_cones = trajectory.points.size();
  vis_debug_publisher_.publish(state);
#endif // SGT_DEBUG_STATE
}

/**
 * @brief Main function to handle RRT.
 */
bool PathPlanning::rrtRun()
{
  if(once_)
  {
    once_ = false;
    timer_avg_ = 0;
    timer_avg_count_ = 0;
    
    Eigen::Vector2f start_pos = ((left_cones_interpolated_[0] + right_cones_interpolated_[0]) * 0.5f);
    short cone_iter = left_cones_interpolated_.size();
    Eigen::Vector2f end_pos = 
      ((left_cones_interpolated_[left_cones_interpolated_.size()-1] 
      + right_cones_interpolated_[right_cones_interpolated_.size()-1]) * 0.5f);
    
    rrt_star_obj_.init(left_cones_interpolated_, right_cones_interpolated_,0, cone_iter, start_pos, end_pos);
  }

  //execution timer
  auto start = std::chrono::high_resolution_clock::now();

  bool end_reached = rrt_star_obj_.update();

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  timer_avg_ += duration.count()/ 1000000.f;
  timer_avg_count_ +=1;
  ROS_DEBUG("\nRRT Timer: %f s", timer_avg_/timer_avg_count_);
  ROS_DEBUG("Path jumps: %ld", rrt_star_obj_.getPath().points.size());
  ROS_DEBUG("Total nodes: %ld\n",rrt_star_obj_.getNodes().size());

  return end_reached;	
}

/**
 * @brief Sorting raw cone data.
 * @param message_cones
 */
void PathPlanning::sortCones(const PathPlanningMsg &msg)
{
  left_cones_.clear();
  right_cones_.clear();

  left_cones_interpolated_.clear();
  right_cones_interpolated_.clear();


  for(const auto &cone : msg.cone_map->cones)
  {
    Eigen::Vector2f conePos(cone.coords.x, cone.coords.y);

    switch(cone.color)
    {
      case 'y':
      case 1 :
        right_cones_.push_back(conePos);
        {
          if(right_cones_.size() > 2)
          {
            const double d1 = (*std::prev(right_cones_.end(), 1) - *std::prev(right_cones_.end(), 3)).norm();
            const double d2 = (*std::prev(right_cones_.end(), 2) - *std::prev(right_cones_.end(), 3)).norm();
            if(d1 < d2) std::swap(*std::prev(right_cones_.end(),1), *std::prev(right_cones_.end(),2));
          }
        }
        break;
      case 'b':
      case 2 :
        left_cones_.push_back(conePos);
        {
          if(left_cones_.size() > 2)
          {
            const double d1 = (*std::prev(left_cones_.end(), 1) - *std::prev(left_cones_.end(), 3)).norm();
            const double d2 = (*std::prev(left_cones_.end(), 2) - *std::prev(left_cones_.end(), 3)).norm();
            if(d1 < d2) std::swap(*std::prev(left_cones_.end(),1), *std::prev(left_cones_.end(),2));
          }
        }
        break;
      case 's':
      case 'g':
      case 3 :
        break;
      default:
        ROS_ERROR("Unknown color of cone\n");
    }
  }

  if(is_yellow_on_left_)
  {
    std::swap(left_cones_, right_cones_);
  }
}

/**
 * @brief Linear interpolation for single colored cones.
 * @param sorted_cones
 * @return
 */
std::vector<Eigen::Vector2f> PathPlanning::linearInterpolation(std::vector<Eigen::Vector2f> points) const
{
  std::vector<Eigen::Vector2f> temp_pts;
  Eigen::Vector2f temp;

  for(size_t i = 0; i < points.size(); i++)
  {	
    const float max_dist = sqrt((pow(points[i+1](0) - points[i](0), 2) + pow(points[i+1](1) - points[i](1), 2)) * 1.0);
    float step = BEZIER_RESOLUTION;

    Eigen::Vector2f endpoint;
    if(i + 1 < points.size())
    {
      endpoint = points[i+1];
    }
    else
    {
      // closing cone loop - last cone is the first cone
      if(full_map_)
      {
        endpoint = points[0];
      }
      else
      {
        endpoint << 2 * points[i][0] - points[i-1][0], 2 * points[i][1] - points[i-1][1];
      }
    }

    if(max_dist > 6) step /=2;

    for(float j = 0 ; j < 1 ; j += step)
    {	
      temp(0) = points[i](0) + ((endpoint(0)-points[i](0))*j);
      temp(1) = points[i](1) + ((endpoint(1)-points[i](1))*j);
      temp_pts.push_back(temp);
    }
  }	

  return temp_pts; 
}

/**
 * @brief Publishing message for middleline trajectory.
 * @return
 */
sgtdv_msgs::Point2DArr PathPlanning::findMiddlePoints()
{
  sgtdv_msgs::Point2DArr trajectory;
  sgtdv_msgs::Point2D point;

  middle_line_points_.clear();

  for(size_t i = 0; i < std::min(right_cones_interpolated_.size(), left_cones_interpolated_.size()); i++)
  {
    Eigen::Vector2f new_point = ((left_cones_interpolated_[i] + right_cones_interpolated_[i]) / 2.f);
    middle_line_points_.push_back(new_point);		
  }

  for(size_t i = 0; i < middle_line_points_.size()-2; i+=4)
  {	

    Eigen::Vector2f endpoint2 = middle_line_points_[i+2];
    Eigen::Vector2f endpoint3 = middle_line_points_[i+3];
    Eigen::Vector2f endpoint4 = middle_line_points_[i+4];

    if((i+2) == middle_line_points_.size()) endpoint2 = middle_line_points_[0];
    if((i+3)>middle_line_points_.size()) endpoint3 = middle_line_points_[1];	
    if((i+4)>middle_line_points_.size()) endpoint4 = middle_line_points_[2];


    for( float j = 0 ; j < 1 ; j += 0.01)
    {
      float xa = middle_line_points_[i](0) + ((middle_line_points_[i+1](0)-middle_line_points_[i](0))*j);
      float ya = middle_line_points_[i](1) + ((middle_line_points_[i+1](1)-middle_line_points_[i](1))*j);
      float xb= middle_line_points_[i+1](0) + ((endpoint2(0)-middle_line_points_[i+1](0))*j);
      float yb = middle_line_points_[i+1](1) + ((endpoint2(1)-middle_line_points_[i+1](1))*j);
      float xc = endpoint2(0) + ((endpoint3(0)-endpoint2(0))*j);
      float yc = endpoint2(1) + ((endpoint3(1)-endpoint2(1))*j);
      float xd = endpoint3(0) + ((endpoint4(0)-endpoint3(0))*j);
      float yd = endpoint3(1) + ((endpoint4(1)-endpoint3(1))*j);

      float xe = xa + ((xb-xa)*j);
      float ye = ya + ((yb-ya)*j);
      float xf= xb + ((xc-xb)*j);
      float yf = yb + ((yc-yb)*j);
      float xg= xc + ((xd-xc)*j);
      float yg = yc + ((yd-yc)*j);

      float xh = xe + ((xf-xe)*j);
      float yh = ye + ((yf-ye)*j);
      float xi= xf + ((xg-xf)*j);
      float yi = yf + ((yg-yf)*j);
      
      point.x = xh + ((xi-xh)*j);
      point.y = yh + ((yi-yh)*j);
      trajectory.points.push_back(point);
    }
    if(!i) i-=2;
  }

  return trajectory;	
}

#ifdef SGT_VISUALIZATION
/**
 * @brief Publishing message for sorted and interpolated cone data.
 * @return
 */
void PathPlanning::visualizeInterpolatedCones()
{	
  visualization_msgs::MarkerArray marker_arr;
  geometry_msgs::Point temp;

  visualization_msgs::Marker left_points;
  left_points.type = visualization_msgs::Marker::POINTS;
  left_points.header.frame_id = "map";
  left_points.id = 0;
  left_points.ns = "left_interpolated";
  left_points.scale.x = 0.2;
  left_points.scale.y = 0.2;
  left_points.color.r = 0.8f;
  left_points.color.g = 0.8f;
  left_points.color.a = 1.0;

  visualization_msgs::Marker right_points;
  right_points.type = visualization_msgs::Marker::POINTS;
  right_points.header.frame_id = "map";
  right_points.id = 1;
  right_points.ns = "right_interpolated";
  right_points.scale.x = 0.2;
  right_points.scale.y = 0.2;
  right_points.color.b = 1.0f;
  right_points.color.a = 1.0;

  visualization_msgs::Marker left_points_big;
  left_points_big.type = visualization_msgs::Marker::POINTS;
  left_points_big.header.frame_id = "map";
  left_points_big.id = 2;
  left_points_big.ns = "left_cones";
  left_points_big.scale.x = 0.4;
  left_points_big.scale.y = 0.4;
  left_points_big.color.r = 0.8f;
  left_points_big.color.g = 0.8f;
  left_points_big.color.a = 1.0;
  left_points_big.pose.orientation.w = 1.0;

  visualization_msgs::Marker right_points_big;
  right_points_big.type = visualization_msgs::Marker::POINTS;
  right_points_big.header.frame_id = "map";
  right_points_big.id = 3;
  right_points_big.ns = "right_cones";
  right_points_big.scale.x = 0.4;
  right_points_big.scale.y = 0.4;
  right_points_big.color.b = 1.0f;
  right_points_big.color.a = 1.0;
  right_points_big.pose.orientation.w = 1.0;


  for(const auto &cone : left_cones_interpolated_)
  {
    temp.x = cone(0);
    temp.y = cone(1);
    right_points.points.push_back(temp);	
  }
  marker_arr.markers.push_back(right_points);


  for(const auto &cone : right_cones_interpolated_)
  {
    temp.x = cone(0);
    temp.y = cone(1);
    left_points.points.push_back(temp);	
  }
  marker_arr.markers.push_back(left_points);

  for(const auto &cone : left_cones_)
  {
    temp.x = cone(0);
    temp.y = cone(1);
    right_points_big.points.push_back(temp);	
  }
  marker_arr.markers.push_back(right_points_big);


  for(const auto &cone : right_cones_)
  {
    temp.x = cone(0);
    temp.y = cone(1);
    left_points_big.points.push_back(temp);	
  }
  marker_arr.markers.push_back(left_points_big);

  visualization_msgs::Marker start;
  start.type = visualization_msgs::Marker::POINTS;
  start.header.frame_id = "map";
  start.id = 4;
  start.ns = "start";
  start.scale.x = 0.5;
  start.scale.y = 0.5;
  start.color.r = 0.7f;
  start.color.a = 1.0;
  temp.x = right_cones_[0](0);
  temp.y = right_cones_[0](1);
  start.points.push_back(temp);
  temp.x = left_cones_[0](0);
  temp.y = left_cones_[0](1);
  start.points.push_back(temp);
  marker_arr.markers.push_back(start);

  visualization_msgs::Marker end;
  end.type = visualization_msgs::Marker::POINTS;
  end.header.frame_id = "map";
  end.id = 5;
  end.ns = "end";
  end.scale.x = 0.5;
  end.scale.y = 0.5;
  end.color.r = 0.0f;
  end.color.g = 0.7f;
  end.color.a = 1.0;
  temp.x = right_cones_[right_cones_.size()-1](0);
  temp.y = right_cones_[right_cones_.size()-1](1);
  end.points.push_back(temp);
  temp.x = left_cones_[left_cones_.size()-1](0);
  temp.y = left_cones_[left_cones_.size()-1](1);
  end.points.push_back(temp);
  marker_arr.markers.push_back(end);

  interpolated_cones_pub_.publish(marker_arr);
}

/**
 * @brief Publishing message for RRT data.
 * @return
 */
void PathPlanning::visualizeRRTPoints()
{
  static visualization_msgs::MarkerArray trajectory_vis_markers;
  trajectory_vis_markers.markers.clear();
  geometry_msgs::Point point_vis;

  visualization_msgs::Marker nodes_marker;
  nodes_marker.type = visualization_msgs::Marker::POINTS;
  nodes_marker.action = visualization_msgs::Marker::ADD;
  nodes_marker.header.frame_id = "map";
  nodes_marker.id = 0;
  nodes_marker.ns = "RRT nodes";
  nodes_marker.scale.x = 0.15;
  nodes_marker.scale.y = 0.15;
  nodes_marker.color.g = 1.0f;
  nodes_marker.color.a = 1.0;
  nodes_marker.pose.orientation.w = 1.0;
  nodes_marker.pose.position.z = -0.2;

  auto nodes = rrt_star_obj_.getNodes();
  nodes_marker.points.reserve(nodes.size());
  for(const auto &node : nodes)
  {
    point_vis.x = node->position(0);
    point_vis.y = node->position(1);
    nodes_marker.points.push_back(point_vis);	
  }

  trajectory_vis_markers.markers.emplace_back(nodes_marker);

  visualization_msgs::Marker trajectory;
  trajectory.type = visualization_msgs::Marker::LINE_STRIP;
  trajectory.action = visualization_msgs::Marker::ADD;
  trajectory.header.frame_id = "map";
  trajectory.id = 1;
  trajectory.ns = "RRT trajectory";
  trajectory.scale.x = 0.2;
  trajectory.scale.y = 0.2;
  trajectory.color.g = 1.0f;
  trajectory.color.a = 1.0;
  trajectory.pose.orientation.w = 1.0;
  trajectory.pose.position.z = -0.2;
  
  auto path = rrt_star_obj_.getPath().points;
  trajectory.points.reserve(path.size());
  for(const auto &path_it : path)
  {
    point_vis.x = path_it.x;
    point_vis.y = path_it.y;
    trajectory.points.push_back(point_vis);
  }

  trajectory_vis_markers.markers.emplace_back(trajectory);
  rrt_vis_pub_.publish(trajectory_vis_markers);
}
#endif /* SGT_VISUALIZATION */
