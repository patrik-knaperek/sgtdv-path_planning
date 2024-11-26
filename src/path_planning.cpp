/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Samuel Mazur, Patrik Knaperek
/*****************************************************/

/* SGT-DV */
#include "SGT_Utils.h"

/* Header */
#include "path_planning.h"

PathPlanning::PathPlanning() :
  once_(true),
  full_map_(false)
{
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
 * @brief Main function in class.
 * @param incoming_ros_msg
 */
sgtdv_msgs::Point2DArr PathPlanning::update(const sgtdv_msgs::PathPlanningMsg &msg)
{
//m_publisher.publish(m_pathPlanningDiscipline->Do(msg));
  
  sgtdv_msgs::Point2DArr trajectory;

  sortCones(msg);
  
  if(left_cones_.size() && right_cones_.size())
  {
    left_cones_interpolated_ = linearInterpolation(left_cones_, msg.car_pose);
    right_cones_interpolated_ = linearInterpolation(right_cones_, msg.car_pose);

    static bool rrt_completed(false);
    if(full_map_)
    {
      rrt_completed = rrtRun();
    }

    if(rrt_completed)
    {
      trajectory = rrt_star_obj_.getPath();
      ref_speed_ = params_.ref_speed_fast;
    }
    else
    {
      trajectory = findMiddlePoints();
      ref_speed_ = params_.ref_speed_slow;
    }
  }
  else
  {
    ROS_WARN("Invalid map obtained. Cannot distinguish track borders.");
  }
  
  return trajectory;
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
    
    rrt_star_obj_.init(left_cones_interpolated_, right_cones_interpolated_);
  }

  //execution timer
  const auto start_marker_ = std::chrono::high_resolution_clock::now();

  const bool end_reached = rrt_star_obj_.update();

  const auto stop = std::chrono::high_resolution_clock::now();
  const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start_marker_);
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
void PathPlanning::sortCones(const sgtdv_msgs::PathPlanningMsg &msg)
{
  left_cones_.clear();
  right_cones_.clear();

  left_cones_interpolated_.clear();
  right_cones_interpolated_.clear();


  for(const auto &cone : msg.cone_map.cones)
  {
    const Eigen::Vector2f cone_pos(cone.coords.x, cone.coords.y);

    switch(cone.color)
    {
      case 'y':
        right_cones_.push_back(cone_pos);
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
        left_cones_.push_back(cone_pos);
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
        break;
      default:
        ROS_WARN("Unknown color of cone");
    }
  }
}

/**
 * @brief Linear interpolation for single colored cones.
 * @param sorted_cones
 * @return
 */
Points PathPlanning::linearInterpolation(const Points& points, const sgtdv_msgs::CarPose& car_pose) const
{
  Points temp_pts;
  Eigen::Vector2f temp;

  for(size_t i = 0; i < points.size(); i++)
  {	
    // const float max_dist = sqrt((pow(points[i+1](0) - points[i](0), 2) + pow(points[i+1](1) - points[i](1), 2)) * 1.0);
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
      else if(points.size() > 1)
      {
        endpoint << 2 * points[i][0] - points[i-1][0], 2 * points[i][1] - points[i-1][1];
      }
      else // points.size() == 1
      {
        endpoint << points[i][0] + 4 * cos(car_pose.yaw), points[i][1] + 4 * sin(car_pose.yaw);
      }
    }

    // if(max_dist > 6) step /=2;

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
    const Eigen::Vector2f new_point((left_cones_interpolated_[i] + right_cones_interpolated_[i]) / 2.f);
    middle_line_points_.emplace_back(new_point);		
  }

  for(size_t i = 0; i < middle_line_points_.size()-4; i += 4)
  {	

    Eigen::Vector2f endpoint2 = middle_line_points_[i+2];
    Eigen::Vector2f endpoint3 = middle_line_points_[i+3];
    Eigen::Vector2f endpoint4 = middle_line_points_[i+4];

    if((i+2) == middle_line_points_.size()) endpoint2 = middle_line_points_[0];
    if((i+3) > middle_line_points_.size()) endpoint3 = middle_line_points_[1];	
    if((i+4) > middle_line_points_.size()) endpoint4 = middle_line_points_[2];


    for(float j = 0 ; j < 1 ; j += BEZIER_RESOLUTION)
    {
      const float xa = middle_line_points_[i](0) + ((middle_line_points_[i+1](0) - middle_line_points_[i](0)) * j);
      const float ya = middle_line_points_[i](1) + ((middle_line_points_[i+1](1) - middle_line_points_[i](1)) * j);
      const float xb = middle_line_points_[i+1](0) + ((endpoint2(0)-middle_line_points_[i+1](0)) * j);
      const float yb = middle_line_points_[i+1](1) + ((endpoint2(1)-middle_line_points_[i+1](1)) * j);
      const float xc = endpoint2(0) + ((endpoint3(0)-endpoint2(0)) * j);
      const float yc = endpoint2(1) + ((endpoint3(1)-endpoint2(1)) * j);
      const float xd = endpoint3(0) + ((endpoint4(0)-endpoint3(0)) * j);
      const float yd = endpoint3(1) + ((endpoint4(1)-endpoint3(1)) * j);

      const float xe = xa + ((xb - xa) * j);
      const float ye = ya + ((yb - ya) * j);
      const float xf = xb + ((xc - xb) * j);
      const float yf = yb + ((yc - yb) * j);
      const float xg = xc + ((xd - xc) * j);
      const float yg = yc + ((yd - yc) * j);

      const float xh = xe + ((xf - xe) * j);
      const float yh = ye + ((yf - ye) * j);
      const float xi = xf + ((xg - xf) * j);
      const float yi = yf + ((yg - yf) * j);
      
      point.x = xh + ((xi - xh) * j);
      point.y = yh + ((yi - yh) * j);
      trajectory.points.push_back(point);
    }
    if(!i) i -= 2;
  }

  return trajectory;	
}
