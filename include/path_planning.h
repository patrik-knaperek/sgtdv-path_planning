/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský, Samuel Mazur, Patrik Knaperek
/*****************************************************/

#pragma once

/* SGT */
//#include "path_planning_disciplines.h"
#include <sgtdv_msgs/PathPlanningMsg.h>
#include "SGT_Macros.h"
#include "rrt_star.h"

constexpr float BEZIER_RESOLUTION = 0.125;

class PathPlanning
{ 
public:
  struct Params
  {
    float ref_speed_slow;
    float ref_speed_fast;
  };

public:
  explicit PathPlanning();
  ~PathPlanning() = default;

  sgtdv_msgs::Point2DArr update(const sgtdv_msgs::PathPlanningMsg &msg);  
  void fullMap() { full_map_ = true; };
  //void setDiscipline(Discipline discipline);

  /* Setters & Getters */

  void setParams(const Params& params, const RRTStar::RRTconf rrt_conf)
  {
    params_ = params;
    rrt_star_obj_.setConf(rrt_conf);
  }

  float getRefSpeed(void)
  {
    return ref_speed_;
  };

  std::pair<Points,Points> getCones(void) const
  {
    return std::pair<Points, Points>(left_cones_, right_cones_);
  };

  std::pair<Points,Points> getConesInterpolated(void) const
  {
    return std::pair<Points, Points>(left_cones_interpolated_, right_cones_interpolated_);
  };

  const std::vector<RRTStar::Node::Ptr> getRRTNodes(void) const
  { 
    return rrt_star_obj_.getNodes(); 
  };
  
  const sgtdv_msgs::Point2DArr getRRTPath(void) const
  {
    return rrt_star_obj_.getPath();
  }

private:
  bool rrtRun();
  void sortCones(const sgtdv_msgs::PathPlanningMsg &msg);
  Points linearInterpolation(const Points& points, const sgtdv_msgs::CarPose& car_pose) const;
  sgtdv_msgs::Point2DArr findMiddlePoints();    
  
  RRTStar rrt_star_obj_;
  Params params_;
	
  float timer_avg_;
  int timer_avg_count_;

  bool once_;
  bool full_map_;

  float ref_speed_;

  Points left_cones_, left_cones_interpolated_, right_cones_, 
        right_cones_interpolated_, middle_line_points_;
  //PathPlanningDiscipline *path_planning_discipline_obj = nullptr;
};
