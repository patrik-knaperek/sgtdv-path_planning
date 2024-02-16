/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Samuel Mazur, Patrik Knaperek
/*****************************************************/

#pragma once

/* C++ */
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <limits>
#include <time.h>
#include <chrono>
//#include "opencv2/core/core.hpp"
#include <Eigen/Eigen>

/* ROS */
#include <ros/ros.h>

/* SGT DV */
#include <sgtdv_msgs/Point2DArr.h>
#include "../include/messages.h"
#include "../../SGT_Utils.h"

class RRTStar
{
public:
  struct Node
  {
    // Node() : children(nullptr), parent(nullptr), position(Eigen::Vector2f::Zero()), orientation(0.f), cost(0.0)
    // {};
    std::vector<std::shared_ptr<Node>> children;
    std::shared_ptr<Node> parent;
    Eigen::Vector2f position;
    float orientation;
    double cost;
  };
  typedef std::shared_ptr<Node> NodeSPtr;

  struct RRTconf
  {
    float car_width;
    float node_step_size;
    float neighbor_radius;
    float max_angle;
    float max_iter;
  };

public:
  RRTStar(ros::NodeHandle& nh);
  ~RRTStar() = default;

  bool update();
  void init(const std::vector<Eigen::Vector2f> &outside_cones, const std::vector<Eigen::Vector2f> &inside_cones, 
            const int start_index, const int end_index, const Eigen::Vector2f start_position,
            const Eigen::Vector2f end_position);
  const std::vector<NodeSPtr> getNodes() const { return nodes_; };
  const sgtdv_msgs::Point2DArr getPath() const;

private:
  void initializeRootNode(const Eigen::Vector2f start_pos);
  void setWorldSize(const std::vector<Eigen::Vector2f> &cones, const int start_index, const int end_index);
  bool getRandNodePos(Eigen::Vector2f *point) const;
  NodeSPtr findNearestNode(const Eigen::Vector2f point) const;
  void findNearNodes(const Eigen::Vector2f point, std::vector<NodeSPtr> *out_nodes) const;
  double pathCost(const Node &q_from, const Node &q_to) const;
  void normalizePosition(const Node &q_nearest, Eigen::Vector2f *q_pos) const;
  void add(const double cost_min, const NodeSPtr &q_nearest, const NodeSPtr &q_new);
  bool isOnTrack(const Eigen::Vector2f target_point) const;
  double computeDistance(const Eigen::Vector2f p, const Eigen::Vector2f q) const;
  double computeOrientation(const Eigen::Vector2f p_from, const Eigen::Vector2f p_to) const;
  double computeAngleDiff(const Node &q_from, const Eigen::Vector2f p_to) const;

  RRTconf conf_;

  std::vector<NodeSPtr> nodes_, path_reverse_;

  std::vector<Eigen::Vector2f> in_cones_, out_cones_;

  NodeSPtr root_, last_node_;
  Eigen::Vector2f end_pos_;
  float x_min, x_max_, y_min, y_max_;
  float world_width_;
  float world_height_;
};

