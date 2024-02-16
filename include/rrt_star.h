/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Samuel Mazur, Patrik Knaperek
/*****************************************************/

#pragma once

/* C++ */
#include <Eigen/Eigen>

/* ROS */
#include <ros/ros.h>

/* SGT DV */
#include <sgtdv_msgs/Point2DArr.h>
#include "messages.h"
#include "SGT_Utils.h"

class RRTStar
{
public:
  struct Node
  {
    typedef std::shared_ptr<RRTStar::Node> Ptr;
    
    Node();

    Node(const Eigen::Ref<const Eigen::Vector2f>& pos) :
      parent(nullptr),
      position(pos),
      orientation(0.f),
      cost(0.f)
    {
    };

    Node::Ptr parent;
    std::vector<Node::Ptr> children;
    Eigen::Vector2f position;
    float orientation;
    double cost;
  };
  

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
  void init(const std::vector<Eigen::Vector2f> &outside_cones, const std::vector<Eigen::Vector2f> &inside_cones);
  const std::vector<Node::Ptr> getNodes() const { return nodes_; };
  const sgtdv_msgs::Point2DArr getPath() const;

private:
  void initializeRootNode(const Eigen::Ref<const Eigen::Vector2f>& start_pos);
  void setWorldSize(const std::vector<Eigen::Vector2f> &cones);
  bool getRandNodePos(Eigen::Vector2f *point) const;
  Node::Ptr findNearestNode(const Eigen::Ref<const Eigen::Vector2f>& point) const;
  void findNearNodes(const Eigen::Vector2f point, std::vector<Node::Ptr> *out_nodes) const;
  double pathCost(const Node &q_from, const Node &q_to) const;
  void normalizePosition(const Node &q_nearest, Eigen::Vector2f *q_pos) const;
  void add(const double cost, const Node::Ptr &q_nearest, const Node::Ptr &q_new);
  bool isOnTrack(const Eigen::Ref<const Eigen::Vector2f>& target_point) const;
  float computeDistance(const Eigen::Ref<const Eigen::Vector2f>& p, const Eigen::Ref<const Eigen::Vector2f>& q) const;
  float computeOrientation(const Eigen::Ref<const Eigen::Vector2f>& p_from,
                            const Eigen::Ref<const Eigen::Vector2f>& p_to) const;
  float computeAngleDiff(const Node &q_from, const Eigen::Ref<const Eigen::Vector2f>& p_to) const;

  RRTconf conf_;

  std::vector<Node::Ptr> nodes_, path_reverse_;

  std::vector<Eigen::Vector2f> in_cones_, out_cones_;

  Node::Ptr root_ = nullptr;
  Eigen::Vector2f end_pos_;
  Utils::Range<float> world_x_, world_y_;
  float world_width_, world_height_;
};
