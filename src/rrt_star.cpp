/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Samuel Mazur, Patrik Knaperek
/*****************************************************/

#include "../include/rrt_star.h"

RRTStar::RRTStar(ros::NodeHandle& handle)
{
  Utils::loadParam(handle, "/rrt_conf/car_width", &conf_.car_width);
  Utils::loadParam(handle, "/rrt_conf/node_step_size", &conf_.node_step_size);
  conf_.neighbor_radius = 5 * conf_.node_step_size;
  Utils::loadParam(handle, "/rrt_conf/max_angle", &conf_.max_angle);
  Utils::loadParam(handle, "/rrt_conf/max_iter", &conf_.max_iter);
};

/**
 * @brief Initialization of whole object.
 * @param outside_cones - reference for world size computing
 * @param inside_cones
 */
void RRTStar::init(const std::vector<Eigen::Vector2f> &outside_cones, const std::vector<Eigen::Vector2f> &inside_cones)
{
	out_cones_ = outside_cones;
  in_cones_ = inside_cones;
	end_pos_ = (out_cones_.back() + inside_cones.back()) * 0.5;
  const Eigen::Vector2f start_pos = (outside_cones.front() + inside_cones.front()) * 0.5;
	
  setWorldSize(outside_cones);
	initializeRootNode(start_pos);
	srand48(time(0));	
}

bool RRTStar::update()
{
  path_reverse_.clear();
  for(int i = 0; i < conf_.max_iter; i++)
  {
    // generate random position
    Eigen::Vector2f new_pos;
    if(getRandNodePos(&new_pos))
    {
      Node::Ptr q_nearest_ptr(findNearestNode(new_pos));

      if(q_nearest_ptr.get()!= nullptr
          && computeDistance(new_pos, q_nearest_ptr->position) > conf_.node_step_size  // check minimum distance from closest node
        )
      {
        // compute new node position based on vehicle model restrictions
        normalizePosition(*q_nearest_ptr, &new_pos);
        
        if(isOnTrack(new_pos))
        {
          // initialize new node
          auto q_new = std::make_shared<Node>(new_pos);
          
          // From the nearby nodes, choose the one with which the connection will have the lowest cost.
          std::vector<Node::Ptr> q_near;
          findNearNodes(q_new->position, &q_near);
          Node::Ptr q_min = q_nearest_ptr;
          double cost_min = q_nearest_ptr->cost + pathCost(*q_nearest_ptr, *q_new);
          double cost_near;
          for(const auto& q : q_near)
          {
            if(std::abs(computeAngleDiff(*q, q_new->position)) < conf_.max_angle   // angle restriction
              && (cost_near = (q->cost + pathCost(*q, *q_new))) < cost_min)
            {
              q_min = q;
              cost_min = cost_near;
            }
          }

          // add new node to tree
          add(cost_min, q_min, q_new);

          for(auto& q : q_near)
          {  /* trajectory cost optimization
              * - rebase the nearby nodes if having the new node as the parent node would result in lower cost
              */
            if(std::abs(computeAngleDiff(*q_new, q->position)) < conf_.max_angle    // angle restriction
                && (q_new->cost + pathCost(*q_new, *q)) < q->cost)
            {
              Node::Ptr q_parent = q->parent;
              q_parent->children.erase(std::remove(q_parent->children.begin(), q_parent->children.end(), q), 
                                      q_parent->children.end());
              q->cost = q_new->cost + pathCost(*q_new, *q);
              q->parent = q_new;
              q->orientation = computeOrientation(q_new->position, q->position);
              q_new->children.push_back(q);
            }
          }
        }
        else i--; // to reset iteration counter in case new point is not on track
      }
    }
  }

  /* Check if the tree has reached the end position */
  bool end_reached(false);
  Node::Ptr q_last(nullptr);
  
  float min_dist = std::numeric_limits<float>::max();
  for(const auto &node : nodes_)
  {
    const double dist = computeDistance(end_pos_, node->position); 
    if(dist < min_dist                                               /* Find nearest node to the end position witch: */
      && node->parent != nullptr                                     /* has a parent node (is connected to the tree and we can build up a reverse path from it) */
      && cos(computeOrientation(end_pos_, node->position)) < 0.0)    /* is behind the end pos in the direction of the track */
    { 
      min_dist = dist;
      q_last = node;
    }
  }
  
  if(min_dist < conf_.neighbor_radius)
  {
    end_reached = true;
  }

  // generate reverse path
  while(q_last != nullptr) 
  {
    path_reverse_.push_back(q_last);
    q_last = q_last->parent;
  }

  return end_reached;
}

const sgtdv_msgs::Point2DArr RRTStar::getPath() const
{
  sgtdv_msgs::Point2DArr path;
  sgtdv_msgs::Point2D point;
  path.points.reserve(path_reverse_.size());
  
  if(path_reverse_.size() != 0)
  {
    for(size_t i = path_reverse_.size(); i != 0; i--)
    {
        point.x = path_reverse_[i-1]->position(0);
        point.y = path_reverse_[i-1]->position(1);
        path.points.push_back(point);
    }
  }
  return path;
}

/**
 * @brief Initialize root node of RRT*.
 * @param start_pos Position of the root node.
 */
void RRTStar::initializeRootNode(const Eigen::Ref<const Eigen::Vector2f>& start_pos)
{
  root_ = std::make_shared<Node>(start_pos);
  nodes_.push_back(root_);
}

/**
 * @brief Set world size for generating Nodes.
 * @param outside_cones
 * @param start_cone_index
 * @param end_cone_index
 */
void RRTStar::setWorldSize(const std::vector<Eigen::Vector2f> &cones)
{
  world_x_.min = std::numeric_limits<float>::max();
  world_x_.max = std::numeric_limits<float>::lowest();
  world_y_.min = std::numeric_limits<float>::max();
  world_y_.max = std::numeric_limits<float>::lowest();
  
  for(const auto& cone : cones)
  {
    if(world_x_.min > cone(0)) world_x_.min = cone(0);
    if(world_x_.max < cone(0)) world_x_.max = cone(0);

    if(world_y_.min > cone(1)) world_y_.min = cone(1);
    if(world_y_.max < cone(1)) world_y_.max = cone(1);
  }

  world_width_ = world_x_.max - world_x_.min;
  world_height_ = world_y_.max - world_y_.min;
}

/**
 * @brief Generate a random node position in the field.
 * @param point
 * @return true if generated position is in map
 */
bool RRTStar::getRandNodePos(Eigen::Vector2f *point) const
{
  (*point)(0) = (drand48() * world_width_) + world_x_.min;
  (*point)(1) = (drand48() * world_height_) + world_y_.min;
  
  return ((*point)(0) >= world_x_.min && 
          (*point)(0) <= world_x_.max && 
          (*point)(1) >= world_y_.min && 
          (*point)(1) <= world_y_.max);
}

/**
 * @brief Helper method to find distance between two positions.
 * @param p
 * @param q
 * @return
 */
float RRTStar::computeDistance(const Eigen::Ref<const Eigen::Vector2f>& p, 
                                const Eigen::Ref<const Eigen::Vector2f>& q) const
{
  return Eigen::Vector2f(p - q).norm();
}

float RRTStar::computeOrientation(const Eigen::Ref<const Eigen::Vector2f>& p_from,
                                  const Eigen::Ref<const Eigen::Vector2f>& p_to) const
{
  return std::atan2((p_to - p_from)(1), (p_to - p_from)(0));
}

/**
 * @brief Get nearest node from a given configuration/position.
 * @param point
 * @return
 */
RRTStar::Node::Ptr RRTStar::findNearestNode(const Eigen::Ref<const Eigen::Vector2f>& point) const
{
  float min_dist = std::numeric_limits<float>::max();
  Node::Ptr closest(nullptr);
  for(const auto &node : nodes_)
  {
    const double dist = computeDistance(point, node->position);
    
    if(dist < min_dist)
    {
      min_dist = dist;
      closest = node;
    }
  }
  
  return closest;
}

/**
 * @brief Get neighborhood nodes of a given configuration/position.
 * @param point
 * @param out_nodes
 * @return
 */
void RRTStar::findNearNodes(const Eigen::Vector2f point, std::vector<Node::Ptr> *out_nodes) const
{
  for(const auto &node : nodes_)
  {
    const double dist = computeDistance(point, node->position);
    
    if(dist < conf_.neighbor_radius)
    {
      (*out_nodes).push_back(node);
    }
  }
}

/**
 * @brief Find a configuration at a distance step_size from nearest node to random node.
 * @param q_nearest_pos
 * @param q_pos
 * @return
 */
void RRTStar::normalizePosition(const Node &q_nearest, Eigen::Vector2f *q_pos) const
{
  // angle restriction
  auto angle_diff = computeAngleDiff(q_nearest, *q_pos);
  angle_diff = std::max(-conf_.max_angle, std::min(conf_.max_angle, angle_diff));

  const double angle = q_nearest.orientation + angle_diff;

  *q_pos = 
    q_nearest.position + Eigen::Vector2f(cosf(angle) * conf_.node_step_size, sinf(angle) * conf_.node_step_size);
}

/**
 * @brief Compute path cost.
 * @param q_from
 * @param q_to
 * @return
 */
double RRTStar::pathCost(const Node &q_from, const Node &q_to) const
{
  const double distance = computeDistance(q_to.position, q_from.position);
  return distance;
}

float RRTStar::computeAngleDiff(const Node &q_from, const Eigen::Ref<const Eigen::Vector2f>& p_to) const
{
  float angle_diff = computeOrientation(q_from.position, p_to) - q_from.orientation;
  
  /* cut angle to interval <-PI, PI> */
  if(std::abs(angle_diff) > M_PI)
  {
    angle_diff = std::pow(-1, static_cast<int>(angle_diff > 0)) * (2 * M_PI - std::abs(angle_diff));
  }
  return angle_diff;
}

/**
 * @brief Add a node to the tree.
 * @param cost
 * @param q_nearest
 * @param q_new
 */
void RRTStar::add(const double cost, const Node::Ptr &q_nearest, const Node::Ptr &q_new)
{
  q_new->parent = q_nearest;
  q_new->orientation = computeOrientation(q_nearest->position, q_new->position);
  q_new->cost = cost;
  q_nearest->children.push_back(q_new);
  nodes_.push_back(q_new);
}

/**
 * @brief Validate if point is on track.
 * @param target_point
 * @return
 */
bool RRTStar::isOnTrack(const Eigen::Ref<const Eigen::Vector2f>& target_point) const
{
  bool c(false);
  int i, j = 0;
  for(i = 0, j = out_cones_.size() - 1; i < out_cones_ .size(); j = i++) 
  {
    if(((out_cones_ [i](1) > target_point(1)) != (out_cones_ [j](1) > target_point(1))) && 
      ((target_point(0) < (out_cones_ [j](0) - out_cones_ [i](0)) * (target_point(1) - out_cones_ [i](1)) /
        (out_cones_ [j](1) - out_cones_ [i](1)) + out_cones_ [i](0))))
      c = !c;
  }
  if(c)
  {
    for(const auto &cone : out_cones_ ) 
    {
      double dist = computeDistance(cone, target_point);
      if(dist < conf_.car_width / 2) 
      {
        c = false;
        break;
      }
    }
  }

  if(c)
  {
    for(i = 0, j = in_cones_.size() - 1; i < in_cones_.size(); j = i++) 
    {
      if(((in_cones_ [i](1) > target_point(1)) != (in_cones_ [j](1) > target_point(1))) && 
        ((target_point(0) < (in_cones_ [j](0) - in_cones_ [i](0)) * (target_point(1) - in_cones_ [i](1)) /
          (in_cones_ [j](1) - in_cones_ [i](1)) + in_cones_ [i](0))))
        c = !c;
    }

    if(c)
    {
      for(const auto &cone : in_cones_ ) 
      {
        double dist = computeDistance(cone, target_point);
        if(dist < conf_.car_width / 2) 
        {
          c = false;
          break;
        }
      }
    }
  }

  return c;
}
