/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Samuel Mazur, Patrik Knaperek
/*****************************************************/

#include "../include/rrt_star.h"

RRTStar::RRTStar(ros::NodeHandle& handle) :
  last_node_(nullptr)
{
  Utils::loadParam(handle, "/rrt_conf/car_width", &conf_.car_width);
  Utils::loadParam(handle, "/rrt_conf/node_step_size", &conf_.node_step_size);
  conf_.neighbor_radius = 5 * conf_.node_step_size;
  Utils::loadParam(handle, "/rrt_conf/max_angle", &conf_.max_angle);
  Utils::loadParam(handle, "/rrt_conf/max_iter", &conf_.max_iter);
};

/**
 * @brief Initialization of whole object.
 * @param outside_cones
 * @param inside_cones
 * @param start_index
 * @param end_index
 * @param start_position
 * @param end_position
  */
void RRTStar::init(const std::vector<Eigen::Vector2f> &outside_cones, const std::vector<Eigen::Vector2f> &inside_cones,
			            const int start_index, const int end_index, const Eigen::Vector2f start_position, 
                  const Eigen::Vector2f end_position)
{
	out_cones_ = outside_cones;
  in_cones_ = inside_cones;
	end_pos_ = end_position;	
	
  setWorldSize(outside_cones, start_index, end_index);  
	initializeRootNode(start_position);
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
      NodeSPtr q_nearest_ptr(findNearestNode(new_pos));

      if(q_nearest_ptr.get()!= nullptr
          && computeDistance(new_pos, q_nearest_ptr->position) > conf_.node_step_size  // check minimum distance from closest node
        )
      {
        // compute new node position based on vehicle model restrictions
        normalizePosition(*q_nearest_ptr, &new_pos);
        
        if(isOnTrack(new_pos))
        {
          // initialize new node
          auto q_new = std::make_shared<Node>();;
          q_new->position = new_pos;
          q_new->orientation = 0;
          
          // find near node with minimal cost
          std::vector<NodeSPtr> q_near;
          findNearNodes(q_new->position, &q_near);
          NodeSPtr q_min = q_nearest_ptr;
          double cost_min = q_nearest_ptr->cost + pathCost(*q_nearest_ptr, *q_new);
          double cost_near;
          for(const auto q_near : q_near)
          {
            if(std::abs(computeAngleDiff(*q_near, q_new->position)) < conf_.max_angle   // angle restriction
              && (cost_near = (q_near->cost + pathCost(*q_near, *q_new))) < cost_min)
            {
              q_min = q_near;
              cost_min = cost_near;
            }
          }

          // add new node to tree
          add(cost_min, q_min, q_new);

          for(auto q_near : q_near)
          {  // trajectory cost optimization
            if(std::abs(computeAngleDiff(*q_new, q_near->position)) < conf_.max_angle    // angle restriction
                && (q_new->cost + pathCost(*q_new, *q_near)) < q_near->cost)
            {
              NodeSPtr q_parent = q_near->parent;
              q_parent->children.erase(std::remove(q_parent->children.begin(), q_parent->children.end(), q_near), 
                                      q_parent->children.end());
              q_near->cost = q_new->cost + pathCost(*q_new, *q_near);
              q_near->parent = q_new;
              q_near->orientation = computeOrientation(q_new->position, q_near->position);
              q_new->children.push_back(q_near);
              // m_lastNode = qNear;
            }
          }
        }
        else i--; //to reset iteration counter in case new point is not on track
      }
    }
  }

  bool end_reached(false);
  NodeSPtr q(nullptr);
  float min_dist = std::numeric_limits<float>::max();
  for(const auto &node : nodes_)
  {
    double dist = computeDistance(end_pos_, node->position);
    if(dist < min_dist && node->parent != nullptr
      && cos(computeOrientation(end_pos_, node->position)) < 0.0)
    {
      min_dist = dist;
      q = node;
    }
  }
  last_node_ = q;
  if(min_dist < conf_.neighbor_radius)
  {
    end_reached = true;
  }

  // generate shortest path to destination.
  while(q != nullptr) 
  {
    path_reverse_.push_back(q);
    q = q->parent;
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
 * @brief Initialize root node of RRTSTAR.
 */
void RRTStar::initializeRootNode(const Eigen::Vector2f start_pos)
{
  root_ = std::make_shared<Node>();
  root_->parent = nullptr;
  root_->position = start_pos;
  root_->orientation = 0.0;
  root_->cost = 0.0;
  last_node_ = root_;
  nodes_.push_back(root_);
}

/**
 * @brief Set world size for generating Nodes.
 * @param outside_cones
 * @param start_cone_index
 * @param end_cone_index
 */
void RRTStar::setWorldSize(const std::vector<Eigen::Vector2f> &cones, const int start_index, const int end_index)
{
  x_min = std::numeric_limits<float>::max();
  x_max_ = std::numeric_limits<float>::lowest();
  y_min = std::numeric_limits<float>::max();
  y_max_ = std::numeric_limits<float>::lowest();
  world_width_ = 0;
  world_height_ = 0;

  for(size_t i = start_index; i<end_index; i++)
  {
    if(x_min > cones[i](0)) x_min = cones[i](0);
    if(x_max_ < cones[i](0)) x_max_ = cones[i](0);

    if(y_min > cones[i](1)) y_min = cones[i](1);
    if(y_max_ < cones[i](1)) y_max_ = cones[i](1);
  }

  world_width_ = x_max_ - x_min;
  world_height_ = y_max_ - y_min;
}

/**
 * @brief Generate a random node position in the field.
 * @param point
 * @return true if generated position is in map
 */
bool RRTStar::getRandNodePos(Eigen::Vector2f *point) const
{
  (*point)(0) = (drand48() * world_width_) + x_min;
  (*point)(1) = (drand48() * world_height_) + y_min;
  
  return ((*point)(0) >= x_min && (*point)(0) <= x_max_ && (*point)(1) >= y_min && (*point)(1) <= y_max_);
}

/**
 * @brief Helper method to find distance between two positions.
 * @param p
 * @param q
 * @return
 */
double RRTStar::computeDistance(const Eigen::Vector2f p, const Eigen::Vector2f q) const
{
  return sqrt(powf(p(0) - q(0), 2) + powf(p(1) - q(1), 2));
}

double RRTStar::computeOrientation(const Eigen::Vector2f p_from, const Eigen::Vector2f p_to) const
{
  return std::atan2((p_to(1) - p_from(1)), (p_to(0) - p_from(0)));
}

/**
 * @brief Get nearest node from a given configuration/position.
 * @param point
 * @return
 */
RRTStar::NodeSPtr RRTStar::findNearestNode(const Eigen::Vector2f point) const
{
  float min_dist = std::numeric_limits<float>::max();
  NodeSPtr closest(nullptr);
  for(const auto &node : nodes_)
  {
    double dist = computeDistance(point, node->position);
    
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
 * @param radius
 * @param out_nodes
 * @return
 */
void RRTStar::findNearNodes(const Eigen::Vector2f point, std::vector<NodeSPtr> *out_nodes) const
{
  for(const auto &node : nodes_)
  {
    double dist = computeDistance(point, node->position);
    double angle = computeAngleDiff(*node, point);
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
  double angle_diff = computeAngleDiff(q_nearest, *q_pos);
  if(angle_diff > conf_.max_angle)
  {
    angle_diff = conf_.max_angle;
  } 
  else if(angle_diff < -conf_.max_angle)
  {
    angle_diff = -conf_.max_angle;
  }
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

double RRTStar::computeAngleDiff(const Node &q_from, const Eigen::Vector2f p_to) const
{
  double angle_diff = computeOrientation(q_from.position, p_to) - q_from.orientation;
  if(std::abs(angle_diff) > M_PI)
  {
    angle_diff = std::pow(-1, static_cast<int>(angle_diff > 0)) * (2 * M_PI - std::abs(angle_diff));
  }
  return angle_diff;
}

/**
 * @brief Add a node to the tree.
 * @param q_nearest
 * @param q_new
 */
void RRTStar::add(const double cost_min, const NodeSPtr &q_nearest, const NodeSPtr &q_new)
{
  q_new->parent = q_nearest;
  q_new->orientation = computeOrientation(q_nearest->position, q_new->position);
  q_new->cost = cost_min;
  q_nearest->children.push_back(q_new);
  nodes_.push_back(q_new);
  // m_lastNode = qNew;
}

/**
 * @brief Validate if point is on track.
 * @param target_point
 * @return
 */
bool RRTStar::isOnTrack(const Eigen::Vector2f target_point) const
{
  bool c = false;
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
