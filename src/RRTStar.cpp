/*****************************************************/
//Organization: Stuba Green Team
//Authors: Samuel Mazur, Patrik Knaperek
/*****************************************************/

#include "../include/RRTStar.h"

RRTStar::RRTStar()
: m_lastNode(nullptr)
{};

RRTStar::~RRTStar() = default;

/**
 * @brief Initialization of whole object.
 * @param outsideCones
 * @param insideCones
 * @param startIndex
 * @param endIndex
 * @param startPosition
 * @param endPosition
  */
void RRTStar::Init(const std::vector<cv::Vec2f> &outsideCones, const std::vector<cv::Vec2f> &insideCones, 
			const int startIndex, const int endIndex, const cv::Vec2f startPosition, const cv::Vec2f endPosition)
{
	m_outCones = outsideCones;
    m_inCones = insideCones;
	m_endPos = endPosition;	
	
    SetWorldSize(outsideCones, startIndex, endIndex);  
	InitializeRootNode(startPosition);
	srand48(time(0));	
}

void RRTStar::Do()
{
	m_pathReverse.clear();
    for(int i = 0; i < MAX_ITER; i++)
    {
        // generate random position
        cv::Vec2f newPos;
        if (GetRandNodePos(&newPos))
        {
            NodeSPtr qNearestPtr(FindNearestNode(newPos));

            if (qNearestPtr.get() != nullptr
                && ComputeDistance(newPos, qNearestPtr->position) > NODE_STEP_SIZE  // check minimum distance from closest node
                )
            {
                // compute new node position based on vehicle model restrictions
                NormalizePosition(*qNearestPtr, &newPos);
                
                if (isOnTrack(newPos))
                {
                    // initialize new node
                    auto qNew = std::make_shared<Node>();;
                    qNew->position = newPos;
                    qNew->orientation = 0;
                    
                    // find near node with minimal cost
                    std::vector<NodeSPtr> Qnear;
                    FindNearNodes(qNew->position, &Qnear);
                    NodeSPtr qMin = qNearestPtr;
                    double costMin = qNearestPtr->cost + PathCost(*qNearestPtr, *qNew);
                    double costNear;
                    for (const auto qNear : Qnear)
					{
                        if (std::abs(computeAngleDiff(*qNear, qNew->position)) < ANGLE_TH   // angle restriction
                            && (costNear = (qNear->cost + PathCost(*qNear, *qNew))) < costMin)
						{
                            qMin = qNear;
                            costMin = costNear;
                        }
                    }

                    // add new node to tree
                    Add(costMin, qMin, qNew);

                    for (auto qNear : Qnear)
                    {   // trajectory cost optimization
                        if(std::abs(computeAngleDiff(*qNew, qNear->position)) < ANGLE_TH    // angle restriction
                            && (qNew->cost + PathCost(*qNew, *qNear)) < qNear->cost)
                        {
                            NodeSPtr qParent = qNear->parent;
                            qParent->children.erase(std::remove(qParent->children.begin(), qParent->children.end(), qNear), qParent->children.end());
                            qNear->cost = qNew->cost + PathCost(*qNew, *qNear);
                            qNear->parent = qNew;
                            qNear->orientation = ComputeOrientation(qNew->position, qNear->position);
                            qNew->children.push_back(qNear);
                            // m_lastNode = qNear;
                        }
                    }
                }
		        else i--; //to reset iteration counter in case new point is not on track
            }
        }
    }
    NodeSPtr q(nullptr);
    // if (ComputeDistance(m_lastNode->position, m_endPos) < END_DIST_THRESHOLD)
    // {
    //     ROS_INFO("END REACHED");
    //     q = m_lastNode;
    // }
    // else
    {
        // if not reached yet
        // ROS_INFO_STREAM("end pos: " << m_endPos);
        // q = FindNearestNode(m_endPos);

        float minDist = std::numeric_limits<float>::max();
        for (const auto &node : m_nodes) {
            double dist = ComputeDistance(m_endPos, node->position);
            if (dist < minDist && node->parent != nullptr
                && cos(ComputeOrientation(m_endPos, node->position)) < 0.0) {
                minDist = dist;
                q = node;
            }
        }
        m_lastNode = q;
        if (minDist < 2*NODE_STEP_SIZE)
            ROS_INFO_STREAM("END REACHED " << minDist);
    }

    // generate shortest path to destination.
    while (q != nullptr) {
        m_pathReverse.push_back(q);
        q = q->parent;
    }
}

const std::vector<cv::Vec2f> RRTStar::GetPath() const
{
    std::vector<cv::Vec2f> path;
    path.reserve(m_pathReverse.size());
    
    if (m_pathReverse.size() != 0)
    {
        for (size_t i = m_pathReverse.size(); i != 0; i--)
        {
            path.push_back(cv::Vec2f(m_pathReverse[i-1]->position[0], m_pathReverse[i-1]->position[1]));
        }
    
    }
    
    return path;
}

/**
 * @brief Initialize root node of RRTSTAR.
 */
void RRTStar::InitializeRootNode(const cv::Vec2f startPos)
{
    m_root = std::make_shared<Node>();
    m_root->parent = nullptr;
    m_root->position = startPos;
    m_root->orientation = 0.0;
    m_root->cost = 0.0;
    m_lastNode = m_root;
    m_nodes.push_back(m_root);
    }

/**
 * @brief Set world size for generating Nodes.
 * @param outsideCones
 * @param startConeIndex
 * @param endConeIndex
 */
void RRTStar::SetWorldSize(const std::vector<cv::Vec2f> &cones, const int startIndex, const int endIndex)
{
	m_xMin = std::numeric_limits<float>::max();
	m_xMax = std::numeric_limits<float>::lowest();
	m_yMin = std::numeric_limits<float>::max();
	m_yMax = std::numeric_limits<float>::lowest();
	m_world_width = 0;
	m_world_height = 0;

	for(size_t i = startIndex; i<endIndex; i++)
	{
		if(m_xMin > cones[i][0]) m_xMin = cones[i][0];
		if(m_xMax < cones[i][0]) m_xMax = cones[i][0];

		if(m_yMin > cones[i][1]) m_yMin = cones[i][1];
		if(m_yMax < cones[i][1]) m_yMax = cones[i][1];
	}

	m_world_width = m_xMax - m_xMin;
	m_world_height = m_yMax - m_yMin;
}

/**
 * @brief Generate a random node position in the field.
 * @param point
 * @return true if generated position is in map
 */
bool RRTStar::GetRandNodePos(cv::Vec2f *point) const
{
    (*point)[0] = (drand48() * m_world_width) + m_xMin;
    (*point)[1] = (drand48() * m_world_height) + m_yMin;
    
    return ((*point)[0] >= m_xMin && (*point)[0] <= m_xMax && (*point)[1] >= m_yMin && (*point)[1] <= m_yMax);
}

/**
 * @brief Helper method to find distance between two positions.
 * @param p
 * @param q
 * @return
 */
double RRTStar::ComputeDistance(const cv::Vec2f p, const cv::Vec2f q) const
{
    return sqrt(powf(p[0] - q[0], 2) + powf(p[1] - q[1], 2));
}

double RRTStar::ComputeOrientation(const cv::Vec2f pFrom, const cv::Vec2f pTo) const
{
    return std::atan2((pTo[1] - pFrom[1]), (pTo[0] - pFrom[0]));
}

/**
 * @brief Get nearest node from a given configuration/position.
 * @param point
 * @return
 */
RRTStar::NodeSPtr RRTStar::FindNearestNode(const cv::Vec2f point) const
{
    float minDist = std::numeric_limits<float>::max();
    NodeSPtr closest(nullptr);
    for (const auto &node : m_nodes) {
        double dist = ComputeDistance(point, node->position);
        
        if (dist < minDist) {
            minDist = dist;
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
void RRTStar::FindNearNodes(const cv::Vec2f point, std::vector<NodeSPtr> *out_nodes) const
{
    for (const auto &node : m_nodes) {
        double dist = ComputeDistance(point, node->position);
        double angle = computeAngleDiff(*node, point);
        if (dist < NODE_STEP_SIZE * RRTSTAR_NEIGHBOR_FACTOR) {
            (*out_nodes).push_back(node);
        }
    }
}

/**
 * @brief Find a configuration at a distance step_size from nearest node to random node.
 * @param qNearestPos
 * @param qPos
 * @return
 */
void RRTStar::NormalizePosition(const Node &qNearest, cv::Vec2f *qPos) const
{
    // angle restriction
    double angleDiff = computeAngleDiff(qNearest, *qPos);
    if (angleDiff > ANGLE_TH)
    {
        angleDiff = ANGLE_TH;
    } else if (angleDiff < -ANGLE_TH)
    {
        angleDiff = -ANGLE_TH;
    }
    const double angle = qNearest.orientation + angleDiff;

    *qPos = qNearest.position + cv::Vec2f(cos(angle), sin(angle)) * NODE_STEP_SIZE;
    }

/**
 * @brief Compute path cost.
 * @param qFrom
 * @param qTo
 * @return
 */
double RRTStar::PathCost(const Node &qFrom, const Node &qTo) const
{
    const double distance = ComputeDistance(qTo.position, qFrom.position);
    return distance;
}

double RRTStar::computeAngleDiff(const Node &qFrom, const cv::Vec2f pTo) const
{
    double angleDiff = ComputeOrientation(qFrom.position, pTo) - qFrom.orientation;
    if (std::abs(angleDiff) > M_PI)
    {
        angleDiff = std::pow(-1, static_cast<int>(angleDiff > 0)) * (2 * M_PI - std::abs(angleDiff));
    }
    return angleDiff;
}

/**
 * @brief Add a node to the tree.
 * @param qNearest
 * @param qNew
 */
void RRTStar::Add(const double costMin, const NodeSPtr &qNearest, const NodeSPtr &qNew)
{
    qNew->parent = qNearest;
    qNew->orientation = ComputeOrientation(qNearest->position, qNew->position);
    qNew->cost = costMin;
    qNearest->children.push_back(qNew);
    m_nodes.push_back(qNew);
    // m_lastNode = qNew;
}

/**
 * @brief Validate if point is on track.
 * @param targetPoint
 * @return
 */
bool RRTStar::isOnTrack(const cv::Vec2f targetPoint) const
{
	bool c = false;
	int i, j = 0;
	for (i = 0, j = m_outCones.size() - 1; i < m_outCones .size(); j = i++) 
	{
		if ( ((m_outCones [i][1] > targetPoint[1]) != (m_outCones [j][1] > targetPoint[1])) && 
		((targetPoint[0] < (m_outCones [j][0] - m_outCones [i][0]) * (targetPoint[1] - m_outCones [i][1]) / (m_outCones [j][1] - m_outCones [i][1]) + m_outCones [i][0])) )
			c = !c;
		
	}
	if(c)
	{
		for (const auto &cone : m_outCones ) 
		{
        	double dist = ComputeDistance(cone, targetPoint);
        	if (dist < CAR_WIDTH / 2) 
			{
            	c = false;
				break;
        	}
    	}
	}

	if(c)
	{
		for (i = 0, j = m_inCones.size() - 1; i < m_inCones.size(); j = i++) 
		{
			if ( ((m_inCones [i][1] > targetPoint[1]) != (m_inCones [j][1] > targetPoint[1])) && 
			((targetPoint[0] < (m_inCones [j][0] - m_inCones [i][0]) * (targetPoint[1] - m_inCones [i][1]) / (m_inCones [j][1] - m_inCones [i][1]) + m_inCones [i][0])) )
				c = !c;
		}

		if(c)
		{
			for (const auto &cone : m_inCones ) 
			{
        		double dist = ComputeDistance(cone, targetPoint);
        		if (dist < CAR_WIDTH / 2) 
				{
            		c = false;
					break;
        		}
    		}
		}
	}

	return c;
}
