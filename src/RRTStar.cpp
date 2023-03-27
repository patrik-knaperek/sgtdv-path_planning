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
 * @param ParentNode (experimental - use in case linking previous tree end point to root node of this)
 */
void RRTStar::Init(std::vector<cv::Vec2f> outsideCones, std::vector<cv::Vec2f> insideCones, int startIndex, int endIndex, cv::Vec2f startPosition, cv::Vec2f endPosition)
{
	m_outCones = outsideCones;
    m_inCones = insideCones;
	m_endPos = endPosition;	
	
    SetWorldSize(outsideCones, startIndex, endIndex);  
	Initialize(startPosition);
	srand48(time(0));	
}

/**
 * @brief Validate if point is on track.
 * @param insideCones
 * @param outsideCones
 * @param targetPoint
 * @param carWidth
 */
void RRTStar::Do()
{
	m_pathReverse.clear();
    for(int i = 0; i < MAX_ITER; i++)
    {
        cv::Vec2f newPos;
        if (GetRandNodePos(newPos))
        {
            NodeSPtr qNearestPtr = FindNearestNode(newPos);
            if (Distance(newPos, qNearestPtr->position) > NODE_STEP_SIZE)
            {
                NormalizePosition(qNearestPtr->position, &newPos);
                
                if (PnPoly(m_inCones, m_outCones, newPos))
                {
                    auto qNew = boost::make_shared<Node>();;
                    qNew->position = newPos;
                    qNew->orientation = 0;
                    
                    std::vector<NodeSPtr> Qnear;
                    FindNearNodes(qNew->position, NODE_STEP_SIZE * RRTSTAR_NEIGHBOR_FACTOR, Qnear);
                    NodeSPtr qMin = qNearestPtr;
                    double cmin = qNearestPtr->cost + PathCost(*qNearestPtr, *qNew);
                    for (const auto qNear : Qnear)
					{
                        if ((PnPoly(m_inCones, m_outCones, qNew->position)) && (qNear->cost + PathCost(*qNear, *qNew)) < cmin)
						{
                            qMin = qNear;
                            cmin = qNear->cost + PathCost(*qNear, *qNew);
                        }
                    }
                    Add(qMin, qNew);

                    for (auto qNear : Qnear){
                        if(PnPoly(m_inCones, m_outCones, qNew->position) &&
                                (qNew->cost + PathCost(*qNew, *qNear)) < qNear->cost){
                            NodeSPtr qParent = qNear->parent;
                            qParent->children.erase(std::remove(qParent->children.begin(), qParent->children.end(), qNear), qParent->children.end());
                            qNear->cost = qNew->cost + PathCost(*qNew, *qNear);
                            qNear->parent = qNew;
                            qNew->children.push_back(qNear);
                        }
                    }

                }
		        else i--; //to reset iteration counter in case new point is not on track
            }
        }
    }
    NodeSPtr q(nullptr);
    if (Distance(m_lastNode->position, m_endPos) < END_DIST_THRESHOLD)
    {
        q = m_lastNode;
    }
    else
    {
        // if not reached yet
        q = FindNearestNode(m_endPos);
    }
    // generate shortest path to destination.
    while (q != nullptr) {
        m_pathReverse.push_back(q);
        q = q->parent;
    }
}

const std::vector<RRTStar::NodeSPtr> RRTStar::GetNodes() const
{
    return m_nodes;
}

const std::vector<cv::Vec2f> RRTStar::GetPath() const
{
    std::vector<cv::Vec2f> path;
    path.reserve(m_pathReverse.size());
    for (size_t i = m_pathReverse.size() - 1; i != 0; i--)
    {
        path.push_back(cv::Vec2f(m_pathReverse[i]->position[0], m_pathReverse[i]->position[1]));
    }
    
    return path;
}

/**
 * @brief Initialize root node of RRTSTAR.
 */
void RRTStar::Initialize(cv::Vec2f startPos)
{
    m_root = boost::make_shared<Node>();
    m_root->parent = nullptr;
    m_root->position = startPos;
    m_root->orientation = 0;
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
void RRTStar::SetWorldSize(std::vector<cv::Vec2f> cones, int startIndex, int endIndex)
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
bool RRTStar::GetRandNodePos(cv::Vec2f &point) const
{
    point[0] = (drand48() * m_world_width) + m_xMin;
    point[1] = (drand48() * m_world_height) + m_yMin;
    
    return (point[0] >= m_xMin && point[0] <= m_xMax && point[1] >= m_yMin && point[1] <= m_yMax);
}

/**
 * @brief Helper method to find distance between two positions.
 * @param p
 * @param q
 * @return
 */
double RRTStar::Distance(const cv::Vec2f &p, const cv::Vec2f &q) const
{
    return sqrt(powf(p[0] - q[0], 2) + powf(p[1] - q[1], 2));
}

/**
 * @brief Get nearest node from a given configuration/position.
 * @param point
 * @return
 */
RRTStar::NodeSPtr RRTStar::FindNearestNode(const cv::Vec2f &point) const
{
    float minDist = std::numeric_limits<float>::max();
    NodeSPtr closest(nullptr);
    for (const auto &node : m_nodes) {
        double dist = Distance(point, node->position);
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
void RRTStar::FindNearNodes(const cv::Vec2f &point, const float radius, std::vector<NodeSPtr>& out_nodes) const
{
    for (const auto &node : m_nodes) {
        double dist = Distance(point, node->position);
        if (dist < radius) {
            out_nodes.push_back(node);
        }
    }
}

/**
 * @brief Find a configuration at a distance step_size from nearest node to random node.
 * @param qNearestPos
 * @param qPos
 * @return
 */
void RRTStar::NormalizePosition(const cv::Vec2f &qNearestPos, cv::Vec2f *qPos) const
{
    cv::Vec2f to = *qPos;
    cv::Vec2f from = qNearestPos;
    cv::Vec2f intermediate = to - from;
	intermediate = intermediate / cv::norm(intermediate);
    *qPos = from + NODE_STEP_SIZE * intermediate;
}

/**
 * @brief Compute path cost.
 * @param qFrom
 * @param qTo
 * @return
 */
double RRTStar::PathCost(const Node &qFrom, const Node &qTo) const
{
    return Distance(qTo.position, qFrom.position);
}

/**
 * @brief Add a node to the tree.
 * @param qNearest
 * @param qNew
 */
void RRTStar::Add(NodeSPtr qNearest, NodeSPtr qNew)
{
    qNew->parent = qNearest;
    qNew->cost = qNearest->cost + PathCost(*qNearest, *qNew);
    qNearest->children.push_back(qNew);
    m_nodes.push_back(qNew);
    m_lastNode = qNew;
}

/**
 * @brief Validate if point is on track.
 * @param insideCones
 * @param outsideCones
 * @param targetPoint
 * @param carWidth
 * @return
 */
bool RRTStar::PnPoly(const std::vector<cv::Vec2f> &insideCones, const std::vector<cv::Vec2f> &outsideCones, 
                    const cv::Vec2f &targetPoint) const
{
	bool c = false;
	int i, j = 0;
	for (i = 0, j = outsideCones.size() - 1; i < outsideCones.size(); j = i++) 
	{
		if ( ((outsideCones[i][1] > targetPoint[1]) != (outsideCones[j][1] > targetPoint[1])) && 
		((targetPoint[0] < (outsideCones[j][0] - outsideCones[i][0]) * (targetPoint[1] - outsideCones[i][1]) / (outsideCones[j][1] - outsideCones[i][1]) + outsideCones[i][0])) )
			c = !c;
		
	}
	if(c)
	{
		for (const auto &cone : outsideCones) 
		{
        	double dist = Distance(cone, targetPoint);
        	if (dist < CAR_WIDTH / 2) 
			{
            	c = false;
				break;
        	}
    	}
	}

	if(c)
	{
		for (i = 0, j = insideCones.size() - 1; i < insideCones.size(); j = i++) 
		{
			if ( ((insideCones[i][1] > targetPoint[1]) != (insideCones[j][1] > targetPoint[1])) && 
			((targetPoint[0] < (insideCones[j][0] - insideCones[i][0]) * (targetPoint[1] - insideCones[i][1]) / (insideCones[j][1] - insideCones[i][1]) + insideCones[i][0])) )
				c = !c;
		}

		if(c)
		{
			for (const auto &cone : insideCones) 
			{
        		double dist = Distance(cone, targetPoint);
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
