/*****************************************************/
//Organization: Stuba Green Team
//Authors: Samuel Mazur
/*****************************************************/

#include "../include/RRTStar.h"

RRTStar::RRTStar()
{
  
}

RRTStar::~RRTStar()
{
	deleteNodes(root);
}

/**
 * @brief Initialization of whole object.
 * @param outsideCones
 * @param insideCones
 * @param startConeIndex
 * @param endConeIndex
 * @param startPosition
 * @param endPosition
 * @param ParentNode (experimental - use in case linking previous tree end point to root node of this)
 */
void RRTStar::init(std::vector<cv::Vec2f> outsideCones, std::vector<cv::Vec2f> insideCones, int startIndex, int endIndex, cv::Vec2f startPosition, cv::Vec2f endPosition, Node *Parent)
{
	outCones = outsideCones;
	inCones = insideCones;
	
	startPos = startPosition;
	endPos = endPosition;	
	
	setStepSize(NODE_STEP_SIZE);
	setMaxIterations(MAX_ITER);
	getWorldSize(outsideCones, startIndex, endIndex);
	
	initialize(startPos, Parent);
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
	path.clear();
	for(int i = 0; i < max_iter; i++) {
        Node *q = getRandomNode();
        if (q) {
            Node *qNearest = nearest(q->position);
            if (distance(q->position, qNearest->position) > step_size) {
                cv::Vec2f newConfigPos;       
                newConfigPos = newConfig(q, qNearest, step_size);

                if (PnPoly(inCones, outCones, newConfigPos, CAR_WIDTH)) 
		{
                    Node *qNew = new Node;
                    qNew->position = newConfigPos;
                    qNew->orientation = 0;

                    std::vector<Node *> Qnear;
                    near(qNew->position, step_size*RRTSTAR_NEIGHBOR_FACTOR, Qnear);
                    Node *qMin = qNearest;
                    double cmin = Cost(qNearest) + PathCost(qNearest, qNew);
                    for(int j = 0; j < Qnear.size(); j++)
					{
                        Node *qNear = Qnear[j];
                        if((PnPoly(inCones, outCones, qNew->position, CAR_WIDTH)) && (Cost(qNear)+PathCost(qNear, qNew)) < cmin )
						{
                            qMin = qNear; cmin = Cost(qNear)+PathCost(qNear, qNew);
                        }
                    }
                    add(qMin, qNew);

                    for(int j = 0; j < Qnear.size(); j++){
                        Node *qNear = Qnear[j];
                        if(PnPoly(inCones, outCones, qNew->position, CAR_WIDTH) &&
                                (Cost(qNew)+PathCost(qNew, qNear)) < Cost(qNear) ){
                            Node *qParent = qNear->parent;
                            qParent->children.erase(std::remove(qParent->children.begin(), qParent->children.end(), qNear), qParent->children.end());
                            qNear->cost = Cost(qNew) + PathCost(qNew, qNear);
                            qNear->parent = qNew;
                            qNew->children.push_back(qNear);
                        }
                    }

                }
		else i--; //to reset iteration counter in case new point is not on track
            }
        }
    }
    Node *q;
    if (reached(endPos)) {
        q = lastNode;
		//std::cout<<"Reached Destination!\n";
    }
    else
    {
        // if not reached yet
        q = nearest(endPos);
		//std::cout<<"Last point: "<< q->position<<"\n";
    }
    // generate shortest path to destination.
    while (q != NULL) {
        path.push_back(q);
        q = q->parent;
    }

}

/**
 * @brief Initialize root node of RRTSTAR.
 */
void RRTStar::initialize(cv::Vec2f startPos, Node *Parent)
{
    root = new Node;
    root->parent = Parent;
    root->position = startPos;
    root->orientation = 0;
    root->cost = 0.0;
    lastNode = root;
    nodes.push_back(root);
}

/**
 * @brief Set world size for generating Nodes.
 * @param outsideCones
 * @param startConeIndex
 * @param endConeIndex
 */
void RRTStar::getWorldSize(std::vector<cv::Vec2f> cones, int startIndex, int endIndex)
{
	xMin = std::numeric_limits<float>::max();
	xMax = -std::numeric_limits<float>::max();
	yMin = std::numeric_limits<float>::max();
	yMax = -std::numeric_limits<float>::max();
	world_width = 0;
	world_height = 0;

	for(size_t i = startIndex; i<endIndex; i++)
	{
		if(xMin > cones[i][0]) xMin = cones[i][0];
		if(xMax < cones[i][0]) xMax = cones[i][0];

		if(yMin > cones[i][1]) yMin = cones[i][1];
		if(yMax < cones[i][1]) yMax = cones[i][1];
	}

	world_width = xMax - xMin;
	world_height = yMax - yMin;
}

/**
 * @brief Generate a random node in the field.
 * @return
 */
Node* RRTStar::getRandomNode()
{
    Node* ret;
    cv::Vec2f point((drand48() * world_width) + xMin, (drand48() * world_height) + yMin);
    float orient = drand48() * 2 * 3.142;
    if (point[0] >= xMin && point[0] <= xMax && point[1] >= yMin && point[1] <= yMax && orient > 0 && orient < 2*3.142) 
	{
        ret = new Node;
        ret->position = point;
        ret->orientation = orient;
        return ret;
    }
    return NULL;
}

/**
 * @brief Helper method to find distance between two positions.
 * @param p
 * @param q
 * @return
 */
double RRTStar::distance(cv::Vec2f &p, cv::Vec2f &q)
{
    cv::Vec2f v = p - q;
    return sqrt(powf(v[0], 2) + powf(v[1], 2));
}

/**
 * @brief Get nearest node from a given configuration/position.
 * @param point
 * @return
 */
Node* RRTStar::nearest(cv::Vec2f point)
{
    float minDist = std::numeric_limits<float>::max();
    Node *closest = NULL;
    for(int i = 0; i < (int)nodes.size(); i++) {
        double dist = distance(point, nodes[i]->position);
        if (dist < minDist) {
            minDist = dist;
            closest = nodes[i];
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
void RRTStar::near(cv::Vec2f point, float radius, std::vector<Node *>& out_nodes)
{
    for(int i = 0; i < (int)nodes.size(); i++) {
        double dist = distance(point, nodes[i]->position);
        if (dist < radius) {
            out_nodes.push_back(nodes[i]);
        }
    }
}

/**
 * @brief Find a configuration at a distance step_size from nearest node to random node.
 * @param q
 * @param qNearest
 * @return
 */
cv::Vec2f RRTStar::newConfig(Node *q, Node *qNearest, float step_size)
{
    cv::Vec2f to = q->position;
    cv::Vec2f from = qNearest->position;
    cv::Vec2f intermediate = to - from;
	intermediate = intermediate / norm(intermediate);
    cv::Vec2f pos = from + step_size * intermediate;
    cv::Vec2f ret(pos[0], pos[1]);
    return ret;
}

/**
 * @brief Return trajectory cost.
 * @param q
 * @return
 */
double RRTStar::Cost(Node *q)
{
    return q->cost;
}

/**
 * @brief Compute path cost.
 * @param qFrom
 * @param qTo
 * @return
 */
double RRTStar::PathCost(Node *qFrom, Node *qTo)
{
    return distance(qTo->position, qFrom->position);
}

/**
 * @brief Add a node to the tree.
 * @param qNearest
 * @param qNew
 */
void RRTStar::add(Node *qNearest, Node *qNew)
{
    qNew->parent = qNearest;
    qNew->cost = qNearest->cost + PathCost(qNearest, qNew);
    qNearest->children.push_back(qNew);
    nodes.push_back(qNew);
    lastNode = qNew;
}

/**
 * @brief Check if the last node is close to the end position.
 * @return
 */
bool RRTStar::reached(cv::Vec2f endPos)
{
    if (distance(lastNode->position, endPos) < END_DIST_THRESHOLD)
	{
		//std::cout << "Final reached\n";
        return true;
	}
    return false;
}

void RRTStar::setStepSize(float step)
{
    step_size = step;
}

void RRTStar::setMaxIterations(int iter)
{
    max_iter = iter;
}

/**
 * @brief Delete all nodes using DFS technique.
 * @param root
 */
void RRTStar::deleteNodes(Node *root)
{
    for(int i = 0; i < (int)root->children.size(); i++) {
        deleteNodes(root->children[i]);
    }
    delete root;
}

/**
 * @brief Validate if point is on track.
 * @param insideCones
 * @param outsideCones
 * @param targetPoint
 * @param carWidth
 * @return
 */
bool RRTStar::PnPoly(std::vector<cv::Vec2f> insideCones, std::vector<cv::Vec2f> outsideCones, cv::Vec2f targetPoint, float car_width)
{
	bool c = false;
	int i, j = 0;
	for (i = 0, j = outsideCones.size()-1; i < outsideCones.size(); j = i++) 
	{
		if ( ((outsideCones[i][1]>targetPoint[1]) != (outsideCones[j][1]>targetPoint[1])) && 
		((targetPoint[0] < (outsideCones[j][0]-outsideCones[i][0]) * (targetPoint[1]-outsideCones[i][1]) / (outsideCones[j][1]-outsideCones[i][1]) + outsideCones[i][0])) )
			c = !c;
		
	}
	if(c)
	{
		for(size_t i = 0; i < outsideCones.size(); i++) 
		{
        	double dist = distance(outsideCones[i], targetPoint);
        	if (dist < car_width/2) 
			{
            	c = false;
				break;
        	}
    	}
	}

	if(c)
	{
		for (i = 0, j = insideCones.size()-1; i < insideCones.size(); j = i++) 
		{
			if ( ((insideCones[i][1]>targetPoint[1]) != (insideCones[j][1]>targetPoint[1])) && 
			((targetPoint[0] < (insideCones[j][0]-insideCones[i][0]) * (targetPoint[1]-insideCones[i][1]) / (insideCones[j][1]-insideCones[i][1]) + insideCones[i][0])) )
				c = !c;
		}

		if(c)
		{
			for(size_t i = 0; i < insideCones.size(); i++) 
			{
        		double dist = distance(insideCones[i], targetPoint);
        		if (dist < car_width/2) 
				{
            		c = false;
					break;
        		}
    		}
		}
	}

	return c;
}
