/*****************************************************/
//Organization: Stuba Green Team
//Authors: Samuel Mazur
/*****************************************************/

// C++
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <limits>
#include <time.h>
#include <chrono>
#include "opencv2/core/core.hpp"

// ROS
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

constexpr float END_DIST_THRESHOLD = 0.25;
constexpr float RRTSTAR_NEIGHBOR_FACTOR = 5;
constexpr float CAR_WIDTH = 1.5;
constexpr float NODE_STEP_SIZE = 0.3;
constexpr int MAX_ITER = 200;

struct Node {
	std::vector<Node *> children;
	Node *parent;
	cv::Vec2f position;
	float orientation;
	double cost;
};

class RRTStar
{
public:
	RRTStar();
	~RRTStar();

	void Do();
	void init(std::vector<cv::Vec2f> outsideCones, std::vector<cv::Vec2f> insideCones,int startIndex, int endIndex, cv::Vec2f startPosition, cv::Vec2f endPosition, Node *Parent);

	void initialize(cv::Vec2f startPos, Node *Parent);
	void getWorldSize(std::vector<cv::Vec2f> cones, int startIndex, int endIndex);
	Node* getRandomNode();
	Node* nearest(cv::Vec2f point);
	void near(cv::Vec2f point, float radius, std::vector<Node *>& out_nodes);
	double distance(cv::Vec2f &p, cv::Vec2f &q);
	double Cost(Node *q);
	double PathCost(Node *qFrom, Node *qTo);
	cv::Vec2f newConfig(Node *q, Node *qNearest, float step_size);
	void add(Node *qNearest, Node *qNew);
	bool reached(cv::Vec2f endPos);
	void setStepSize(float step);
	void setMaxIterations(int iter);
	void deleteNodes(Node *root);
	bool PnPoly(std::vector<cv::Vec2f> insideCones, std::vector<cv::Vec2f> outsideCones, cv::Vec2f targetPoint, float car_width);

	std::vector<Node *> nodes;
	std::vector<Node *> path;

	std::vector<cv::Vec2f> inCones, outCones;

	Node *root, *lastNode, *newRandNode, *newNode;
	cv::Vec2f startPos, endPos;
	int max_iter, iter;
	float xMin, xMax, yMin, yMax;
	float step_size;
	float world_width;
	float world_height;
};

