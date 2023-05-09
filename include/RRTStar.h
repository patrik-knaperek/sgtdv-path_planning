/*****************************************************/
//Organization: Stuba Green Team
//Authors: Samuel Mazur, Patrik Knaperek
/*****************************************************/

// C++
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <limits>
#include <time.h>
#include <chrono>
//#include "opencv2/core/core.hpp"
#include <Eigen/Eigen>

// ROS
#include <ros/ros.h>

// SGT DV
#include <sgtdv_msgs/Point2DArr.h>

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

	static constexpr float NODE_STEP_SIZE		   = 0.3;
	static constexpr float END_DIST_THRESHOLD	   = 0.25;
	static constexpr float RRTSTAR_NEIGHBOR_RADIUS = 5 * NODE_STEP_SIZE;
	static constexpr float CAR_WIDTH		       = 2.0;
	static constexpr int   MAX_ITER 			   = 500;
	static constexpr double ANGLE_TH			   = 0.1 * M_PI;

public:
	RRTStar();
	~RRTStar();

	bool Do();
	void Init(const std::vector<Eigen::Vector2f> &outsideCones, const std::vector<Eigen::Vector2f> &insideCones, 
			const int startIndex, const int endIndex, const Eigen::Vector2f startPosition, const Eigen::Vector2f endPosition);
	const std::vector<NodeSPtr> GetNodes() const { return m_nodes; };
	const sgtdv_msgs::Point2DArr GetPath() const;

private:
	void InitializeRootNode(const Eigen::Vector2f startPos);
	void SetWorldSize(const std::vector<Eigen::Vector2f> &cones, const int startIndex, const int endIndex);
	bool GetRandNodePos(Eigen::Vector2f *point) const;
	NodeSPtr FindNearestNode(const Eigen::Vector2f point) const;
	void FindNearNodes(const Eigen::Vector2f point, std::vector<NodeSPtr> *out_nodes) const;
	double PathCost(const Node &qFrom, const Node &qTo) const;
	void NormalizePosition(const Node &qNearest, Eigen::Vector2f *qPos) const;
	void Add(const double costMin, const NodeSPtr &qNearest, const NodeSPtr &qNew);
	bool isOnTrack(const Eigen::Vector2f targetPoint) const;
	double ComputeDistance(const Eigen::Vector2f p, const Eigen::Vector2f q) const;
	double ComputeOrientation(const Eigen::Vector2f pFrom, const Eigen::Vector2f pTo) const;
	double computeAngleDiff(const Node &qFrom, const Eigen::Vector2f pTo) const;

	std::vector<NodeSPtr> m_nodes, m_pathReverse;
	
	std::vector<Eigen::Vector2f> m_inCones, m_outCones;

	NodeSPtr m_root, m_lastNode;
	Eigen::Vector2f m_endPos;
	float m_xMin, m_xMax, m_yMin, m_yMax;
	float m_world_width;
	float m_world_height;
};

