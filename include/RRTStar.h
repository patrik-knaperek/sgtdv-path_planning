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
#include "opencv2/core/core.hpp"

// ROS
#include <ros/ros.h>

class RRTStar
{
public:
	struct Node {
	std::vector<boost::shared_ptr<Node>> children;
	boost::shared_ptr<Node> parent;
	cv::Vec2f position;
	float orientation;
	double cost;
	};
	typedef boost::shared_ptr<Node> NodeSPtr;

	static constexpr float END_DIST_THRESHOLD	   = 0.25;
	static constexpr float RRTSTAR_NEIGHBOR_FACTOR = 5;
	static constexpr float CAR_WIDTH		       = 2.0;
	static constexpr float NODE_STEP_SIZE		   = 0.3;
	static constexpr int   MAX_ITER 			   = 200;

public:
	RRTStar();
	~RRTStar();

	void Do();
	void Init(std::vector<cv::Vec2f> outsideCones, std::vector<cv::Vec2f> insideCones,int startIndex, int endIndex, cv::Vec2f startPosition, cv::Vec2f endPosition);
	const std::vector<NodeSPtr> GetNodes() const;
	const std::vector<cv::Vec2f> GetPath() const;

private:
	void Initialize(cv::Vec2f startPos);
	void SetWorldSize(std::vector<cv::Vec2f> cones, int startIndex, int endIndex);
	bool GetRandNodePos(cv::Vec2f &point) const;
	NodeSPtr FindNearestNode(const cv::Vec2f &point) const;
	void FindNearNodes(const cv::Vec2f &point, const float radius, std::vector<NodeSPtr>& out_nodes) const;
	double PathCost(const Node &qFrom, const Node &qTo) const;
	void NormalizePosition(const cv::Vec2f &qNearestPos, cv::Vec2f *qPos) const;
	void Add(NodeSPtr qNearest, NodeSPtr qNew);
	bool PnPoly(const std::vector<cv::Vec2f> &insideCones, const std::vector<cv::Vec2f> &outsideCones, 
                const cv::Vec2f &targetPoint) const;
	double Distance(const cv::Vec2f &p, const cv::Vec2f &q) const;

	std::vector<NodeSPtr> m_nodes, m_pathReverse;
	
	std::vector<cv::Vec2f> m_inCones, m_outCones;

	NodeSPtr m_root, m_lastNode;
	cv::Vec2f m_endPos;
	float m_xMin, m_xMax, m_yMin, m_yMax;
	float m_world_width;
	float m_world_height;
};

