/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Samuel Mazur, Patrik Knaperek
/*****************************************************/

#pragma once

// C++
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <vector>
#include <map>
#include <chrono>

#include "opencv2/core/core.hpp"

// ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// SGT
#include "Messages.h"
//#include "PathPlanningDisciplines.h"
#include "../include/RRTStar.h"
#include <sgtdv_msgs/PathPlanningMsg.h>
#include <sgtdv_msgs/Cone.h>
#include <sgtdv_msgs/Point2D.h>
#include <sgtdv_msgs/Point2DArr.h>

constexpr float BEZIER_RESOLUTION = 0.125;
constexpr bool FULL_MAP = true;

class PathPlanning
{
public:
    PathPlanning();
    ~PathPlanning() = default;

    void SetPublisher(const ros::Publisher &trajectoryPub
                    , const ros::Publisher &trajectoryVisPub
                    , const ros::Publisher &interpolatedConesPub
                    // , const ros::Publisher &treeVisPub
                    );
    void Do(const PathPlanningMsg &msg);  
    void YellowOnLeft(bool value);
    //void SetDiscipline(Discipline discipline);

private:
    void RRTRun();
    void SortCones(const PathPlanningMsg &msg);
	std::vector<cv::Vec2f> LinearInterpolation(std::vector<cv::Vec2f> points) const;
    visualization_msgs::MarkerArray FindMiddlePoints();
    visualization_msgs::MarkerArray VisualizeInterpolatedCones() const;
    visualization_msgs::MarkerArray VisualizeRRTPoints() const;

    RRTStar m_rrtStar;
	RRTStar m_rrtStar2;
	RRTStar m_rrtStar3;

	float m_timeravg;
	int m_timeravgcount;

    ros::Publisher m_trajectoryPub, m_trajectoryVisPub, m_interpolatedConesPub/*, m_treeVisPub*/;
    bool m_isYellowOnLeft;
	bool m_once;

    std::vector<cv::Vec2f> m_leftCones, m_leftConesInterpolated,  m_rightCones, m_rightConesInterpolated, m_middleLinePoints;
    //PathPlanningDiscipline *m_pathPlanningDiscipline = nullptr;
};