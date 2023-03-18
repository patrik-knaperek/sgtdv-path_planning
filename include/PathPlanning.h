/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Samuel Mazur
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

constexpr float BEZIER_RESOLUTION = 0.125;
constexpr bool FULL_MAP = false;

class PathPlanning
{
public:
    PathPlanning();
    ~PathPlanning() = default;

    void SetPublisher(const ros::Publisher &trajectoryPub, const ros::Publisher &interpolatedConesPub);
    void Do(const PathPlanningMsg &msg);  
    void RRTRun();
    void YellowOnLeft(bool value);
    //void SetDiscipline(Discipline discipline);

private:
    RRTStar m_rrtStar1;
	RRTStar m_rrtStar2;
	RRTStar m_rrtStar3;

	float timeravg;
	int timeravgcount;

    ros::Publisher m_trajectoryPub, m_interpolatedConesPub;
    bool m_isYellowOnLeft;
	bool m_once;

    std::vector<cv::Vec2f> m_leftCones;
    std::vector<cv::Vec2f> m_leftConesInterpolated;
    std::vector<cv::Vec2f> m_rightCones;
    std::vector<cv::Vec2f> m_rightConesInterpolated;
    std::vector<cv::Vec2f> m_middleLinePoints;

    void SortCones(const PathPlanningMsg &msg);
	std::vector<cv::Vec2f> LinearInterpolation(std::vector<cv::Vec2f> points) const;
    visualization_msgs::MarkerArray FindMiddlePoints();
    visualization_msgs::MarkerArray InterpolatedCones() const;
    visualization_msgs::MarkerArray RRTPoints() const;

    //PathPlanningDiscipline *m_pathPlanningDiscipline = nullptr;
};