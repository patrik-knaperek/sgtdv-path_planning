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

// #include "opencv2/core/core.hpp"

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

class PathPlanning
{
public:
    PathPlanning();
    ~PathPlanning() = default;

    void SetPublisher(const ros::Publisher &trajectoryPub
                    , const ros::Publisher &trajectoryVisPub
                    , const ros::Publisher &interpolatedConesPub
                    );
    void Do(const PathPlanningMsg &msg);  
    void YellowOnLeft(bool value);
    void FullMap() { m_fullMap = true; };
    //void SetDiscipline(Discipline discipline);

private:
    bool RRTRun();
    void SortCones(const PathPlanningMsg &msg);
	std::vector<Eigen::Vector2f> LinearInterpolation(std::vector<Eigen::Vector2f> points) const;
    sgtdv_msgs::Point2DArr FindMiddlePoints();
    visualization_msgs::MarkerArray VisualizeInterpolatedCones() const;
    void VisualizeRRTPoints();
    void VisualizeTrajectory(const sgtdv_msgs::Point2DArr &trajectory);

    RRTStar m_rrtStar;
	
	float m_timeravg;
	int m_timeravgcount;

    ros::Publisher m_trajectoryPub, m_trajectoryVisPub, m_interpolatedConesPub;
    bool m_isYellowOnLeft;
	bool m_once;
    bool m_fullMap;

    std::vector<Eigen::Vector2f> m_leftCones, m_leftConesInterpolated,  m_rightCones, m_rightConesInterpolated, m_middleLinePoints;
    //PathPlanningDiscipline *m_pathPlanningDiscipline = nullptr;

    visualization_msgs::MarkerArray m_trajectoryVisMarkers;
};