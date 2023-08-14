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
#include <sgtdv_msgs/Float32Srv.h>
#include "../../SGT_Utils.h"
#include "../../SGT_Macros.h"

constexpr float BEZIER_RESOLUTION = 0.125;

class PathPlanning
{
public:
    PathPlanning(const ros::NodeHandle& handle);
    ~PathPlanning() = default;

    void SetPublisher(const ros::Publisher &trajectoryPub
                #ifdef SGT_VISUALIZATION
                    , const ros::Publisher &pathPlanningVisPub
                    , const ros::Publisher &interpolatedConesPub
                #endif /* SGT_VISUALIZATION */
                    );
    void SetServiceClient(const ros::ServiceClient &setSpeedClient)
    {
        m_setSpeedClient = setSpeedClient;
    };
    void Do(const PathPlanningMsg &msg);  
    void YellowOnLeft(bool value);
    void FullMap() { m_fullMap = true; };
    //void SetDiscipline(Discipline discipline);

private:
    bool RRTRun();
    void SortCones(const PathPlanningMsg &msg);
	std::vector<Eigen::Vector2f> LinearInterpolation(std::vector<Eigen::Vector2f> points) const;
    sgtdv_msgs::Point2DArr FindMiddlePoints();

#ifdef SGT_VISUALIZATION
    void VisualizeInterpolatedCones();
    void VisualizeRRTPoints();
#endif /* SGT_VISUALIZATION */
    
    RRTStar m_rrtStar;
    Params m_params;
	
	float m_timeravg;
	int m_timeravgcount;

    ros::Publisher m_trajectoryPub;
    ros::ServiceClient m_setSpeedClient;
    sgtdv_msgs::Float32Srv m_setSpeedMsg;
    bool m_isYellowOnLeft;
	bool m_once;
    bool m_fullMap;

    std::vector<Eigen::Vector2f> m_leftCones, m_leftConesInterpolated,  m_rightCones, m_rightConesInterpolated, m_middleLinePoints;
    //PathPlanningDiscipline *m_pathPlanningDiscipline = nullptr;

#ifdef SGT_VISUALIZATION
    ros::Publisher m_interpolatedConesPub, m_pathPlanningVisPub;
#endif /* SGT_VISUALIZATION */
};