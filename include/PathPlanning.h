/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#pragma once

#include <ros/ros.h>
#include <sgtdv_msgs/PathPlanningMsg.h>
#include <sgtdv_msgs/PathTrackingMsg.h>
#include <sgtdv_msgs/Cone.h>
#include <sgtdv_msgs/Point2D.h>
#include "../include/Messages.h"
#include <vector>
#include <map>
#include "opencv2/core/core.hpp"

#define deg2rad(x) (x*M_PI/180.f)

constexpr size_t MAX_PREDICT_POINTS = 3;

class PathPlanning
{
public:
    PathPlanning();
    ~PathPlanning();

    void Do(const PathPlanningMsg &msg);
    void SetPublisher(ros::Publisher publisher);
    void YellowOnLeft(bool value);

private:
    ros::Publisher m_publisher;
    std::vector<cv::Vec2f> m_leftCones;
    std::vector<cv::Vec2f> m_rightCones;
    std::map<float, size_t> m_leftDistances;
    std::map<float, size_t> m_rightDistances;
    bool m_isYellowOnLeft;

    void Clear();
    void SortCones(const PathPlanningMsg &msg);
    void FindMiddlePoints(std::vector<sgtdv_msgs::Point2D> &points);
    bool IsLessOnLeft() const;
    float Norm(const cv::Vec2f &point) const;
    cv::Vec2f Rotate90Clockwise(const cv::Vec2f &point) const;
};