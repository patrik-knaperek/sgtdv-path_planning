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
#include <vector>
#include <map>
// #include "opencv2/core/core.hpp"
#include <Eigen/Eigen>
#include "../include/Messages.h"

#define deg2rad(x) (x*M_PI/180.f)

constexpr size_t MAX_PREDICT_POINTS = 3;

enum Discipline
{
    UNKNOWN_TRACK = 0,
    SKIDPAD
};

class PathPlanningDiscipline
{
public:
    virtual sgtdv_msgs::Point2DArrPtr Do(const PathPlanningMsg &msg) = 0;  
    void YellowOnLeft(bool value);

protected:
    PathPlanningDiscipline();
    ~PathPlanningDiscipline();

    bool m_isYellowOnLeft;
};


class UnknownTrack : public PathPlanningDiscipline
{
public:
    UnknownTrack();
    ~UnknownTrack();

    virtual sgtdv_msgs::Point2DArrPtr Do(const PathPlanningMsg &msg);
    
private:
   std::vector<Eigen::Vector2f> m_leftCones;
    std::vector<Eigen::Vector2f> m_rightCones;
    std::map<float, size_t> m_leftDistances;
    std::map<float, size_t> m_rightDistances;

    void Clear();
    void SortCones(const PathPlanningMsg &msg);
    void FindMiddlePoints(std::vector<sgtdv_msgs::Point2D> &points);
    bool IsLessOnLeft() const;
    float Norm(const Eigen::Vector2f &point) const;
    Eigen::Vector2f Rotate90Clockwise(const Eigen::Vector2f &point) const;
};


//////////////////////////////
//////////////////////////////
///////// SKIDPAD ////////////
//////////////////////////////


class Skidpad : public PathPlanningDiscipline
{
public:
    Skidpad();
    ~Skidpad();

    virtual sgtdv_msgs::Point2DArrPtr Do(const PathPlanningMsg &msg);
};