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
#include "../include/messages.h"

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
    virtual sgtdv_msgs::Point2DArrPtr update(const PathPlanningMsg &msg) = 0;  
    void yellowOnLeft(bool value);

protected:
    PathPlanningDiscipline();
    ~PathPlanningDiscipline();

    bool is_yellow_on_left_;
};


class UnknownTrack : public PathPlanningDiscipline
{
public:
    UnknownTrack();
    ~UnknownTrack();

    virtual sgtdv_msgs::Point2DArrPtr update(const PathPlanningMsg &msg);
    
private:
   std::vector<Eigen::Vector2f> left_cones_;
    std::vector<Eigen::Vector2f> right_cones_;
    std::map<float, size_t> left_distances_;
    std::map<float, size_t> right_distances_;

    void clear();
    void sortCones(const PathPlanningMsg &msg);
    void findMiddlePoints(std::vector<sgtdv_msgs::Point2D> &points);
    bool isLessOnLeft() const;
    float norm(const Eigen::Vector2f &point) const;
    Eigen::Vector2f rotate90Clockwise(const Eigen::Vector2f &point) const;
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

    virtual sgtdv_msgs::Point2DArrPtr update(const PathPlanningMsg &msg);
};