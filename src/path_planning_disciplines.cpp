/*****************************************************/
//Organization: Stuba Green Team
//Authors: Pavel Sadloň, Juraj Krasňanský
/*****************************************************/

#include "../include/path_planning_disciplines.h"


PathPlanningDiscipline::PathPlanningDiscipline()
{

}

PathPlanningDiscipline::~PathPlanningDiscipline()
{

}

void PathPlanningDiscipline::yellowOnLeft(bool value)
{
    is_yellow_on_left_ = value;
}


//////////////////////////////
//////////////////////////////
////// UNKNOWN_TRACK /////////
//////////////////////////////

UnknownTrack::UnknownTrack()
{
    is_yellow_on_left_ = false;
}

UnknownTrack::~UnknownTrack()
{

}

sgtdv_msgs::Point2DArrPtr UnknownTrack::update(const PathPlanningMsg &msg)
{
    sgtdv_msgs::Point2DArrPtr trajectory( new sgtdv_msgs::Point2DArr );
    trajectory->points.reserve(MAX_PREDICT_POINTS);

    sortCones(msg);
    findMiddlePoints(trajectory->points);
    return trajectory;
}

bool UnknownTrack::isLessOnLeft() const
{
    return left_cones_.size() < right_cones_.size();
}

float UnknownTrack::norm(const Eigen::Vector2f &point) const
{
    return sqrt(point.dot(point));
}

void UnknownTrack::sortCones(const PathPlanningMsg &msg)
{
    clear();
    left_cones_.reserve(msg.cone_map->cones.size());         //overkill, but better than reallocate whole block
    right_cones_.reserve(msg.cone_map->cones.size());

    for (size_t i = 0; i < msg.cone_map->cones.size(); i++)
    {
        Eigen::Vector2f cone_pos(msg.cone_map->cones[i].coords.x, msg.cone_map->cones[i].coords.y);
        Eigen::Vector2f vehicle_pos(msg.car_pose->position.x, msg.car_pose->position.y);
        Eigen::Vector2f forward_vec(cosf(deg2rad(msg.car_pose->yaw)), sinf(deg2rad(msg.car_pose->yaw)));
        Eigen::Vector2f vehicle_cone_vec(cone_pos - vehicle_pos);

        if (vehicle_cone_vec.dot(forward_vec) > 0)     //if cone is in front of the vehicle
        {
            bool is_on_the_left = true;
            Eigen::Vector2f right_vec;

            switch (msg.cone_map->cones[i].color)
            {
                case 'y':
                    right_distances_.insert(std::pair<float, size_t>(norm(vehicle_cone_vec), right_cones_.size())); 
                    right_cones_.push_back(cone_pos);
                    break;
                case 'b':
                    left_distances_.insert(std::pair<float, size_t>(norm(vehicle_cone_vec), left_cones_.size())); 
                    left_cones_.push_back(cone_pos);
                    break;
                case 'o':
                    right_vec = rotate90Clockwise(forward_vec);
                    is_on_the_left = right_vec.dot(vehicle_cone_vec) < 0;
                    
                    is_on_the_left ? left_distances_.insert(std::pair<float, size_t>(norm(vehicle_cone_vec), left_cones_.size()))
                        :  right_distances_.insert(std::pair<float, size_t>(norm(vehicle_cone_vec), right_cones_.size()));
                    is_on_the_left ? left_cones_.push_back(cone_pos) : right_cones_.push_back(cone_pos);
                    break;
                default:
                    std::cerr << "Unknown color of cone\n";
            }
        }
    }

    if (is_yellow_on_left_)
    {
        std::swap(left_cones_, right_cones_);
        std::swap(left_distances_, right_distances_);
    }
}

Eigen::Vector2f UnknownTrack::rotate90Clockwise(const Eigen::Vector2f &point) const
{
    return Eigen::Vector2f(point[1], -point[0]);
}

void UnknownTrack::clear()
{
    left_cones_.clear();
    right_cones_.clear();
    left_distances_.clear();
    right_distances_.clear();
}

void UnknownTrack::findMiddlePoints(std::vector<sgtdv_msgs::Point2D> &points)
{
    std::map<float, size_t>::iterator left_it(left_distances_.begin());
    std::map<float, size_t>::iterator right_it(right_distances_.begin());

    for (size_t i = 0; i < MAX_PREDICT_POINTS; i++)
    {
        if (left_distances_.size() <= i && right_distances_.size() <= i) 
            continue;
        sgtdv_msgs::Point2D temp;
        Eigen::Vector2f new_pos = ((left_cones_[left_it->second] + right_cones_[right_it->second]) / 2.f);

        temp.x = new_pos[0];
        temp.y = new_pos[1];
        if (left_it == left_distances_.end())
            break;
        left_it++;
        if (right_it == right_distances_.end())
            break;
        right_it++;
        points.push_back(temp);
    }
}


//////////////////////////////
//////////////////////////////
///////// SKIDPAD ////////////
//////////////////////////////


Skidpad::Skidpad()
{

}

Skidpad::~Skidpad()
{

}

sgtdv_msgs::Point2DArrPtr Skidpad::update(const PathPlanningMsg &msg)
{
    sgtdv_msgs::Point2DArrPtr trajectory ( new sgtdv_msgs::Point2DArr );

    return trajectory;
}