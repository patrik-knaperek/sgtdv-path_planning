/*****************************************************/
//Organization: Stuba Green Team
//Authors: Pavel Sadloň, Juraj Krasňanský
/*****************************************************/

#include "../include/PathPlanningDisciplines.h"


PathPlanningDiscipline::PathPlanningDiscipline()
{

}

PathPlanningDiscipline::~PathPlanningDiscipline()
{

}

void PathPlanningDiscipline::YellowOnLeft(bool value)
{
    m_isYellowOnLeft = value;
}


//////////////////////////////
//////////////////////////////
////// UNKNOWN_TRACK /////////
//////////////////////////////

UnknownTrack::UnknownTrack()
{
    m_isYellowOnLeft = false;
}

UnknownTrack::~UnknownTrack()
{

}

sgtdv_msgs::Point2DArrPtr UnknownTrack::Do(const PathPlanningMsg &msg)
{
    sgtdv_msgs::Point2DArrPtr trajectory( new sgtdv_msgs::Point2DArr );
    trajectory->points.reserve(MAX_PREDICT_POINTS);

    SortCones(msg);
    FindMiddlePoints(trajectory->points);
    return trajectory;
}

bool UnknownTrack::IsLessOnLeft() const
{
    return m_leftCones.size() < m_rightCones.size();
}

float UnknownTrack::Norm(const cv::Vec2f &point) const
{
    return sqrt(point.dot(point));
}

void UnknownTrack::SortCones(const PathPlanningMsg &msg)
{
    Clear();
    m_leftCones.reserve(msg.coneMap->cones.size());         //overkill, but better than reallocate whole block
    m_rightCones.reserve(msg.coneMap->cones.size());

    for (size_t i = 0; i < msg.coneMap->cones.size(); i++)
    {
        cv::Vec2f conePos(msg.coneMap->cones[i].coords.x, msg.coneMap->cones[i].coords.y);
        cv::Vec2f vehiclePos(msg.carState->position.x, msg.carState->position.y);
        cv::Vec2f forwardVec(cosf(deg2rad(msg.carState->yaw)), sinf(deg2rad(msg.carState->yaw)));
        cv::Vec2f vehicleConeVec(conePos - vehiclePos);

        if (vehicleConeVec.dot(forwardVec) > 0)     //if cone is in front of the vehicle
        {
            bool isOnTheLeft = true;
            cv::Vec2f rightVec;

            switch (msg.coneMap->cones[i].color)
            {
                case 'y':
                    m_rightDistances.insert(std::pair<float, size_t>(Norm(vehicleConeVec), m_rightCones.size())); 
                    m_rightCones.push_back(conePos);
                    break;
                case 'b':
                    m_leftDistances.insert(std::pair<float, size_t>(Norm(vehicleConeVec), m_leftCones.size())); 
                    m_leftCones.push_back(conePos);
                    break;
                case 'o':
                    rightVec = Rotate90Clockwise(forwardVec);
                    isOnTheLeft = rightVec.dot(vehicleConeVec) < 0;
                    
                    isOnTheLeft ? m_leftDistances.insert(std::pair<float, size_t>(Norm(vehicleConeVec), m_leftCones.size()))
                        :  m_rightDistances.insert(std::pair<float, size_t>(Norm(vehicleConeVec), m_rightCones.size()));
                    isOnTheLeft ? m_leftCones.push_back(conePos) : m_rightCones.push_back(conePos);
                    break;
                default:
                    std::cerr << "Unknown color of cone\n";
            }
        }
    }

    if (m_isYellowOnLeft)
    {
        std::swap(m_leftCones, m_rightCones);
        std::swap(m_leftDistances, m_rightDistances);
    }
}

cv::Vec2f UnknownTrack::Rotate90Clockwise(const cv::Vec2f &point) const
{
    return cv::Vec2f(point[1], -point[0]);
}

void UnknownTrack::Clear()
{
    m_leftCones.clear();
    m_rightCones.clear();
    m_leftDistances.clear();
    m_rightDistances.clear();
}

void UnknownTrack::FindMiddlePoints(std::vector<sgtdv_msgs::Point2D> &points)
{
    std::map<float, size_t>::iterator leftIt(m_leftDistances.begin());
    std::map<float, size_t>::iterator rightIt(m_rightDistances.begin());

    for (size_t i = 0; i < MAX_PREDICT_POINTS; i++)
    {
        if (m_leftDistances.size() <= i && m_rightDistances.size() <= i) continue;
        sgtdv_msgs::Point2D temp;
        cv::Vec2f newPos = ((m_leftCones[leftIt->second] + m_rightCones[rightIt->second]) / 2.f);

        temp.x = newPos[0];
        temp.y = newPos[1];
        leftIt++;
        rightIt++;
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

sgtdv_msgs::Point2DArrPtr Skidpad::Do(const PathPlanningMsg &msg)
{
    sgtdv_msgs::Point2DArrPtr trajectory ( new sgtdv_msgs::Point2DArr );

    return trajectory;
}