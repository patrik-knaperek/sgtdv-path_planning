/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#pragma once

#include "Messages.h"
#include "PathPlanningDisciplines.h"
#include "opencv2/core/core.hpp"
#include <ros/ros.h>


class PathPlanning
{
public:
    PathPlanning();
    ~PathPlanning();

    void SetPublisher(ros::Publisher publisher);
    void Do(const PathPlanningMsg &msg);  
    void YellowOnLeft(bool value);
    void SetDiscipline(Discipline discipline);

private:    
    ros::Publisher m_publisher;
    PathPlanningDiscipline *m_pathPlanningDiscipline = nullptr;
};