/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Juraj Krasňanský
/*****************************************************/

#include "../include/path_planning_synch.h"
//#include "../include/path_planning_disciplines.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_planning");
  ros::NodeHandle handle;

  PathPlanningSynch synch_obj(handle);

  //if(/*arg from launchfile*/true)
  //{
  //    synch_obj.setDiscipline(UNKNOWN_TRACK);
  //}   
  //else
  //{
  //    synch_obj.setDiscipline(SKIDPAD);
  //}

  ros::spin();

  return 0;
}