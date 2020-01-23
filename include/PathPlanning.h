#include <ros/ros.h>
#include <sgtdv_msgs/PathPlanningMsg.h>
#include <sgtdv_msgs/PathTrackingMsg.h>

class PathPlanning
{
public:
    PathPlanning();
    ~PathPlanning();

    void Do(const sgtdv_msgs::PathPlanningMsgPtr &msg);
    void SetPublisher(ros::Publisher publisher);

private:
    ros::Publisher m_publisher;
};