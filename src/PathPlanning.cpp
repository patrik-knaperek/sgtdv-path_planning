/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Samuel Mazur, Patrik Knaperek
/*****************************************************/


#include "../include/PathPlanning.h"

PathPlanning::PathPlanning()
	: m_isYellowOnLeft(true)
	, m_once(true)
{
    
}

/**
 * @brief Seting ROS publishers.
 * @param trajectoryPub
 * @param interpolatedConesPub
 */
//TODO: replace multiple arguments with single one
void PathPlanning::SetPublisher(const ros::Publisher &trajectoryPub, const ros::Publisher &trajectoryVisPub, const ros::Publisher &interpolatedConesPub)
{
    m_trajectoryPub = trajectoryPub;
	m_trajectoryVisPub = trajectoryVisPub;
    m_interpolatedConesPub = interpolatedConesPub;
}

/*void PathPlanning::SetDiscipline(Discipline discipline)
{
    switch (discipline)
    {
        case UNKNOWN_TRACK: m_pathPlanningDiscipline = new UnknownTrack; break;
        case SKIDPAD: m_pathPlanningDiscipline = new Skidpad; break;
        default: m_pathPlanningDiscipline = nullptr;
    }

    if (!m_pathPlanningDiscipline) ros::shutdown();
}*/

/**
 * @brief Swap color of cones in arrays.
 * @param isYellowOnLeft
 */
void PathPlanning::YellowOnLeft(bool value)
{
    //m_pathPlanningDiscipline->YellowOnLeft(value);
    m_isYellowOnLeft = value;
}

/**
 * @brief Main function in class.
 * @param incomingROSMsg
 */
void PathPlanning::Do(const PathPlanningMsg &msg)
{
    //m_publisher.publish(m_pathPlanningDiscipline->Do(msg));

    SortCones(msg);
	//ROS_INFO("SortCones(msg) completed");
    m_leftConesInterpolated = LinearInterpolation(m_leftCones);
	//ROS_INFO("LinearInterpolation(m_leftCones) completed");
    m_rightConesInterpolated = LinearInterpolation(m_rightCones);
	//ROS_INFO("LinearInterpolation(m_rightCones) completed");
	m_interpolatedConesPub.publish(VisualizeInterpolatedCones());
	
	if(FULL_MAP)
	{
		ROS_INFO("Starting RRTRun algorithm");
		RRTRun();
		m_trajectoryVisPub.publish(VisualizeRRTPoints());
	}
	
	else m_trajectoryVisPub.publish(FindMiddlePoints());
}

/**
 * @brief Main function to handle RRT.
 */
void PathPlanning::RRTRun()
{
	if(m_once)
	{
	m_once = false;
	m_timeravg = 0;
	m_timeravgcount = 0;
	//ROS_INFO_STREAM("first left: " << m_leftConesInterpolated[0]);
	//ROS_INFO_STREAM("first right: " << m_rightConesInterpolated[0]);
	cv::Vec2f startPos1 = ((m_leftConesInterpolated[0] + m_rightConesInterpolated[0]) / 2.f);

	short cone_iter = m_leftConesInterpolated.size()/3.f;
	cv::Vec2f startPos2 = ((m_leftConesInterpolated[cone_iter] + m_rightConesInterpolated[cone_iter]) / 2.f);

	short cone_iter2 = 2* m_leftConesInterpolated.size()/3.f;
	cv::Vec2f startPos3 = ((m_leftConesInterpolated[cone_iter2] + m_rightConesInterpolated[cone_iter2]) / 2.f);

	cv::Vec2f endPos = ((m_leftConesInterpolated[1] + m_rightConesInterpolated[1]) / 2.f);

	m_rrtStar1.Init(m_leftConesInterpolated, m_rightConesInterpolated,0, cone_iter+1, startPos1, startPos2);
	m_rrtStar2.Init(m_leftConesInterpolated, m_rightConesInterpolated,cone_iter-1, cone_iter2+1, startPos2, startPos3);
	m_rrtStar3.Init(m_leftConesInterpolated, m_rightConesInterpolated,cone_iter2-1, m_leftConesInterpolated.size(), startPos3, endPos);
	}
	ROS_INFO("RRT Star initialization complete");

	//execution timer
	auto start = std::chrono::high_resolution_clock::now();
	
	m_rrtStar1.Do();
	//ROS_INFO("RRT Star 1 completed with %ld nodes", m_rrtStar1.GetPath().size());
	m_rrtStar2.Do();
	//ROS_INFO("RRT Star 2 completed with %ld nodes", m_rrtStar2.GetPath().size());
	m_rrtStar3.Do();
	//ROS_INFO("RRT Star 3 completed with %ld nodes", m_rrtStar3.GetPath().size());

	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
	m_timeravg += duration.count()/ 1000000.f;
	m_timeravgcount +=1;
	ROS_INFO("\nRRT Timer: %f s\n", m_timeravg/m_timeravgcount);
	ROS_INFO("Path jumps: %ld", m_rrtStar1.GetPath().size() + m_rrtStar2.GetPath().size() + m_rrtStar3.GetPath().size());
	ROS_INFO("Total nodes: %ld",m_rrtStar1.GetPath().size() + m_rrtStar2.GetPath().size() + m_rrtStar3.GetPath().size());
	
}

/**
 * @brief Publishing message for RRT data.
 * @return
 */
visualization_msgs::MarkerArray PathPlanning::VisualizeRRTPoints() const
{
    visualization_msgs::MarkerArray trajectoryVis;
	geometry_msgs::Point pointVis;
	sgtdv_msgs::Point2DArr trajectory;
	sgtdv_msgs::Point2D point;

	visualization_msgs::Marker tree1;
    tree1.type = visualization_msgs::Marker::LINE_STRIP;
    tree1.header.frame_id = "map";
	tree1.id = 0;
	tree1.ns = "trajectory1";
    tree1.scale.x = 0.2;
    tree1.scale.y = 0.2;
    tree1.color.r = 0.0f;
	tree1.color.g = 1.0f;
 	tree1.color.b = 0.0f;
    tree1.color.a = 1.0;
	tree1.pose.orientation.w = 1.0;
    
	for(const auto &pathIt : m_rrtStar1.GetPath())
	{
		point.x = pointVis.x = pathIt[0];
		point.y = pointVis.y = pathIt[1];
		tree1.points.push_back(pointVis);
		trajectory.points.push_back(point);
	}

	visualization_msgs::Marker tree2;
    tree2.type = visualization_msgs::Marker::LINE_STRIP;
    tree2.header.frame_id = "map";
	tree2.id = 1;
	tree2.ns = "trajectory2";
    tree2.scale.x = 0.2;
    tree2.scale.y = 0.2;
    tree2.color.r = 1.0f;
	tree2.color.g = 0.0f;
 	tree2.color.b = 0.0f;
    tree2.color.a = 1.0;
	tree2.pose.orientation.w = 1.0;

	for(const auto &pathIt : m_rrtStar2.GetPath())
	{
		point.x = pointVis.x = pathIt[0];
		point.y = pointVis.y = pathIt[1];
		tree2.points.push_back(pointVis);
		trajectory.points.push_back(point);
	}

	visualization_msgs::Marker tree3;
    tree3.type = visualization_msgs::Marker::LINE_STRIP;
    tree3.header.frame_id = "map";
	tree3.id = 2;
	tree3.ns = "trajectory3";
    tree3.scale.x = 0.2;
    tree3.scale.y = 0.2;
    tree3.color.r = 1.0f;
	tree3.color.g = 0.0f;
 	tree3.color.b = 1.0f;
    tree3.color.a = 1.0;
	tree3.pose.orientation.w = 1.0;

	for(const auto &pathIt : m_rrtStar3.GetPath())
	{
		point.x = pointVis.x = pathIt[0];
		point.y = pointVis.y = pathIt[1];
		tree3.points.push_back(pointVis);
		trajectory.points.push_back(point);
	}


	visualization_msgs::Marker points1;
    points1.type = visualization_msgs::Marker::POINTS;
    points1.header.frame_id = "map";
	points1.id = 3;
	points1.ns = "nodes1";
    points1.scale.x = 0.15;
    points1.scale.y = 0.15;
    points1.color.r = 0.0f;
	points1.color.g = 1.0f;
 	points1.color.b = 0.0f;
    points1.color.a = 1.0;

	visualization_msgs::Marker points2;
    points2.type = visualization_msgs::Marker::POINTS;
    points2.header.frame_id = "map";
	points2.id = 4;
	points2.ns = "nodes2";
    points2.scale.x = 0.15;
    points2.scale.y = 0.15;
    points2.color.r = 1.0f;
	points2.color.g = 0.0f;
 	points2.color.b = 0.0f;
    points2.color.a = 1.0;

	visualization_msgs::Marker points3;
    points3.type = visualization_msgs::Marker::POINTS;
    points3.header.frame_id = "map";
	points3.id = 5;
	points3.ns = "nodes3";
    points3.scale.x = 0.15;
    points3.scale.y = 0.15;
    points3.color.r = 1.0f;
	points3.color.g = 0.0f;
 	points3.color.b = 1.0f;
    points3.color.a = 1.0;

	for(const auto &node : m_rrtStar1.GetNodes())
	{
		pointVis.x = node->position[0];
		pointVis.y = node->position[1];
		points1.points.push_back(pointVis);
	}
	for(const auto &node : m_rrtStar2.GetNodes())
	{
		pointVis.x = node->position[0];
		pointVis.y = node->position[1];
		points2.points.push_back(pointVis);
	}
	for(const auto &nodes : m_rrtStar3.GetNodes())
	{
		pointVis.x = nodes->position[0];
		pointVis.y = nodes->position[1];
		points3.points.push_back(pointVis);
	}

	trajectoryVis.markers.push_back(tree1);
	trajectoryVis.markers.push_back(tree2);
	trajectoryVis.markers.push_back(tree3);
	trajectoryVis.markers.push_back(points1);
	trajectoryVis.markers.push_back(points2);
	trajectoryVis.markers.push_back(points3);

	m_trajectoryPub.publish(trajectory);

	return trajectoryVis;

}

/**
 * @brief Sorting raw cone data.
 * @param messageCones
 */
void PathPlanning::SortCones(const PathPlanningMsg &msg)
{
    m_leftCones.clear();
    m_rightCones.clear();

    m_leftConesInterpolated.clear();
    m_rightConesInterpolated.clear();
	

    for (const auto &cone : msg.coneMap->cones)
    {
        cv::Vec2f conePos(cone.coords.x, cone.coords.y);

        switch (cone.color)
        {
            	case 'y':
				case 1 :
                    m_rightCones.push_back(conePos);
                    break;
                case 'b':
				case 2 :
                    m_leftCones.push_back(conePos);
                    break;
                case 's':
				case 'g':
				case 3 :
                    break;
                default:
                    ROS_ERROR("Unknown color of cone\n");
        }
    }

    if (m_isYellowOnLeft)
    {
        std::swap(m_leftCones, m_rightCones);
    }
}

/**
 * @brief Linear interpolation for single colored cones.
 * @param sortedCones
 * @return
 */
std::vector<cv::Vec2f> PathPlanning::LinearInterpolation(std::vector<cv::Vec2f> points) const
{
	std::vector<cv::Vec2f> temp_pts;
	cv::Vec2f temp;

	for(size_t i = 0; i < points.size(); i++)
	{	
		float maxDist = sqrt((pow(points[i+1][0] - points[i][0], 2) + pow(points[i+1][1] - points[i][1], 2)) * 1.0);
		float step = BEZIER_RESOLUTION;

		cv::Vec2f endpoint = points[i+1];
		//closing cone loop - last cone is the first cone
		if((i+1) == points.size()) endpoint = points[0];	

		if(maxDist > 6) step /=2;

		for(float j = 0 ; j < 1 ; j += step)
		{
			
			temp[0] = points[i][0] + ((endpoint[0]-points[i][0])*j);
			temp[1] = points[i][1] + ((endpoint[1]-points[i][1])*j);
			temp_pts.push_back(temp);
		}
	}	
	temp[0] = points[0][0];
	temp[1] = points[0][1];
	temp_pts.push_back(temp);

    return temp_pts; 
}

/**
 * @brief Publishing message for sorted and interpolated cone data.
 * @return
 */
visualization_msgs::MarkerArray PathPlanning::VisualizeInterpolatedCones() const
{	
	visualization_msgs::MarkerArray markerArr;
	geometry_msgs::Point temp;

	visualization_msgs::Marker leftpoints;
    leftpoints.type = visualization_msgs::Marker::POINTS;
    leftpoints.header.frame_id = "map";
	leftpoints.id = 0;
	leftpoints.ns = "left_interpolated";
    leftpoints.scale.x = 0.2;
    leftpoints.scale.y = 0.2;
    leftpoints.color.r = 0.8f;
	leftpoints.color.g = 0.8f;
 	leftpoints.color.b = 0.0f;
    leftpoints.color.a = 1.0;

	visualization_msgs::Marker rightpoints;
    rightpoints.type = visualization_msgs::Marker::POINTS;
    rightpoints.header.frame_id = "map";
	rightpoints.id = 1;
	rightpoints.ns = "right_interpolated";
    rightpoints.scale.x = 0.2;
    rightpoints.scale.y = 0.2;
    rightpoints.color.r = 0.0f;
	rightpoints.color.g = 0.0f;
 	rightpoints.color.b = 1.0f;
    rightpoints.color.a = 1.0;

	visualization_msgs::Marker leftpointsBIG;
    leftpointsBIG.type = visualization_msgs::Marker::POINTS;
    leftpointsBIG.header.frame_id = "map";
	leftpointsBIG.id = 2;
	leftpointsBIG.ns = "left_cones";
    leftpointsBIG.scale.x = 0.4;
    leftpointsBIG.scale.y = 0.4;
    leftpointsBIG.color.r = 0.8f;
	leftpointsBIG.color.g = 0.8f;
 	leftpointsBIG.color.b = 0.0f;
    leftpointsBIG.color.a = 1.0;

	visualization_msgs::Marker rightpointsBIG;
    rightpointsBIG.type = visualization_msgs::Marker::POINTS;
    rightpointsBIG.header.frame_id = "map";
	rightpointsBIG.id = 3;
	rightpointsBIG.ns = "right_cones";
   	rightpointsBIG.scale.x = 0.4;
    rightpointsBIG.scale.y = 0.4;
    rightpointsBIG.color.r = 0.0f;
	rightpointsBIG.color.g = 0.0f;
 	rightpointsBIG.color.b = 1.0f;
    rightpointsBIG.color.a = 1.0;


	for(const auto &cone : m_leftConesInterpolated)
		{
			temp.x = cone[0];
			temp.y = cone[1];
			rightpoints.points.push_back(temp);	
		}
	markerArr.markers.push_back(rightpoints);


	for(const auto &cone : m_rightConesInterpolated)
		{
			temp.x = cone[0];
			temp.y = cone[1];
			leftpoints.points.push_back(temp);	
		}
	markerArr.markers.push_back(leftpoints);
	
	for(const auto &cone : m_leftCones)
		{
			temp.x = cone[0];
			temp.y = cone[1];
			rightpointsBIG.points.push_back(temp);	
		}
	markerArr.markers.push_back(rightpointsBIG);


	for(const auto &cone : m_rightCones)
		{
			temp.x = cone[0];
			temp.y = cone[1];
			leftpointsBIG.points.push_back(temp);	
		}
	markerArr.markers.push_back(leftpointsBIG);

	return markerArr;
}

/**
 * @brief Publishing message for middleline trajectory.
 * @return
 */
visualization_msgs::MarkerArray PathPlanning::FindMiddlePoints()
{
	visualization_msgs::MarkerArray markerArr;
	sgtdv_msgs::Point2DArr trajectory;
	sgtdv_msgs::Point2D point;

    visualization_msgs::Marker trajectoryVis;
    trajectoryVis.type = visualization_msgs::Marker::LINE_STRIP;
    trajectoryVis.header.frame_id = "map";
    trajectoryVis.id = 0;
    trajectoryVis.scale.x = 0.2;
    trajectoryVis.scale.y = 0.2;
    trajectoryVis.color.g = 1.0f;
    trajectoryVis.color.a = 1.0;
	trajectoryVis.pose.orientation.w = 1.0;

    trajectoryVis.points.reserve(3);
    geometry_msgs::Point pointVis;

    m_middleLinePoints.clear();


    for (size_t i = 0; i < m_rightConesInterpolated.size(); i++)
    {
		cv::Vec2f newPoint = ((m_leftConesInterpolated[i] + m_rightConesInterpolated[i]) / 2.f);
		m_middleLinePoints.push_back(newPoint);		
    }

	for(size_t i = 0; i < m_middleLinePoints.size()-3; i+=4)
	{	

		cv::Vec2f endpoint2 = m_middleLinePoints[i+2];
		cv::Vec2f endpoint3 = m_middleLinePoints[i+3];
		cv::Vec2f endpoint4 = m_middleLinePoints[i+4];

		if((i+2) == m_middleLinePoints.size()) endpoint2 = m_middleLinePoints[0];
		if((i+3)>m_middleLinePoints.size()) endpoint3 = m_middleLinePoints[1];	
		if((i+4)>m_middleLinePoints.size()) endpoint4 = m_middleLinePoints[2];


    	for( float j = 0 ; j < 1 ; j += 0.01)
		{

			float xa = m_middleLinePoints[i][0] + ((m_middleLinePoints[i+1][0]-m_middleLinePoints[i][0])*j);
			float ya = m_middleLinePoints[i][1] + ((m_middleLinePoints[i+1][1]-m_middleLinePoints[i][1])*j);
			float xb= m_middleLinePoints[i+1][0] + ((endpoint2[0]-m_middleLinePoints[i+1][0])*j);
			float yb = m_middleLinePoints[i+1][1] + ((endpoint2[1]-m_middleLinePoints[i+1][1])*j);
			float xc = endpoint2[0] + ((endpoint3[0]-endpoint2[0])*j);
			float yc = endpoint2[1] + ((endpoint3[1]-endpoint2[1])*j);
			float xd = endpoint3[0] + ((endpoint4[0]-endpoint3[0])*j);
			float yd = endpoint3[1] + ((endpoint4[1]-endpoint3[1])*j);

			float xe = xa + ((xb-xa)*j);
			float ye = ya + ((yb-ya)*j);
			float xf= xb + ((xc-xb)*j);
			float yf = yb + ((yc-yb)*j);
			float xg= xc + ((xd-xc)*j);
			float yg = yc + ((yd-yc)*j);

			float xh = xe + ((xf-xe)*j);
			float yh = ye + ((yf-ye)*j);
			float xi= xf + ((xg-xf)*j);
			float yi = yf + ((yg-yf)*j);

			
			point.x = pointVis.x = xh + ((xi-xh)*j);
			point.y = pointVis.y = yh + ((yi-yh)*j);
			trajectoryVis.points.push_back(pointVis);
			trajectory.points.push_back(point);
		}
		if(!i) i-=2;
	}

   markerArr.markers.push_back(trajectoryVis);
   m_trajectoryPub.publish(trajectory);
   return markerArr;	
}
