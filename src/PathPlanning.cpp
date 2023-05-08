/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Samuel Mazur, Patrik Knaperek
/*****************************************************/


#include "../include/PathPlanning.h"

PathPlanning::PathPlanning()
	: m_isYellowOnLeft(true)
	, m_once(true)
	, m_fullMap(false)
{
    
}

/**
 * @brief Seting ROS publishers.
 * @param trajectoryPub
 * @param interpolatedConesPub
 */
void PathPlanning::SetPublisher(const ros::Publisher &trajectoryPub
								, const ros::Publisher &trajectoryVisPub
								, const ros::Publisher &interpolatedConesPub
								)
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
	m_leftConesInterpolated = LinearInterpolation(m_leftCones);
	m_rightConesInterpolated = LinearInterpolation(m_rightCones);
	
	m_interpolatedConesPub.publish(VisualizeInterpolatedCones());
	
	bool RRTCompleted(false);
	if(m_fullMap)
	{
		RRTCompleted = RRTRun();
	}

	sgtdv_msgs::Point2DArr trajectory;
	if (RRTCompleted)
		trajectory = m_rrtStar.GetPath();
	else
		trajectory = FindMiddlePoints();

	m_trajectoryPub.publish(trajectory);
	VisualizeTrajectory(trajectory);
	VisualizeRRTPoints();

	m_trajectoryVisPub.publish(m_trajectoryVisMarkers);
}

/**
 * @brief Main function to handle RRT.
 */
bool PathPlanning::RRTRun()
{
	if(m_once)
	{
		m_once = false;
		m_timeravg = 0;
		m_timeravgcount = 0;
		
		cv::Vec2f startPos = ((m_leftConesInterpolated[0] + m_rightConesInterpolated[0]) / 2.f);
		short cone_iter = m_leftConesInterpolated.size();
		cv::Vec2f endPos = ((m_leftConesInterpolated[m_leftConesInterpolated.size()-1] + m_rightConesInterpolated[m_rightConesInterpolated.size()-1]) / 2.f);
		
		m_rrtStar.Init(m_leftConesInterpolated, m_rightConesInterpolated,0, cone_iter, startPos, endPos);
	}
	
	//execution timer
	auto start = std::chrono::high_resolution_clock::now();
	
	bool endReached = m_rrtStar.Do();
	
	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
	m_timeravg += duration.count()/ 1000000.f;
	m_timeravgcount +=1;
	ROS_INFO("\nRRT Timer: %f s", m_timeravg/m_timeravgcount);
	ROS_INFO("Path jumps: %ld", m_rrtStar.GetPath().points.size());
	ROS_INFO("Total nodes: %ld\n",m_rrtStar.GetNodes().size());

	return endReached;	
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
		const float maxDist = sqrt((pow(points[i+1][0] - points[i][0], 2) + pow(points[i+1][1] - points[i][1], 2)) * 1.0);
		float step = BEZIER_RESOLUTION;

		cv::Vec2f endpoint = points[i+1];
		
		// closing cone loop - last cone is the first cone
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
 * @brief Publishing message for middleline trajectory.
 * @return
 */
sgtdv_msgs::Point2DArr PathPlanning::FindMiddlePoints()
{
	sgtdv_msgs::Point2DArr trajectory;
	sgtdv_msgs::Point2D point;

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

			
			point.x = xh + ((xi-xh)*j);
			point.y = yh + ((yi-yh)*j);
			trajectory.points.push_back(point);
		}
		if(!i) i-=2;
	}

   return trajectory;	
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
 	leftpoints.color.a = 1.0;
	
	visualization_msgs::Marker rightpoints;
    rightpoints.type = visualization_msgs::Marker::POINTS;
    rightpoints.header.frame_id = "map";
	rightpoints.id = 1;
	rightpoints.ns = "right_interpolated";
    rightpoints.scale.x = 0.2;
    rightpoints.scale.y = 0.2;
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
 	leftpointsBIG.color.a = 1.0;
	leftpointsBIG.pose.orientation.w = 1.0;

	visualization_msgs::Marker rightpointsBIG;
    rightpointsBIG.type = visualization_msgs::Marker::POINTS;
    rightpointsBIG.header.frame_id = "map";
	rightpointsBIG.id = 3;
	rightpointsBIG.ns = "right_cones";
   	rightpointsBIG.scale.x = 0.4;
    rightpointsBIG.scale.y = 0.4;
    rightpointsBIG.color.b = 1.0f;
    rightpointsBIG.color.a = 1.0;
	rightpointsBIG.pose.orientation.w = 1.0;


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

	visualization_msgs::Marker start;
	start.type = visualization_msgs::Marker::POINTS;
    start.header.frame_id = "map";
	start.id = 4;
	start.ns = "start";
    start.scale.x = 0.5;
    start.scale.y = 0.5;
    start.color.r = 0.7f;
	start.color.a = 1.0;
	temp.x = m_rightCones[0][0];
	temp.y = m_rightCones[0][1];
	start.points.push_back(temp);
	temp.x = m_leftCones[0][0];
	temp.y = m_leftCones[0][1];
	start.points.push_back(temp);
	markerArr.markers.push_back(start);

	visualization_msgs::Marker end;
	end.type = visualization_msgs::Marker::POINTS;
    end.header.frame_id = "map";
	end.id = 5;
	end.ns = "end";
    end.scale.x = 0.5;
    end.scale.y = 0.5;
    end.color.r = 0.0f;
	end.color.g = 0.7f;
 	end.color.a = 1.0;
	temp.x = m_rightCones[m_rightCones.size()-1][0];
	temp.y = m_rightCones[m_rightCones.size()-1][1];
	end.points.push_back(temp);
	temp.x = m_leftCones[m_leftCones.size()-1][0];
	temp.y = m_leftCones[m_leftCones.size()-1][1];
	end.points.push_back(temp);
	markerArr.markers.push_back(end);


	return markerArr;
}

void PathPlanning::VisualizeTrajectory(const sgtdv_msgs::Point2DArr &trajectory)
{
	m_trajectoryVisMarkers.markers.clear();

	visualization_msgs::Marker trajectoryVis;
    trajectoryVis.type = visualization_msgs::Marker::LINE_STRIP;
    trajectoryVis.header.frame_id = "map";
    trajectoryVis.id = 0;
	trajectoryVis.ns = "trajectory";
    trajectoryVis.scale.x = 0.2;
    trajectoryVis.scale.y = 0.2;
    trajectoryVis.color.r = 1.0f;
    trajectoryVis.color.a = 1.0;
	trajectoryVis.pose.orientation.w = 1.0;

    trajectoryVis.points.reserve(trajectory.points.size());
    geometry_msgs::Point pointVis;

	for (auto &point : trajectory.points)
	{
		pointVis.x = point.x;
		pointVis.y = point.y;
		trajectoryVis.points.push_back(pointVis);
	}

	m_trajectoryVisMarkers.markers.emplace_back(trajectoryVis);
}

/**
 * @brief Publishing message for RRT data.
 * @return
 */
void PathPlanning::VisualizeRRTPoints()
{
    geometry_msgs::Point pointVis;

	visualization_msgs::Marker nodes;
    nodes.type = visualization_msgs::Marker::POINTS;
    nodes.header.frame_id = "map";
	nodes.id = 2;
	nodes.ns = "RRT nodes";
    nodes.scale.x = 0.15;
    nodes.scale.y = 0.15;
    nodes.color.g = 1.0f;
 	nodes.color.a = 1.0;

	for(const auto &node : m_rrtStar.GetNodes())
	{
		pointVis.x = node->position[0];
		pointVis.y = node->position[1];
		nodes.points.push_back(pointVis);	
	}
	
	m_trajectoryVisMarkers.markers.emplace_back(nodes);
	

	visualization_msgs::Marker trajectory;
    trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory.header.frame_id = "map";
	trajectory.id = 3;
	trajectory.ns = "RRT trajectory";
    trajectory.scale.x = 0.2;
    trajectory.scale.y = 0.2;
    trajectory.color.g = 1.0f;
 	trajectory.color.a = 1.0;
	trajectory.pose.orientation.w = 1.0;
    
	for(const auto &pathIt : m_rrtStar.GetPath().points)
	{
		pointVis.x = pathIt.x;
		pointVis.y = pathIt.y;
		trajectory.points.push_back(pointVis);
	}
	
	m_trajectoryVisMarkers.markers.emplace_back(trajectory);
}
