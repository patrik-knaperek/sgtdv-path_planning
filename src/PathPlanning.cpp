/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský, Samuel Mazur
/*****************************************************/


#include "../include/PathPlanning.h"

PathPlanning::PathPlanning() :
	m_once(false)
{
    
}

/**
 * @brief Seting ROS publishers.
 * @param trajectoryPub
 * @param interpolatedConesPub
 */
//TODO: replace multiple arguments with single one
void PathPlanning::SetPublisher(const ros::Publisher &trajectoryPub, const ros::Publisher &interpolatedConesPub)
{
    m_trajectoryPub = trajectoryPub;
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
	m_interpolatedConesPub.publish(InterpolatedCones());
	
	if(FULL_MAP)
	{
		RRTRun();
		m_trajectoryPub.publish(RRTPoints());
	}
	
	else m_trajectoryPub.publish(FindMiddlePoints());
}

/**
 * @brief Main function to handle RRT.
 */
void PathPlanning::RRTRun()
{
	if(m_once)
	{
	m_once = false;
	timeravg = 0;
	timeravgcount = 0;
	cv::Vec2f startPos1 = ((m_leftConesInterpolated[0] + m_rightConesInterpolated[0]) / 2.f);

	short cone_iter = m_leftConesInterpolated.size()/3.f;
	cv::Vec2f startPos2 = ((m_leftConesInterpolated[cone_iter] + m_rightConesInterpolated[cone_iter]) / 2.f);

	short cone_iter2 = 2* m_leftConesInterpolated.size()/3.f;
	cv::Vec2f startPos3 = ((m_leftConesInterpolated[cone_iter2] + m_rightConesInterpolated[cone_iter2]) / 2.f);

	cv::Vec2f endPos = ((m_leftConesInterpolated[1] + m_rightConesInterpolated[1]) / 2.f);

	m_rrtStar1.init(m_leftConesInterpolated, m_rightConesInterpolated,0, cone_iter+1, startPos1, startPos2, NULL);
	m_rrtStar2.init(m_leftConesInterpolated, m_rightConesInterpolated,cone_iter-1, cone_iter2+1, startPos2, startPos3, NULL);
	m_rrtStar3.init(m_leftConesInterpolated, m_rightConesInterpolated,cone_iter2-1, m_leftConesInterpolated.size(), startPos3, endPos, NULL);
	}

	//execution timer
	auto start = std::chrono::high_resolution_clock::now();
	
	m_rrtStar1.Do();
	m_rrtStar2.Do();
	m_rrtStar3.Do();

	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
	timeravg += duration.count()/ 1000000.f;
	timeravgcount +=1;
	std::cout <<"\nRRT Timer:" << timeravg/timeravgcount <<" s\n";
	std::cout<<"Path jumps: "<<m_rrtStar1.path.size() + m_rrtStar2.path.size() + m_rrtStar3.path.size()<<"\n";
	std::cout<<"Total nodes: "<<m_rrtStar1.nodes.size() + m_rrtStar2.nodes.size() + m_rrtStar3.nodes.size()<<"\n";
	
}

/**
 * @brief Publishing message for RRT data.
 * @return
 */
visualization_msgs::MarkerArray PathPlanning::RRTPoints() const
{
    visualization_msgs::MarkerArray trajectory;
	geometry_msgs::Point temp;

	visualization_msgs::Marker tree1;
    tree1.type = visualization_msgs::Marker::LINE_STRIP;
    tree1.header.frame_id = "map";
	tree1.id = 0;
    tree1.scale.x = 0.2;
    tree1.scale.y = 0.2;
    tree1.color.r = 0.0f;
	tree1.color.g = 1.0f;
 	tree1.color.b = 0.0f;
    tree1.color.a = 1.0;
    
	for(size_t i = 0; i<m_rrtStar1.path.size(); i++)
	{
		temp.x = m_rrtStar1.path[i]->position[0];
		temp.y = m_rrtStar1.path[i]->position[1];
		tree1.points.push_back(temp);
	}

	visualization_msgs::Marker tree2;
    tree2.type = visualization_msgs::Marker::LINE_STRIP;
    tree2.header.frame_id = "map";
	tree2.id = 1;
    tree2.scale.x = 0.2;
    tree2.scale.y = 0.2;
    tree2.color.r = 1.0f;
	tree2.color.g = 0.0f;
 	tree2.color.b = 0.0f;
    tree2.color.a = 1.0;

	for(size_t i = 0; i<m_rrtStar2.path.size(); i++)
	{
		temp.x = m_rrtStar2.path[i]->position[0];
		temp.y = m_rrtStar2.path[i]->position[1];
		tree2.points.push_back(temp);
	}

	visualization_msgs::Marker tree3;
    tree3.type = visualization_msgs::Marker::LINE_STRIP;
    tree3.header.frame_id = "map";
	tree3.id = 2;
    tree3.scale.x = 0.2;
    tree3.scale.y = 0.2;
    tree3.color.r = 1.0f;
	tree3.color.g = 0.0f;
 	tree3.color.b = 1.0f;
    tree3.color.a = 1.0;

	for(size_t i = 0; i<m_rrtStar3.path.size(); i++)
	{
		temp.x = m_rrtStar3.path[i]->position[0];
		temp.y = m_rrtStar3.path[i]->position[1];
		tree3.points.push_back(temp);
	}


	visualization_msgs::Marker points;
    points.type = visualization_msgs::Marker::POINTS;
    points.header.frame_id = "map";
	points.id = 3;
    points.scale.x = 0.15;
    points.scale.y = 0.15;
    points.color.r = 0.0f;
	points.color.g = 1.0f;
 	points.color.b = 0.0f;
    points.color.a = 1.0;

	visualization_msgs::Marker points2;
    points2.type = visualization_msgs::Marker::POINTS;
    points2.header.frame_id = "map";
	points2.id = 4;
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
    points3.scale.x = 0.15;
    points3.scale.y = 0.15;
    points3.color.r = 1.0f;
	points3.color.g = 0.0f;
 	points3.color.b = 1.0f;
    points3.color.a = 1.0;

	for(size_t i = 0; i<m_rrtStar1.nodes.size(); i++)
	{
		temp.x = m_rrtStar1.nodes[i]->position[0];
		temp.y = m_rrtStar1.nodes[i]->position[1];
		points.points.push_back(temp);
	}
	for(size_t i = 0; i<m_rrtStar2.nodes.size(); i++)
	{
		temp.x = m_rrtStar2.nodes[i]->position[0];
		temp.y = m_rrtStar2.nodes[i]->position[1];
		points2.points.push_back(temp);
	}
	for(size_t i = 0; i<m_rrtStar3.nodes.size(); i++)
	{
		temp.x = m_rrtStar3.nodes[i]->position[0];
		temp.y = m_rrtStar3.nodes[i]->position[1];
		points3.points.push_back(temp);
	}

	trajectory.markers.push_back(tree1);
	trajectory.markers.push_back(tree2);
	trajectory.markers.push_back(tree3);
	trajectory.markers.push_back(points);
	trajectory.markers.push_back(points2);
	trajectory.markers.push_back(points3);

	return trajectory;

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
	

    for (size_t i = 0; i < msg.coneMap->cones.size(); i++)
    {
        cv::Vec2f conePos(msg.coneMap->cones[i].coords.x, msg.coneMap->cones[i].coords.y);

        switch (msg.coneMap->cones[i].color)
        {
            	case 1:
                    m_rightCones.push_back(conePos);
                    break;
                case 2:
                    m_leftCones.push_back(conePos);
                    break;
                case 3:
                    break;
                default:
                    std::cerr << "Unknown color of cone\n";
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

	for(size_t i=0;i<points.size();i++)
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
visualization_msgs::MarkerArray PathPlanning::InterpolatedCones() const
{	
	visualization_msgs::MarkerArray markerArr;
	geometry_msgs::Point temp;

	visualization_msgs::Marker leftpoints;
    leftpoints.type = visualization_msgs::Marker::POINTS;
    leftpoints.header.frame_id = "map";
	leftpoints.id = 0;
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
   	rightpointsBIG.scale.x = 0.4;
    rightpointsBIG.scale.y = 0.4;
    rightpointsBIG.color.r = 0.0f;
	rightpointsBIG.color.g = 0.0f;
 	rightpointsBIG.color.b = 1.0f;
    rightpointsBIG.color.a = 1.0;


	for(size_t i = 0; i< m_leftConesInterpolated.size(); i++)
		{
			temp.x = m_leftConesInterpolated[i][0];
			temp.y = m_leftConesInterpolated[i][1];
			rightpoints.points.push_back(temp);	
		}
	markerArr.markers.push_back(rightpoints);


	for(size_t i = 0; i< m_rightConesInterpolated.size(); i++)
		{
			temp.x = m_rightConesInterpolated[i][0];
			temp.y = m_rightConesInterpolated[i][1];
			leftpoints.points.push_back(temp);	
		}
	markerArr.markers.push_back(leftpoints);
	
	for(size_t i = 0; i< m_leftCones.size(); i++)
		{
			temp.x = m_leftCones[i][0];
			temp.y = m_leftCones[i][1];
			rightpointsBIG.points.push_back(temp);	
		}
	markerArr.markers.push_back(rightpointsBIG);


	for(size_t i = 0; i< m_rightCones.size(); i++)
		{
			temp.x = m_rightCones[i][0];
			temp.y = m_rightCones[i][1];
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

    visualization_msgs::Marker trajectory;
    trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory.header.frame_id = "map";
    trajectory.id = 0;
    trajectory.scale.x = 0.2;
    trajectory.scale.y = 0.2;
    trajectory.color.g = 1.0f;
    trajectory.color.a = 1.0;

    trajectory.points.reserve(3);
    geometry_msgs::Point temp;

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

			
			temp.x = xh + ((xi-xh)*j);
			temp.y = yh + ((yi-yh)*j);
			trajectory.points.push_back(temp);	
		}
		if(!i) i-=2;
	}

   markerArr.markers.push_back(trajectory);
   return markerArr;	
}
