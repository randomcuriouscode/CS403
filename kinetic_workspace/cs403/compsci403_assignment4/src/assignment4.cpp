#include "assignment4.h"

// Include any additional header or service/message files

// Declare class variables, subscribers, publishers, messages

ros::ServiceServer g_CheckPointSrv; // Service /COMPSCI403/CheckPoint
ros::ServiceServer g_GetFreePathSrv; // Service /COMPSCI403/GetFreePath
ros::ServiceServer g_GetCommandVelSrv; // Service /COMPSCI403/GetCommandVel
ros::Subscriber g_LaserSub; // Subscriber /Cobot/Laser

ros::Publisher g_ObstaclesPub; // Publisher /COMPSCI403/Obstacles

ros::Publisher g_VisPub; // For Visualizations

static sensor_msgs::PointCloud g_TranslatedPC; // store translated point cloud from result of part 3
																							 // could make a class wrapper for access, but not sure
																							 // how to deal with multiple copies occurring on stack
static int32_t DISCRETIZATIONS = 10; // total discretizations >= DISCRETIZATIONS^2

using namespace std;

// Define service and callback functions

bool CheckPointCallback (compsci403_assignment4::CheckPointSrv::Request &req,
												 compsci403_assignment4::CheckPointSrv::Response &res)
{
	Eigen::Vector2f p (req.P.x, req.P.y); // z value is always 0, 2d point

	float f = -1.0f;

	bool obstacle = t_helpers::PointIsObstacle(p, req.v, req.w, &f);

	stringstream logstr;
	logstr << p;

	if (obstacle)
	{
		ROS_DEBUG("CheckPointCallback: point (%s) OBSTACLE for w: %f, v: %f, f: %f", logstr.str().c_str(), 
							req.v, req.w, f);
		res.is_obstacle = true;
		res.free_path_length = f;
	}
	else
	{
		ROS_DEBUG("CheckPointCallback: point (%s) NOT OBSTACLE for w: %f, v: %f", logstr.str().c_str(), req.v, req.w);
		res.is_obstacle = false;
	}

	return true;
}

bool GetFreePathCallback (compsci403_assignment4::GetFreePathSrv::Request &req,
													compsci403_assignment4::GetFreePathSrv::Response &res)
{
	sensor_msgs::PointCloud pc;
	// laser scan is already in the reference frame of the robot, just convert it to a pointcloud
	t_helpers::LaserScanToPointCloud(req.laser_scan, pc); 

	vector< pointdistpair > obstacles;
	pointdistpair closest_pt;
	bool obstacle = t_helpers::ObstacleExist(pc, req.v, req.w, obstacles, closest_pt); // true if not obstacle free

	// publish to visualization_marker_array
	g_VisPub.publish(t_helpers::GenPointListMarkers(pc, obstacles, "/base_footprint"));

	if (obstacle) // at least one obstacle was found
	{
		ROS_DEBUG("GetFreePathCallback: At least 1 obstacle: %f", closest_pt.second);
		res.is_obstacle = true;
		res.free_path_length = closest_pt.second;
	}
	else // no obstacles along the path
	{
		ROS_DEBUG("GetFreePathCallback: No obstacles found along path");
		res.is_obstacle = false;
	}

	return true;
} // end GetFreePathCallback

void ScanOccurredCallback (const sensor_msgs::LaserScan &msg)
{
	// convert scan depths into robot ref point cloud
	sensor_msgs::PointCloud translated_pc;

	t_helpers::ProjectRangeFinderToRobotRef(msg, translated_pc);

	// publish converted points

	compsci403_assignment4::ObstacleMsg res;
	res.header = msg.header;
	res.obstacle_points = translated_pc.points;

	g_TranslatedPC = translated_pc; // set global for part 4. gross, but necessary.

	g_ObstaclesPub.publish(res);
}

bool GetCommandVelCallback (compsci403_assignment4::GetCommandVelSrv::Request &req,
													 	compsci403_assignment4::GetCommandVelSrv::Response &res)
{
	if (g_TranslatedPC.points.size() == 0)
	{
		ROS_ERROR("GetCommandVelCallback: FATAL ERROR, /Cobot/Laser was NOT published to before this call. PointCloud global size is 0");
		return true; // TODO: this should be return false by ROS spec, but instructor says false is bad.
	}

	sensor_msgs::PointCloud translated_pc = g_TranslatedPC;
	
	vector< pointdistpair > obstacle_points;
	pointdistpair closest_pt;

	bool obstacle = t_helpers::ObstacleExist(translated_pc, req.v_0, req.w_0, obstacle_points, closest_pt);
	
	if (obstacle)
	{	
		vector<Eigen::Vector2f> disc_window = t_helpers::GenDiscDynWind(Eigen::Vector2f(req.v_0, req.w_0), DISCRETIZATIONS);
		Eigen::Vector2f best_vel;
		float best_cost = numeric_limits<float>::min();

		for (auto it = disc_window.begin(); it != disc_window.end(); it++)
		{ // iterate over each val in discrete window.
			
		}

	}
	else // no obstacles
	{ // robot can continue on its merry way
		res.C_v = req.v_0;
		res.C_w = req.w_0;
	}

	g_TranslatedPC = sensor_msgs::PointCloud(); // reset the cached pointcloud
	return true;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "assignment4");
  ros::NodeHandle n;

  if (argc > 1)
  {
  	DISCRETIZATIONS = atoi(argv[1]) > 0 ? atoi(argv[1]) : DISCRETIZATIONS;
  	ROS_INFO("DISCRETIZATIONS set to %i", DISCRETIZATIONS);
  }
  else
  {
  	ROS_INFO("Param: Discretizations not set, defaulting");
  }

	// Perform operations defined in Assignment 4
	// 0. publish topic visualization_marker_array
	g_VisPub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1000);

	// 1. Provide Service /COMPSCI403/CheckPoint
  g_CheckPointSrv = n.advertiseService("/COMPSCI403/CheckPoint", CheckPointCallback);

  // 2. Provide Service /COMPSCI403/GetFreePath
  g_GetFreePathSrv = n.advertiseService("/COMPSCI403/GetFreePath", GetFreePathCallback);

  // 3. Create Publisher /COMPSCI403/Obstacles
  g_ObstaclesPub = n.advertise<sensor_msgs::LaserScan>("/COMPSCI403/Obstacles", 1000);

  // 3. Create Subscriber /Cobot/Laser
  g_LaserSub = n.subscribe("/Cobot/Laser", 1000, ScanOccurredCallback);

  // 4. Create Service /COMPSCI403/GetCommandVel
  g_GetCommandVelSrv = n.advertiseService("/COMPSCI403/GetCommandVel", GetCommandVelCallback);

  ros::Rate spin_rate (20); // 20 hz
	
	while (ros::ok())
	{
		ros::spinOnce();
		spin_rate.sleep();
	}

  return(0);
}
