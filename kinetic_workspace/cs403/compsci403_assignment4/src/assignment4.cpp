#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>

#include <visualization_msgs/Marker.h>

#include "cobot_msgs/CobotDriveMsg.h"
#include "compsci403_assignment4/ObstacleMsg.h"
#include "compsci403_assignment4/CheckPointSrv.h"
#include "compsci403_assignment4/GetFreePathSrv.h"
#include "compsci403_assignment4/GetCommandVelSrv.h"

// Include any additional header or service/message files

// Declare class variables, subscribers, publishers, messages

ros::ServiceServer g_CheckPointSrv; // Service /COMPSCI403/CheckPoint
ros::ServiceServer g_GetFreePathSrv; // Service /COMPSCI403/GetFreePath
ros::ServiceServer g_GetCommandVelSrv; // Service /COMPSCI403/GetCommandVel
ros::Subscriber g_LaserSub; // Subscriber /Cobot/Laser

ros::Publisher g_ObstaclesPub; // Publisher /COMPSCI403/Obstacles

ros::Publisher g_VisPub; // For Visualizations

const float TIME_DELTA = .05f; // velocity update every .05 seconds
const float V_MAX = .5f; // max velocity is .5 m/s
const float W_MAX = 1.5f; // max angular velocity is 1.5 rad/s
const float AC_MAX = .5f; // max linear acceleration is .5 m/s^2
const float WC_MAX = 2.0f; // max angular acceleration is 2 rad/s^2
const float S_MAX = .25f; // max stopping distance .25m by V_max^2 / (2a_max)
const float R_ROBOT = .18f; // radius of robot is .18m

const Eigen::Matrix3f M_ROTATION = (Eigen::Matrix3f() << 1.f, 0.f, 0.f,
																												 0.f, 1.f, 0.f,
																												 0.f, 0.f, 1.f).finished();

const Eigen::MatrixXf M_TRANSLATION = (Eigen::MatrixXf(3,1) << .145f, 0.f, 0.23f).finished();

static sensor_msgs::PointCloud g_TranslatedPC; // store translated point cloud from result of part 3
																							 // could make a class wrapper for access, but not sure
																							 // how to deal with multiple copies occurring on stack

using namespace std;

typedef pair<geometry_msgs::Point32, float> pointdistpair;


namespace t_helpers
{

visualization_msgs::Marker GenPointListMarkers(const sensor_msgs::PointCloud all_pts, 
																							const vector< pointdistpair > obstacle_pts,
																							const string frame_id  )
{
	static size_t id = 0;
	visualization_msgs::Marker m;
	m.header.frame_id = frame_id;
	m.header.stamp = ros::Time();
	m.ns = "t_helpers";
	m.id = id ++;

	return m;
}

// function prototypes ONLY
bool PointIsObstacle(Eigen::Vector2f p, float v, float w, float *out_f);

// end function prototypes

/*
	@param pc input PointCloud
	@param v velocity scalar
	@param w angular velocity scalar
	@param out_pointmap output vector of points to their free path distance
	@param out_closest output closest point and free path distance
*/
bool ObstacleExist(const sensor_msgs::PointCloud pc, const float v, const float w,
					vector< pointdistpair > &out_pointmap, 
					pointdistpair &out_closest)
{
	bool obstacle = false; // true if not obstacle free
	float min_f = numeric_limits<float>::max(); // keep track of min free path len
	geometry_msgs::Point32 closest_pt;

	for (vector<geometry_msgs::Point32>::const_iterator it = pc.points.begin(); it != pc.points.end(); 
				it++) // iterate over all converted points to find obstacle
	{
		float temp_f;
		bool temp_obstacle = PointIsObstacle(Eigen::Vector2f(it->x, it->y), 
																					v, w, &temp_f);
		if (temp_obstacle) // found an obstacle
		{
			obstacle = true;

			out_pointmap.push_back(pointdistpair(*it, temp_f)); // push to output vector

			if (temp_f < min_f) // obstacle is closer than current closest
			{
				ROS_DEBUG("GetFreePathCallback: Found an obstacle: %f, cur min: %f", temp_f, min_f);
				closest_pt = *it;
				min_f = temp_f;
			}
		}
	}

	if (obstacle) // at least one obstacle was found
	{
		ROS_DEBUG("GetFreePathCallback: At least 1 obstacle: %f", min_f);
		out_closest.first = closest_pt;
		out_closest.second = min_f;
		return true;
	}
	else // no obstacles along the path
	{
		ROS_DEBUG("GetFreePathCallback: No obstacles found along path");
		return false;
	}
}

/*
	Convenience function to output only the free path distance, discarding other output.
*/
bool ObstacleExist(const sensor_msgs::PointCloud pc, const float v, const float w, float *out_f)
{
	vector< pointdistpair > dont_care;
	pointdistpair closest_pt;

	bool obstacle = ObstacleExist(pc, v, w, dont_care, closest_pt);

	if (obstacle)
	{
		*out_f = closest_pt.second;
	}

	return obstacle;
}

/*
	@param p 2x1 Vector input point
	@param v velocity scalar
	@param w angular velocity scalar
	@param out_f output free path scalar
*/
bool PointIsObstacle(Eigen::Vector2f p, float v, float w, float *out_f)
{
	if (!w) // 0 angular vel, calculate straight line free path to p
	{	// let robot be moving along x axis

		// if the y value of p is within radius of robot, it is an obstacle
		if (abs(p.y()) <= R_ROBOT)
		{
			// since p is obstacle, compute free path length
			float f = p.x() - sqrt(pow(R_ROBOT, 2.0f) - pow(p.y(), 2.0f));

			ROS_DEBUG("p.y: %f is less than R_ROBOT: %f, free path length: %f", p.y(), R_ROBOT, f);

			*out_f = f;

			return true; // free path and obstacle found with 0 angular velocity
		} //end if (abs(p.y() < R_ROBOT)
		else // not an obstacle
		{
			ROS_DEBUG("p.y: %f is greater than R_ROBOT: %f, not obstacle", p.y(), R_ROBOT);

			return false; // not an obstacle
		} // end else
	} // end if (!w)
	else
	{ // not a straight line path


		Eigen::Vector2f c (0, v / w);	// calculate center of rotation

		float r = c.norm(); // radius of rotation is the L1 norm of the center of rotation

		float p_dist_from_robot = abs((c - p).norm() - r);

		if (p_dist_from_robot < R_ROBOT) // obstacle
		{

		Eigen::Vector2f cp = p - c; // terminal point - initial point = vector CP
		Eigen::Vector2f co = -c; // vector CO

		float pco = acos(cp.dot(co) / (cp.norm() * co.norm())); // total angle covered about c

		// compute angle PCL using R and r (angle of center of robot to end of free path)

		float pcl = atan(R_ROBOT / r); // arctan (robot / radius of rotation)

		float lco = pco - pcl; // free path angle

		float f = lco * r; // free path arclength = free path angle * radius of rotation

		*out_f = f;
		return true; // free path obstacle found along curve
		} // end if p_dist_from_robot < R_ROBOT
		else // not an obstacle
		{
			return false; // no obstacle along path
		} // end else
	} // end else
} // end PointIsObstacle

/*
	1. Convert a laser scan dataset into a point cloud. Makes NO translation of the points, just a raw depth transform
	2. copies laser scan msg header to point cloud.
*/
void LaserScanToPointCloud(const sensor_msgs::LaserScan &msg, sensor_msgs::PointCloud &pc)
{
	float angle_min = msg.angle_min;
  float angle_max = msg.angle_max;
  float angle_increment = msg.angle_increment;
  float range_min = msg.range_min;
  float range_max = msg.range_max;
  std::vector<float> ranges = msg.ranges;

  std::size_t m = ranges.size();

  ROS_INFO("Angle_min: %f, angle max: %f, angle_increment: %f, range_min: %f, range_max: %f, ranges_size: %lu",
      angle_min, angle_max, angle_increment, range_min, range_max, m);

  pc.header = msg.header;

  float cur_angle = angle_min;

  for(std::size_t i = 0; i < m; ++i){
    if (ranges[i] < range_max && ranges[i] > range_min){
      float x_i = ranges[i] * cos(cur_angle);
      float y_i = ranges[i] * sin(cur_angle);

      geometry_msgs::Point32 p;
      p.x = x_i;
      p.y = y_i;

      ROS_INFO("X_%lu: %f,Y_%lu: %f", i, x_i, i, y_i);

      pc.points.push_back(p);
    }
    cur_angle += angle_increment;
  }
} // end LaserScanToPointCloud

/*
	1. Project a laser scan dataset measured in the reference frame of the scanner to the reference frame of the robot
	2. Copy msg header to point cloud header.
*/
void ProjectRangeFinderToRobotRef(const sensor_msgs::LaserScan &msg, sensor_msgs::PointCloud &translated_pc)
{
	translated_pc.header = msg.header; // copy msg header

	sensor_msgs::PointCloud scanner_pc; // point cloud in ref of scanner

	LaserScanToPointCloud(msg, scanner_pc); // translate laser scan to a point cloud in refframe of scanner

	for (vector<geometry_msgs::Point32>::const_iterator it = scanner_pc.points.begin(); it != scanner_pc.points.end();
			it++) // iterate through scanner pointcloud, translate to robot reference frame with P' = RP + T
	{
		// assume it->z is 0 since this is in  2-D
		Eigen::MatrixXf p  = (Eigen::MatrixXf(3,1) << it->x, it->y, 0).finished();
		Eigen::Vector3f p_prime = M_ROTATION * p + M_TRANSLATION;

		geometry_msgs::Point32 pt_p_prime;
		pt_p_prime.x = p_prime.x();
		pt_p_prime.y = p_prime.y();
		pt_p_prime.z = 0; // 2-d, z value is 0.

		translated_pc.points.push_back(pt_p_prime);
	}
}

} // end helper mainspace

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

	float closest_pt = -1.0f;
	bool obstacle = t_helpers::ObstacleExist(pc, req.v, req.w, &closest_pt); // true if not obstacle free
	

	if (obstacle) // at least one obstacle was found
	{
		ROS_DEBUG("GetFreePathCallback: At least 1 obstacle: %f", closest_pt);
		res.is_obstacle = true;
		res.free_path_length = closest_pt;
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

	}
	else // no obstacles
	{

	}

	g_TranslatedPC = sensor_msgs::PointCloud(); // reset the cached pointcloud
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "assignment4");
  ros::NodeHandle n;

	// Perform operations defined in Assignment 4

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
