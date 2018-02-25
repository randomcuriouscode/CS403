#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>

#include "cobot_msgs/CobotDriveMsg.h"
#include "compsci403_assignment4/ObstacleMsg.h"
#include "compsci403_assignment4/CheckPointSrv.h"
#include "compsci403_assignment4/GetFreePathSrv.h"
#include "compsci403_assignment4/GetCommandVelSrv.h"

// Include any additional header or service/message files

// Declare class variables, subscribers, publishers, messages

ros::ServiceServer g_CheckPointSrv;
ros::ServiceServer g_GetFreePathSrv;

const float TIME_DELTA = .05f; // velocity update every .05 seconds
const float S_MAX = .25f; // max stopping distance .25m by V_max^2 / (2a_max)
const float R_ROBOT = .18f; // radius of robot is .18m

using namespace std;

namespace t_helpers
{
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
	Convert a laser scan dataset into a point cloud, assuming reference frame of the robot.
*/
void LaserScanToPointCloud_RobotRef(const sensor_msgs::LaserScan &msg, sensor_msgs::PointCloud &pc)
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
}

} // end helper mainspace

// Define service and callback functions

bool CheckPointCallback (compsci403_assignment4::CheckPointSrv::Request &req,
												 compsci403_assignment4::CheckPointSrv::Response &res)
{
	Eigen::Vector2f p (req.P.x, req.P.y); // z value is always 0, 2d point

	float f = -1.0f;

	bool obstacle = t_helpers::PointIsObstacle(p, req.v, req.w, &f);

	if (obstacle)
	{
		res.is_obstacle = true;
		res.free_path_length = f;
	}
	else
	{
		res.is_obstacle = false;
	}

	return true;
}

bool GetFreePathCallback (compsci403_assignment4::GetFreePathSrv::Request &req,
													compsci403_assignment4::GetFreePathSrv::Response &res)
{
	sensor_msgs::PointCloud pc;
	t_helpers::LaserScanToPointCloud_RobotRef(req.laser_scan, pc);

	bool obstacle = false; // true if not obstacle free
	float min_f = numeric_limits<float>::max(); // keep track of min free path len

	for (vector<geometry_msgs::Point32>::iterator it = pc.points.begin(); it != pc.points.end(); 
				it++)
	{
		float temp_f;
		bool temp_obstacle = t_helpers::PointIsObstacle(Eigen::Vector2f(it->x, it->y), 
																										req.v, req.w, &temp_f);
		if (temp_obstacle) // found an obstacle
		{
			obstacle = true;
			if (temp_f < min_f) // obstacle is closer than current closest
			{
				ROS_DEBUG("GetFreePathCallback: Found an obstacle: %f, cur min: %f", temp_f, min_f);
				min_f = temp_f;
			}
		}
	}

	if (obstacle) // at least one obstacle was found
	{
		res.is_obstacle = true;
		res.free_path_length = min_f;
	}
	else // no obstacles along the path
	{
		res.is_obstacle = false;
	}

	return true;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "assignment4");
  ros::NodeHandle n;

	// Perform operations defined in Assignment 4

	// 1. Provide Service /COMPSCI403/CheckPoint
  g_CheckPointSrv = n.advertiseService("/COMPSCI403/CheckPoint", CheckPointCallback);

  // 2. Provide Service /COMPSCI403/GetFreePath
  g_GetFreePathSrv = n.advertiseService("/COMPSCI403/GetFreePath", GetFreePathCallback);

	ros::spin();

  return(0);
}
