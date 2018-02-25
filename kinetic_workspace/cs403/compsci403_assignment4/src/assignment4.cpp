#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
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

const float TIME_DELTA = .05f; // velocity update every .05 seconds
const float S_MAX = .25f; // max stopping distance .25m by V_max^2 / (2a_max)
const float R_ROBOT = .18f; // radius of robot is .18m

using namespace std;

namespace t_helpers
{
/*
	@param p 2x1 Vector input point
	@param 
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

int main(int argc, char **argv) {

  ros::init(argc, argv, "assignment4");
  ros::NodeHandle n;

	// Perform operations defined in Assignment 4

	// 1. Provide Service /COMPSCI403/CheckPoint
  g_CheckPointSrv = n.advertiseService("/COMPSCI403/CheckPoint", CheckPointCallback);

	ros::spin();

  return(0);
}
