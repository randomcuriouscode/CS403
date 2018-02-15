#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>

#include "compsci403_assignment3/TransformPointSrv.h"
#include "compsci403_assignment3/FitMinimalPlaneSrv.h"
#include "compsci403_assignment3/FindInliersSrv.h"
#include "compsci403_assignment3/FitBestPlaneSrv.h"
#include "compsci403_assignment3/PlaneParametersMsg.h"

// Include any additional header or service/message files

using Eigen::Matrix3f;
using Eigen::Vector3f;
using geometry_msgs::Point32;

// Declare class variables, publishers, messages

ros::ServiceServer g_TransformPointSrv; // /COMPSCI403/TransformPoint
ros::ServiceServer g_FitMinimalPlaneSrv; // /COMPSCI403/FitMinimalPlane
ros::ServiceServer g_FindInliersSrv; // /COMPSCI403/FindInliers
ros::ServiceServer g_FitBestPlaneSrv; // /

const float ESTIMATED_FIT_POINTS = .75f;
const size_t RANSAC_MAX_ITER = 40;

// Define service and callback functions

bool TransformPointCallback(compsci403_assignment3::TransformPointSrv::Request &req,
							compsci403_assignment3::TransformPointSrv::Response &res)
{
	Vector3f P(req.P.x, req.P.y, req.P.z); // point in kinect ref frame

	Matrix3f R; // rotation matrix

	for (int row = 0; row < 3; row++)
	{
		for (int col = 0; col < 3; col++)
		{
			R(row, col) = req.R[3*row+col];
		}
	}

	Vector3f T(req.T.x, req.T.y, req.T.z); // translation vector

	std::stringstream logstr;

	logstr << "P: " << P << "R: " << R << "T: " << T << std::endl;

	ROS_DEBUG("TransformPointCallback(): %s", logstr.str().c_str());

	Vector3f P_prime = (R * P) + T;

	res.P_prime.x = (float)P_prime.x();
	res.P_prime.y = (float)P_prime.y();
	res.P_prime.z = (float)P_prime.z();

	return true;
}

bool FitMinimalPlaneCallback(compsci403_assignment3::FitMinimalPlaneSrv::Request &req,
							compsci403_assignment3::FitMinimalPlaneSrv::Response &res)
{
	Vector3f v1 (req.P2.x - req.P1.x, 
				req.P2.y - req.P1.y,
				req.P2.z - req.P1.z ); //Vector P1P2
	Vector3f v2 (req.P3.x - req.P1.x,
				req.P3.y - req.P1.y,
				req.P3.z - req.P1.z);

	Vector3f n = v1.cross(v2); // calc normal
	n = n / n.norm(); // normalize normal

	Point32 p_n;

	p_n.x = n.x();
	p_n.y = n.y();
	p_n.z = n.z();

	res.n = p_n;
	res.P0 = req.P1; // any point on plane can be P0

	return true;
}

bool FindInliersCallback(compsci403_assignment3::FindInliersSrv::Request &req,
						compsci403_assignment3::FindInliersSrv::Response &res)
{
	Vector3f n (req.n.x, req.n.y, req.n.z);

	for (std::vector<Point32>::iterator it = req.P.begin(); 
		 it != req.P.end(); it++)
	{
		Vector3f M  (it->x - req.P0.x,
					 it->y - req.P0.y,
					 it->z - req.P0.z); // M = (P - P0)

		float d = M.dot(n);  // calculate distance

		if (d <= req.epsilon)
		{   // add to response inlier points list
			res.P.push_back(*it); 
		}
	} 

	return true;
}

/*
  Helper to randomly select an item from a STL container, from stackoverflow.
*/
template <typename I>
I random_element(I begin, I end)
{
    const unsigned long n = std::distance(begin, end);
    const unsigned long divisor = (RAND_MAX + 1) / n;

    unsigned long k;
    do { k = std::rand() / divisor; } while (k >= n);

    std::advance(begin, k);
    return begin;
}

bool FitBestPlaneCallback(compsci403_assignment3::FitBestPlaneSrv::Request &req,
						  compsci403_assignment3::FitBestPlaneSrv::Response &res)
{
	std::vector<Point32> all_points = req.P;
	for (int iterations = 0; iterations < RANSAC_MAX_ITER; iterations ++)
	{
		// get 3 random points 
		Point32 p1 = *random_element(all_points.begin(), all_points.end());
		Point32 p2 = *random_element(all_points.begin(), all_points.end());
		Point32 p3 = *random_element(all_points.begin(), all_points.end());
	}
	return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "assignment3");
  ros::NodeHandle n;

  // Perform operations defined in Assignment 3

  // 1. Provide service TransformPoint
  g_TransformPointSrv = n.advertiseService("COMPSCI403/TransformPoint", TransformPointCallback);

  //2. Provide service FitMinimalPlane
  g_FitMinimalPlaneSrv = n.advertiseService("/COMPSCI403/FitMinimalPlane", FitMinimalPlaneCallback);

  //3. Provide Service FindInliers
  g_FindInliersSrv = n.advertiseService("/COMPSCI403/FindInliers", FindInliersCallback);

  //4. Provide /COMPSCI403/FitBestPlane
  g_FitBestPlaneSrv = n.advertiseService("/COMPSCI403/FitBestPlane", FitBestPlaneCallback);

  ros::spin();

  return 0;
}
