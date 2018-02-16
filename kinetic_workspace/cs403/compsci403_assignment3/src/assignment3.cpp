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

#include <ros/console.h>

#define DEBUG



// Include any additional header or service/message files

using Eigen::Matrix3f;
using Eigen::Vector3f;
using geometry_msgs::Point32;

// Declare class variables, publishers, messages

ros::ServiceServer g_TransformPointSrv; // /COMPSCI403/TransformPoint
ros::ServiceServer g_FitMinimalPlaneSrv; // /COMPSCI403/FitMinimalPlane
ros::ServiceServer g_FindInliersSrv; // /COMPSCI403/FindInliers
ros::ServiceServer g_FitBestPlaneSrv; // /COMPSCI403/FitBestPlane

ros::Subscriber g_PointCloudSub; // /COMPSCI403/PointCloud

ros::Publisher g_FilteredPointCloudPub; // /COMPSCI403/FilteredPointCloud
ros::Publisher g_PlaneParametersPub; // /COMPSCI403/PlaneParameters

const float RANSAC_ESTIMATED_FIT_POINTS = .80f; // % points estimated to fit the model
const size_t RANSAC_MAX_ITER = 100; // max RANSAC iterations
const size_t RANDOM_MAX_TRIES = 100; // max RANSAC random point tries per iteration
const float RANSAC_THRESHOLD = 0.00000000000001f; // threshold to determine what constitutes a close point to a plane

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
	// n will already be normalized in the request.

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
    const unsigned long divisor = ((long)RAND_MAX + 1) / n;

    unsigned long k;
    do { k = std::rand() / divisor; } while (k >= n);

    std::advance(begin, k);
    return begin;
}

bool run_RANSAC(const std::vector<Point32> all_points, 
	Vector3f *out_p0, Vector3f *out_n,
	 std::vector<Point32> *out_inlier_points)
{
	for (size_t iterations = 0; iterations < RANSAC_MAX_ITER; iterations ++)
	{
		Point32 p1,p2,p3;
		Vector3f v1;
		Vector3f v2;

		Vector3f n_hat; // keep track of the current plane model
		Vector3f P0;
		std::vector<Point32> points_agree; // list of points that agree with model within 

		bool found = false;

		// try RANDOM_MAX_TRIES times to get random 3 points
		for (size_t tries = 0; tries < RANDOM_MAX_TRIES; tries ++) // try to get unique random points 100 times
		{
			// get 3 random points 
			p1 = *random_element(all_points.begin(), all_points.end());
			p2 = *random_element(all_points.begin(), all_points.end());
			p3 = *random_element(all_points.begin(), all_points.end());

			v1 = Vector3f (p2.x - p1.x, 
					p2.y - p1.y,
					p2.z - p1.z ); //Vector P1P2
			v2 = Vector3f (p3.x - p1.x,
					p3.y - p1.y,
					p3.z - p1.z); //Vector P1P3

			if (std::abs(v1.dot(v2)) != 1.f) // dot product != 1 means we've found 3 nonlinear points
			{
				found = true;
				break; 
			}
		} // end try random element loop
		if (!found) // could not find 3 random nonlinear points in 100 tries, go to next iteration
		{
			ROS_ERROR("run_RANSAC(): Could not find 3 random nonlinear points in %ld tries, going on to iteration %ld", RANDOM_MAX_TRIES, iterations + 1);
			continue;
		}

		// nonlinear random points exist past here

		// fit a plane to p1, p2, p3

		Vector3f n = v1.cross(v2); // calculate normal of plane
		n_hat = n / n.norm();
		P0 = Vector3f(p1.x, p1.y, p1.z); 

		// at some point, the original p0, p1, p2 will be iterated over and added to agreed points

		// loop over all points, find points that are inliers to plane
		for (std::vector<Point32>::const_iterator it = all_points.begin(); 
		 it != all_points.end(); it++)
		{
			Vector3f M  (it->x - P0.x(),
						 it->y - P0.y(),
						 it->z - P0.z()); // M = (P - P0)

			float d = M.dot(n_hat);  // calculate distance

			if (d <= RANSAC_THRESHOLD)
			{   // add to inlier points list
				points_agree.push_back(*it); 
			}
		} // end points loop 

		ROS_DEBUG("run_RANSAC() POINTS AGREED: %li=%f, RANSAC_ESTIMATED_FIT_POINTS: %f", points_agree.size(), 
				(float) points_agree.size() / all_points.size(), RANSAC_ESTIMATED_FIT_POINTS);
		if (((float) points_agree.size()) / all_points.size() > RANSAC_ESTIMATED_FIT_POINTS)
			{	// if points agree / total points > estimated % points fitting
				// fit to points_agree.size() points

				size_t n = points_agree.size();

				Vector3f sum(0.0f, 0.0f, 0.0f);

				for (std::vector<Point32>::iterator iter = points_agree.begin();
					iter != points_agree.end(); iter++)
				{
					sum += Vector3f(iter->x, iter->y, iter->z);
				}

				Vector3f centroid = sum / n; // calculate centroid

				Eigen::MatrixXf M(points_agree.size(), 3);

				for (size_t row = 0; row < points_agree.size(); row++)
				{ // build distance vector matrix
					Vector3f point(points_agree[row].x, 
									points_agree[row].y,
									points_agree[row].z);

					for (size_t col = 0; col < 3; col ++)
					{
						M(row, col) = point(col) - centroid(col);
					}
				}

				Matrix3f covariance_matrix = M.transpose() * M;

				Eigen::EigenSolver<Matrix3f> eigen_solver;
				eigen_solver.compute(covariance_matrix);

				Vector3f eigen_values = eigen_solver.eigenvalues().real();

				Matrix3f eigen_vectors = eigen_solver.eigenvectors().real();

				// find eigenvalue that is closest to 0

				size_t idx;

				// find minimum eigenvalue, get index
				float closest_eval = eigen_values.cwiseAbs().minCoeff(&idx);

				// find corresponding eigenvector
				Vector3f closest_evec = eigen_vectors.col(idx);

				std::stringstream logstr;
				logstr << "Closest eigenvalue : " << closest_eval << std::endl <<
					"Corresponding eigenvector : " << std::endl << closest_evec << std::endl <<
					"Centroid : " << std::endl << centroid;

				ROS_DEBUG("run_RANSAC(): %s", logstr.str().c_str());

				Vector3f all_fitted_n_hat = closest_evec / closest_evec.norm();

				// invoke copy constructors for outbound 
				*out_n = Vector3f(all_fitted_n_hat);
				*out_p0 = Vector3f(centroid); 
				*out_inlier_points = std::vector<Point32>(points_agree);

				ROS_INFO("run_RANSAC():: Success, total_size: %li, inlier_size: %li, %% agreement %f", 
					all_points.size(), out_inlier_points->size(), (float) out_inlier_points->size() / all_points.size());

				return true;
			}
	} // end iterations loop
	return false;
}

bool FitBestPlaneCallback(compsci403_assignment3::FitBestPlaneSrv::Request &req,
						  compsci403_assignment3::FitBestPlaneSrv::Response &res)
{
	std::vector<Point32> all_points = req.P;

	Vector3f fitted_n_hat;
	Vector3f fitted_p0;

	std::vector<Point32> inlier_points;

	if(!run_RANSAC(all_points, &fitted_p0, &fitted_n_hat, &inlier_points))
	{
		ROS_ERROR("FitBestPlaneCallback(): Something went wrong with RANSAC");
		return false;
	}

	res.P0.x = fitted_p0.x();
	res.P0.y = fitted_p0.y();
	res.P0.z = fitted_p0.z();

	res.n.x = fitted_n_hat.x();
	res.n.y = fitted_n_hat.y();
	res.n.z = fitted_n_hat.z();

	return true;
}

void PointCloudCallback(const sensor_msgs::PointCloud &pc)
{
	Vector3f fitted_n_hat;
	Vector3f fitted_p0;
	std::vector<Point32> inlier_points;

	if (!run_RANSAC(pc.points, &fitted_p0, &fitted_n_hat, &inlier_points))
	{
		ROS_ERROR("PointCloudCallback(): RANSAC failed to fit the plane, nothing to publish");
		return;
	}

	// construct filtered pointcloud msg
	sensor_msgs::PointCloud filtered_pc;
	filtered_pc.header = pc.header;
	filtered_pc.points = inlier_points;

	g_FilteredPointCloudPub.publish(filtered_pc); //publish filtered

	// construct plane parameters msg
	compsci403_assignment3::PlaneParametersMsg plane_params;
	plane_params.n.x = fitted_n_hat.x();
	plane_params.n.y = fitted_n_hat.y();
	plane_params.n.z = fitted_n_hat.z();

	plane_params.P0.x = fitted_p0.x();
	plane_params.P0.y = fitted_p0.y();
	plane_params.P0.z = fitted_p0.z();

	g_PlaneParametersPub.publish(plane_params); // publish plane params
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

  //5. Publisher to /COMPSCI403/FilteredPointCloud
  g_FilteredPointCloudPub = n.advertise<sensor_msgs::PointCloud>("/COMPSCI403/FilteredPointCloud", 1000);

  //5. Publisher to /COMPSCI403/PlaneParameters
  g_PlaneParametersPub = n.advertise<compsci403_assignment3::PlaneParametersMsg>("/COMPSCI403/PlaneParameters", 1000);

  //5. Subscriber to /COMPSCI403/PointCloud
  g_PointCloudSub = n.subscribe("/COMPSCI403/PointCloud", 1000, PointCloudCallback);

#ifdef DEBUG
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
	   ros::console::notifyLoggerLevelsChanged();
	}
#endif

  ROS_INFO("All globals initialized, spinning...");

  ros::spin();

  return 0;
}
