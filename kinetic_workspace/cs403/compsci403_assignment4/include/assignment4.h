#pragma once
#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "cobot_msgs/CobotDriveMsg.h"
#include "compsci403_assignment4/ObstacleMsg.h"
#include "compsci403_assignment4/CheckPointSrv.h"
#include "compsci403_assignment4/GetFreePathSrv.h"
#include "compsci403_assignment4/GetCommandVelSrv.h"

#include <cmath>

using namespace std;

typedef pair<geometry_msgs::Point32, float> pointdistpair;

const float TIME_DELTA = .05f; // velocity update every .05 seconds
const float V_MAX = .5f; // max velocity is .5 m/s
const float W_MAX = 1.5f; // max angular velocity is 1.5 rad/s
const float AC_MAX = .5f; // max linear acceleration is .5 m/s^2
const float WC_MAX = 2.0f; // max angular acceleration is 2 rad/s^2
const float S_MAX = .25f; // max stopping distance .25m by V_max^2 / (2a_max)
const float R_ROBOT = .18f; // radius of robot is .18m
const float V_CRIT = std::sqrt(2.0f * AC_MAX * S_MAX); // critical velocity
const float SW_MAX = std::pow(W_MAX, 2.0f) / (2.0f * WC_MAX); // max stopping angle
const float W_CRIT = std::sqrt(2.0f * WC_MAX * SW_MAX); // critical angular velocity

const Eigen::Matrix3f M_ROTATION = (Eigen::Matrix3f() << 1.f, 0.f, 0.f,
																												 0.f, 1.f, 0.f,
																												 0.f, 0.f, 1.f).finished();

const Eigen::MatrixXf M_TRANSLATION = (Eigen::MatrixXf(3,1) << .145f, 0.f, 0.23f).finished();


namespace t_helpers
{

class ObstacleInfo
{

private:
	geometry_msgs::Point32 p_point;
	float p_f;
	float p_m;
	float p_final_angle;

public:
	ObstacleInfo(geometry_msgs::Point32 p, float f_p, float m, float f_ang) : 
		p_point(p), p_f(f_p), p_m(m), p_final_angle(f_ang) {}

	ObstacleInfo() = default;

	void setpoint(geometry_msgs::Point32 p)
	{
		p_point = p;
	}

	void setf(float f)
	{
		p_f = f;
	}

	void setmargin(float m)
	{
		p_m = m;
	}

	void setfinal_angle(float f_ang)
	{
		p_final_angle = f_ang;
	}

	geometry_msgs::Point32 point()
	{
		return p_point;
	}

	float f()
	{
		return p_f;
	}

	float margin()
	{
		return p_m;
	}

	float final_angle()
	{
		return p_final_angle;
	}
};

/*
	Constrain an angle between 0, 2pi
*/
float ConstrainAngle(float x)
{
	x = fmod(x, 2 * M_PI);
	if (x < 0)
		x += 2 * M_PI;

	return x;
}

/*
	@param p 2x1 Vector input point
	@param v velocity scalar
	@param w angular velocity scalar
	@param out_f output free path scalar
	@param out_margin output margin between free path center and point
	@param out_finangle output final angle
*/
bool PointIsObstacle(Eigen::Vector2f p, float v, float w, float *out_f, float *out_margin, float *out_finangle)
{
	if (!w) // 0 angular vel, calculate straight line free path to p
	{	// let robot be moving along x axis

		// if the y value of p is within radius of robot, it is an obstacle
		if (abs(p.y()) <= R_ROBOT && signbit(p.x()) == signbit(v))
		{
			// since p is obstacle, compute free path length
			float f = p.x() - sqrt(pow(R_ROBOT, 2.0f) - pow(p.y(), 2.0f));

			ROS_DEBUG("PointIsObstacle: p.y: %f is less than R_ROBOT: %f, free path length: %f", p.y(), R_ROBOT, f);

			*out_f = f;

			return true; // free path and obstacle found with 0 angular velocity
		} //end if (abs(p.y() < R_ROBOT)
		else // not an obstacle
		{
			ROS_DEBUG("PointIsObstacle: p.y: %f is greater than R_ROBOT: %f or sign(px)%d =sign(v) %d, not obstacle", 
							p.y(), R_ROBOT, signbit(p.x()), signbit(v));

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

		pcl = std::abs(pcl);

		float lco = pco - pcl; // free path angle

		float f = lco * r; // free path arclength = free path angle * radius of rotation

		ROS_DEBUG("PointIsObstacle: p.x:%f, p.y:%f, pco: %f, pcl: %f, lco: %f, f: %f", 
			p.x(), p.y(), pco, pcl, lco, f);

		// calculate angle error margin from straight liine angle and predicted angle

		Eigen::Vector2f op_trans = p + p; // translate vectors to p to calc angle
		Eigen::Vector2f cp_trans = p + cp;

		float phi = acos((cp_trans).dot(op_trans) / (cp_trans.norm() * op_trans.norm()));

		if (phi >= 0)
		{
			*out_finangle = lco - phi;
		}
		else
		{
			*out_finangle = ConstrainAngle(lco + phi);
		}

		*out_f = f;
		*out_margin = p_dist_from_robot;

		return true; // free path obstacle found along curve
		} // end if p_dist_from_robot < R_ROBOT
		else // not an obstacle
		{
			return false; // no obstacle along path
		} // end else
	} // end else
} // end PointIsObstacle

/*
	@param p 2x1 Vector input point
	@param v velocity scalar
	@param w angular velocity scalar
	@param out_f output free path scalar
*/
bool PointIsObstacle(Eigen::Vector2f p, float v, float w, float *out_f)
{
	float margin_dontcare;
	float angle_dontcare;

	return PointIsObstacle(p, v, w, out_f, &margin_dontcare, &angle_dontcare);
}

visualization_msgs::MarkerArray GenPointListMarkers(const sensor_msgs::PointCloud all_pts, 
																							vector< ObstacleInfo > obstacle_pts,
																							const string frame_id  )
{
	visualization_msgs::MarkerArray arr;

	static size_t id = 0;
	visualization_msgs::Marker m;
	if (all_pts.points.size()){
		m.header.frame_id = frame_id;
		m.header.stamp = ros::Time();
		m.ns = "t_helpers";
		m.id = id ++;
		m.type = visualization_msgs::Marker::POINTS;
		m.action = visualization_msgs::Marker::ADD;
		m.scale.x = .02f;
		m.scale.y = .02f;
		m.color.a = 1.f;
		m.color.r = 0.f;
		m.color.g = 1.f;
		m.color.b = 0.f;

		// convert all_points.points to point64
		for (vector<geometry_msgs::Point32>::const_iterator it = all_pts.points.begin(); it != all_pts.points.end(); it ++)
		{
			geometry_msgs::Point p64; 
			p64.x = static_cast<double>(it->x);
			p64.y = static_cast<double>(it->y);
			p64.z = 0.0f;
			m.points.push_back(p64);
		}

		arr.markers.push_back(m); // add all points to marker vector
	}
	if (obstacle_pts.size())
	{
		m = visualization_msgs::Marker();
		m.header.frame_id = frame_id;
		m.header.stamp = ros::Time();
		m.ns = "t_helpers.obstacle";
		m.id = id ++;
		m.type = visualization_msgs::Marker::POINTS;
		m.action = visualization_msgs::Marker::ADD;
		m.scale.x = .02f;
		m.scale.y = .02f;
		m.color.a = 1.f;
		m.color.r = 1.f;
		m.color.g = 0.f;
		m.color.b = 0.f;

		for (vector< ObstacleInfo >::iterator it = obstacle_pts.begin(); it != obstacle_pts.end(); it ++)
		{
			geometry_msgs::Point32 p = it->point();
			geometry_msgs::Point p64;
			p64.x = static_cast<double>(p.x);
			p64.y = static_cast<double>(p.y);
			p64.z = 0.0f;
			remove_if(arr.markers[0].points.begin(), arr.markers[0].points.end(), 
								[p64](geometry_msgs::Point &otherp){return otherp.x == p64.x && otherp.y == p64.y;});
			m.points.push_back(p64);
		}

		arr.markers.push_back(m);
	}

	return arr;
}

/*
	Generate a dynamic discretized set of viable (v,w) values given initial velocity
	and how many subdivisions to make. Returns on average subdivisions^2 grid values.
	Only returns admissible velocities within critical velocities.
	Guaranteed to return a window containing at least v_0.
	@param v_0 input linear/angular velocity
	@param subdivisions amount of discretizations per row and per col.
	@returns a list of discretized possible linear/angular velocity values
*/
inline vector<Eigen::Vector2f> GenDiscDynWind (Eigen::Vector2f v_0, int subdivisions)
{ // v_0.x = linear vel, v_0.y = angular vel
	vector<Eigen::Vector2f> discretized;

	// calc left window bound, must be within the critical window
	float window_temp = -WC_MAX * TIME_DELTA + v_0.y();
	float window_left = window_temp >= -W_CRIT ? window_temp : -W_CRIT;

	// calc top window bound
	window_temp = AC_MAX * TIME_DELTA + v_0.x();
	float window_top = window_temp <= V_CRIT ? window_temp : V_CRIT;

	// calc right window bound
	window_temp = WC_MAX * TIME_DELTA + v_0.y();
	float window_right = window_temp <= W_CRIT ? window_temp : W_CRIT;

	// calc bottom window bound
	window_temp = -AC_MAX * TIME_DELTA + v_0.x();
	float window_bottom = window_temp >= 0 ? window_temp : 0;

	float row_step = (window_top - window_bottom) / subdivisions;
	float col_step = (window_right - window_left) / subdivisions;

	// go bottom to top, left to right, increment v and w by step
	for (float v = window_bottom; v < window_top; 
			v =	v + row_step < window_top ? v + row_step : window_top) // always include top
	{
		for (float w = window_left; w < window_right; 
			w = w + col_step < window_right ? w + col_step : window_right) // always include bottom
		{
			Eigen::Vector2f disc_val (v, w);
			discretized.push_back(disc_val);
		}
	}

	discretized.push_back(v_0); // add initial velocity, just in case.

	#ifdef GTEST
	std::cerr << "Discretized " << discretized.size() << "[" << window_left <<
				"," << window_top << "," << window_right << "," << window_bottom << "]" << 
				endl;
	#endif

	ROS_DEBUG("GenDiscDynWind: generated %ld values. window: [%f,%f,%f,%f]",
						discretized.size(), window_left, window_top, window_right, window_bottom);

	return discretized;
}


/*
	@param pc input PointCloud
	@param v velocity scalar
	@param w angular velocity scalar
	@param out_pointmap output vector of points to their free path distance
	@param out_closest output closest point and free path distance
	@param out_margin output margin
*/
bool ObstacleExist(const sensor_msgs::PointCloud pc, const float v, const float w,
					vector< ObstacleInfo > &out_pointmap, 
					ObstacleInfo &out_closest)
{
	bool obstacle = false; // true if not obstacle free
	float min_f = numeric_limits<float>::max(); // keep track of min free path len
	float min_margin = numeric_limits<float>::max();
	float min_finangle = numeric_limits<float>::max();

	geometry_msgs::Point32 closest_pt;

	for (vector<geometry_msgs::Point32>::const_iterator it = pc.points.begin(); it != pc.points.end(); 
				it++) // iterate over all converted points to find obstacle
	{
		float temp_f;
		float temp_margin;
		float temp_finangle;
		bool temp_obstacle = PointIsObstacle(Eigen::Vector2f(it->x, it->y), 
																		v, w, &temp_f, &temp_margin, &temp_finangle);
		if (temp_obstacle) // found an obstacle
		{
			obstacle = true;

			out_pointmap.push_back(ObstacleInfo(*it, temp_f, temp_margin, temp_finangle)); // push to output vector

			if (temp_f < min_f) // obstacle is closer than current closest
			{
				ROS_DEBUG("ObstacleExist: Found an obstacle: %f, cur min: %f", temp_f, min_f);
				closest_pt = *it;
				min_f = temp_f;
				min_margin = temp_margin;
				min_finangle = temp_finangle;
			}
		}
	}

	if (obstacle) // at least one obstacle was found
	{
		ROS_DEBUG("ObstacleExist: At least 1 obstacle: %f", min_f);
		out_closest.setpoint(closest_pt);
		out_closest.setf(min_f);
		out_closest.setmargin(min_margin);
		out_closest.setfinal_angle(min_finangle);
		return true;
	}
	else // no obstacles along the path
	{
		ROS_DEBUG("ObstacleExist: No obstacles found along path");
		return false;
	}
}

/*
	Convenience function to output only the free path distance, discarding other output.
*/
bool ObstacleExist(const sensor_msgs::PointCloud pc, const float v, const float w, float *out_f)
{
	vector< ObstacleInfo > dont_care;
	ObstacleInfo closest_pt;

	bool obstacle = ObstacleExist(pc, v, w, dont_care, closest_pt);

	if (obstacle)
	{
		*out_f = closest_pt.f();
	}

	return obstacle;
}

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

  ROS_DEBUG("Angle_min: %f, angle max: %f, angle_increment: %f, range_min: %f, range_max: %f, ranges_size: %lu",
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

      #ifdef GTEST
      ROS_DEBUG("X_%lu: %f,Y_%lu: %f", i, x_i, i, y_i);
      #endif

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