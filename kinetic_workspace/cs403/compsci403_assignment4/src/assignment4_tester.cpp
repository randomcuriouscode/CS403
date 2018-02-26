#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <cstdlib>
#include <random>
#include <iostream>

#include "cobot_msgs/CobotDriveMsg.h"
#include "compsci403_assignment4/ObstacleMsg.h"
#include "compsci403_assignment4/CheckPointSrv.h"
#include "compsci403_assignment4/GetFreePathSrv.h"
#include "compsci403_assignment4/GetCommandVelSrv.h"

#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;

// Include any additional header or service/message files

// Declare class variables, subscribers, publishers, messages

ros::ServiceClient g_GetFreePathCl;

const uint32_t num_readings = 100;
const float laser_frequency = 40;

// Define service and callback functions

// pub random laser scan for pt 2
sensor_msgs::LaserScan GenRandomLaserScan(string frame_id)
{
	float ranges[num_readings];
	float intensities[num_readings] = {0.f};

	random_device rd;
	mt19937 gen(rd());
	std::uniform_real_distribution<float> rdm(0.019999999553f, 4.0f);

	for (size_t i = 0; i < num_readings; i ++)
	{
		ranges[i] = rdm(gen);
	}

	ros::Time scan_time = ros::Time::now();

	sensor_msgs::LaserScan scan;
	scan.header.stamp = scan_time;
	scan.header.frame_id = frame_id;
	scan.angle_min = -1.57f;
	scan.angle_max = 1.57f;
	scan.angle_increment = M_PI / num_readings;
	scan.time_increment = (1 / laser_frequency) / (num_readings);
	scan.range_min = 0.25f;
	scan.range_max = 4.0f;

	scan.ranges.resize(num_readings);
	scan.intensities.resize(num_readings);
	for(size_t i = 0; i < num_readings; ++i){
	 scan.ranges[i] = ranges[i];
	 scan.intensities[i] = intensities[i];
	}

	return scan;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "assignment4_tester");
  ros::NodeHandle n;

	// Perform operations defined in Assignment 4

  if (argc > 1)
  {
  	int option = atoi(argv[1]);

  	switch (option)
  	{
  		case 2:
  		{
  		g_GetFreePathCl = n.serviceClient<compsci403_assignment4::GetFreePathSrv>("/COMPSCI403/GetFreePath");
  		float v = atof(argv[2]);
  		float w = atof(argv[3]);

  		string input;
  		while (ros::ok())
  		{
        getline(cin, input);
  			if (input.find("g") != string::npos)
  			{
  				// call the GetFreePath service
  				compsci403_assignment4::GetFreePathSrv msg;
  				msg.request.laser_scan = GenRandomLaserScan("/base_footprint");
  				msg.request.v = v;
  				msg.request.w = w;

  				auto res = g_GetFreePathCl.call(msg);

  				if (res)
  				{
  					ROS_INFO("SVC RETURNED %d, %f:", msg.response.is_obstacle, msg.response.free_path_length);
  				}
  				else
  				{
  					ROS_ERROR("Error occurred with GetFreePath");
  				}		
  			}
  		}
  		break;
  		}
		default:
 		ROS_INFO("Option Invalid");
 		return(0);
  	}

  }

	ros::spin();

  return(0);
}