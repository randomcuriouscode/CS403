#include <iostream>
#include <ros/ros.h>
#include <sstream>

#include <std_msgs/String.h>

#include "compsci403_assignment1/Assignment1CallerSrv.h"
#include "compsci403_assignment1/Assignment1ProviderSrv.h"

// Include any additional header or service files

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <vector>
#include <cmath>

using std::cout;
using std::string;

// Declare class variables and publishers
std_msgs::String string_to_publish_;

ros::Publisher string_publisher_;

ros::Publisher pointPub; 

// Define service and callback functions 

bool add(compsci403_assignment1::Assignment1ProviderSrv::Request &req,
					compsci403_assignment1::Assignment1ProviderSrv::Response &res)
{
	res.result = req.x + req.y;
	ROS_INFO("Request: x=%f, y=%f", req.x, req.y);
	ROS_INFO("Response: %f", res.result);
	return true;
}

void scanOccuredCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	float angle_min = msg->angle_min;
  float angle_max = msg->angle_max;
  float angle_increment = msg->angle_increment;
  float range_min = msg->range_min;
  float range_max = msg->range_max;
  std::vector<float> ranges = msg->ranges;

  std::size_t m = ranges.size();

  ROS_INFO("Angle_min: %f, angle max: %f, angle_increment: %f, range_min: %f, range_max: %f, ranges_size: %d",
      angle_min, angle_max, angle_increment, range_min, range_max, m);

  sensor_msgs::PointCloud pc;
  pc.header = msg->header;

  float cur_angle = angle_min;

  for(std::size_t i = 0; i < m; ++i){
    if (ranges[i] < range_max && ranges[i] > range_min){
      float x_i = ranges[i] * cos(cur_angle);
      float y_i = ranges[i] * sin(cur_angle);

      geometry_msgs::Point32 p;
      p.x = x_i;
      p.y = y_i;

      ROS_INFO("X_%d: %f,Y_%d: %f", i, x_i, i, y_i);

      pc.points.push_back(p);
    }
    cur_angle += angle_increment;
  }

  pointPub.publish(pc); // publish pointcloud to /COMPSCI403/PointCloud
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "assignment1");
  ros::NodeHandle n;

  // Perform operations defined in Assignment 1

  // 1. call /COMPSCI403/CallerTest

  ros::ServiceClient callerTestClient = 
  	n.serviceClient<compsci403_assignment1::Assignment1CallerSrv>("COMPSCI403/CallerTest");

  compsci403_assignment1::Assignment1CallerSrv callTestSrv;

  std::string resp = "";

  if (callerTestClient.call(callTestSrv))
  {
  	 resp = (std::string)callTestSrv.response.str;
  	 resp = resp.substr(0, 8); // extract first 8 chars
  	 ROS_INFO("CallerTest Resp: %s", resp.c_str());
  }
  else
  {
  	ROS_ERROR("Failed to call COMPSCI403/CallerTest");
  }

  pointPub = n.advertise<sensor_msgs::PointCloud>("COMPSCI403/PointCloud", 1000);

  // 1. create publisher to PublishTest
  ros::Publisher testPub = n.advertise<std_msgs::String>("COMPSCI403/PublishTest", 1000);

  std_msgs::String testMsg;
  std::stringstream ss;
  ss << resp;
  testMsg.data = ss.str();
  
  // 2. create service /COMPSCI403/ProviderTest

  ros::ServiceServer providerService = n.advertiseService("COMPSCI403/ProviderTest", add);
  ROS_INFO("/COMPSCI403/ProviderTest/ service created.");

  // 3. subscribe to /COMPSCI403/LaserScan

  ros::Subscriber sub = n.subscribe("COMPSCI403/LaserScan", 1000, scanOccuredCallback);

  ros::Rate loop(10);

  while (ros::ok()) {
		ROS_INFO("Publishing %s to COMPSCI403/PublishTest", testMsg.data.c_str());

		testPub.publish(testMsg); // 1. publish
    ros::spinOnce();
    loop.sleep();
  }

  return 0;
}
