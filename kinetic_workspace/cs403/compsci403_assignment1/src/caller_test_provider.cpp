#include <iostream>
#include <ros/ros.h>
#include <sstream>

#include <std_msgs/String.h>

#include "compsci403_assignment1/Assignment1CallerSrv.h"

using std::cout;
using std::string;


bool CallerTestService(
    compsci403_assignment1::Assignment1CallerSrv::Request& req,
    compsci403_assignment1::Assignment1CallerSrv::Response& res) {
  res.str = "Test string";
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "compsci403_assignment1_caller_test");
  ros::NodeHandle n;

  ros::ServiceServer caller_test = n.advertiseService(
      "/COMPSCI403/CallerTest", CallerTestService);

  ros::spin();

  return 0;
}
