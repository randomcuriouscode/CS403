#include <algorithm>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

#include "compsci403_assignment2/GetIntrinsicsSrv.h"

namespace kinect_intrinsics {
  float a = 3.008;
  float b = -0.002745;
  float p_x = 320.0;
  float p_y = 240.0;
  float f_x = 588.446;
  float f_y = -564.227;
}  // namespace kinect_intrinsics

bool GetIntrinsicsService(
    compsci403_assignment2::GetIntrinsicsSrv::Request& req,
    compsci403_assignment2::GetIntrinsicsSrv::Response& res) {
  res.fx = kinect_intrinsics::f_x;
  res.fy = kinect_intrinsics::f_y;
  res.px = kinect_intrinsics::p_x;
  res.py = kinect_intrinsics::p_y;
  res.a = kinect_intrinsics::a;
  res.b = kinect_intrinsics::b;
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "kinect_intrinsics_service");
  ros::NodeHandle n;

  ros::ServiceServer get_intrisics_service =
    n.advertiseService("/COMPSCI403/GetIntrinsics", GetIntrinsicsService);

  ros::spin();

  return 0;
}
