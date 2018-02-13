#include <algorithm>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>

#include "compsci403_assignment2/Get3DPointFromDepthSrv.h"
#include "compsci403_assignment2/Get3DPointFromDisparitySrv.h"
#include "compsci403_assignment2/GetDepthFromDisparitySrv.h"
#include "compsci403_assignment2/GetIntrinsicsSrv.h"
#include "compsci403_assignment2/GetPixelFrom3DPointSrv.h"

// Include any additional header or service files

// Declare class variables, messages, publishers

ros::ServiceServer g_GetPixelFrom3DPointSrv; // Service /COMPSCI403/GetPixelFrom3DPoint
ros::ServiceServer g_Get3DPointFromDepthSrv; // Service /COMPSCI403/Get3DPointFromDepth


// Define service and callback functions

/*
 The coordinates of the 3D point and the pin-hole camera intrinsics will be the 
 inputs, and the pixel coordinates of the image point will be the output of the service
*/
bool GetPixelFrom3DPointCallback(compsci403_assignment2::GetPixelFrom3DPointSrv::Request &req,
                                 compsci403_assignment2::GetPixelFrom3DPointSrv::Response &res)
{
  float X = req.X, Y = req.Y, Z = req.Z, 
        fx = req.fx, fy = req.fy, px = req.px,
        py = req.py;

  ROS_DEBUG("GetPixelFrom3DPointCallback(): X: %f, Y: %f, Z: %f, fx: %f fy: %f, px: %f, py: %f", 
            X, Y, Z, fx, fy, px, py);

  if(Z != 0)
  {
    res.x = fx * X / Z + px;
    res.y = fy * Y / Z + py;
  }
  else
  {
    return false;
  }

  return true;
}

bool Get3DPointFromDepthCallback(compsci403_assignment2::Get3DPointFromDepthSrv::Request &req,
                                  compsci403_assignment2::Get3DPointFromDepthSrv::Response &res)
{
  float x = req.x, y = req.y, depth = req.depth, fx = req.fx,
        fy = req.fy, px = req.px, py = req.py;

  int XZ_ratio = (x - px) / fx; // gives X/Z
  int YZ_ratio = (y - py) / fy; // gives Y/Z
  
  ROS_DEBUG("Get3DPointFromDepthCallback(): x: %f, y: %f, depth: %f, fx: %f, fy: %f, px: %f, py: %f",
            x, y, depth, fx, fy, px, py);

  res.X = XZ_ratio * depth;
  res.Y = YZ_ratio * depth;
  res.Z = depth;

  return true;
}

void DepthImageCallback(const sensor_msgs::Image& image) {
  const int image_width = image.width;
  const int image_height = image.height;
  for (int y = 0; y < image_height; ++y) {
    for (int x = 0; x < image_width; ++x) {
      uint16_t byte0 = image.data[2 * (x + y * image_width) + 0];
      uint16_t byte1 = image.data[2 * (x + y * image_width) + 1];
      if (!image.is_bigendian) {
        std::swap(byte0, byte1);
      }
      // Combine the two bytes to form a 16 bit value, and disregard the
      // most significant 4 bits to extract the lowest 12 bits.
      const uint16_t raw_depth = ((byte0 << 8) | byte1) & 0x7FF;

      // Reconstruct 3D point from x, y, raw_depth, and sensor intrinsics

    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "assignment2");
  ros::NodeHandle n;

  // Perform operations defined in Assignment 2

  // 1. Provide service named /COMPSCI403/GetPixelFrom3DPoint

  g_GetPixelFrom3DPointSrv = n.advertiseService("/COMPSCI403/GetPixelFrom3DPoint", GetPixelFrom3DPointCallback);

  // 2. Provide service named /COMPSCI403/Get3DPointFromDepth
  g_Get3DPointFromDepthSrv = n.advertiseService("/COMPSCI403/Get3DPointFromDepth", Get3DPointFromDepthCallback);

  ros::spin();

  return 0;
}
