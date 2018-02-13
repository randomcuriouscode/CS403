#include <algorithm>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <vector>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>

#include "compsci403_assignment2/Get3DPointFromDepthSrv.h"
#include "compsci403_assignment2/Get3DPointFromDisparitySrv.h"
#include "compsci403_assignment2/GetDepthFromDisparitySrv.h"
#include "compsci403_assignment2/GetIntrinsicsSrv.h"
#include "compsci403_assignment2/GetPixelFrom3DPointSrv.h"

// Include any additional header or service files

// Declare class variables, messages, publishers

ros::ServiceServer g_GetPixelFrom3DPointSrv; // Service /COMPSCI403/GetPixelFrom3DPoint
ros::ServiceServer g_Get3DPointFromDepthSrv; // Service /COMPSCI403/Get3DPointFromDepth
ros::ServiceServer g_GetDepthFromDisparitySrv; // Service /COMPSCI403/GetDepthFromDisparity
ros::ServiceServer g_Get3DPointFromDisparitySrv; // Service /COMPSCI403/Get3DPointFromDisparity

ros::Publisher g_PointCloudPub;

ros::Subscriber g_DepthImageSub;

ros::ServiceClient g_GetIntrinsicsCl; // Client /COMPSCI403/GetIntrinsics

float g_fx, g_fy, g_px, g_py, g_a, g_b;

// Define service and callback functions

/*
1.
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
    ROS_ERROR("GetPixelFrom3DPointCallback recieved a Z=0");
    return false;
  }

  return true;
}

/*
  2.
*/
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

/*
  3.
*/
bool GetDepthFromDisparityCallback(compsci403_assignment2::GetDepthFromDisparitySrv::Request &req,
                                    compsci403_assignment2::GetDepthFromDisparitySrv::Response &res)
{
  int32_t disparity = req.disparity;
  float a = req.a;
  float b = req.b;

  float denom = (a + (b * disparity));

  ROS_DEBUG("GetDepthFromDisparityCallback(): disparity: %i, a: %f, b:%f, denom: %f", 
              disparity, a, b ,denom);

  if (denom != 0)
  {
    res.depth = 1 / denom; // d = 1 / (a + br)
  }
  else
  {
    ROS_ERROR("GetDepthFromDisparityCallback(): divide by zero denominator");
    return false;
  }
  
  return true;
}

/*
  4.
*/
bool Get3DPointFromDisparityCallback(compsci403_assignment2::Get3DPointFromDisparitySrv::Request &req,
                                      compsci403_assignment2::Get3DPointFromDisparitySrv::Response &res)
{
  int32_t x = req.x, y = req.y, disparity = req.disparity;
  float fx = req.fx, fy = req.fy, px = req.px, py = req.py,
              a = req.a, b = req.b;

  int XZ_ratio = (x - px) / fx; // gives X/Z
  int YZ_ratio = (y - py) / fy; // gives Y/Z

  float denom = (a + (b * disparity));

  int depth = 0;
  if (denom != 0)
  {
    depth = 1 / denom; // d = 1 / (a + br)
  }
  else
  {
    ROS_ERROR("Get3DPointFromDisparityCallback(): divide by zero denominator");
    return false;
  }

  res.X = XZ_ratio * depth;
  res.Y = YZ_ratio * depth;
  res.Z = depth;

  return true;
}

/*
  5.
*/
void DepthImageCallback(const sensor_msgs::Image& image) {
  const int image_width = image.width;
  const int image_height = image.height;

  sensor_msgs::PointCloud pc; // create pointcloud

  for (int y = 0; y < image_height; ++y) {
    for (int x = 0; x < image_width; ++x) {
      uint16_t byte0 = image.data[2 * (x + y * image_width) + 0];
      uint16_t byte1 = image.data[2 * (x + y * image_width) + 1];
      if (!image.is_bigendian) {
        std::swap(byte0, byte1);
      }
      // Combine the two bytes to form a 16 bit value, and disregard the
      // most significant 4 bits to extract the lowest 11 bits.
      const uint16_t raw_depth = ((byte0 << 8) | byte1) & 0x7FF; // raw depth disparity

      // Reconstruct 3D point from x, y, raw_depth, and sensor intrinsics

      float denom = (g_a + (g_b * (float)raw_depth));

      ROS_DEBUG("DepthImageCallback(): disparity: %i, a: %f, b:%f, denom: %f", 
                  raw_depth, g_a, g_b , denom);

      if (denom == 0)
      {
        ROS_ERROR("GetDepthFromDisparityCallback(): divide by zero denominator");
      }

      float depth = 1 / denom; // calculate depth from kinect formula

      int XZ_ratio = (x - g_px) / g_fx; // gives X/Z
      int YZ_ratio = (y - g_py) / g_fy; // gives Y/Z

      geometry_msgs::Point32 p; // create point
      p.x = XZ_ratio * depth;
      p.y = YZ_ratio * depth;
      p.z = depth;

      pc.points.push_back(p); // add to point cloud points vector
    }
  }

  g_PointCloudPub.publish(pc); // publish the point cloud
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "assignment2");
  ros::NodeHandle n;

  // Perform operations defined in Assignment 2

  // 1. Provide service named /COMPSCI403/GetPixelFrom3DPoint

  g_GetPixelFrom3DPointSrv = n.advertiseService("/COMPSCI403/GetPixelFrom3DPoint", GetPixelFrom3DPointCallback);

  // 2. Provide service named /COMPSCI403/Get3DPointFromDepth
  g_Get3DPointFromDepthSrv = n.advertiseService("/COMPSCI403/Get3DPointFromDepth", Get3DPointFromDepthCallback);

  // 3. Provide service named /COMPSCI403/GetDepthFromDisparity
  g_GetDepthFromDisparitySrv = n.advertiseService("/COMPSCI403/GetDepthFromDisparity", GetDepthFromDisparityCallback);

  // 4. Provide service named /COMPSCI403/Get3DPointFromDisparity
  g_Get3DPointFromDisparitySrv = n.advertiseService("/COMPSCI403/Get3DPointFromDisparity", Get3DPointFromDepthCallback);

  // 5a. Create client to /COMPSCI403/GetIntrinsics
  g_GetIntrinsicsCl = n.serviceClient<compsci403_assignment2::GetIntrinsicsSrv>("/COMPSCI403/GetIntrinsics");

  compsci403_assignment2::GetIntrinsicsSrv pcp_payload;

  if (g_GetIntrinsicsCl.call(pcp_payload))
  {
    g_fx = (float)pcp_payload.response.fx;
    g_fy = (float)pcp_payload.response.fy;
    g_px = (float)pcp_payload.response.px;
    g_py = (float)pcp_payload.response.py;
    g_a = (float)pcp_payload.response.a;
    g_b = (float)pcp_payload.response.b;

    ROS_INFO("DepthImageCallback()::GetIntrinsics(): fx: %f, fy: %f, px: %f, py: %f, a: %f, b: %f", 
      g_fx, g_fy, g_px, g_py, g_a, g_b);
  }
  else
  {
    ROS_ERROR("DepthImageCallback(): Failed to call COMPSCI403/GetIntrinsics");
  }

  // 5b. Create publisher to /COMPSCI403/PointCloud
  g_PointCloudPub = n.advertise<sensor_msgs::PointCloud>("COMPSCI403/PointCloud", 1000);

  // 5c. Subscribe to topic named /COMPSCI403/DepthImage
  g_DepthImageSub = n.subscribe("/COMPSCI403/DepthImage", 1000, DepthImageCallback);
  
  ros::spin();

  return 0;
}
