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

// Define service and callback functions

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

  ros::spin();

  return 0;
}
