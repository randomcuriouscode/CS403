#pragma once

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

using namespace std;

typedef pair<geometry_msgs::Point32, float> pointdistpair;


namespace t_helpers
{

bool PointIsObstacle(Eigen::Vector2f p, float v, float w, float *out_f);

visualization_msgs::MarkerArray GenPointListMarkers(const sensor_msgs::PointCloud all_pts, 
																							const vector< pointdistpair > obstacle_pts,
																							const string frame_id  );

inline vector<Eigen::Vector2f> GenDiscDynWind (Eigen::Vector2f v_0, int subdivisions);


bool ObstacleExist(const sensor_msgs::PointCloud pc, const float v, const float w,
					vector< pointdistpair > &out_pointmap, 
					pointdistpair &out_closest);

bool ObstacleExist(const sensor_msgs::PointCloud pc, const float v, const float w, float *out_f);

bool PointIsObstacle(Eigen::Vector2f p, float v, float w, float *out_f);

void LaserScanToPointCloud(const sensor_msgs::LaserScan &msg, sensor_msgs::PointCloud &pc);

void ProjectRangeFinderToRobotRef(const sensor_msgs::LaserScan &msg, sensor_msgs::PointCloud &translated_pc);

}