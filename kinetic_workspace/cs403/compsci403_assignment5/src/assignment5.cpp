#include "assignment5.h"

using Eigen::Matrix3f;
using Eigen::MatrixXf;
using Eigen::MatrixXd;
using Eigen::Vector3f;
using Eigen::Vector2f;
using geometry_msgs::Point32;
using geometry_msgs::Point;
using geometry_msgs::Twist;
using nav_msgs::Odometry;
using visualization_msgs::Marker;
using sensor_msgs::LaserScan;
using sensor_msgs::PointCloud;
using std::cout;
using std::vector;
using namespace std;

// Global Parameters
const float gRobotRadius = 0.18;
const float gRobotHeight = 0.36;
const float gEpsilon = 0.15;
const float gAMaxLin = 1 * 0.5; // m/s^2
const float gAMaxRot = 1 * 2.0; // rad/s^2
const float gVMaxLin = 0.5; // m/s
const float gVMaxRot = 1.5; // rad/s
const float gDT = 0.02; // s

ros::ServiceServer g_CheckPointSrv; // Service /COMPSCI403/CheckPoint
ros::ServiceServer g_GetFreePathSrv; // Service /COMPSCI403/GetFreePath
ros::ServiceServer g_ObstacleLaserScanSrv; // Service /COMPSCI403/ObstacleLaserScan
ros::ServiceServer g_GetCommandVelSrv; // Service /COMPSCI403/GetCommandVel

ros::Publisher g_VisPub; // For Visualizations

static sensor_msgs::PointCloud g_TranslatedPC; // store translated point cloud from result of part 3
                                               // could make a class wrapper for access, but not sure
                                               // how to deal with multiple copies occurring on stack
static int32_t DISCRETIZATIONS = 10; // total discretizations >= DISCRETIZATIONS^2

static Eigen::Vector2f g_v; // x: linear y: angular
static Eigen::Vector2f g_robotPos; // current position of the robot, use to transform scan to origin


/// 1. Provide a service named /COMPSCI403/CheckPoint of type CheckPointSrv
bool CheckPointCallback (compsci403_assignment5::CheckPointSrv::Request &req,
                         compsci403_assignment5::CheckPointSrv::Response &res)
{
  Eigen::Vector2f p (req.P.x, req.P.y); // z value is always 0, 2d point

  float f = -1.0f;

  bool obstacle = t_helpers::PointIsObstacle(p, req.v, req.w, &f);

  stringstream logstr;
  logstr << p;

  if (obstacle)
  {
    ROS_DEBUG("CheckPointCallback: point (%s) OBSTACLE for w: %f, v: %f, f: %f", logstr.str().c_str(), 
              req.v, req.w, f);
    res.is_obstacle = true;
    res.free_path_length = f;
  }
  else
  {
    ROS_DEBUG("CheckPointCallback: point (%s) NOT OBSTACLE for w: %f, v: %f", logstr.str().c_str(), req.v, req.w);
    res.is_obstacle = false;
  }

  return true;
} // end CheckPointCallback

// 2.
bool GetFreePathCallback (compsci403_assignment5::GetFreePathSrv::Request &req,
                          compsci403_assignment5::GetFreePathSrv::Response &res)
{
  sensor_msgs::PointCloud pc;
  // laser scan is already in the reference frame of the robot, just convert it to a pointcloud
  t_helpers::LaserScanToPointCloud(req.laser_scan, pc); 

  vector< t_helpers::ObstacleInfo > obstacles;
  t_helpers::ObstacleInfo closest_pt;
  bool obstacle = t_helpers::ObstacleExist(pc, req.v, req.w, obstacles, closest_pt); // true if not obstacle free

  // publish to visualization_marker_array
  g_VisPub.publish(t_helpers::GenPointListMarkers(pc, obstacles, "/base_footprint"));

  if (obstacle) // at least one obstacle was found
  {
    ROS_DEBUG("GetFreePathCallback: At least 1 obstacle: %f", closest_pt.f());
    res.is_obstacle = true;
    res.free_path_length = closest_pt.f();
  }
  else // no obstacles along the path
  {
    ROS_DEBUG("GetFreePathCallback: No obstacles found along path");
    res.is_obstacle = false;
  }

  return true;
} // end GetFreePathCallback

// 3.
bool ObstacleLaserScanCallback(compsci403_assignment5::ObstacleLaserScanSrv::Request &req,
                               compsci403_assignment5::ObstacleLaserScanSrv::Response &res)
{
  Eigen::Matrix3f R;

  for (size_t row = 0; row < 3; row ++)
  {
    for (size_t col = 0; col < 3; col ++)
    {
      R(row,col) = req.R[3 * row + col];
    }
  }

  Eigen::Vector3f T(req.T.x, req.T.y, req.T.z);

  sensor_msgs::PointCloud translated_pc;

  t_helpers::ProjectRangeFinderToRobotRef(req.S, R, T, translated_pc);

  // TODO wait for TA response on this, translate pc back to laserscan?

  sensor_msgs::LaserScan s_prime;
  s_prime.header = req.S.header;
  s_prime.header.frame_id = "/base_footprint";
  s_prime.angle_min = req.S.angle_min;
  s_prime.angle_max = req.S.angle_max;
  s_prime.time_increment = req.S.time_increment;
  s_prime.scan_time = req.S.scan_time;
  s_prime.range_min = req.S.range_min;
  s_prime.range_max = req.S.range_max;

  g_TranslatedPC = translated_pc;

  t_helpers::PointCloudToLaserScan(translated_pc, s_prime);

  res.S_prime = s_prime;

  return true;
}

// 4.
bool GetCommandVelCallback (compsci403_assignment5::GetCommandVelSrv::Request &req,
                            compsci403_assignment5::GetCommandVelSrv::Response &res)
{
  if (g_TranslatedPC.points.size() == 0)
  {
    ROS_ERROR("GetCommandVelCallback: FATAL ERROR, /Cobot/Laser was NOT published to before this call. PointCloud global size is 0");
    return true; // TODO: this should be return false by ROS spec, but instructor says false is bad.
  }

  sensor_msgs::PointCloud translated_pc = g_TranslatedPC;

  vector<Eigen::Vector2f> disc_window = t_helpers::GenDiscDynWind(Eigen::Vector2f(req.v_0, req.w_0), DISCRETIZATIONS);
  
  Eigen::Vector2f best_vel = Eigen::Vector2f(req.v_0, req.w_0);
  float best_score = numeric_limits<float>::min();

  for (auto it_wind = disc_window.begin(); it_wind != disc_window.end(); it_wind++)
    { // iterate over each velocity in discrete window
      vector< t_helpers::ObstacleInfo > obstacles; // computed obstacles for dyn window
      t_helpers::ObstacleInfo closest_pt; // closest point for dyn window
      t_helpers::ObstacleExist(translated_pc, it_wind->x(), it_wind->y(), obstacles, closest_pt); // true if not obstacle free

      auto v_admissible = find_if(obstacles.begin(), obstacles.end(), 
        [](t_helpers::ObstacleInfo &obstacle){
        return obstacle.f() < S_MAX; // admissible if no free path is less than max stopping dist
      }); 

      if (v_admissible == obstacles.end())
      { // velocity is admissible
        // compute score for each obstacle
        for (auto it_ob = obstacles.begin(); it_ob != obstacles.end(); it_ob ++)
        {
          float score = t_helpers::CalculateScore(*it_wind, *it_ob, closest_pt);
          if (score > best_score)
          {
            best_score = score;
            best_vel = *it_wind;
          }
        }
      }
    }

  ROS_DEBUG("GetCommandVelCallback: C_v: %f, C_w: %f, best_score: %f", 
    best_vel.x(), best_vel.y(), best_score);

  res.C_v = best_vel.x();
  res.C_w = best_vel.y();

  g_TranslatedPC = sensor_msgs::PointCloud(); // reset the cached pointcloud
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "compsci403_assignment5");
  ros::NodeHandle n;

  if (argc > 1)
  {
    DISCRETIZATIONS = atoi(argv[1]) > 0 ? atoi(argv[1]) : DISCRETIZATIONS;
    ROS_INFO("DISCRETIZATIONS set to %i", DISCRETIZATIONS);

    if (argc > 3)
    {
      ALPHA = atof(argv[2]);
      BETA = atof(argv[3]) >= 0 ? atof(argv[3]) : BETA;
      GAMMA = atof(argv[4]) >= 0 ? atof(argv[4]) : GAMMA;

      ROS_INFO("Set ALPHA: %f, BETA: %f, GAMMA: %f", ALPHA, BETA, GAMMA);
    }
    else
    {
      ROS_INFO("Param: Not set ALPHA: %f, BETA: %f, GAMMA: %f", ALPHA, BETA, GAMMA);
    }
  }
  else
  {
    ROS_INFO("Param: Discretizations not set, defaulting %i", DISCRETIZATIONS);
  }

  // 0. publish topic visualization_marker_array
  g_VisPub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1000);

  // 1. Provide Service /COMPSCI403/CheckPoint
  g_CheckPointSrv = n.advertiseService("/COMPSCI403/CheckPoint", CheckPointCallback);

  // 2. Provide Service /COMPSCI403/GetFreePath
  g_GetFreePathSrv = n.advertiseService("/COMPSCI403/GetFreePath", GetFreePathCallback);

  // 3. Provide a service named /COMPSCI403/ObstacleLaserScan
  g_ObstacleLaserScanSrv = n.advertiseService("/COMPSCI403/ObstacleLaserScan", ObstacleLaserScanCallback);
  
  // 4. Create Service /COMPSCI403/GetCommandVel
  g_GetCommandVelSrv = n.advertiseService("/COMPSCI403/GetCommandVel", GetCommandVelCallback);


  ros::Rate spin_rate (20); // 20 hz
  
  while (ros::ok())
  {
    ros::spinOnce();
    spin_rate.sleep();
  }

  return 0;
}
