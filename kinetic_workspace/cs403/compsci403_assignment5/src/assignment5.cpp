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

ros::ServiceClient g_GetTransformationCl;

ros::Subscriber g_OdomSub; // Subscriber /odom
ros::Subscriber g_LaserSub; // Subscriber /COMPSCI403/LaserScan

ros::Publisher g_DrivePub; // publisher /cmd_vel_mux/input/navi
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

  g_TranslatedPC = translated_pc;

  res.S_prime = translated_pc.points;

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
  float max_disc_v;
  vector<Eigen::Vector2f> disc_window = t_helpers::GenDiscDynWind(Eigen::Vector2f(req.v_0, req.w_0), DISCRETIZATIONS, &max_disc_v);
  
  Eigen::Vector2f best_vel = Eigen::Vector2f(req.v_0, req.w_0);
  float best_score = numeric_limits<float>::min();

  bool atleast_one_admissible = false;

  for (auto it_wind = disc_window.begin(); it_wind != disc_window.end(); it_wind++)
    { // iterate over each velocity in discrete window
      vector< t_helpers::ObstacleInfo > obstacles; // computed obstacles for dyn window
      t_helpers::ObstacleInfo closest_pt; // closest point for dyn window
      bool obstacle = t_helpers::ObstacleExist(translated_pc, it_wind->x(), it_wind->y(), obstacles, closest_pt); // true if not obstacle free
      if (obstacle)
      {
      auto v_admissible = find_if(obstacles.begin(), obstacles.end(), 
        [](t_helpers::ObstacleInfo &obstacle){
        return obstacle.f() < S_MAX; // admissible if no free path is less than max stopping dist
      }); 

        if (v_admissible == obstacles.end())
        { // velocity is admissible
          // compute score for each obstacle
          atleast_one_admissible = true;
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
      else
      {
        res.C_v = max_disc_v;
        res.C_w = 0.0f;
        
        return true;
      }
    }

  if (atleast_one_admissible)
  {
  ROS_DEBUG("GetCommandVelCallback: C_v: %f, C_w: %f, best_score: %f", 
    best_vel.x(), best_vel.y(), best_score);

  res.C_v = best_vel.x();
  res.C_w = best_vel.y();
  }
  else
  {
    res.C_w = 0.1f; // spin in place 
    res.C_v = 0.0f;
  }

  g_TranslatedPC = sensor_msgs::PointCloud(); // reset the cached pointcloud
  return true;
}

// 5.
void ScanOccurredCallback(const sensor_msgs::LaserScan &msg)
{
  // Call GetTransformation service
  compsci403_assignment5::GetTransformationSrv params;
  if (g_GetTransformationCl.call(params))
  {
    // get rigid body transform constants
    Matrix3f R;
    Vector3f T (params.response.T.x, params.response.T.y, params.response.T.z);

    for (size_t row = 0; row < 3; row ++)
    {
      for (size_t col = 0; col < 3; col ++)
      {
        R(row,col) = params.response.R[3 * row + col];
      }
    }

    // transform laser scan from sensor to robot reference frame
    sensor_msgs::PointCloud translated_pc;
    bool atleast_one_admissible = false;
    t_helpers::ProjectRangeFinderToRobotRef(msg, R, T, translated_pc);
    float max_disc_v;
    vector<Eigen::Vector2f> disc_window = t_helpers::GenDiscDynWind(Eigen::Vector2f(g_v.x(), g_v.y()), DISCRETIZATIONS, &max_disc_v);
    
    Eigen::Vector2f best_vel = Eigen::Vector2f(g_v.x(), g_v.y());
    float best_score = numeric_limits<float>::min();

    for (auto it_wind = disc_window.begin(); it_wind != disc_window.end(); it_wind++)
    { // iterate over each velocity in discrete window
      vector< t_helpers::ObstacleInfo > obstacles; // computed obstacles for dyn window
      t_helpers::ObstacleInfo closest_pt; // closest point for dyn window
      bool obstacle = t_helpers::ObstacleExist(translated_pc, it_wind->x(), it_wind->y(), obstacles, closest_pt); // true if not obstacle free

      if (obstacle)
      {
      float delta_theta = closest_pt.r() != 0 ? closest_pt.f() / closest_pt.r() : 3.14f;
      float v_constraint = sqrt(2.0f * AC_MAX * closest_pt.f());
      bool v_admissible =  (it_wind->x() <= v_constraint) && 
          it_wind->y() <= sqrt(2.0f * WC_MAX * delta_theta);


        if (v_admissible)
        { // velocity is admissible
          // compute score for each obstacle
          atleast_one_admissible = true;
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
      else
      {
        geometry_msgs::Twist msg;
        msg.linear.x = max_disc_v;
        msg.angular.z = 0.0f;
        g_DrivePub.publish(msg);
        return;
      }
    }
/*
    ROS_INFO("ScanOccurredCallback: C_v: %f, C_w: %f, best_score: %f", 
      best_vel.x(), best_vel.y(), best_score);
*/
    if (atleast_one_admissible)
    {
      geometry_msgs::Twist msg;
      msg.linear.x = best_vel.x();
      msg.angular.z = best_vel.y();
      g_DrivePub.publish(msg);
    }
    else
    {
      geometry_msgs::Twist msg;
      msg.linear.x = 0.0f;
      msg.angular.z = .3f;
      g_DrivePub.publish(msg);
    }
  }
  else
  {
    ROS_ERROR("ScanOccurredCallback: FATAL ERROR GetTransformationSrv is DEAD");
  }
}

// 5.
void OdometryOccurredCallback(const nav_msgs::Odometry &odom)
{ // odometry is published before laser scan, save it into a global
  g_v.x() = sqrt(pow(odom.twist.twist.linear.x, 2.0f) + pow(odom.twist.twist.linear.y, 2.0f));
  g_v.y() = odom.twist.twist.angular.z;
  g_robotPos.x() = odom.pose.pose.position.x;
  g_robotPos.y() = odom.pose.pose.position.y;
  ROS_DEBUG("OdometryOccurredCallback: set to: v: %f, w: %f, p_x: %f, p_y: %f", g_v.x(), g_v.y(), g_robotPos.x(), g_robotPos.y());
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

  // 5. Create Publisher /cmd_vel_mux/input/navi type geometry_msgs/Twist
  g_DrivePub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);

  // 5. Client for service /COMPSCI403/GetTransformation
  g_GetTransformationCl = n.serviceClient<compsci403_assignment5::GetTransformationSrv>("/COMPSCI403/GetTransformation");

  // 5. Create Subscriber /odom
  g_OdomSub = n.subscribe("/odom", 1, OdometryOccurredCallback);

  // 5. Create Subscriber /COMPSCI403/LaserScan
  g_LaserSub = n.subscribe("/COMPSCI403/LaserScan", 5, ScanOccurredCallback);

  ros::Rate spin_rate (20); // 20 hz
  
  while (ros::ok())
  {
    ros::spinOnce();
    spin_rate.sleep();
  }

  return 0;
}
