#pragma once

#include <string>

#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Dense>

#include "localization_ndt/types.hpp"
#include "localization_ndt/simple_predictor.hpp"
#include "localization_ndt/ndt_matcher.hpp"   

namespace localization_ndt {

class NdtLocalizerNode {
public:
  NdtLocalizerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  void spin();

private:
  bool loadMap(const std::string& path);

  void scanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg);
  void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg);
  void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg);

  void publishOdom(const ros::Time& stamp, const Eigen::Matrix4f& T_map_base);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::string map_pcd_path_;
  std::string scan_topic_;
  std::string odom_topic_;
  std::string imu_topic_;

  PointCloudT::Ptr map_;

  ros::Subscriber scan_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber imu_sub_;

  ros::Publisher pose_pub_;

  laser_geometry::LaserProjection projector_;
  SimplePredictor predictor_;
  NdtMatcher ndt_matcher_;          // 新增

  double ndt_resolution_ = 1.0;     // 参数
  double ndt_max_fitness_score_ = 2.0;
};

}  // namespace localization_ndt
