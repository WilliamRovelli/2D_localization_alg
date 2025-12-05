#pragma once

#include <string>

#include <laser_geometry/laser_geometry.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>

#include <Eigen/Dense>

#include "localization_ndt/ndt_matcher.hpp"
#include "localization_ndt/simple_predictor.hpp"
#include "localization_ndt/types.hpp"
// 用于将雷达转换到baselink下
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
// 用于手动初始化重定位
#include <geometry_msgs/PoseWithCovarianceStamped.h>
// 发布map到base_footprint的关系
#include <tf2_ros/transform_broadcaster.h>

#include "localization_ndt/initial_pose_manager.hpp"

namespace localization_ndt {

enum class NdtQualityLevel {
  BAD = 0,     // 拒绝本次 NDT，使用预测
  SUSPECT = 1, // 可疑：先接受，但可以上层降权
  GOOD = 2     // 匹配很好
};

struct NdtQualityMetrics {
  double fitness =
      std::numeric_limits<double>::quiet_NaN(); // NDT 自身的代价
  double delta_trans =
      std::numeric_limits<double>::quiet_NaN(); // 预测 vs NDT 的平移差
  double delta_yaw =
      std::numeric_limits<double>::quiet_NaN(); // 预测 vs NDT 的航向差（rad）
  NdtQualityLevel level = NdtQualityLevel::BAD;
};

class NdtLocalizerNode {
public:
  NdtLocalizerNode(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  void spin();

private:
  bool loadMap(const std::string &path);

  /// 计算质量指标并给出 GOOD / SUSPECT / BAD 判断
  NdtQualityMetrics evaluateNdtQuality(
      double fitness,
      const Eigen::Matrix4f &T_pred,
      const Eigen::Matrix4f &T_ndt,
      bool ndt_converged) const;

  void scanCallback(const sensor_msgs::LaserScanConstPtr &scan_msg);
  void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg);
  void imuCallback(const sensor_msgs::ImuConstPtr &imu_msg);

  void publishOdom(const ros::Time &stamp, const Eigen::Matrix4f &T_map_base);
  void initialPoseCallback(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::string map_pcd_path_;
  std::string scan_topic_;
  std::string odom_topic_;
  std::string imu_topic_;
  std::string urdf_file_path_; 

  PointCloudT::Ptr map_;

  ros::Subscriber scan_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber imu_sub_;
  ros::Publisher pose_pub_;
  ros::Subscriber initialpose_sub_;
  laser_geometry::LaserProjection projector_;
  SimplePredictor predictor_;
  NdtMatcher ndt_matcher_;

  double ndt_resolution_ = 1.0; // 参数
  double ndt_max_fitness_score_ = 2.0;

  /// 质量评估开关及参数（可通过 rosparam 调）
  bool quality_enable_ = true;
  double quality_fitness_bad_ = 2.0;    // fitness > 这个 → BAD
  double quality_fitness_warn_ = 0.7;   // fitness > 这个 → SUSPECT
  double quality_max_delta_trans_ = 1.5; // 预测 vs NDT 平移差阈值[m]
  double quality_max_delta_yaw_deg_ = 20.0; // 预测 vs NDT 航向差阈值[deg]

  std::string base_frame_id_ = "base_footprint";

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // 初始位姿管理
  InitialPoseManager initial_pose_mgr_;
  std::string pose_config_dir_;

  // 最近一次里程计（给 resetWithCorrection 用）
  nav_msgs::Odometry last_odom_msg_;
  bool has_last_odom_msg_ = false;
  double latest_linear_speed_ = 0.0;
  ros::Time last_odom_stamp_;
  bool initial_pose_applied_ = false;

  // 质量/静止阈值
  double ndt_good_fitness_score_ = 0.5;    // "GOOD" 的 fitness 上限，可调
  double stationary_speed_thresh_ = 0.02;  // m/s，认为车停住

  // ---- 静止检测 & last_good_pose 写入控制 ----
  double stationary_min_duration_;   // 静止超过多少秒才写入 [s]
  bool stationary_phase_;            // 当前是否处于“静止阶段”
  bool stationary_pose_written_;     // 当前这次静止阶段是否已经写过一次
  ros::Time stationary_start_time_;  // 当前静止阶段开始的时间
};

} // namespace localization_ndt
