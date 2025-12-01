#include "localization_ndt/ndt_localizer_node.hpp"

#include <limits>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

namespace localization_ndt {

NdtLocalizerNode::NdtLocalizerNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh), tf_buffer_(), tf_listener_(tf_buffer_) {

  // ------- 读取参数 -------
  pnh_.param<std::string>("map_pcd_path", map_pcd_path_, std::string(""));
  pnh_.param<std::string>("scan_topic", scan_topic_,
                          std::string("/Y991B020189BC0018/scan"));
  pnh_.param<std::string>("odom_topic", odom_topic_,
                          std::string("/Y991B020189BC0018/odom/wheel"));
  pnh_.param<std::string>("imu_topic", imu_topic_,
                          std::string("/Y991B020189BC0018/imu"));
  pnh_.param<std::string>("base_frame_id", base_frame_id_,
                          std::string("base_footprint"));
  // NDT 参数
  int ndt_num_threads = 4;
  pnh_.param("ndt_resolution", ndt_resolution_, 1.0);
  pnh_.param("ndt_max_fitness_score", ndt_max_fitness_score_, 2.0);
  pnh_.param("ndt_num_threads", ndt_num_threads, 4);

  ndt_matcher_.setResolution(ndt_resolution_);
  ndt_matcher_.setNumThreads(ndt_num_threads);

  if (map_pcd_path_.empty()) {
    ROS_FATAL("~map_pcd_path is empty. Please set it in the launch file.");
    throw std::runtime_error("map_pcd_path is empty");
  }

  if (!loadMap(map_pcd_path_)) {
    ROS_FATAL("Failed to load map from '%s'", map_pcd_path_.c_str());
    throw std::runtime_error("map load failed");
  }

  // ------- 订阅传感器 -------
  odom_sub_ =
      nh_.subscribe(odom_topic_, 100, &NdtLocalizerNode::odomCallback, this);
  imu_sub_ =
      nh_.subscribe(imu_topic_, 200, &NdtLocalizerNode::imuCallback, this);
  scan_sub_ =
      nh_.subscribe(scan_topic_, 5, &NdtLocalizerNode::scanCallback, this);

  // ------- 发布位姿 -------
  pose_pub_ = nh_.advertise<nav_msgs::Odometry>("ndt_odom", 10, false);

  ROS_INFO_STREAM("NdtLocalizerNode initialized. "
                  << "map_pcd_path=" << map_pcd_path_ << ", scan_topic="
                  << scan_topic_ << ", odom_topic=" << odom_topic_
                  << ", imu_topic=" << imu_topic_
                  << ", ndt_resolution=" << ndt_resolution_
                  << ", ndt_max_fitness_score=" << ndt_max_fitness_score_);
}

void NdtLocalizerNode::spin() { ros::spin(); }

bool NdtLocalizerNode::loadMap(const std::string &path) {
  map_.reset(new PointCloudT);

  ROS_INFO_STREAM("Loading map PCD: " << path);
  if (pcl::io::loadPCDFile<PointT>(path, *map_) != 0) {
    ROS_ERROR_STREAM("Could not read PCD file: " << path);
    map_.reset();
    return false;
  }

  ROS_INFO_STREAM("Map loaded. Point size = " << map_->size());

  if (!map_->empty()) {
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*map_, min_pt, max_pt);
    ROS_INFO_STREAM("Map bounds: "
                    << "min[" << min_pt.x << ", " << min_pt.y << ", "
                    << min_pt.z << "], "
                    << "max[" << max_pt.x << ", " << max_pt.y << ", "
                    << max_pt.z << "]");
  }

  // 关键：把地图给 NDT 作为 target
  ndt_matcher_.setTargetMap(map_);

  return true;
}

void NdtLocalizerNode::odomCallback(
    const nav_msgs::OdometryConstPtr &odom_msg) {
  predictor_.updateOdom(odom_msg);
}

void NdtLocalizerNode::imuCallback(const sensor_msgs::ImuConstPtr &imu_msg) {
  predictor_.updateImu(imu_msg);
}

void NdtLocalizerNode::scanCallback(
    const sensor_msgs::LaserScanConstPtr &scan_msg) {
  // 1) LaserScan -> PointCloud2
  sensor_msgs::PointCloud2 cloud_msg;
  try {
    // 把 LaserScan 直接变成 base_frame_id_ 下的点云
    projector_.transformLaserScanToPointCloud(
        base_frame_id_, // 目标坐标系：车体基准
        *scan_msg, cloud_msg, tf_buffer_);
  } catch (const std::exception &e) {
    ROS_WARN_STREAM_THROTTLE(1.0, "Laser projection (to "
                                      << base_frame_id_
                                      << ") failed: " << e.what());
    return;
  }

  // 2) PointCloud2 -> PCL PointCloud
  PointCloudT::Ptr cloud(new PointCloudT);
  pcl::fromROSMsg(cloud_msg, *cloud);

  if (cloud->empty()) {
    ROS_WARN("Projected cloud is empty.");
    return;
  }

  ROS_INFO_STREAM_THROTTLE(
      1.0, "Received scan, projected cloud size = " << cloud->size());

  // 3) odom+IMU 预测（initial guess）
  Eigen::Matrix4f T_init = Eigen::Matrix4f::Identity();
  bool has_pred = predictor_.predict(T_init);

  if (has_pred) {
    Eigen::Vector3f t = T_init.block<3, 1>(0, 3);
    float yaw = std::atan2(T_init(1, 0), T_init(0, 0));
    ROS_INFO_STREAM_THROTTLE(1.0, "Predicted pose (odom+imu): x="
                                      << t.x() << ", y=" << t.y()
                                      << ", yaw=" << yaw);
  } else {
    ROS_WARN_THROTTLE(5.0, "Predictor not ready yet (waiting for odom+imu). "
                           "Using Identity as init guess.");
  }

  // 4) NDT_OMP 匹配
  Eigen::Matrix4f T_ndt = T_init;
  double fitness = std::numeric_limits<double>::max();
  bool ndt_ok = ndt_matcher_.align(cloud, T_init, T_ndt, fitness);

  Eigen::Matrix4f T_output = T_init;

  if (ndt_ok && fitness < ndt_max_fitness_score_) {
    ROS_INFO_STREAM_THROTTLE(1.0, "NDT converged. fitness=" << fitness);

    T_output = T_ndt;
    // 用这次匹配结果刷新预测器的“锚点”
    predictor_.commitPredictionAsCorrection(T_ndt);
  } else {
    if (!ndt_ok) {
      ROS_WARN_THROTTLE(1.0, "NDT did not converge. Using prediction only.");
    } else {
      ROS_WARN_THROTTLE(1.0,
                        "NDT bad fitness (%f >= %f). Using prediction only.",
                        fitness, ndt_max_fitness_score_);
    }
  }
  // 5) 发布最终位姿
  publishOdom(scan_msg->header.stamp, T_output);
} // namespace localization_ndt

void NdtLocalizerNode::publishOdom(const ros::Time &stamp,
                                   const Eigen::Matrix4f &T_map_base) {
  nav_msgs::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = "map";
  odom.child_frame_id = "base_footprint"; // 之后可以改成你真实的 baselink

  Eigen::Matrix3f R = T_map_base.block<3, 3>(0, 0);
  Eigen::Vector3f t = T_map_base.block<3, 1>(0, 3);

  Eigen::Quaternionf q(R);
  q.normalize();

  odom.pose.pose.position.x = t.x();
  odom.pose.pose.position.y = t.y();
  odom.pose.pose.position.z = 0.0f; // 2D
  odom.pose.pose.orientation.w = q.w();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();

  pose_pub_.publish(odom);
}
} // namespace localization_ndt
// namespace localization_ndt

// ---------------------- main ----------------------

int main(int argc, char **argv) {
  ros::init(argc, argv, "ndt_localizer_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    localization_ndt::NdtLocalizerNode node(nh, pnh);
    node.spin();
  } catch (const std::exception &e) {
    ROS_FATAL_STREAM("Exception in NdtLocalizerNode: " << e.what());
    return 1;
  }

  return 0;
}
