#include "localization_ndt/ndt_localizer_node.hpp"

#include <cmath>
#include <limits>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

// 需要 roslib 依赖
#include <ros/package.h>

namespace localization_ndt {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

NdtLocalizerNode::NdtLocalizerNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh), tf_buffer_(), tf_listener_(tf_buffer_) {

  // ---------- 先拿包路径 ----------
  const std::string pkg_path = ros::package::getPath("localization_ndt");
  if (pkg_path.empty()) {
    ROS_FATAL("Failed to get package path for 'localization_ndt'.");
    throw std::runtime_error("ros::package::getPath failed");
  }

  // ---------- 读取基本参数 ----------
  // 地图名（不含扩展名），例如 "xw"
  std::string map_name;
  pnh_.param<std::string>("map_name", map_name, std::string(""));

  // 可选：允许用户直接指定完整 pcd 路径（优先级更高）
  pnh_.param<std::string>("map_pcd_path", map_pcd_path_, std::string(""));

  pnh_.param<std::string>("scan_topic", scan_topic_,
                          std::string("/Y991B020189BC0018/scan"));
  pnh_.param<std::string>("odom_topic", odom_topic_,
                          std::string("/Y991B020189BC0018/odom/wheel"));
  pnh_.param<std::string>("imu_topic", imu_topic_,
                          std::string("/Y991B020189BC0018/imu"));
  pnh_.param<std::string>("base_frame_id", base_frame_id_,
                          std::string("base_footprint"));

  int ndt_num_threads = 4;
  pnh_.param("ndt_resolution", ndt_resolution_, 1.0);
  pnh_.param("ndt_max_fitness_score", ndt_max_fitness_score_, 2.0);
  pnh_.param("ndt_num_threads", ndt_num_threads, 4);

  // ---------- pose_config_dir_ 自动推导 ----------
  // 1) 如果用户在 launch 里给了 pose_config_dir，则使用用户路径
  // 2) 否则，默认用 <pkg_path>/config
  if (!pnh_.getParam("pose_config_dir", pose_config_dir_)) {
    pose_config_dir_ = pkg_path + "/config";
  }

  // ---------- map_pcd_path_ 自动推导 ----------
  // 1) 如果用户在 launch 里直接给了 map_pcd_path_（完整路径），优先用它
  // 2) 否则，如果 map_name 非空，则按约定目录拼路径：
  //    <workspace_root>/map/<map_name>.pcd
  //    workspace_root = pkg_path + "/../.."
  if (map_pcd_path_.empty()) {
    if (map_name.empty()) {
      ROS_FATAL("Neither ~map_pcd_path nor ~map_name is set. "
                "You must set one of them.");
      throw std::runtime_error("map_pcd_path_ is empty and map_name not set");
    }

    const std::string workspace_root = pkg_path + "/../..";
    map_pcd_path_ = workspace_root + "/map/" + map_name + ".pcd";
  }

  ROS_INFO_STREAM("Resolved map_pcd_path = " << map_pcd_path_);
  ROS_INFO_STREAM("Resolved pose_config_dir = " << pose_config_dir_);

  // ---------- 质量评估相关参数 ----------
  pnh_.param("quality_enable", quality_enable_, true);
  pnh_.param("quality_fitness_bad", quality_fitness_bad_,
             ndt_max_fitness_score_);
  pnh_.param("quality_fitness_warn", quality_fitness_warn_, 0.7);
  pnh_.param("quality_max_delta_trans", quality_max_delta_trans_, 1.5);
  pnh_.param("quality_max_delta_yaw_deg", quality_max_delta_yaw_deg_, 20.0);

  // ---------- 写 last_good_pose 的相关门槛 ----------
  pnh_.param("ndt_good_fitness_score", ndt_good_fitness_score_, 0.5);
  pnh_.param("stationary_speed_thresh", stationary_speed_thresh_, 0.02);
  pnh_.param("stationary_min_duration", stationary_min_duration_, 2.0); // 秒

  // ---------- 一些状态变量初始化 ----------
  initial_pose_applied_ = false;
  has_last_odom_msg_ = false;
  latest_linear_speed_ = 0.0;
  stationary_phase_ = false;
  stationary_pose_written_ = false;
  stationary_start_time_ = ros::Time(0);

    // ---------- YAML 管理器 / URDF 路径解析 ----------

  // 支持两种写法：
  // 1) ~urdf_file: 绝对路径，例如  /home/wr/localization_alg/agv_urdf/agv01.urdf
  // 2) ~urdf_file: 名字       ，例如  "agv01" 或 "agv01.urdf"
  std::string urdf_param;
  pnh_.param<std::string>("urdf_file", urdf_param, std::string(""));

  if (urdf_param.empty()) {
    ROS_FATAL("~urdf_file is not set. Both map and urdf must be provided.");
    throw std::runtime_error("urdf_file param missing");
  }

  // 如果包含 '/' 或 '\'，认为是完整路径，直接使用
  if (urdf_param.find('/') != std::string::npos ||
      urdf_param.find('\\') != std::string::npos) {
    urdf_file_path_ = urdf_param;
  } else {
    // 否则认为只是文件名，在 <workspace_root>/agv_urdf/ 下自动拼
    // workspace_root = pkg_path + "/../.."
    const std::string workspace_root = pkg_path + "/../..";

    std::string urdf_name = urdf_param;
    const std::string suffix = ".urdf";

    // 如果没有 .urdf 后缀，自动补上
    if (urdf_name.size() < suffix.size() ||
        urdf_name.substr(urdf_name.size() - suffix.size()) != suffix) {
      urdf_name += suffix;
    }

    urdf_file_path_ = workspace_root + "/agv_urdf/" + urdf_name;
  }

  ROS_INFO_STREAM("Resolved urdf_file_path_ = " << urdf_file_path_);

  initial_pose_mgr_.setConfigDir(pose_config_dir_);
  if (!initial_pose_mgr_.initializeForMap(map_pcd_path_, urdf_file_path_)) {
    ROS_FATAL("InitialPoseManager::initializeForMap failed. Check map/urdf "
              "path and config_dir.");
    throw std::runtime_error("InitialPoseManager init failed");
  }



  if (initial_pose_mgr_.hasLastGoodPose()) {
    const auto &p = initial_pose_mgr_.lastGoodPose();
    ROS_INFO_STREAM("InitialPoseManager: use last_good_pose from "
                    << initial_pose_mgr_.yamlPath() << " : x=" << p.x
                    << ", y=" << p.y << ", yaw=" << p.yaw);
  } else {
    const auto &p = initial_pose_mgr_.fixedPose();
    ROS_INFO_STREAM("InitialPoseManager: use fixed_pose from "
                    << initial_pose_mgr_.yamlPath() << " : x=" << p.x
                    << ", y=" << p.y << ", yaw=" << p.yaw);
  }

  // ---------- 加载地图 ----------
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
  initialpose_sub_ = nh_.subscribe(
      "initialpose", 1, &NdtLocalizerNode::initialPoseCallback, this);

  // ------- 发布位姿 -------
  pose_pub_ = nh_.advertise<nav_msgs::Odometry>("ndt_odom", 10, false);

  ndt_matcher_.setResolution(ndt_resolution_);
  ndt_matcher_.setNumThreads(ndt_num_threads);

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

  // 把地图给 NDT 作为 target
  ndt_matcher_.setTargetMap(map_);

  return true;
}

void NdtLocalizerNode::odomCallback(
    const nav_msgs::OdometryConstPtr &odom_msg) {
  predictor_.updateOdom(odom_msg);

  // 记录最近 odom（给 resetWithCorrection、静止检测用）
  last_odom_msg_ = *odom_msg;
  has_last_odom_msg_ = true;

  latest_linear_speed_ = std::hypot(odom_msg->twist.twist.linear.x,
                                    odom_msg->twist.twist.linear.y);
  last_odom_stamp_ = odom_msg->header.stamp;
}

void NdtLocalizerNode::imuCallback(const sensor_msgs::ImuConstPtr &imu_msg) {
  predictor_.updateImu(imu_msg);
}

void NdtLocalizerNode::scanCallback(
    const sensor_msgs::LaserScanConstPtr &scan_msg) {

  // ========= 1) LaserScan -> PointCloud2 =========
  sensor_msgs::PointCloud2 cloud_msg;
  try {
    projector_.transformLaserScanToPointCloud(base_frame_id_, *scan_msg,
                                              cloud_msg, tf_buffer_);
  } catch (const std::exception &e) {
    ROS_WARN_STREAM_THROTTLE(1.0, "Laser projection (to "
                                      << base_frame_id_
                                      << ") failed: " << e.what());
    return;
  }

  // ========= 2) PointCloud2 -> PCL PointCloud =========
  PointCloudT::Ptr cloud(new PointCloudT);
  pcl::fromROSMsg(cloud_msg, *cloud);

  if (cloud->empty()) {
    ROS_WARN("Projected cloud is empty.");
    return;
  }

  ROS_INFO_STREAM_THROTTLE(
      1.0, "Received scan, projected cloud size = " << cloud->size());

  // ========= 3) 初始重定位：fixed_pose & last_good_pose 双候选，只跑一次
  // =========
  if (!initial_pose_applied_) {
    if (!has_last_odom_msg_) {
      ROS_WARN_THROTTLE(
          1.0, "Waiting for odom to apply initial pose. Skip this scan.");
      return;
    }

    // ---- fixed_pose 候选 ----
    InitialPoseManager::Pose2D fixed = initial_pose_mgr_.fixedPose();
    Eigen::Matrix4f T_guess_fixed = Eigen::Matrix4f::Identity();
    {
      float c = std::cos(fixed.yaw);
      float s = std::sin(fixed.yaw);
      T_guess_fixed(0, 0) = c;
      T_guess_fixed(0, 1) = -s;
      T_guess_fixed(1, 0) = s;
      T_guess_fixed(1, 1) = c;
      T_guess_fixed(0, 3) = fixed.x;
      T_guess_fixed(1, 3) = fixed.y;
    }

    Eigen::Matrix4f T_ndt_fixed = T_guess_fixed;
    double fitness_fixed = std::numeric_limits<double>::max();
    bool ndt_ok_fixed =
        ndt_matcher_.align(cloud, T_guess_fixed, T_ndt_fixed, fitness_fixed);
    bool have_fixed = ndt_ok_fixed;

    if (ndt_ok_fixed) {
      ROS_INFO_STREAM(
          "[Init] fixed_pose NDT converged. fitness=" << fitness_fixed);
    } else {
      ROS_WARN("[Init] NDT from fixed_pose did not converge.");
    }

    // ---- last_good_pose 候选 ----
    bool has_last = initial_pose_mgr_.hasLastGoodPose();
    Eigen::Matrix4f T_guess_last = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f T_ndt_last = Eigen::Matrix4f::Identity();
    double fitness_last = std::numeric_limits<double>::max();
    bool ndt_ok_last = false;
    bool have_last = false;

    if (has_last) {
      auto last = initial_pose_mgr_.lastGoodPose();
      {
        float c = std::cos(last.yaw);
        float s = std::sin(last.yaw);
        T_guess_last(0, 0) = c;
        T_guess_last(0, 1) = -s;
        T_guess_last(1, 0) = s;
        T_guess_last(1, 1) = c;
        T_guess_last(0, 3) = last.x;
        T_guess_last(1, 3) = last.y;
      }

      ndt_ok_last =
          ndt_matcher_.align(cloud, T_guess_last, T_ndt_last, fitness_last);
      have_last = ndt_ok_last;

      if (ndt_ok_last) {
        ROS_INFO_STREAM(
            "[Init] last_good_pose NDT converged. fitness=" << fitness_last);
      } else {
        ROS_WARN("[Init] NDT from last_good_pose did not converge.");
      }
    }

    // ---- 根据 fitness 选择更好的那一个 ----
    Eigen::Matrix4f T_best = T_guess_fixed; // 默认回退到 fixed_guess
    if (have_fixed || have_last) {
      bool use_last = false;

      if (have_last && !have_fixed) {
        use_last = true;
      } else if (!have_last && have_fixed) {
        use_last = false;
      } else {
        // 两个都可用：直接比较 fitness，谁小用谁
        use_last = (fitness_last < fitness_fixed);
      }

      if (use_last) {
        T_best = T_ndt_last;
        ROS_INFO("[Init] Choose last_good_pose as initial pose.");
      } else {
        T_best = T_ndt_fixed;
        ROS_INFO("[Init] Choose fixed_pose as initial pose.");
      }
    } else {
      ROS_ERROR("[Init] Both initial NDT (fixed_pose & last_good_pose) failed. "
                "Fallback to fixed_pose guess.");
      T_best = T_guess_fixed;
    }

    // ---- 用选出来的姿态锚定 predictor，并发布这一帧 ----
    predictor_.resetWithCorrection(T_best, last_odom_msg_);
    initial_pose_applied_ = true;
    stationary_phase_ = false;
    stationary_pose_written_ = false;

    publishOdom(scan_msg->header.stamp, T_best);
    return; // 这一帧到此结束，不再走下面正常流程
  }

  // ========= 4) 正常流程：predict → NDT → 质量门控 → 静止 2s 写 yaml =========

  Eigen::Matrix4f T_init = Eigen::Matrix4f::Identity();
  bool has_pred = predictor_.predict(T_init);

  if (has_pred) {
    Eigen::Vector3f t = T_init.block<3, 1>(0, 3);
    float yaw = std::atan2(T_init(1, 0), T_init(0, 0));
    ROS_INFO_STREAM_THROTTLE(1.0, "Predicted pose (odom+imu): x="
                                      << t.x() << ", y=" << t.y()
                                      << ", yaw=" << yaw);
  } else {
    ROS_WARN_THROTTLE(5.0, "Predictor not ready yet after initial pose. "
                           "Using Identity as init guess.");
  }

  Eigen::Matrix4f T_ndt = T_init;
  double fitness = std::numeric_limits<double>::max();
  bool ndt_ok = ndt_matcher_.align(cloud, T_init, T_ndt, fitness);

  Eigen::Matrix4f T_output = T_init;

  if (!ndt_ok) {
    ROS_WARN_STREAM_THROTTLE(
        1.0,
        "NDT did not converge. Using prediction only. fitness=" << fitness);
  } else {
    // 计算质量：只用 fitness + 预测 vs NDT 的差值
    NdtQualityMetrics q = evaluateNdtQuality(fitness, T_init, T_ndt, ndt_ok);

    ROS_INFO_STREAM_THROTTLE(
        1.0, "NDT quality: fitness="
                 << fitness << ", delta_trans=" << q.delta_trans
                 << ", delta_yaw_deg=" << (q.delta_yaw * 180.0 / kPi)
                 << ", level="
                 << (q.level == NdtQualityLevel::GOOD
                         ? "GOOD"
                         : (q.level == NdtQualityLevel::SUSPECT ? "SUSPECT"
                                                                : "BAD")));

    if (q.level == NdtQualityLevel::BAD) {
      // 完全不信 NDT，只用预测
      ROS_WARN_STREAM_THROTTLE(
          1.0, "NDT rejected by quality gate. "
               "Use prediction only. fitness="
                   << fitness << ", delta_trans=" << q.delta_trans
                   << ", delta_yaw_deg=" << (q.delta_yaw * 180.0 / kPi));
      // T_output 保持为 T_init
    } else {
      // GOOD 或 SUSPECT：都要用到 NDT，只是权重不同
      if (q.level == NdtQualityLevel::GOOD) {
        // 匹配很好：完全信任 NDT
        ROS_INFO_STREAM_THROTTLE(1.0,
                                 "NDT accepted (GOOD). Use pure NDT result.");
        T_output = T_ndt;
        predictor_.commitPredictionAsCorrection(T_ndt);
      } else {
        // SUSPECT：部分信任 NDT，部分信预测
        ROS_WARN_STREAM_THROTTLE(
            1.0, "NDT accepted (SUSPECT). Blending NDT and prediction.");

        // ---- 简单固定权重 + 一点按 fitness 调整 ----
        double w_ndt = 0.5;  // NDT 权重
        double w_pred = 0.5; // 预测权重

        // fitness 越大，说明匹配越差，就多信一点预测
        if (fitness > quality_fitness_warn_) {
          // warn 以上了，多信一点预测
          w_ndt = 0.4;
          w_pred = 0.6;
        }

        // 平移部分：简单线性插值
        Eigen::Vector3f t_pred = T_init.block<3, 1>(0, 3);
        Eigen::Vector3f t_ndt = T_ndt.block<3, 1>(0, 3);
        Eigen::Vector3f t_mix = static_cast<float>(w_pred) * t_pred +
                                static_cast<float>(w_ndt) * t_ndt;

        // 航向角：用“预测角 + NDT 修正的一部分”
        float yaw_pred = std::atan2(T_init(1, 0), T_init(0, 0));
        float dyaw = static_cast<float>(q.delta_yaw); // 已 wrap 到 [-pi,pi]
        float yaw_mix = yaw_pred + static_cast<float>(w_ndt) * dyaw;

        Eigen::Matrix4f T_mix = Eigen::Matrix4f::Identity();
        float c = std::cos(yaw_mix);
        float s = std::sin(yaw_mix);
        T_mix(0, 0) = c;
        T_mix(0, 1) = -s;
        T_mix(1, 0) = s;
        T_mix(1, 1) = c;
        T_mix(0, 3) = t_mix.x();
        T_mix(1, 3) = t_mix.y();
        T_mix(2, 3) = t_mix.z(); // 2D，用不到

        T_output = T_mix;
        predictor_.commitPredictionAsCorrection(T_output);

        ROS_INFO_STREAM_THROTTLE(
            1.0, "Blended pose: w_ndt=" << w_ndt << ", w_pred=" << w_pred);
      }

      // ===== 静止 2 秒只写一次 last_good_pose（只在 GOOD 情况下写） =====
      bool quality_good = (q.level == NdtQualityLevel::GOOD) &&
                          (fitness < ndt_good_fitness_score_);

      double v = latest_linear_speed_;
      bool now_stationary = (v < stationary_speed_thresh_);

      if (!now_stationary) {
        stationary_phase_ = false;
        stationary_pose_written_ = false;
      } else {
        if (!stationary_phase_) {
          stationary_phase_ = true;
          stationary_pose_written_ = false;
          stationary_start_time_ = scan_msg->header.stamp;
        }
      }

      if (quality_good && stationary_phase_ && !stationary_pose_written_) {

        double dt_stop =
            (scan_msg->header.stamp - stationary_start_time_).toSec();

        if (dt_stop >= stationary_min_duration_) {
          InitialPoseManager::Pose2D pose2d;
          pose2d.x = T_output(0, 3);
          pose2d.y = T_output(1, 3);
          pose2d.yaw = std::atan2(T_output(1, 0), T_output(0, 0));

          initial_pose_mgr_.updateLastGoodPose(pose2d);
          stationary_pose_written_ = true;

          ROS_INFO_STREAM(
              "InitialPoseManager: update last_good_pose at stop: x="
              << pose2d.x << ", y=" << pose2d.y << ", yaw=" << pose2d.yaw);
        }
      }
      // ===== 静止写 YAML 结束 =====
    }
  }

  // ========= 5) 发布最终位姿 =========
  publishOdom(scan_msg->header.stamp, T_output);
}

void NdtLocalizerNode::initialPoseCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
  // 一般 RViz 这里 frame_id 会是 "map"
  if (msg->header.frame_id != "map") {
    ROS_WARN_STREAM("initialpose frame_id = '"
                    << msg->header.frame_id
                    << "', expected 'map'. Using it as map frame anyway.");
  }

  // 1) 把 Pose 转成 T_map_base
  Eigen::Matrix4f T_map_base = Eigen::Matrix4f::Identity();

  const auto &p = msg->pose.pose.position;
  const auto &q_msg = msg->pose.pose.orientation;

  Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
  q.normalize();
  Eigen::Matrix3d R = q.toRotationMatrix();

  T_map_base.block<3, 3>(0, 0) = R.cast<float>();
  T_map_base(0, 3) = static_cast<float>(p.x);
  T_map_base(1, 3) = static_cast<float>(p.y);
  T_map_base(2, 3) = 0.0f; // 2D 地图，高度忽略

  // 2) 用这个姿态重设预测器的锚点
  if (!has_last_odom_msg_) {
    ROS_WARN("Received initialpose but no odom yet. "
             "Resetting predictor with dummy odom.");
    nav_msgs::Odometry dummy;
    dummy.pose.pose.orientation.w = 1.0;
    predictor_.resetWithCorrection(T_map_base, dummy);
  } else {
    predictor_.resetWithCorrection(T_map_base, last_odom_msg_);
  }

  // 标记：已经有了可靠初始位姿，scanCallback 里就不要再自动套 YAML 了
  initial_pose_applied_ = true;
  stationary_phase_ = false;
  stationary_pose_written_ = false;

  ROS_INFO_STREAM("Manual initial pose received: x=" << p.x << ", y=" << p.y);
}

NdtQualityMetrics NdtLocalizerNode::evaluateNdtQuality(
    double fitness, const Eigen::Matrix4f &T_pred, const Eigen::Matrix4f &T_ndt,
    bool ndt_converged) const {

  NdtQualityMetrics q;
  q.fitness = fitness;

  // 如果关闭质量评估，只用 "是否收敛" 这一条
  if (!quality_enable_) {
    q.level = ndt_converged ? NdtQualityLevel::GOOD : NdtQualityLevel::BAD;
    return q;
  }

  if (!ndt_converged) {
    q.level = NdtQualityLevel::BAD;
    return q;
  }

  // 预测 vs NDT 的位姿差：平移 + yaw
  Eigen::Vector2f t_pred = T_pred.block<3, 1>(0, 3).head<2>();
  Eigen::Vector2f t_ndt = T_ndt.block<3, 1>(0, 3).head<2>();
  q.delta_trans = (t_ndt - t_pred).norm();

  float yaw_pred = std::atan2(T_pred(1, 0), T_pred(0, 0));
  float yaw_ndt = std::atan2(T_ndt(1, 0), T_ndt(0, 0));
  float dyaw = yaw_ndt - yaw_pred;
  while (dyaw > kPi)
    dyaw -= 2.0f * static_cast<float>(kPi);
  while (dyaw < -kPi)
    dyaw += 2.0f * static_cast<float>(kPi);
  q.delta_yaw = dyaw;

  bool bad = false;
  bool suspect = false;

  // 1) fitness 太差 → 直接 BAD
  if (fitness > quality_fitness_bad_) {
    bad = true;
  }

  // 2) 与预测的差值 gating：太离谱就 BAD
  const double max_delta_yaw_rad = quality_max_delta_yaw_deg_ * kPi / 180.0;
  if (q.delta_trans > quality_max_delta_trans_ ||
      std::fabs(q.delta_yaw) > max_delta_yaw_rad) {
    bad = true;
  }

  // 3) fitness 落在 warn 区：标记 SUSPECT
  if (fitness > quality_fitness_warn_) {
    suspect = true;
  }

  if (bad) {
    q.level = NdtQualityLevel::BAD;
  } else if (suspect) {
    q.level = NdtQualityLevel::SUSPECT;
  } else {
    q.level = NdtQualityLevel::GOOD;
  }

  return q;
}

void NdtLocalizerNode::publishOdom(const ros::Time &stamp,
                                   const Eigen::Matrix4f &T_map_base) {
  nav_msgs::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = "map";
  odom.child_frame_id = base_frame_id_; // 用参数，不要写死

  Eigen::Matrix3f R = T_map_base.block<3, 3>(0, 0);
  Eigen::Vector3f t = T_map_base.block<3, 1>(0, 3);

  Eigen::Quaternionf q(R);
  q.normalize();

  odom.pose.pose.position.x = t.x();
  odom.pose.pose.position.y = t.y();
  odom.pose.pose.position.z = 0.0f;
  odom.pose.pose.orientation.w = q.w();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();

  pose_pub_.publish(odom);

  // ==== 同步发布 TF: map -> base_frame_id_ ====
  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = base_frame_id_;

  tf_msg.transform.translation.x = t.x();
  tf_msg.transform.translation.y = t.y();
  tf_msg.transform.translation.z = 0.0;

  tf_msg.transform.rotation.x = q.x();
  tf_msg.transform.rotation.y = q.y();
  tf_msg.transform.rotation.z = q.z();
  tf_msg.transform.rotation.w = q.w();

  tf_broadcaster_.sendTransform(tf_msg);
}

} // namespace localization_ndt

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
