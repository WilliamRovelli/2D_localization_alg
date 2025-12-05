#include "localization_ndt/initial_pose_manager.hpp"

#include <fstream>
#include <ros/ros.h>

namespace localization_ndt {

std::string InitialPoseManager::extractMapName(const std::string& map_pcd_path) const {
  // /home/wr/xxx/xw.pcd  ->  xw
  std::size_t slash_pos = map_pcd_path.find_last_of("/\\");
  std::size_t start = (slash_pos == std::string::npos) ? 0 : slash_pos + 1;

  std::size_t dot_pos = map_pcd_path.find_last_of('.');
  if (dot_pos == std::string::npos || dot_pos <= start) {
    // 没有扩展名，就整个当名字
    return map_pcd_path.substr(start);
  }
  return map_pcd_path.substr(start, dot_pos - start);
}

bool InitialPoseManager::initializeForMap(const std::string& map_pcd_path,
                                          const std::string& urdf_file_path) {
  // 1) 基本检查：不能为空
  if (map_pcd_path.empty()) {
    ROS_ERROR("InitialPoseManager: map_pcd_path is empty.");
    return false;
  }
  if (urdf_file_path.empty()) {
    ROS_ERROR("InitialPoseManager: urdf_file_path is empty.");
    return false;
  }

  // 2) 文件是否存在（map & urdf 必须都存在）
  {
    std::ifstream f_map(map_pcd_path.c_str());
    if (!f_map.good()) {
      ROS_ERROR_STREAM("InitialPoseManager: map PCD does not exist: "
                       << map_pcd_path);
      return false;
    }
  }
  {
    std::ifstream f_urdf(urdf_file_path.c_str());
    if (!f_urdf.good()) {
      ROS_ERROR_STREAM("InitialPoseManager: URDF file does not exist: "
                       << urdf_file_path);
      return false;
    }
  }

  // 3) config_dir_ 检查（可以保留你之前的兜底）
  if (config_dir_.empty()) {
    // 你之前的默认行为就保留
    config_dir_ = "/home/wr/localization_alg/config";
    ROS_WARN_STREAM("InitialPoseManager: config_dir_ is empty, "
                    "fallback to default: " << config_dir_);
  }

  // 4) 提取前缀：xw.pcd -> xw, agv01.urdf -> agv01
  std::string map_stem   = extractMapName(map_pcd_path);
  std::string robot_stem = extractMapName(urdf_file_path);

  if (map_stem.empty() || robot_stem.empty()) {
    ROS_ERROR_STREAM("InitialPoseManager: failed to extract map/urdf stem. "
                     "map_pcd_path=" << map_pcd_path
                     << ", urdf_file_path=" << urdf_file_path);
    return false;
  }

  map_name_ = map_stem;

  // 5) 严格命名：map + "_" + urdf + ".yaml"，不再有兜底
  std::string yaml_name = map_stem + "_" + robot_stem + ".yaml";
  yaml_path_ = config_dir_ + "/" + yaml_name;

  // 默认 pose：原点
  fixed_pose_         = Pose2D();
  last_good_pose_     = Pose2D();
  has_last_good_pose_ = false;

  // 6) 如果 yaml 不存在 -> 写一个默认；存在 -> 加载
  std::ifstream fin(yaml_path_.c_str());
  if (!fin.good()) {
    fin.close();
    ROS_INFO_STREAM("InitialPoseManager: yaml not found, create default: "
                    << yaml_path_);
    return writeFile();
  }
  fin.close();

  ROS_INFO_STREAM("InitialPoseManager: loading initial pose yaml: "
                  << yaml_path_);
  return loadFromFile();
}


bool InitialPoseManager::loadFromFile() {
  try {
    YAML::Node root = YAML::LoadFile(yaml_path_);

    if (root["fixed_pose"]) {
      auto node = root["fixed_pose"];
      if (node["x"])   fixed_pose_.x   = node["x"].as<double>();
      if (node["y"])   fixed_pose_.y   = node["y"].as<double>();
      if (node["yaw"]) fixed_pose_.yaw = node["yaw"].as<double>();
    } else {
      fixed_pose_ = Pose2D();  // 原点
    }

    if (root["last_good_pose"]) {
      auto node = root["last_good_pose"];
      if (node["x"])   last_good_pose_.x   = node["x"].as<double>();
      if (node["y"])   last_good_pose_.y   = node["y"].as<double>();
      if (node["yaw"]) last_good_pose_.yaw = node["yaw"].as<double>();
      has_last_good_pose_ = true;
    } else {
      last_good_pose_ = Pose2D();
      has_last_good_pose_ = false;
    }

    return true;
  } catch (const std::exception& e) {
    ROS_WARN_STREAM("InitialPoseManager: failed to load yaml: "
                    << yaml_path_ << " error: " << e.what());
    fixed_pose_ = Pose2D();
    last_good_pose_ = Pose2D();
    has_last_good_pose_ = false;
    return false;
  }
}

bool InitialPoseManager::writeFile() {
  try {
    YAML::Emitter out;
    out << YAML::BeginMap;

    // fixed_pose
    out << YAML::Key << "fixed_pose"
        << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "x"   << YAML::Value << fixed_pose_.x;
    out << YAML::Key << "y"   << YAML::Value << fixed_pose_.y;
    out << YAML::Key << "yaw" << YAML::Value << fixed_pose_.yaw;
    out << YAML::EndMap;

    // last_good_pose
    out << YAML::Key << "last_good_pose"
        << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "x"   << YAML::Value << last_good_pose_.x;
    out << YAML::Key << "y"   << YAML::Value << last_good_pose_.y;
    out << YAML::Key << "yaw" << YAML::Value << last_good_pose_.yaw;
    out << YAML::EndMap;

    out << YAML::EndMap;

    std::ofstream fout(yaml_path_.c_str());
    fout << out.c_str();
    fout.close();

    return true;
  } catch (const std::exception& e) {
    ROS_WARN_STREAM("InitialPoseManager: failed to write yaml: "
                    << yaml_path_ << " error: " << e.what());
    return false;
  }
}

void InitialPoseManager::updateLastGoodPose(const Pose2D& pose) {
  last_good_pose_ = pose;
  has_last_good_pose_ = true;

  if (!writeFile()) {
    ROS_WARN_STREAM("InitialPoseManager: failed to update yaml: "
                    << yaml_path_);
  } else {
    ROS_INFO_STREAM("InitialPoseManager: last_good_pose updated: "
                    << "x=" << pose.x << ", y=" << pose.y
                    << ", yaw=" << pose.yaw
                    << " -> " << yaml_path_);
  }
}

}  // namespace localization_ndt
