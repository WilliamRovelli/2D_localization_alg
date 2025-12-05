#pragma once

#include <string>
#include <yaml-cpp/yaml.h>

namespace localization_ndt {

class InitialPoseManager {
public:
  struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;  // rad
  };

  InitialPoseManager() = default;

  /// 设置 config 目录，比如 /home/wr/localization_alg/config
  void setConfigDir(const std::string& dir) {
    config_dir_ = dir;
  }

  /// 根据地图 pcd 路径（例如 /xxx/xw.pcd）初始化：
  ///  - 生成 map_name = xw
  ///  - yaml_path = config_dir/xw_initial_pose.yaml
  ///  - 如果不存在则创建，并把 fixed_pose 和 last_good_pose 都写成原点
  ///  - 如果存在则从文件中读出两个 pose
  bool initializeForMap(const std::string& map_pcd_path,
                        const std::string& urdf_file_path = std::string());

  const Pose2D& fixedPose() const { return fixed_pose_; }
  const Pose2D& lastGoodPose() const { return last_good_pose_; }
  bool hasLastGoodPose() const { return has_last_good_pose_; }

  /// 更新 last_good_pose，并写回 yaml 文件
  void updateLastGoodPose(const Pose2D& pose);

  /// 方便 log
  const std::string& yamlPath() const { return yaml_path_; }

private:
  std::string extractMapName(const std::string& map_pcd_path) const;
  bool loadFromFile();
  bool writeFile();

private:
  std::string config_dir_;   // e.g. /home/wr/localization_alg/config
  std::string map_name_;     // e.g. xw
  std::string yaml_path_;    // e.g. /home/.../xw_initial_pose.yaml

  Pose2D fixed_pose_;
  Pose2D last_good_pose_;
  bool has_last_good_pose_ = false;
};

}  // namespace localization_ndt
