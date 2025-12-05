#pragma once

#include <memory>
#include <vector>
#include <limits>

#include <Eigen/Dense>

#include <pclomp/ndt_omp.h>
#include <pcl/registration/registration.h>   // 为了 getSearchMethodTarget

#include "localization_ndt/types.hpp"

namespace localization_ndt {

class NdtMatcher {
public:
  using PointT      = localization_ndt::PointT;
  using PointCloudT = localization_ndt::PointCloudT;

  NdtMatcher() {
    ndt_.reset(new pclomp::NormalDistributionsTransform<PointT, PointT>());
    // 一些默认参数，可以在外面再覆盖
    ndt_->setTransformationEpsilon(0.01);
    ndt_->setResolution(1.0);
    ndt_->setStepSize(0.1);
    ndt_->setMaximumIterations(30);
    ndt_->setNeighborhoodSearchMethod(pclomp::DIRECT7);
  }

  void setResolution(double res) {
    ndt_->setResolution(res);
  }

  void setNumThreads(int n_threads) {
    ndt_->setNumThreads(n_threads);
  }

  void setTransformationEpsilon(double eps) {
    ndt_->setTransformationEpsilon(eps);
  }

  void setStepSize(double step) {
    ndt_->setStepSize(step);
  }

  void setMaxIterations(int iters) {
    ndt_->setMaximumIterations(iters);
  }

  void setTargetMap(const PointCloudT::ConstPtr& map) {
    ndt_->setInputTarget(map);
    has_target_ = true;
  }

  /// @brief 对当前帧做一次 NDT 匹配
  /// @param cloud       当前帧（在 base_frame 下）
  /// @param init_guess  初始位姿 T_map_base
  /// @param final_pose  输出的最终位姿 T_map_base
  /// @param fitness     NDT 自带的 fitness_score
  /// @param aligned_out （可选）输出对齐到 map 坐标系下的点云
  /// @return 是否成功（收敛 && 有 target map）
  bool align(const PointCloudT::ConstPtr& cloud,
             const Eigen::Matrix4f& init_guess,
             Eigen::Matrix4f&       final_pose,
             double&                fitness,
             PointCloudT::Ptr       aligned_out = nullptr) {
    if (!has_target_) {
      return false;
    }

    ndt_->setInputSource(cloud);

    PointCloudT aligned;
    ndt_->align(aligned, init_guess);

    if (!ndt_->hasConverged()) {
      return false;
    }

    final_pose   = ndt_->getFinalTransformation();
    fitness      = ndt_->getFitnessScore();

    if (aligned_out) {
      *aligned_out = aligned;
    }

    return true;
  }

  /// @brief 参考 hdl_localization 计算 matching_error / inlier_fraction
  /// @param aligned  已经对齐到 map 坐标系下的当前帧点云（align 输出）
  /// @param max_correspondence_dist  匹配内点距离阈值（默认 0.5m）
  /// @param max_valid_point_dist     有效点距离阈值（默认 25m）
  /// @param matching_error           输出：inliers 的平均平方距离
  /// @param inlier_fraction          输出：inliers / 有效点 比例
  /// @return 是否成功（map 可用 && 至少有一个 inlier）
  bool computeMatchQuality(const PointCloudT::ConstPtr& aligned,
                           double max_correspondence_dist,
                           double max_valid_point_dist,
                           double& matching_error,
                           double& inlier_fraction) const {
    if (!has_target_ || !aligned || aligned->empty()) {
      matching_error  = std::numeric_limits<double>::quiet_NaN();
      inlier_fraction = 0.0;
      return false;
    }

    auto tree = ndt_->getSearchMethodTarget();  // KdTree or DIRECT*
    if (!tree) {
      matching_error  = std::numeric_limits<double>::quiet_NaN();
      inlier_fraction = 0.0;
      return false;
    }

    const double max_corr_sq = max_correspondence_dist * max_correspondence_dist;

    int    num_inliers      = 0;
    int    num_valid_points = 0;
    double error_sum        = 0.0;

    std::vector<int>   k_indices(1);
    std::vector<float> k_sq_dists(1);

    for (const auto& pt : *aligned) {
      // 1) 先过滤太远的点
      if (pt.getVector3fMap().norm() > max_valid_point_dist) {
        continue;
      }
      ++num_valid_points;

      // 2) 查最近邻
      if (tree->nearestKSearch(pt, 1, k_indices, k_sq_dists) <= 0) {
        continue;
      }

      if (k_sq_dists[0] < max_corr_sq) {
        error_sum += k_sq_dists[0];
        ++num_inliers;
      }
    }

    if (num_inliers > 0) {
      matching_error = error_sum / static_cast<double>(num_inliers);
    } else {
      matching_error = std::numeric_limits<double>::infinity();
    }

    if (num_valid_points > 0) {
      inlier_fraction =
          static_cast<double>(num_inliers) /
          static_cast<double>(num_valid_points);
    } else {
      inlier_fraction = 0.0;
    }

    return num_inliers > 0;
  }

private:
  pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt_;
  bool has_target_ = false;
};

}  // namespace localization_ndt
