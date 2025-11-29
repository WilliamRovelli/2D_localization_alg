#pragma once

#include <memory>
#include <Eigen/Dense>

#include <pclomp/ndt_omp.h>

#include "localization_ndt/types.hpp"

namespace localization_ndt {

class NdtMatcher {
public:
  using PointT = localization_ndt::PointT;
  using PointCloudT = localization_ndt::PointCloudT;

  NdtMatcher() {
    ndt_.reset(new pclomp::NormalDistributionsTransform<PointT, PointT>());
    // 这里给一些合理的默认值，具体的可以在 Node 里再 set 一次
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
    // 如果不想管，就传 1 或 4 都行
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
  /// @return 是否成功（收敛 && 有 target map）
  bool align(const PointCloudT::ConstPtr& cloud,
             const Eigen::Matrix4f& init_guess,
             Eigen::Matrix4f& final_pose,
             double& fitness_score) {
    if (!has_target_) {
      return false;
    }

    ndt_->setInputSource(cloud);

    pcl::PointCloud<PointT> aligned;
    ndt_->align(aligned, init_guess);

    if (!ndt_->hasConverged()) {
      return false;
    }

    final_pose = ndt_->getFinalTransformation();
    fitness_score = ndt_->getFitnessScore();

    return true;
  }

private:
  pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt_;
  bool has_target_ = false;
};

}  // namespace localization_ndt
