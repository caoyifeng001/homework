/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "../mapping_2d/scan_matching/ceres_scan_matcher.h"

#include <utility>
#include <vector>

#include "eigen3/Eigen/Core"
#include "../common/ceres_solver_options.h"
#include "../common/lua_parameter_dictionary.h"
#include "../kalman_filter/pose_tracker.h"
#include "../mapping_2d/probability_grid.h"
#include "../mapping_2d/scan_matching/occupied_space_cost_functor.h"
#include "../mapping_2d/scan_matching/rotation_delta_cost_functor.h"
#include "../mapping_2d/scan_matching/translation_delta_cost_functor.h"
#include "../transform/transform.h"
#include "ceres/ceres.h"
#include "glog/logging.h"


/*
 * 用优化的方式进行scan-match。也就是说这里是用梯度下降的方式来进行scan-match
 * 因此这里的作用实际上和gmapping的hill-climb方法是差不多的。
 * 这种局部优化的方法很容易陷入到局部极小值当中。因此这个方法能正常工作的前提是初始值离全局最优值比较近。
 * 因此这个方法一般是用作其他方法的优化。
 * 比如在cartographer中　在调用这个方法之前，首先会用CSM方法来进行搜索出来一个初值，然后再用这个优化的方法来进行优化
*/

namespace cartographer {
namespace mapping_2d {
namespace scan_matching {

proto::CeresScanMatcherOptions CreateCeresScanMatcherOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::CeresScanMatcherOptions options;
  options.set_occupied_space_cost_functor_weight(
      parameter_dictionary->GetDouble("occupied_space_cost_functor_weight"));
  options.set_previous_pose_translation_delta_cost_functor_weight(
      parameter_dictionary->GetDouble(
          "previous_pose_translation_delta_cost_functor_weight"));
  options.set_initial_pose_estimate_rotation_delta_cost_functor_weight(
      parameter_dictionary->GetDouble(
          "initial_pose_estimate_rotation_delta_cost_functor_weight"));
  options.set_covariance_scale(
      parameter_dictionary->GetDouble("covariance_scale"));
  *options.mutable_ceres_solver_options() =
      common::CreateCeresSolverOptionsProto(
          parameter_dictionary->GetDictionary("ceres_solver_options").get());
  return options;
}

CeresScanMatcher::CeresScanMatcher(
    const proto::CeresScanMatcherOptions& options)
    : options_(options),
      ceres_solver_options_(
          common::CreateCeresSolverOptions(options.ceres_solver_options())) {
  ceres_solver_options_.linear_solver_type = ceres::DENSE_QR;
}

CeresScanMatcher::~CeresScanMatcher() {}

void CeresScanMatcher::Match(const transform::Rigid2d& previous_pose,
                             const transform::Rigid2d& initial_pose_estimate,
                             const sensor::PointCloud2D& point_cloud,
                             const ProbabilityGrid& probability_grid,
                             transform::Rigid2d* const pose_estimate,
                             kalman_filter::Pose2DCovariance* const covariance,
                             ceres::Solver::Summary* const summary) const
{
  double ceres_pose_estimate[3] = {initial_pose_estimate.translation().x(),
                                   initial_pose_estimate.translation().y(),
                                   initial_pose_estimate.rotation().angle()};
  ceres::Problem problem;
  CHECK_GT(options_.occupied_space_cost_functor_weight(), 0.);

  //构造残差--栅格
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<OccupiedSpaceCostFunctor, ceres::DYNAMIC,
                                      3>(
          new OccupiedSpaceCostFunctor(
              options_.occupied_space_cost_functor_weight() /
                  std::sqrt(static_cast<double>(point_cloud.size())),
              point_cloud, probability_grid),
          point_cloud.size()),
      nullptr, ceres_pose_estimate);
  CHECK_GT(options_.previous_pose_translation_delta_cost_functor_weight(), 0.);

  //构造残差--平移
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<TranslationDeltaCostFunctor, 2, 3>(
          new TranslationDeltaCostFunctor(
              options_.previous_pose_translation_delta_cost_functor_weight(),
              previous_pose)),
      nullptr, ceres_pose_estimate);
  CHECK_GT(options_.initial_pose_estimate_rotation_delta_cost_functor_weight(),
           0.);

  //构造残差--旋转
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<RotationDeltaCostFunctor, 1,
                                      3>(new RotationDeltaCostFunctor(
          options_.initial_pose_estimate_rotation_delta_cost_functor_weight(),
          ceres_pose_estimate[2])),
      nullptr, ceres_pose_estimate);

  //求解器
  ceres::Solve(ceres_solver_options_, &problem, summary);

  //优化完毕之后得到的最优位姿
  *pose_estimate = transform::Rigid2d(
      {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);

  //计算位姿的方差
  ceres::Covariance::Options options;
  ceres::Covariance covariance_computer(options);
  std::vector<std::pair<const double*, const double*>> covariance_blocks;
  covariance_blocks.emplace_back(ceres_pose_estimate, ceres_pose_estimate);
  CHECK(covariance_computer.Compute(covariance_blocks, &problem));

  double ceres_covariance[3 * 3];
  covariance_computer.GetCovarianceBlock(ceres_pose_estimate,
                                         ceres_pose_estimate, ceres_covariance);

  *covariance = Eigen::Map<kalman_filter::Pose2DCovariance>(ceres_covariance);
  *covariance *= options_.covariance_scale();
}

}  // namespace scan_matching
}  // namespace mapping_2d
}  // namespace cartographer
