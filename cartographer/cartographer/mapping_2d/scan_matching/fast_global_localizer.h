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

#ifndef CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_FAST_GLOBAL_LOCALIZER_H_
#define CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_FAST_GLOBAL_LOCALIZER_H_

#include <vector>

#include "eigen3/Eigen/Geometry"
#include "../mapping_2d/scan_matching/fast_correlative_scan_matcher.h"
#include "../sensor/voxel_filter.h"

namespace cartographer {
namespace mapping_2d {
namespace scan_matching {

// Perform global localization against the provided 'matchers'.
// The 'cutoff' specifies the minimum correlation that will be accepted.
// This function does not take ownership of the pointers passed in
// 'matchers'; they are passed as a vector of raw pointers to give maximum
// flexibility to callers.
//
// Returns true in the case of successful localization. The output parameters
// should not be trusted if the function returns false. The 'cutoff' and
// 'best_score' are in the range [0.0, 1.0].
// 用来进行全局定位的函数
// 基本功能就是和地图中的每一个submap进行匹配。如果能匹配上则全局定位成功。
// 否则全局定位失败
// 每一个submap都和一个FastCorrelativeScanMatcher绑定在一起。
// 因此传入的直接就是FastCorrelativeScanMatcher
//
/**
 * @brief PerformGlobalLocalization
 * 用来进行全局定位的函数
 * @param cutoff                接受定位成功的阈值
 * @param voxel_filter          网格滤波器
 * @param matchers              submap对应的FastCorrelativeScanMatcher
 * @param point_cloud           用来定位的激光数据
 * @param best_pose_estimate    估计出来的最好的位姿
 * @param best_score            估计出来的位姿对应的分数
 * @return
 */
bool PerformGlobalLocalization(
    float cutoff, const cartographer::sensor::AdaptiveVoxelFilter& voxel_filter,
    const std::vector<
        cartographer::mapping_2d::scan_matching::FastCorrelativeScanMatcher*>&
        matchers,
    const cartographer::sensor::PointCloud2D& point_cloud,
    transform::Rigid2d* best_pose_estimate, float* best_score);

}  // namespace scan_matching
}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_FAST_GLOBAL_LOCALIZER_H_
