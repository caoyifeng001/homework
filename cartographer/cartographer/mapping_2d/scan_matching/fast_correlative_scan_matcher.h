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

// This is an implementation of the algorithm described in "Real-Time
// Correlative Scan Matching" by Olson.
//
// It is similar to the RealTimeCorrelativeScanMatcher but has a different
// trade-off: Scan matching is faster because more effort is put into the
// precomputation done for a given map. However, this map is immutable(不可变的) after
// construction.
// 这里实现了对应的论文中第三种方法：Multi-Level Resolution方法。
// 因此比起RealTimeCorrelativeScanMatcher方法 这里的方法速度要更快。
// 这里会实现事先几个不同分辨率的地图来加速匹配
// 由于要事先计算不同分辨率的地图来进行加速，因此每一个FastCorrelativeScanMatcher只能对应一个栅格地图(Probability grid)
// 如果要换成不同的地图，则要重新定义一个新的FastCorrelativeScanMatcher
// 可以说每一个FastCorrelativeScanMatcher都和一个栅格地图(Probability Grid)绑定了



#ifndef CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_FAST_CORRELATIVE_SCAN_MATCHER_H_
#define CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_FAST_CORRELATIVE_SCAN_MATCHER_H_

#include <memory>
#include <vector>

#include "eigen3/Eigen/Core"
#include "../common/port.h"
#include "../mapping_2d/probability_grid.h"
#include "../mapping_2d/scan_matching/correlative_scan_matcher.h"
#include "../sensor/point_cloud.h"

#include "cartographer/mapping_2d/scan_matching/proto/fast_correlative_scan_matcher_options.pb.h"

namespace cartographer {
namespace mapping_2d {
namespace scan_matching {

proto::FastCorrelativeScanMatcherOptions
CreateFastCorrelativeScanMatcherOptions(
    common::LuaParameterDictionary* parameter_dictionary);

// A precomputed grid that contains in each cell (x0, y0) the maximum
// probability in the width x width area defined by x0 <= x < x0 + width and
// y0 <= y < y0.
// 这个类表示栅格地图，这个栅格地图是用来加速Multi-Level Resolution来作用的
// 一般会有好几个不同的分辨率的栅格地图。这个类被PrecomputationGridStack里面引用
class PrecomputationGrid
{
 public:
  PrecomputationGrid(const ProbabilityGrid& probability_grid,
                     const CellLimits& limits, int width,
                     std::vector<float>* reusable_intermediate_grid);

  // Returns a value between 0 and 255 to represent probabilities between
  // kMinProbability and kMaxProbability.
  // 返回对应下标的CellValue
  // 可以认为这里传入的index是　概率地图的index
  // 要转换成PrecomputationGrid的index的话　要加上offset
  // 因为precomputationGrid相比与原始地图来说　要
  int GetValue(const Eigen::Array2i& xy_index) const
  {
    const Eigen::Array2i local_xy_index = xy_index - offset_;
    // The static_cast<unsigned> is for performance to check with 2 comparisons
    // xy_index.x() < offset_.x() || xy_index.y() < offset_.y() ||
    // local_xy_index.x() >= wide_limits_.num_x_cells ||
    // local_xy_index.y() >= wide_limits_.num_y_cells
    // instead of using 4 comparisons.
    if (static_cast<unsigned>(local_xy_index.x()) >=
            static_cast<unsigned>(wide_limits_.num_x_cells) ||
        static_cast<unsigned>(local_xy_index.y()) >=
            static_cast<unsigned>(wide_limits_.num_y_cells))
    {
      return 0;
    }
    const int stride = wide_limits_.num_x_cells;
    return cells_[local_xy_index.x() + local_xy_index.y() * stride];
  }

  // Maps values from [0, 255] to [kMinProbability, kMaxProbability].
  // 把CellValue转换为占用概率
  static float ToProbability(float value)
  {
    return mapping::kMinProbability +
           value *
               ((mapping::kMaxProbability - mapping::kMinProbability) / 255.f);
  }

 private:
  //把占用概率转换为CellValue
  uint8 ComputeCellValue(float probability) const;

  // Offset of the precomputation grid in relation to the 'probability_grid'
  // including the additional 'width' - 1 cells.
  // 预计算地图　和　概率地图的唯一关系
  // 预计算地图比概率地图来说多了width-1的cells
  // 预计算的栅格地图　相对于　原始地图　的offset
  const Eigen::Array2i offset_;

  // Size of the precomputation grid.
  // 地图的大小 这个一般是在原始的地图大小上　加入 width
  const CellLimits wide_limits_;

  // Probabilites mapped to 0 to 255.
  // 地图的数据
  std::vector<uint8> cells_;
};

//用来存储一系列不同分辨率的PrecomputationGrid
class PrecomputationGridStack;

// An implementation of "Real-Time Correlative Scan Matching" by Olson.
// 实现了论文中的多分辨率匹配方法(Muiti-Level Resolution)
class FastCorrelativeScanMatcher
{
 public:
  FastCorrelativeScanMatcher(
      const ProbabilityGrid& probability_grid,
      const proto::FastCorrelativeScanMatcherOptions& options);
  ~FastCorrelativeScanMatcher();

  FastCorrelativeScanMatcher(const FastCorrelativeScanMatcher&) = delete;
  FastCorrelativeScanMatcher& operator=(const FastCorrelativeScanMatcher&) =
      delete;

  // Aligns 'point_cloud' within the 'probability_grid' given an
  // 'initial_pose_estimate'. If a score above 'min_score' (excluding equality)
  // is possible, true is returned, and 'score' and 'pose_estimate' are updated
  // with the result.
  // 在规定的搜索窗口中来进行匹配 注意每次进行调用的时候，这里面的地图都是已经固定的了。
  // 在RealTimeCorrelativeScanMatcher里面，在进行Match函数调用的时候，会传入地图。
  // 但是在这里面是不行的。因为要计算多分辨率地图，这个是事先计算好的。
  bool Match(const transform::Rigid2d& initial_pose_estimate,
             const sensor::PointCloud2D& point_cloud, float min_score,
             float* score, transform::Rigid2d* pose_estimate) const;

  // Aligns 'point_cloud' within the full 'probability_grid', i.e., not
  // restricted to the configured search window. If a score above 'min_score'
  // (excluding equality) is possible, true is returned, and 'score' and
  // 'pose_estimate' are updated with the result.
  // 和整个submap来进行匹配，而不是局限在规定的搜索窗口
  bool MatchFullSubmap(const sensor::PointCloud2D& point_cloud, float min_score,
                       float* score, transform::Rigid2d* pose_estimate) const;

 private:
  // The actual implementation of the scan matcher, called by Match() and
  // MatchFullSubmap() with appropriate 'initial_pose_estimate' and
  // 'search_parameters'.
  bool MatchWithSearchParameters(
      SearchParameters search_parameters,
      const transform::Rigid2d& initial_pose_estimate,
      const sensor::PointCloud2D& point_cloud, float min_score, float* score,
      transform::Rigid2d* pose_estimate) const;

  std::vector<Candidate> ComputeLowestResolutionCandidates(
      const std::vector<DiscreteScan>& discrete_scans,
      const SearchParameters& search_parameters) const;

  std::vector<Candidate> GenerateLowestResolutionCandidates(
      const SearchParameters& search_parameters) const;

  void ScoreCandidates(const PrecomputationGrid& precomputation_grid,
                       const std::vector<DiscreteScan>& discrete_scans,
                       const SearchParameters& search_parameters,
                       std::vector<Candidate>* const candidates) const;

  Candidate BranchAndBound(const std::vector<DiscreteScan>& discrete_scans,
                           const SearchParameters& search_parameters,
                           const std::vector<Candidate>& candidates,
                           int candidate_depth, float min_score) const;

  const proto::FastCorrelativeScanMatcherOptions options_;
  MapLimits limits_;
  std::unique_ptr<PrecomputationGridStack> precomputation_grid_stack_;
};

}  // namespace scan_matching
}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_FAST_CORRELATIVE_SCAN_MATCHER_H_
