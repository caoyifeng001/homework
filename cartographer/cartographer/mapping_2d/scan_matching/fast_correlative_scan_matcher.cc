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

#include "../mapping_2d/scan_matching/fast_correlative_scan_matcher.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>

#include "eigen3/Eigen/Geometry"
#include "../common/math.h"
#include "../mapping_2d/probability_grid.h"
#include "../sensor/point_cloud.h"
#include "../transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_2d {
namespace scan_matching {

namespace {

// A collection of values which can be added and later removed, and the maximum
// of the current values in the collection can be retrieved.
// All of it in (amortized) O(1).
// 用来储存一些列数据的结构体，这个结构体可以维护这一些列数据中的最大值。
// 及时对数据进行删除和插入操作　也依然可以得到最大值
// 这个sliding_window是按照从大到小依次排列的，最大的元素位于队首　最小的位于队尾
// 这个滑动窗口主要是用来生成地图的时候　求解一个范围内的所有元素的最大值
class SlidingWindowMaximum
{
 public:

  //插入一个值　找到sliding window中的位置
  void AddValue(const float value)
  {
    while (!non_ascending_maxima_.empty() &&
           value > non_ascending_maxima_.back())
    {
      non_ascending_maxima_.pop_back();
    }
    non_ascending_maxima_.push_back(value);
  }

  //删除值　每次只能删除最大的值
  void RemoveValue(const float value)
  {
    // DCHECK for performance, since this is done for every value in the
    // precomputation grid.
    DCHECK(!non_ascending_maxima_.empty());
    DCHECK_LE(value, non_ascending_maxima_.front());

    if (value == non_ascending_maxima_.front())
    {
      non_ascending_maxima_.pop_front();
    }
  }

  //在预先计算地图的时候　每个栅格都需要用这个函数进行计算
  float GetMaximum() const
  {
    // DCHECK for performance, since this is done for every value in the
    // precomputation grid.
    DCHECK_GT(non_ascending_maxima_.size(), 0);
    return non_ascending_maxima_.front();
  }

  void CheckIsEmpty() const { CHECK_EQ(non_ascending_maxima_.size(), 0); }

 private:
  // Maximum of the current sliding window at the front. Then the maximum of the
  // remaining window that came after this values first occurence, and so on.
  // 这个sliding　window的最大值位于队列的首位
  std::deque<float> non_ascending_maxima_;
};

}  // namespace

//用来做进行CSM搜索的三个参数，线性搜索窗口、角度搜索窗口、分枝定界深度
proto::FastCorrelativeScanMatcherOptions
CreateFastCorrelativeScanMatcherOptions(
    common::LuaParameterDictionary* const parameter_dictionary)
{
  proto::FastCorrelativeScanMatcherOptions options;
  options.set_linear_search_window(
      parameter_dictionary->GetDouble("linear_search_window"));
  options.set_angular_search_window(
      parameter_dictionary->GetDouble("angular_search_window"));
  options.set_branch_and_bound_depth(
      parameter_dictionary->GetInt("branch_and_bound_depth"));
  return options;
}

/**
 * @brief PrecomputationGrid::PrecomputationGrid
 * 栅格地图的构造函数
 * 这个函数就是在原始地图的基础上　生成分辨率位width*origin_resolution的地图
 * 这个函数就是地图分成的最基础的函数　用来生成一系列不同分辨率的地图
 * @param probability_grid                  对应的概率地图(原始地图)
 * @param limits                            地图的参数(原始地图的参数　大小 x方向和y方向的cell的数量)
 * @param width                             地图的宽度(地图是正方形的)　可以认为是地图的分辨率 width*width个原始地图栅格合成一个
 * @param reusable_intermediate_grid        可以重复使用的中间栅格 用来计算最大值的一个中间值
 */
PrecomputationGrid::PrecomputationGrid(
    const ProbabilityGrid& probability_grid,
    const CellLimits& limits,
    const int width,
    std::vector<float>* reusable_intermediate_grid)
    : offset_(-width + 1, -width + 1),
      wide_limits_(limits.num_x_cells + width - 1,
                   limits.num_y_cells + width - 1),
      cells_(wide_limits_.num_x_cells * wide_limits_.num_y_cells)
{
  CHECK_GE(width, 1);
  CHECK_GE(limits.num_x_cells, 1);
  CHECK_GE(limits.num_y_cells, 1);

  //地图的一行的栅格数量　这个地图的栅格数量在原始地图的基础上增加了width-1个 因为原始地图的边界元素也需要做同样的修改
  const int stride = wide_limits_.num_x_cells;

  // First we compute the maximum probability for each (x0, y) achieved in the
  // span defined by x0 <= x < x0 + width.
  //　保持y的分辨率不变　把x0分割为长度为width的线段　并且求解每个线段的中的最大值
  std::vector<float>& intermediate = *reusable_intermediate_grid;

  //重新设置大小
  intermediate.resize(wide_limits_.num_x_cells * limits.num_y_cells);

  //枚举原始地图所有的y　这里是把每一行分为width的段，求解出每个width的段的最大值
  for (int y = 0; y != limits.num_y_cells; ++y)
  {
    SlidingWindowMaximum current_values;

    //设置0
    current_values.AddValue(
        probability_grid.GetProbability(Eigen::Array2i(0, y)));

    //求x=(1,width-1)区间的最大值　上面已经加入了0　因此总的东西是(0,width-1)
    for (int x = -width + 1; x != 0; ++x)
    {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();

      if (x + width < limits.num_x_cells)
      {
        current_values.AddValue(
            probability_grid.GetProbability(Eigen::Array2i(x + width, y)));
      }
    }

    //求x=(width,limits.num_x_cells)区间的最大值
    for (int x = 0; x < limits.num_x_cells - width; ++x)
    {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();

      //求出(x,y)加入(x_width,y) 相当于窗口进行滑动，在x轴进行滑动
      current_values.RemoveValue(
          probability_grid.GetProbability(Eigen::Array2i(x, y)));

      current_values.AddValue(
          probability_grid.GetProbability(Eigen::Array2i(x + width, y)));
    }

    for (int x = std::max(limits.num_x_cells - width, 0);
         x != limits.num_x_cells; ++x)
    {
      intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
      current_values.RemoveValue(
          probability_grid.GetProbability(Eigen::Array2i(x, y)));
    }
    current_values.CheckIsEmpty();
  }


  // For each (x, y), we compute the maximum probability in the width x width
  // region starting at each (x, y) and precompute the resulting bound on the
  // score.
  // 对应每个(x,y)，我们都需要计算周围width*width的范围对应的栅格的最大值　作为当前的值
  // 这里枚举的是要生成的地图的每一个x的栅格值

  // 经过上面的计算　intermediate的每一个行元素存储的都是一个width线段的最大值。
  // 这里的width线段是当前点i和i+width-1这一段的值
  // 下面的功能基本上是一样的，不过因为intermediate已经计算出来了，是width的线段的最大值
  // 这里计算的就是width*width里面的最大值
  for (int x = 0; x != wide_limits_.num_x_cells; ++x)
  {

    SlidingWindowMaximum current_values;
    current_values.AddValue(intermediate[x]);

    //求解第x列的　(0,width-1)的最大值　并且赋值给cells_
    for (int y = -width + 1; y != 0; ++y)
    {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      if (y + width < limits.num_y_cells)
      {
        current_values.AddValue(intermediate[x + (y + width) * stride]);
      }
    }

    //窗口开始滑动　每入一个　就出一个　按照y的方向滑动.

    for (int y = 0; y < limits.num_y_cells - width; ++y)
    {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      current_values.RemoveValue(intermediate[x + y * stride]);
      current_values.AddValue(intermediate[x + (y + width) * stride]);
    }

    for (int y = std::max(limits.num_y_cells - width, 0);
         y != limits.num_y_cells; ++y)
    {
      cells_[x + (y + width - 1) * stride] =
          ComputeCellValue(current_values.GetMaximum());
      current_values.RemoveValue(intermediate[x + y * stride]);
    }
    current_values.CheckIsEmpty();
  }
}

/**
 * @brief PrecomputationGrid::ComputeCellValue
 * 把占用概率转换为cellValue 通过线性插值的方式来做
 * KMInProbability      对应  0
 * KMaxPrabability      对应  255
 * @param probability
 * @return
 */
uint8 PrecomputationGrid::ComputeCellValue(const float probability) const
{
  const int cell_value = common::RoundToInt(
      (probability - mapping::kMinProbability) *
      (255.f / (mapping::kMaxProbability - mapping::kMinProbability)));

  assert()

  CHECK_GE(cell_value, 0);
  CHECK_LE(cell_value, 255);
  return cell_value;
}

/**
 * @brief The PrecomputationGridStack class
 * 这里面用的Multi-Level Resolution方法。
 * 因此要计算很多个尺度长的computation Grid。
 * 这个PrecomputationGridStack()里面就存储了一系列的不同分辨率的Computation Grid
 */
class PrecomputationGridStack
{
 public:
  PrecomputationGridStack(
      const ProbabilityGrid& probability_grid,
      const proto::FastCorrelativeScanMatcherOptions& options)
  {
    //最粗的分辨率　这个是由分枝定界的深度决定的。
    CHECK_GE(options.branch_and_bound_depth(), 1);
    const int max_width = 1 << (options.branch_and_bound_depth() - 1);

    //GridStack中地图的个数 即地图分为多少分辨率的
    precomputation_grids_.reserve(options.branch_and_bound_depth());

    //可以重复使用的中间栅格
    std::vector<float> reusable_intermediate_grid;
    const CellLimits limits = probability_grid.limits().cell_limits();

    reusable_intermediate_grid.reserve((limits.num_x_cells + max_width - 1) *
                                       limits.num_y_cells);

    //构造各个不同分辨率的栅格地图
    //width表示不同分辨率的栅格
    //1表示　最细的分辨率 origin_resolutino
    //2表示　　２*origin_resolution
    //4表示　　4*origin_resolution
    //8表示　　8*origin_resolution
    for (int i = 0; i != options.branch_and_bound_depth(); ++i)
    {
      //表示步长
      const int width = 1 << i;
      precomputation_grids_.emplace_back(probability_grid, limits, width,
                                         &reusable_intermediate_grid);
    }
  }

  //得到index对应的栅格地图　index表示深度
  const PrecomputationGrid& Get(int index)
  {
    return precomputation_grids_[index];
  }

  //最多有多少个分辨率
  int max_depth() const { return precomputation_grids_.size() - 1; }

 private:
  std::vector<PrecomputationGrid> precomputation_grids_;
};

FastCorrelativeScanMatcher::FastCorrelativeScanMatcher(
    const ProbabilityGrid& probability_grid,
    const proto::FastCorrelativeScanMatcherOptions& options)
    : options_(options),
      limits_(probability_grid.limits()),
      precomputation_grid_stack_(
          new PrecomputationGridStack(probability_grid, options)) {}

FastCorrelativeScanMatcher::~FastCorrelativeScanMatcher() {}

/**
 * @brief FastCorrelativeScanMatcher::Match
 * 被外界调用的Scan-Match函数。
 * 在这个函数里面主要就是调用MatchWithSearchParameters()来完成Scan-Match算法来完成匹配工作。
 * @param initial_pose_estimate     机器人的初始位姿(初始位姿的唯一作用就是确定搜索框的位置)
 * @param point_cloud               激光数据
 * @param min_score                 可接受的最低的分数
 * @param score                     最优位姿的分数
 * @param pose_estimate             最优的位姿
 * @return
 */
bool FastCorrelativeScanMatcher::Match(
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud2D& point_cloud, const float min_score,
    float* score, transform::Rigid2d* pose_estimate) const
{
  //设置搜索参数
  const SearchParameters search_parameters(options_.linear_search_window(),
                                           options_.angular_search_window(),
                                           point_cloud, limits_.resolution());

  //用这个参数进行匹配
  return MatchWithSearchParameters(search_parameters, initial_pose_estimate,
                                   point_cloud, min_score, score,
                                   pose_estimate);
}

/**
 * @brief FastCorrelativeScanMatcher::MatchFullSubmap
 * 在整个submap中做匹配 而不是在配置函数中定义的搜索窗口中做匹配。
 * 因为是在整个submap中做匹配，因此没必要定义initial_pose_estimation
 * 直接从整个地图的中点开始搜索就可以了。
 * @param point_cloud
 * @param min_score
 * @param score
 * @param pose_estimate
 * @return
 */
bool FastCorrelativeScanMatcher::MatchFullSubmap(
    const sensor::PointCloud2D& point_cloud,
    float min_score,
    float* score,
    transform::Rigid2d* pose_estimate) const
{
  // Compute a search window around the center of the submap that includes it
  // fully.
  // 计算一个搜索窗口 使得这个搜索串口包含整个Submap
  const SearchParameters search_parameters(
      1e6 * limits_.resolution(),  // Linear search window, 1e6 cells/direction.
      M_PI,  // Angular search window, 180 degrees in both directions.
      point_cloud, limits_.resolution());

  // 计算搜索窗口的中点 把这个中点作为搜索的起点
  const transform::Rigid2d center = transform::Rigid2d::Translation(
      limits_.max() -
      0.5 * limits_.resolution() *
          Eigen::Vector2d(limits_.cell_limits().num_y_cells,
                          limits_.cell_limits().num_x_cells));

  return MatchWithSearchParameters(search_parameters, center, point_cloud,
                                   min_score, score, pose_estimate);
}

/**
 * @brief FastCorrelativeScanMatcher::MatchWithSearchParameters
 * 根据搜索窗口和初始位置进行scan-match来进行位姿的优化。
 * 这个函数是最终的匹配函数　所有的东西都是通过这个函数来进行匹配的。
 * 得到的新的位姿的分数必须大于min_score
 * 这个函数被Match()和MatchFullSubmap()调用
 * @param search_parameters         主要设置搜索窗口的大小
 * @param initial_pose_estimate     初始的位姿
 * @param point_cloud               对应的激光数据
 * @param min_score                 接受位姿的最小的得分
 * @param score                     最优位姿的得分
 * @param pose_estimate             最优位姿
 * @return
 */
bool FastCorrelativeScanMatcher::MatchWithSearchParameters(
    SearchParameters search_parameters,
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud2D& point_cloud,
    float min_score,
    float* score,
    transform::Rigid2d* pose_estimate) const
{
  CHECK_NOTNULL(score);
  CHECK_NOTNULL(pose_estimate);

  //把激光数据旋转到世界坐标系中的0度的位置
  const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();
  const sensor::PointCloud2D rotated_point_cloud =
      sensor::TransformPointCloud2D(
          point_cloud,
          transform::Rigid2d::Rotation(initial_rotation).cast<float>());

  //生成一系列的rotated scans
  const std::vector<sensor::PointCloud2D> rotated_scans =
      GenerateRotatedScans(rotated_point_cloud, search_parameters);

  //把上面的rotated scans转换到世界坐标系中 然后转换到地图坐标系中
  //这里之后，所有激光点的坐标走在世界坐标系中了　或者说地图坐标系中。
  //这里的离散激光点　是在最细的分辨率的地图上面
  const std::vector<DiscreteScan> discrete_scans = DiscretizeScans(
      limits_, rotated_scans,
      Eigen::Translation2f(initial_pose_estimate.translation().x(),
                           initial_pose_estimate.translation().y()));

  search_parameters.ShrinkToFit(discrete_scans, limits_.cell_limits());

  //计算最低分辨率中的所有的候选解 最低分辨率是通过搜索树的层数、地图的分辨率计算出来的。
  //对于地图坐标系来说 最低分辨率=1<<h h表示搜索树的总的层数
  //这里不但对最低分辨率的所有候选解的得分进行了计算　同时还按照从大到小排列
  const std::vector<Candidate> lowest_resolution_candidates =
      ComputeLowestResolutionCandidates(discrete_scans, search_parameters);

  //用分枝定界方法来计算最优的候选解
  const Candidate best_candidate = BranchAndBound(
      discrete_scans, search_parameters, lowest_resolution_candidates,
      precomputation_grid_stack_->max_depth(), min_score);

  //如果计算出来的解大于最小的阈值 则认为匹配成功，返回对应的位姿
  if (best_candidate.score > min_score)
  {
    *score = best_candidate.score;
    *pose_estimate = transform::Rigid2d(
        {initial_pose_estimate.translation().x() + best_candidate.x,
         initial_pose_estimate.translation().y() + best_candidate.y},
        initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation));
    return true;
  }
  return false;
}

/**
 * @brief FastCorrelativeScanMatcher::ComputeLowestResolutionCandidates
 * 计算所有的最低分辨率的可行解 & 计算每个可行解的得分 & 按照从大到小的顺序排列
 * @param discrete_scans
 * @param search_parameters
 * @return
 */
std::vector<Candidate> FastCorrelativeScanMatcher::ComputeLowestResolutionCandidates(
    const std::vector<DiscreteScan>& discrete_scans,
    const SearchParameters& search_parameters) const
{
  //计算最低分辨率的所有的候选解
  std::vector<Candidate> lowest_resolution_candidates =
      GenerateLowestResolutionCandidates(search_parameters);

  //计算每个候选解的分数 注意用来评分的地图是最低分辨率的地图
  ScoreCandidates(
      precomputation_grid_stack_->Get(precomputation_grid_stack_->max_depth()),
      discrete_scans, search_parameters, &lowest_resolution_candidates);

  return lowest_resolution_candidates;
}

/**
 * @brief FastCorrelativeScanMatcher::GenerateLowestResolutionCandidates
 * 生成最低分辨率的所有的可行解 这个和一般的计算Candidates的不同在于在计算线性步长的时候要考虑分辨率的影响。
 * 线性步长不再是1 而是2^(h) h从0开始
 * 最低分辨率的解的个数为:(linear_search_window/linear_step_size)^2 * num_scans
 * 这个函数被ComputeLowestResolutionCandidates()调用
 * @param search_parameters
 * @return
 */
std::vector<Candidate> FastCorrelativeScanMatcher::GenerateLowestResolutionCandidates(
    const SearchParameters& search_parameters) const
{
  //计算步长的增量 在最高的分辨率中增量为1 在最低的分辨率中增量为2^(h) h从0开始
  const int linear_step_size = 1 << precomputation_grid_stack_->max_depth();

  //计算有多少个可行解 按照计算好的分辨率 x有多少的步长 y有多少的步长
  int num_candidates = 0;
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index)
  {
    const int num_lowest_resolution_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + linear_step_size) /
        linear_step_size;

    const int num_lowest_resolution_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + linear_step_size) /
        linear_step_size;

    num_candidates += num_lowest_resolution_linear_x_candidates *
                      num_lowest_resolution_linear_y_candidates;
  }

  //三层for循环  把每一个可行解都存入candidates中
  std::vector<Candidate> candidates;
  candidates.reserve(num_candidates);
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index)
  {
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         x_index_offset += linear_step_size)
    {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           y_index_offset += linear_step_size)
      {
        candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                search_parameters);
      }
    }
  }
  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}

/**
 * @brief FastCorrelativeScanMatcher::ScoreCandidates
 * 计算每个Candidates的得分 根据传入的地图在这个地图上进行搜索来计算得分
 * 这个函数被ComputeLowestResolutionCandidates()调用
 * @param precomputation_grid       用来计算分数的地图
 * @param discrete_scans            离散化的rotated scans
 * @param search_parameters         搜索的参数
 * @param candidates                所有的候选解
 */
void FastCorrelativeScanMatcher::ScoreCandidates(
    const PrecomputationGrid& precomputation_grid,
    const std::vector<DiscreteScan>& discrete_scans,
    const SearchParameters& search_parameters,
    std::vector<Candidate>* const candidates) const
{
  //枚举所有的候选解
  for (Candidate& candidate : *candidates)
  {
    int sum = 0;
    //每个候选解 枚举所有的激光点 累计占用概率log-odd
    //这里都是固定的角度的，因此激光点的坐标就等于激光点在车体坐标系的坐标加上候选解的坐标
    //这里的xy_index是已经激光雷达原始坐标计算出来的，因此需要加上这个候选解相对于原始位置的偏移
    for (const Eigen::Array2i& xy_index :
         discrete_scans[candidate.scan_index])
    {
      //激光点在地图坐标系中的坐标
      const Eigen::Array2i proposed_xy_index(
          xy_index.x() + candidate.x_index_offset,
          xy_index.y() + candidate.y_index_offset);

      sum += precomputation_grid.GetValue(proposed_xy_index);
    }

    //求平均并转换为概率
    candidate.score = PrecomputationGrid::ToProbability(
        sum / static_cast<float>(discrete_scans[candidate.scan_index].size()));
  }
  //按照从大到小的顺序排列所有的candidates
  std::sort(candidates->begin(), candidates->end(), std::greater<Candidate>());
}

/**
 * @brief FastCorrelativeScanMatcher::BranchAndBound
 * @param discrete_scans            离散的旋转激光数据discretescan rotate scan
 * @param search_parameters         搜索窗口的相应的参数
 * @param candidates                所有的可行解
 * @param candidate_depth           地图的层数(Multi-Level里面有多少个Level)　当前节点的深度　也就是当前节点的地图的层数
 * @param min_score                 能接受的最小的分数(也可以认为是当前的最优解的得分 凡是比当前最优解低的分数 一律不要)
 * 在分枝定界的方法中，一节node只表示一个角度。
 * 因此实际构造的束的根节点下面有N个1层子节点，N=rotated scans的数量。
 * 然后每个1层的节点下面都是4个子节点
 * 在进行更新best_socre的时候，只有叶子节点的得分才能被用来更新best_score。
 * best_score体现的是当前的最优解的得分(只有叶子节点才能被当做解)
 * @return
 */
Candidate FastCorrelativeScanMatcher::BranchAndBound(
    const std::vector<DiscreteScan>& discrete_scans,
    const SearchParameters& search_parameters,
    const std::vector<Candidate>& candidates,
    const int candidate_depth,
    float min_score) const
{
  //如果只有一层 那么最低分辨率中最好的就是全局最好的，直接返回
  //相当于是叶子节点 这个分数会用来更新父节点的best_score。
  //这个在返回之后　会用来更新bestScore
  if (candidate_depth == 0)
  {
    // Return the best candidate.
    return *candidates.begin();
  }

  Candidate best_high_resolution_candidate(0, 0, 0, search_parameters);
  best_high_resolution_candidate.score = min_score;

  //枚举所有的候选解　从高到低美剧
  for (const Candidate& candidate : candidates)
  {
    //如果某个候选解小于min_score可不需要再进行计算了。这里的min_score相当于当前搜索过的所有解中的最优解的得分
    //这里相当于定界 如果当前这颗字数的分数小于最优解的分数 则这颗子树可以直接被减枝
    //因为候选解是按照分数从大到小排列的
    //在进行迭代的时候，这个min_score是会不断的进行更新的。因为会不断的进行搜索。每次在子节点搜索到更优的解。这个值就会被更新。
    //min_score只有最最底层的叶子节点的时候，才会进行更新。
    if (candidate.score <= min_score)
    {
      break;
    }

    //开始进行分支
    std::vector<Candidate> higher_resolution_candidates;
    const int half_width = 1 << (candidate_depth - 1);
    //该节点分解为四个子节点 这里就是分枝
    for (int x_offset : {0, half_width})
    {
      //超出范围则不需要进行操作了
      if (candidate.x_index_offset + x_offset >
          search_parameters.linear_bounds[candidate.scan_index].max_x)
      {
        break;
      }

      for (int y_offset : {0, half_width})
      {
        //超出范围则不需要进行计算了
        if (candidate.y_index_offset + y_offset >
            search_parameters.linear_bounds[candidate.scan_index].max_y)
        {
          break;
        }

        //把这个可能存在更优解的地方放入队列中
        higher_resolution_candidates.emplace_back(
            candidate.scan_index, candidate.x_index_offset + x_offset,
            candidate.y_index_offset + y_offset, search_parameters);
      }
    }

    //计算所有的候选解(4个)的得分
    ScoreCandidates(precomputation_grid_stack_->Get(candidate_depth - 1),
                    discrete_scans, search_parameters,
                    &higher_resolution_candidates);

    //进行迭代求解最优值 这里相当于传进去最新的best_score来作为子节点的min_score
    //注意这个best_score相当于这颗子树下面的所有叶子节点的best_score
    best_high_resolution_candidate = std::max(
        best_high_resolution_candidate,
        BranchAndBound(discrete_scans, search_parameters,
                       higher_resolution_candidates, candidate_depth - 1,
                       best_high_resolution_candidate.score));
  }
  return best_high_resolution_candidate;
}

}  // namespace scan_matching
}  // namespace mapping_2d
}  // namespace cartographer
