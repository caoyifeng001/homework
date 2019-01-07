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

#include "../mapping_2d/scan_matching/real_time_correlative_scan_matcher.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

#include "eigen3/Eigen/Geometry"
#include "../common/lua_parameter_dictionary.h"
#include "../common/math.h"
#include "../mapping_2d/probability_grid.h"
#include "../sensor/point_cloud.h"
#include "../transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_2d {
namespace scan_matching {

//参数配置
proto::RealTimeCorrelativeScanMatcherOptions
CreateRealTimeCorrelativeScanMatcherOptions(
    common::LuaParameterDictionary* const parameter_dictionary)
{
  proto::RealTimeCorrelativeScanMatcherOptions options;

  //CSM来进行搜索的窗口的大小
  options.set_linear_search_window(
      parameter_dictionary->GetDouble("linear_search_window"));
  options.set_angular_search_window(
      parameter_dictionary->GetDouble("angular_search_window"));

  //这两个参数是用来惩罚离初始值比较远的可行解的 离初始值越远 惩罚越严重
  //这种情况应该是对于环境比较相似的环境中会有比较好的效果，例如长走廊
  options.set_translation_delta_cost_weight(
      parameter_dictionary->GetDouble("translation_delta_cost_weight"));
  options.set_rotation_delta_cost_weight(
      parameter_dictionary->GetDouble("rotation_delta_cost_weight"));

  CHECK_GE(options.translation_delta_cost_weight(), 0.);
  CHECK_GE(options.rotation_delta_cost_weight(), 0.);
  return options;
}

RealTimeCorrelativeScanMatcher::RealTimeCorrelativeScanMatcher(
    const proto::RealTimeCorrelativeScanMatcherOptions& options)
    : options_(options) {}

/**
 * @brief RealTimeCorrelativeScanMatcher::GenerateExhaustiveSearchCandidates
 * 计算出来所有的要进行搜索的解。
 * 即得到三层for循环的所有的组合
 * 在Match()里面被调用
 * @param search_parameters
 * @return
 */
std::vector<Candidate>
RealTimeCorrelativeScanMatcher::GenerateExhaustiveSearchCandidates(
    const SearchParameters& search_parameters) const
{
  //计算 一共有多少的candidates。即三层循环中一共要计算多少次score
  //相当于num_scans*num_linear_x_candidates*num_linear_y_candidates
  //这里的num_scans表示一共由多少的角度搜索次数
  int num_candidates = 0;
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index)
  {
    //计算x的候选解
    const int num_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + 1);

    //计算y的候选解
    const int num_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + 1);

    //累加候选解的个数
    num_candidates += num_linear_x_candidates * num_linear_y_candidates;
  }

  //获得三层for循环的组合起来的所有的解　这里所有的点角度都是下标　xy都是grid_index
  std::vector<Candidate> candidates;
  candidates.reserve(num_candidates);
  //最外层循环表示角度
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index)
  {
    //内部两层循环表示线性搜索空间
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         ++x_index_offset)
    {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           ++y_index_offset)
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
 * @brief RealTimeCorrelativeScanMatcher::Match
 * 实现的是RealTime CSM论文里面的方法:Computing 2D Slices
 * 这里并没有进行多分辨率地图的构建　因此这里实际上就是进行枚举而已。
 * 并没有进行什么高级的加速策略
 * 用来进行scan-match来优化机器人的位姿
 * @param initial_pose_estimate         初始的机器人的位姿
 * @param point_cloud                   位于平面机器人坐标系中的点云数据
 * @param probability_grid              对应的栅格地图
 * @param pose_estimate                 优化之后用于返回的位姿
 * @return
 */
double RealTimeCorrelativeScanMatcher::Match(
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud2D& point_cloud,
    const ProbabilityGrid& probability_grid,
    transform::Rigid2d* pose_estimate) const
{
  CHECK_NOTNULL(pose_estimate);

  //得到机器人初始的位姿
  const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();

  //把激光数据旋转到和世界坐标系平行的坐标系中
  //只所有要进行这样的旋转是因为把激光数据旋转会0度。
  //因为接下来的搜索中，我们要对角度进行离散化搜索，从0度开始进行搜索
  const sensor::PointCloud2D rotated_point_cloud =
      sensor::TransformPointCloud2D(
          point_cloud,
          transform::Rigid2d::Rotation(initial_rotation).cast<float>());

  //设置scan-match的参数
  const SearchParameters search_parameters(
      options_.linear_search_window(), options_.angular_search_window(),
      rotated_point_cloud, probability_grid.limits().resolution());

  //把激光数据经过一些列的旋转操作 得到一系列的laser_scans。这里相当于已经把所有的不同角度的激光数据进行投影了。
  //后面枚举x,y的时候，就没必要进行投影了。这也是computing 2d slice比直接三层for循环快的原因
  //这一系列的laser_scan都是经过旋转得到的
  const std::vector<sensor::PointCloud2D> rotated_scans =
      GenerateRotatedScans(rotated_point_cloud, search_parameters);

  //把上面旋转得到的激光雷达数据 转换到 地图坐标系中
  //经过这个函数之后，所有的激光数据的原点都和世界坐标系重合。
  //而角度也都是在世界坐标系中描述的。
  //因此对于各个不同的x_offset y_offset只需要进行激光端点的平移就可以
  const std::vector<DiscreteScan> discrete_scans = DiscretizeScans(
      probability_grid.limits(), rotated_scans,
      Eigen::Translation2f(initial_pose_estimate.translation().x(),
                           initial_pose_estimate.translation().y()));


  //得到整个搜索空间里面的所有的候选解
  std::vector<Candidate> candidates =
      GenerateExhaustiveSearchCandidates(search_parameters);

  //计算空间中所有的候选解的得分
  ScoreCandidates(probability_grid, discrete_scans, search_parameters,
                  &candidates);

  //找到得到最大的候选解candidate
  const Candidate& best_candidate =
      *std::max_element(candidates.begin(), candidates.end());

  //候选解的位姿即为优化过后的位姿
  *pose_estimate = transform::Rigid2d(
      {initial_pose_estimate.translation().x() + best_candidate.x,
       initial_pose_estimate.translation().y() + best_candidate.y},
      initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation));

  return best_candidate.score;
}

/**
 * @brief RealTimeCorrelativeScanMatcher::ScoreCandidates
 * 计算每个候选解的得分。
 * 得到的计算方式为候选解中所有击中的栅格的占用概率的平均值。
 * 并且如果对应的候选解离初始位置比较远 则会施加惩罚
 * 在Match()里面被调用
 * @param probability_grid              对应的栅格地图
 * @param discrete_scans                所有的经过一系列旋转的激光数据  地图坐标系中
 * @param search_parameters             搜索的一些参数 包括步长，窗口大小等等参数
 * @param candidates                    所有的候选解
 */
void RealTimeCorrelativeScanMatcher::ScoreCandidates(
    const ProbabilityGrid& probability_grid,
    const std::vector<DiscreteScan>& discrete_scans,
    const SearchParameters& search_parameters,
    std::vector<Candidate>* const candidates) const
{
  //枚举所有的candidate
  for (Candidate& candidate : *candidates)
  {
    candidate.score = 0.f;

    //每个candidate都需要枚举所有的激光点
    for (const Eigen::Array2i& xy_index :
         discrete_scans[candidate.scan_index])
    {
      //因为事先经过旋转，所以这里只需要相加就可以
      const Eigen::Array2i proposed_xy_index(
          xy_index.x() + candidate.x_index_offset,
          xy_index.y() + candidate.y_index_offset);

      //计算占用概率的累加和
      const float probability =
          probability_grid.GetProbability(proposed_xy_index);
      candidate.score += probability;
    }

    //进行平均
    candidate.score /=
        static_cast<float>(discrete_scans[candidate.scan_index].size());

    //施加惩罚 离初始位置越远 惩罚的越严重
    candidate.score *=
        std::exp(-common::Pow2(std::hypot(candidate.x, candidate.y) *
                                   options_.translation_delta_cost_weight() +
                               std::abs(candidate.orientation) *
                                   options_.rotation_delta_cost_weight()));

    CHECK_GT(candidate.score, 0.f);
  }
}

}  // namespace scan_matching
}  // namespace mapping_2d
}  // namespace cartographer
