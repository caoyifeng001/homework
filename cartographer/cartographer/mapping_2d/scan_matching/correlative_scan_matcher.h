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

#ifndef CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_CORRELATIVE_SCAN_MATCHER_H_
#define CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_CORRELATIVE_SCAN_MATCHER_H_

// 这里面实现的函数和数据结构都是真正的real-time correlative scan match做辅助的。
// 这个类里面本身没有scan-match的函数。
// 这里面的函数和数据结构都是为了real_time_correlative_scan_matcher & fast_correlative_scan_matcher写的。
// 这里面的函数也会在real_time_correlative_scan_matcher里面被调用
// real_time_correlative_scan_matcher实现的是论文Real-Time Correlative Scan Match中的Computing 2D Slices方法
// fast_correlative_scan_matcher实现的是论文Real-Time Correlative Scan Match中的Multi-Level Resolution方法
#include <vector>

#include "eigen3/Eigen/Core"
#include "../common/lua_parameter_dictionary.h"
#include "../mapping_2d/map_limits.h"
#include "../mapping_2d/xy_index.h"
#include "../sensor/point_cloud.h"

namespace cartographer {
namespace mapping_2d {
namespace scan_matching {

typedef std::vector<Eigen::Array2i> DiscreteScan;

// Describes the search space.
struct SearchParameters
{
  // Linear search window in pixel offsets; bounds are inclusive.
  // 地图中的线性搜索窗口　表示的是栅格坐标
  struct LinearBounds
  {
    int min_x;
    int max_x;
    int min_y;
    int max_y;
  };

  //设置CSM搜索参数
  SearchParameters(double linear_search_window, double angular_search_window,
                   const sensor::PointCloud2D& point_cloud, double resolution);

  // For testing.
  SearchParameters(int num_linear_perturbations, int num_angular_perturbations,
                   double angular_perturbation_step_size, double resolution);

  // Tightens the search window as much as possible.
  // 把搜索窗口尽可能的弄小
  void ShrinkToFit(const std::vector<DiscreteScan>& scans,
                   const CellLimits& cell_limits);

  int num_angular_perturbations;            //角度搜索一半范围的步长
  double angular_perturbation_step_size;    //角度的搜索步长
  double resolution;                        //占用栅格地图的分辨率
  int num_scans;                            //rotated scan的数量
  std::vector<LinearBounds> linear_bounds;  //每个rotated scan都要有一个对应的linear_bound Per rotated scans.
};

// Generates a collection of rotated scans.
// 生成一系列的经过各个角度旋转的激光雷达数据
std::vector<sensor::PointCloud2D> GenerateRotatedScans(
    const sensor::PointCloud2D& point_cloud,
    const SearchParameters& search_parameters);

// Translates and discretizes the rotated scans into a vector of integer
// indices.
// 把rotate scan平移到世界坐标系中 & 进行离散化得到地图坐标
std::vector<DiscreteScan> DiscretizeScans(
    const MapLimits& map_limits, const std::vector<sensor::PointCloud2D>& scans,
    const Eigen::Translation2f& initial_translation);

// A possible solution.
// 进行scan-match时候的一个可行解 或者说是 一个搜索节点
struct Candidate
{
  Candidate(const int init_scan_index, const int init_x_index_offset,
            const int init_y_index_offset,
            const SearchParameters& search_parameters)
      : scan_index(init_scan_index),
        x_index_offset(init_x_index_offset),
        y_index_offset(init_y_index_offset),
        x(-y_index_offset * search_parameters.resolution),
        y(-x_index_offset * search_parameters.resolution),
        orientation((scan_index - search_parameters.num_angular_perturbations) *
                    search_parameters.angular_perturbation_step_size) {}

  // Index into the rotated scans vector.
  // 属于第多少个旋转的旋转
  int scan_index = 0;

  // Linear offset from the initial pose.
  // 离初始位置的线性位移　这个位移是用栅格坐标的　这个跟上面的scan_index是同一个性质的。
  // 最小的为linearbound.minx,最大为linearbound.maxx
  // 其中linearbound.minx = - linearbound.maxx
  int x_index_offset = 0;
  int y_index_offset = 0;

  // Pose of this Candidate relative to the initial pose.
  // 这个可行解相对于初始位置的位姿 (物理坐标) 把上面的index　转换成物理坐标
  // 不知道为什么这里的x和y跟上面的下标是反过来的。
  // 相对于其实解的物理坐标
  double x = 0.;
  double y = 0.;
  double orientation = 0.;

  // Score, higher is better.
  // 该可行解的得分
  float score = 0.f;

  bool operator<(const Candidate& other) const { return score < other.score; }
  bool operator>(const Candidate& other) const { return score > other.score; }
};

}  // namespace scan_matching
}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_CORRELATIVE_SCAN_MATCHER_H_
