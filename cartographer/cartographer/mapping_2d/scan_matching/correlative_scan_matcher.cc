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

#include "../mapping_2d/scan_matching/correlative_scan_matcher.h"

#include <cmath>

#include "../common/math.h"

namespace cartographer {
namespace mapping_2d {
namespace scan_matching {

/**
 * @brief SearchParameters::SearchParameters
 * 根据设置的线性搜索窗口 & 角度搜索窗口的大小 来计算一些搜索参数。实际的范围是设置的2倍。因为正负都要搜索
 * 这里比较要注意的就是角度搜索步长的确定。
 * 角度步长的确定方法见论文"Real-Time Loop Closure in 2D LIDAR SLAM"的式6,式7
 * @param linear_search_window      线性窗口的大小
 * @param angular_search_window     角度窗口的大小
 * @param point_cloud               对应的点云数据
 * @param resolution                地图的分辨率
 */
SearchParameters::SearchParameters(const double linear_search_window,
                                   const double angular_search_window,
                                   const sensor::PointCloud2D& point_cloud,
                                   const double resolution)
    : resolution(resolution)
{
  // We set this value to something on the order of resolution to make sure that
  // the std::acos() below is defined.
  float max_scan_range = 3.f * resolution;

  //得到激光束中的最大距离
  for (const Eigen::Vector2f& point : point_cloud)
  {
    const float range = point.norm();
    max_scan_range = std::max(range, max_scan_range);
  }

  //为保险起见 乘以一个略小于1的值
  const double kSafetyMargin = 1. - 1e-3;

  //计算搜索时候的角度步长  计算的原则是当角度步长++的时候，最长的激光的唯一不会超过resolution
  //见论文"Real-Time Loop Closure in 2D LIDAR SLAM"的式6,式7
  //对于一个６m的激光雷达来说  2.5cm的分辨率 -->angular_step_size = 0.2degree
  //                         5cm的分辨率 -->angular_step_size = 0.48degree
  angular_perturbation_step_size =
      kSafetyMargin *
      std::acos(1. -
                common::Pow2(resolution) / (2. * common::Pow2(max_scan_range)));

  //根据角度步长和搜索窗口的大小计算角度有多少个离散值
  num_angular_perturbations =
      std::ceil(angular_search_window / angular_perturbation_step_size);

  //由于要正负两个方向都搜索，因此最终的离散值要*2
  num_scans = 2 * num_angular_perturbations + 1;

  //线性搜索的离散值多少
  const int num_linear_perturbations =
      std::ceil(linear_search_window / resolution);

  linear_bounds.reserve(num_scans);

  //每个旋转都需要有一个对应的线性窗口
  for (int i = 0; i != num_scans; ++i)
  {
    linear_bounds.push_back(
        LinearBounds{-num_linear_perturbations, num_linear_perturbations,
                     -num_linear_perturbations, num_linear_perturbations});
  }
}

SearchParameters::SearchParameters(const int num_linear_perturbations,
                                   const int num_angular_perturbations,
                                   const double angular_perturbation_step_size,
                                   const double resolution)
    : num_angular_perturbations(num_angular_perturbations),
      angular_perturbation_step_size(angular_perturbation_step_size),
      resolution(resolution),
      num_scans(2 * num_angular_perturbations + 1)
{
  linear_bounds.reserve(num_scans);
  for (int i = 0; i != num_scans; ++i)
  {
    linear_bounds.push_back(
        LinearBounds{-num_linear_perturbations, num_linear_perturbations,
                     -num_linear_perturbations, num_linear_perturbations});
  }
}

void SearchParameters::ShrinkToFit(const std::vector<DiscreteScan>& scans,
                                   const CellLimits& cell_limits)
{
  CHECK_EQ(scans.size(), num_scans);
  CHECK_EQ(linear_bounds.size(), num_scans);
  for (int i = 0; i != num_scans; ++i)
  {
    Eigen::Array2i min_bound = Eigen::Array2i::Zero();
    Eigen::Array2i max_bound = Eigen::Array2i::Zero();
    for (const Eigen::Array2i& xy_index : scans[i])
    {
      min_bound = min_bound.min(-xy_index);
      max_bound = max_bound.max(Eigen::Array2i(cell_limits.num_x_cells - 1,
                                               cell_limits.num_y_cells - 1) -
                                xy_index);
    }
    linear_bounds[i].min_x = std::max(linear_bounds[i].min_x, min_bound.x());
    linear_bounds[i].max_x = std::min(linear_bounds[i].max_x, max_bound.x());
    linear_bounds[i].min_y = std::max(linear_bounds[i].min_y, min_bound.y());
    linear_bounds[i].max_y = std::min(linear_bounds[i].max_y, max_bound.y());
  }
}

/**
 * @brief GenerateRotatedScans
 * 根据搜索参数把激光数据进行多次旋转 相当于在进行compute 2d slice的时候进行最外层的角度循环
 * 这里的激光进行转化之后，虽然角度和世界坐标系重合，但是原点依然是不重合的
 * @param point_cloud           初始角度为0的点云数据
 * @param search_parameters     对应的搜索函数
 * @return                      经过一系列旋转之后的激光点云
 */
std::vector<sensor::PointCloud2D> GenerateRotatedScans(
    const sensor::PointCloud2D& point_cloud,
    const SearchParameters& search_parameters)
{
  std::vector<sensor::PointCloud2D> rotated_scans;
  rotated_scans.reserve(search_parameters.num_scans);

  //搜索的角度的起点 -offset
  double delta_theta = -search_parameters.num_angular_perturbations *
                       search_parameters.angular_perturbation_step_size;

  for (int scan_index = 0; scan_index < search_parameters.num_scans;
       ++scan_index,
           delta_theta += search_parameters.angular_perturbation_step_size)
  {
    //把角度进行旋转并转换为点云
    rotated_scans.push_back(sensor::TransformPointCloud2D(
        point_cloud, transform::Rigid2f::Rotation(delta_theta)));
  }
  return rotated_scans;
}

/**
 * @brief DiscretizeScans
 * 把一系列的激光数据scans转换到世界坐标系中的坐标(initial_translation)。
 * 因为在进行rotated scans生成的时候，激光数据的角度已经和世界坐标系重合了。
 * 因此这里进行转换的时候只需要进行平移就可以了。
 * 然后把世界坐标系中的物理坐标离散化，转换为地图坐标系的坐标
 * 这个函数返回一些列地图坐标系中的激光扫描数据
 *
 * 经过这个函数之后，各个激光数据角度都和世界坐标系对齐了。
 * 原点都和世界坐标系重合了。
 *
 * @param map_limits            地图的分辨率等信息
 * @param scans                 激光数据
 * @param initial_translation   激光数据的原点
 * @return
 */
std::vector<DiscreteScan> DiscretizeScans(
    const MapLimits& map_limits, const std::vector<sensor::PointCloud2D>& scans,
    const Eigen::Translation2f& initial_translation)
{
  std::vector<DiscreteScan> discrete_scans;
  discrete_scans.reserve(scans.size());

  //all
  for (const sensor::PointCloud2D& scan : scans)
  {
    discrete_scans.emplace_back();
    discrete_scans.back().reserve(scan.size());

    for (const Eigen::Vector2f& point : scan)
    {
      //转换到世界坐标系中
      const Eigen::Vector2f translated_point = initial_translation * point;

      //把世界坐标系中的点转换到地图坐标系中
      discrete_scans.back().push_back(
          map_limits.GetXYIndexOfCellContainingPoint(translated_point.x(),
                                                     translated_point.y()));
    }
  }
  return discrete_scans;
}

}  // namespace scan_matching
}  // namespace mapping_2d
}  // namespace cartographer
