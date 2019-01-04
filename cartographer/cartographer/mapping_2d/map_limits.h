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

#ifndef CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_
#define CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_

#include <utility>
#include <vector>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "../common/math.h"
#include "../mapping/trajectory_node.h"
#include "../mapping_2d/xy_index.h"
#include "../sensor/laser.h"
#include "../sensor/point_cloud.h"
#include "../transform/rigid_transform.h"
#include "../transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_2d {

// Defines the limits of a grid map. This class must remain inlined for
// performance reasons.

/*
 * 这个类主要储存了一些栅格地图的信息．
 * 比如说
 * cell的数量－－地图的最大范围 in cell
 * 地图的分辨率．
 * 地图的最大范围　地图最大范围 in meters
*/

class MapLimits
{
 public:
  MapLimits(const double resolution, const Eigen::Vector2d& max,
            const CellLimits& cell_limits)
      : resolution_(resolution), max_(max), cell_limits_(cell_limits)
  {
    CHECK_GT(resolution_, 0.);
    CHECK_GT(cell_limits.num_x_cells, 0.);
    CHECK_GT(cell_limits.num_y_cells, 0.);
  }

  // Returns the cell size in meters. All cells are square and the resolution is
  // the length of one side.
  // 返回地图的分辨率
  double resolution() const { return resolution_; }

  // Returns the corner of the limits, i.e., all pixels have positions with
  // smaller coordinates.
  // 返回地图最大的物理坐标(包围盒的最大的物理坐标)
  const Eigen::Vector2d& max() const { return max_; }

  // Returns the limits of the grid in number of cells.
  // 返回地图的最大的栅格坐标(包围盒的最大的栅格坐标)
  const CellLimits& cell_limits() const { return cell_limits_; }

  // Returns the index of the cell containing the point ('x', 'y') which may be
  // outside the map, i.e., negative or too large indices that will return
  // false for Contains().
  // 计算物理坐标(x,y)落在哪个栅格中。可以认为这个函数的功能为把物理坐标转换为栅格坐标
  Eigen::Array2i GetXYIndexOfCellContainingPoint(const double x,
                                                 const double y) const
  {
    // Index values are row major and the top left has Eigen::Array2i::Zero()
    // and contains (centered_max_x, centered_max_y). We need to flip and
    // rotate.
    return Eigen::Array2i(
        common::RoundToInt((max_.y() - y) / resolution_ - 0.5),
        common::RoundToInt((max_.x() - x) / resolution_ - 0.5));
  }

  // Returns true of the ProbabilityGrid contains 'xy_index'.
  // 地图是否包含栅格xy_index
  bool Contains(const Eigen::Array2i& xy_index) const
  {
    return (Eigen::Array2i(0, 0) <= xy_index).all() &&
           (xy_index <
            Eigen::Array2i(cell_limits_.num_x_cells, cell_limits_.num_y_cells))
               .all();
  }

  // Computes MapLimits that contain the origin, and all laser rays (both
  // returns and missing echoes) in the 'trajectory'.
  /**
   * @brief ComputeMapLimits
   * 计算目前地图的一些参数，分辨率、MAX_XY_in_meter、MAX_XY_in_cell
   * @param resolution              地图分辨率
   * @param trajectory_nodes        所有的节点(储存了所有的激光数据)
   * @return
   */
  static MapLimits ComputeMapLimits(
      const double resolution,
      const std::vector<mapping::TrajectoryNode>& trajectory_nodes)
  {
    //计算地图的包围盒
    Eigen::AlignedBox2f bounding_box = ComputeMapBoundingBox(trajectory_nodes);

    // Add some padding to ensure all rays are still contained in the map after
    // discretization.
    //增加一点padding，确保所有的范围确保所有的激光束离散化以后 依然在地图的范围内
    const float kPadding = 3.f * resolution;

    //计算地图的最大范围(in meter)
    bounding_box.min() -= kPadding * Eigen::Vector2f::Ones();
    bounding_box.max() += kPadding * Eigen::Vector2f::Ones();

    //计算有多少个栅格 或者说 最大栅格坐标的最大值(in cell)
    const Eigen::Vector2d pixel_sizes =
        bounding_box.sizes().cast<double>() / resolution;

    //对应的地图参数
    return MapLimits(resolution, bounding_box.max().cast<double>(),
                     CellLimits(common::RoundToInt(pixel_sizes.y()),
                                common::RoundToInt(pixel_sizes.x())));
  }

  /**
   * @brief ComputeMapBoundingBox
   * 计算到目前为止地图的包围盒的大小
   * 这个函数主要被上面的ComputeMapLimits()调用
   * @param trajectory_nodes
   * @return
   */
  static Eigen::AlignedBox2f ComputeMapBoundingBox(
      const std::vector<mapping::TrajectoryNode>& trajectory_nodes)
  {
    Eigen::AlignedBox2f bounding_box(Eigen::Vector2f::Zero());

    //枚举所有的节点
    for (const auto& node : trajectory_nodes)
    {
      const auto& data = *node.constant_data;
      //如果是3d的雷达数据
      if (!data.laser_fan_3d.returns.empty())
      {
        //把雷达数据投影到世界坐标系
        const sensor::LaserFan3D laser_fan_3d = sensor::TransformLaserFan3D(
            Decompress(data.laser_fan_3d), node.pose.cast<float>());

        //进行包围盒的扩展
        bounding_box.extend(laser_fan_3d.origin.head<2>());
        for (const Eigen::Vector3f& hit : laser_fan_3d.returns)
        {
          bounding_box.extend(hit.head<2>());
        }
      }
      //如果是2d的雷达数据
      else
      {
        //把雷达数据投影到世界坐标系
        const sensor::LaserFan laser_fan = sensor::TransformLaserFan(
            data.laser_fan, transform::Project2D(node.pose).cast<float>());

        bounding_box.extend(laser_fan.origin);

        //枚举这帧雷达数据中的每一个激光点 来进行包围盒的拓展
        for (const Eigen::Vector2f& hit : laser_fan.point_cloud)
        {
          bounding_box.extend(hit);
        }
        for (const Eigen::Vector2f& miss : laser_fan.missing_echo_point_cloud)
        {
          bounding_box.extend(miss);
        }
      }
    }
    return bounding_box;
  }

 private:
  double resolution_;               //地图的解析数据
  Eigen::Vector2d max_;             //地图最大的物理坐标
  CellLimits cell_limits_;          //地图最大的栅格坐标
};

}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_MAP_LIMITS_H_
