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

#ifndef CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_
#define CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "../common/math.h"
#include "../common/port.h"
#include "../mapping/probability_values.h"
#include "../mapping_2d/map_limits.h"
#include "../mapping_2d/xy_index.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_2d {

// Represents a 2D grid of probabilities.
/**
 * @brief The ProbabilityGrid class
 * 覆盖栅格地图．
 * 每个栅格的值表示该点被障碍物占用的概率
 */
class ProbabilityGrid
{
 public:
  explicit ProbabilityGrid(const MapLimits& limits)
      : limits_(limits),
        cells_(limits_.cell_limits().num_x_cells *
                   limits_.cell_limits().num_y_cells,
               mapping::kUnknownProbabilityValue),
        max_x_(0),
        max_y_(0),
        min_x_(limits_.cell_limits().num_x_cells - 1),
        min_y_(limits_.cell_limits().num_y_cells - 1) {}

  // Returns the limits of this ProbabilityGrid.
  // 返回地图的参数 包括分辨率 最大物理坐标 最大栅格坐标之类的
  const MapLimits& limits() const { return limits_; }

  // Starts the next update sequence.
  void StartUpdate()
  {
    while (!update_indices_.empty())
    {
      DCHECK_GE(cells_[update_indices_.back()], mapping::kUpdateMarker);
      cells_[update_indices_.back()] -= mapping::kUpdateMarker;
      update_indices_.pop_back();
    }
  }

  // Sets the probability of the cell at 'xy_index' to the given 'probability'.
  // Only allowed if the cell was unknown before.
  // 设置index为xy_index的cell的概率　只有这个cell为unknown时才可以这么做
  void SetProbability(const Eigen::Array2i& xy_index, const float probability)
  {
    uint16& cell = cells_[GetIndexOfCell(xy_index)];
    CHECK_EQ(cell, mapping::kUnknownProbabilityValue);
    cell = mapping::ProbabilityToValue(probability);
    UpdateBounds(xy_index);
  }

  // Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
  // to the probability of the cell at 'xy_index' if the cell has not already
  // been updated. Multiple updates of the same cell will be ignored until
  // StartUpdate() is called. Returns true if the cell was updated.
  //
  // If this is the first call to ApplyOdds() for the specified cell, its value
  // will be set to probability corresponding to 'odds'.
  // 进行地图更新xy_index表示一系列的点的坐标　来表示哪些栅格要更新
  //
  bool ApplyLookupTable(const Eigen::Array2i& xy_index,
                        const std::vector<uint16>& table)
  {
    DCHECK_EQ(table.size(), mapping::kUpdateMarker);
    const int cell_index = GetIndexOfCell(xy_index);
    uint16& cell = cells_[cell_index];
    if (cell >= mapping::kUpdateMarker)
    {
      return false;
    }

    //存储下来所有要更新的栅格的坐标
    update_indices_.push_back(cell_index);
    cell = table[cell];

    DCHECK_GE(cell, mapping::kUpdateMarker);

    //更新地图的边界
    UpdateBounds(xy_index);
    return true;
  }

  // Returns the probability of the cell with 'xy_index'.
  // 返回下标为xy_index的栅格的概率值
  float GetProbability(const Eigen::Array2i& xy_index) const
  {
    if (limits_.Contains(xy_index))
    {
      return mapping::ValueToProbability(cells_[GetIndexOfCell(xy_index)]);
    }
    return mapping::kMinProbability;
  }

  // Returns the probability of the cell containing the point ('x', 'y').
  // 得到(x,y)出的cell的概率
  float GetProbability(const double x, const double y) const
  {
    return GetProbability(limits_.GetXYIndexOfCellContainingPoint(x, y));
  }

  // Returns true if the probability at the specified index is known.
  // 返回栅格xy_index是否是unknown
  bool IsKnown(const Eigen::Array2i& xy_index) const
  {
    return limits_.Contains(xy_index) &&
           cells_[GetIndexOfCell(xy_index)] !=
               mapping::kUnknownProbabilityValue;
  }

  // Fills in 'offset' and 'limits' to define a subregion of that contains all
  // known cells.
  // 计算裁剪参数　即计算一个矩形，使得这个矩形包括了所有的cells
  // 相当于已知区域的外接矩形
  // offset　表示起点
  // limits　表示范围
  void ComputeCroppedLimits(Eigen::Array2i* const offset,
                            CellLimits* const limits) const
  {
    *offset = Eigen::Array2i(min_x_, min_y_);
    *limits = CellLimits(std::max(max_x_, min_x_) - min_x_ + 1,
                         std::max(max_y_, min_y_) - min_y_ + 1);
  }

  // Grows the map as necessary to include 'x' and 'y'. This changes the meaning
  // of these coordinates going forward. This method must be called immediately
  // after 'StartUpdate', before any calls to 'ApplyLookupTable'.
  // 拓展地图的边界,使得地图会包含点(x,y)
  // 新扩展出来的点被设置为unknown
  void GrowLimits(const double x, const double y)
  {
    CHECK(update_indices_.empty());
    while (!limits_.Contains(limits_.GetXYIndexOfCellContainingPoint(x, y)))
    {
      const int x_offset = limits_.cell_limits().num_x_cells / 2;
      const int y_offset = limits_.cell_limits().num_y_cells / 2;

      const MapLimits new_limits(
          limits_.resolution(),
          limits_.max() +
              limits_.resolution() * Eigen::Vector2d(y_offset, x_offset),
          CellLimits(2 * limits_.cell_limits().num_x_cells,
                     2 * limits_.cell_limits().num_y_cells));

      const int stride = new_limits.cell_limits().num_x_cells;
      const int offset = x_offset + stride * y_offset;
      const int new_size = new_limits.cell_limits().num_x_cells *
                           new_limits.cell_limits().num_y_cells;
      std::vector<uint16> new_cells(new_size,
                                    mapping::kUnknownProbabilityValue);
      for (int i = 0; i < limits_.cell_limits().num_y_cells; ++i)
      {
        for (int j = 0; j < limits_.cell_limits().num_x_cells; ++j)
        {
          new_cells[offset + j + i * stride] =
              cells_[j + i * limits_.cell_limits().num_x_cells];
        }
      }
      cells_ = new_cells;
      limits_ = new_limits;
      min_x_ += x_offset;
      min_y_ += y_offset;
      max_x_ += x_offset;
      max_y_ += y_offset;
    }
  }

 private:
  // Returns the index of the cell at 'xy_index'.
  // 把(x,y)坐标转化为index
  int GetIndexOfCell(const Eigen::Array2i& xy_index) const
  {
    CHECK(limits_.Contains(xy_index)) << xy_index;
    return limits_.cell_limits().num_x_cells * xy_index.y() + xy_index.x();
  }

  // 扩展地图的边界使得它包含点xy_index
  // 这个跟ros里面的costmap_2d比较像
  void UpdateBounds(const Eigen::Array2i& xy_index)
  {
    min_x_ = std::min(min_x_, xy_index.x());
    min_y_ = std::min(min_y_, xy_index.y());
    max_x_ = std::max(max_x_, xy_index.x());
    max_y_ = std::max(max_y_, xy_index.y());
  }

  MapLimits limits_;                //地图的一些参数
  std::vector<uint16> cells_;       //地图的栅格 Highest bit is update marker.
  std::vector<int> update_indices_; //

  // Minimum and maximum cell coordinates of know cells to efficiently compute
  // cropping limits.
  // 已知栅格的边界值 用来快速计算裁剪参数
  int max_x_;
  int max_y_;
  int min_x_;
  int min_y_;
};

}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_
