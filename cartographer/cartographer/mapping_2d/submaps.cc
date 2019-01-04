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

#include "../mapping_2d/submaps.h"

#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>

#include "eigen3/Eigen/Geometry"
#include "../common/make_unique.h"
#include "../common/port.h"
#include "glog/logging.h"
#include "webp/encode.h"

namespace cartographer {
namespace mapping_2d {

namespace {

void WriteDebugImage(const string& filename,
                     const ProbabilityGrid& probability_grid)
{
  constexpr int kUnknown = 128;
  const mapping_2d::CellLimits& cell_limits =
      probability_grid.limits().cell_limits();
  const int width = cell_limits.num_x_cells;
  const int height = cell_limits.num_y_cells;
  std::vector<uint8_t> rgb;
  for (const Eigen::Array2i& xy_index : mapping_2d::XYIndexRangeIterator(
           probability_grid.limits().cell_limits()))
  {
    CHECK(probability_grid.limits().Contains(xy_index));
    const uint8_t value =
        probability_grid.IsKnown(xy_index)
            ? common::RoundToInt(
                  (1. - probability_grid.GetProbability(xy_index)) * 255 + 0)
            : kUnknown;
    rgb.push_back(value);
    rgb.push_back(value);
    rgb.push_back(value);
  }
  uint8_t* output = nullptr;
  size_t output_size =
      WebPEncodeLosslessRGB(rgb.data(), width, height, 3 * width, &output);
  std::unique_ptr<uint8_t, void (*)(void*)> output_deleter(output, std::free);
  std::ofstream output_file(filename, std::ios::out | std::ios::binary);
  output_file.write(reinterpret_cast<char*>(output), output_size);
  output_file.close();
  CHECK(output_file) << "Writing " << filename << " failed.";
}

}  // namespace

/**
 * @brief ComputeCroppedProbabilityGrid
 * 计算经过裁剪的覆盖栅格地图
 * 这个函数在FinishSubmap()中被调用。
 * 在finished一个submap的时候都会对原本的地图进行裁剪。就需要调用这个函数
 * @param probability_grid  待裁剪的地图
 * @return 裁剪过后的地图
 */
ProbabilityGrid ComputeCroppedProbabilityGrid(
    const ProbabilityGrid& probability_grid)
{
  //计算得到最小外接矩形
  //offset表示起点 limits表示范围
  Eigen::Array2i offset;
  CellLimits limits;
  probability_grid.ComputeCroppedLimits(&offset, &limits);

  //得到分辨率
  const double resolution = probability_grid.limits().resolution();

  //得到物理坐标的范围
  const Eigen::Vector2d max =
      probability_grid.limits().max() -
      resolution * Eigen::Vector2d(offset.y(), offset.x());

  //生成新的覆盖栅格地图
  ProbabilityGrid cropped_grid(MapLimits(resolution, max, limits));

  //对新的覆盖栅格地图进行赋值
  cropped_grid.StartUpdate();
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(limits))
  {
    if (probability_grid.IsKnown(xy_index + offset))
    {
      cropped_grid.SetProbability(
          xy_index, probability_grid.GetProbability(xy_index + offset));
    }
  }
  return cropped_grid;
}

proto::SubmapsOptions CreateSubmapsOptions(
    common::LuaParameterDictionary* const parameter_dictionary)
{
  proto::SubmapsOptions options;
  options.set_resolution(parameter_dictionary->GetDouble("resolution"));
  options.set_half_length(parameter_dictionary->GetDouble("half_length"));
  options.set_num_laser_fans(
      parameter_dictionary->GetNonNegativeInt("num_laser_fans"));
  options.set_output_debug_images(
      parameter_dictionary->GetBool("output_debug_images"));
  *options.mutable_laser_fan_inserter_options() = CreateLaserFanInserterOptions(
      parameter_dictionary->GetDictionary("laser_fan_inserter").get());
  CHECK_GT(options.num_laser_fans(), 0);
  return options;
}

Submap::Submap(const MapLimits& limits, const Eigen::Vector2f& origin,
               const int begin_laser_fan_index)
    : mapping::Submap(Eigen::Vector3f(origin.x(), origin.y(), 0.),
                      begin_laser_fan_index),
      probability_grid(limits) {}

Submaps::Submaps(const proto::SubmapsOptions& options)
    : options_(options),
      laser_fan_inserter_(options.laser_fan_inserter_options())
{
  // We always want to have at least one likelihood field which we can return,
  // and will create it at the origin in absence of a better choice.
  AddSubmap(Eigen::Vector2f::Zero());
}

/**
 * @brief Submaps::InsertLaserFan
 * 插入一个激光雷达到submaps中．
 * @param laser_fan
 */
void Submaps::InsertLaserFan(const sensor::LaserFan& laser_fan)
{
  CHECK_LT(num_laser_fans_, std::numeric_limits<int>::max());

  //激光数据的id
  ++num_laser_fans_;
  //枚举所有的需要被插入的submap 实际上就是最近两个submap。因为只有最近两个submap还没有finish
  for (const int index : insertion_indices())
  {
    Submap* submap = submaps_[index].get();
    CHECK(submap->finished_probability_grid == nullptr);

    laser_fan_inserter_.Insert(laser_fan, &submap->probability_grid);

    submap->end_laser_fan_index = num_laser_fans_;
  }

  //如果最近submap中的激光数量已经满足插入新submap的要求。那么则需要把最近的submap设置为finish。
  //因为最近的submap，size()-1,的激光雷达数据达到了options_.num_laser_fans()就需要把size()-2设置为完成。
  //所以一个submap中的激光帧的数量为options_.num_laser_fans()*2。
  //同时已这帧激光位姿为起点 新建一个submap
  ++num_laser_fans_in_last_submap_;
  if (num_laser_fans_in_last_submap_ == options_.num_laser_fans())
  {
    AddSubmap(laser_fan.origin);
  }
}

/**
 * @brief Submaps::Get
 * 得到下标为index的submap
 * @param index     submap对应的下标
 * @return
 */
const Submap* Submaps::Get(int index) const
{
  CHECK_GE(index, 0);
  CHECK_LT(index, size());
  return submaps_[index].get();
}

/**
 * @brief Submaps::size
 * 得到submap的数量
 * @return
 */
int Submaps::size() const { return submaps_.size(); }

void Submaps::SubmapToProto(
    const int index, const std::vector<mapping::TrajectoryNode>&,
    const transform::Rigid3d&,
    mapping::proto::SubmapQuery::Response* const response)
{
  AddProbabilityGridToResponse(Get(index)->local_pose(),
                               Get(index)->probability_grid, response);
}

/**
 * @brief Submaps::FinishSubmap
 * 把下标为Index的submap设置为finish状态。
 * 为submap的finished_probability_grid赋值。
 * 这个变量被赋值了之后，表示这个submap已经完成。不会再修改了。
 * @param index
 */
void Submaps::FinishSubmap(int index)
{
  // Crop the finished Submap before inserting a new Submap to reduce peak
  // memory usage a bit.
  Submap* submap = submaps_[index].get();
  CHECK(submap->finished_probability_grid == nullptr);

  //对覆盖栅格地图进行裁剪
  submap->probability_grid =
      ComputeCroppedProbabilityGrid(submap->probability_grid);

  //finished_probability_grid设置为自身地图。
  //
  submap->finished_probability_grid = &submap->probability_grid;

  if (options_.output_debug_images())
  {
    // Output the Submap that won't be changed from now on.
    WriteDebugImage("submap" + std::to_string(index) + ".webp",
                    submap->probability_grid);
  }
}

/**
 * @brief Submaps::AddSubmap
 * 增加一个submap.由于每次都有且只能有2个submap来进行更新。
 * 因此增加一个submap的同时也必须finished一个submap
 * 一个submap里面的激光数量为options_.num_laser_fans()*2.
 * @param origin    新增加的submap的原点
 */
void Submaps::AddSubmap(const Eigen::Vector2f& origin)
{
  //到了新增submap的时候，首先要把size()-2的submap来finished掉。
  //因此这个时候说明size()-2的submap里面的激光数据已经有options_.num_laser_fans()*2了.
  //注意！！！！如果这个时候只有1个submap的话，那是不能进行finished的，直接增加一个地图就可以了。
  if (size() > 1)
  {
    FinishSubmap(size() - 2);
  }

  //地图的cellsize大小
  const int num_cells_per_dimension =
      common::RoundToInt(2. * options_.half_length() / options_.resolution()) +
      1;

  //新建一个submap。并且push到submaps中
  submaps_.push_back(common::make_unique<Submap>(
      MapLimits(options_.resolution(),
                origin.cast<double>() +
                    options_.half_length() * Eigen::Vector2d::Ones(),
                CellLimits(num_cells_per_dimension, num_cells_per_dimension)),
      origin, num_laser_fans_));

  LOG(INFO) << "Added submap " << size();
  num_laser_fans_in_last_submap_ = 0;
}

}  // namespace mapping_2d
}  // namespace cartographer
