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

#include "../mapping_2d/laser_fan_inserter.h"

#include <cstdlib>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "../mapping_2d/ray_casting.h"
#include "../mapping_2d/xy_index.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_2d {

proto::LaserFanInserterOptions CreateLaserFanInserterOptions(
    common::LuaParameterDictionary* const parameter_dictionary)
{
  proto::LaserFanInserterOptions options;
  options.set_hit_probability(
      parameter_dictionary->GetDouble("hit_probability"));

  options.set_miss_probability(
      parameter_dictionary->GetDouble("miss_probability"));

  options.set_insert_free_space(
      parameter_dictionary->HasKey("insert_free_space")
          ? parameter_dictionary->GetBool("insert_free_space")
          : true);

  CHECK_GT(options.hit_probability(), 0.5);
  CHECK_LT(options.miss_probability(), 0.5);
  return options;
}

LaserFanInserter::LaserFanInserter(
    const proto::LaserFanInserterOptions& options)
    : options_(options),
      hit_table_(mapping::ComputeLookupTableToApplyOdds(
          mapping::Odds(options.hit_probability()))),

      miss_table_(mapping::ComputeLookupTableToApplyOdds(
          mapping::Odds(options.miss_probability()))) {}

/**
 * @brief LaserFanInserter::Insert
 * 执行插入的函数 这里面会调用RayCast算法来进行地图的更新
 * @param laser_fan
 * @param probability_grid
 */
void LaserFanInserter::Insert(const sensor::LaserFan& laser_fan,
                              ProbabilityGrid* const probability_grid) const
{
  CHECK_NOTNULL(probability_grid)->StartUpdate();

  // By not starting a new update after hits are inserted, we give hits priority
  // (i.e. no hits will be ignored because of a miss in the same cell).
  CastRays(laser_fan, probability_grid->limits(),

           //传入的这个函数的函数体就是调用probability_grid->ApplyLookupTable()
           [this, &probability_grid](const Eigen::Array2i& hit) {
             probability_grid->ApplyLookupTable(hit, hit_table_);
           },

           //传入的这个函数的函数体就是调用probability_grid->ApplyLookupTable()来进行地图更新
           [this, &probability_grid](const Eigen::Array2i& miss) {
             if (options_.insert_free_space()) {
               probability_grid->ApplyLookupTable(miss, miss_table_);
             } 
           });
}

}  // namespace mapping_2d
}  // namespace cartographer
