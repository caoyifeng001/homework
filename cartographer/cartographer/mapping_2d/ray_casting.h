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

#ifndef CARTOGRAPHER_MAPPING_2D_RAY_CASTING_H_
#define CARTOGRAPHER_MAPPING_2D_RAY_CASTING_H_

#include <functional>

#include "../mapping_2d/map_limits.h"
#include "../mapping_2d/xy_index.h"
#include "../sensor/laser.h"
#include "../sensor/point_cloud.h"
#include "../transform/transform.h"

namespace cartographer {
namespace mapping_2d {

// For each ray in 'laser_fan', calls 'hit_visitor' and 'miss_visitor' on the
// appropriate cells. Hits are handled before misses.
// 进行栅格地图构建的时候需要用到的raytrace，基本算法为bresenham2d算法．
// 计算hit_visitor 和 miss_visitor，先处理hits
void CastRays(const sensor::LaserFan& laser_fan, const MapLimits& limits,
              const std::function<void(const Eigen::Array2i&)>& hit_visitor,
              const std::function<void(const Eigen::Array2i&)>& miss_visitor);

}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_RAY_CASTING_H_
