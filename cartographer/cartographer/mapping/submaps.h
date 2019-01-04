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

// Submaps is a sequence of maps to which scans are matched and into which scans
// are inserted.
//
// Except during initialization when only a single submap exists, there are
// always two submaps into which scans are inserted: an old submap that is used
// for matching, and a new one, which will be used for matching next, that is
// being initialized.
//
// Once a certain number of scans have been inserted, the new submap is
// considered initialized: the old submap is no longer changed, the "new" submap
// is now the "old" submap and is used for scan-to-map matching. Moreover,
// a "new" submap gets inserted.
#ifndef CARTOGRAPHER_MAPPING_SUBMAPS_H_
#define CARTOGRAPHER_MAPPING_SUBMAPS_H_

#include <memory>
#include <vector>

#include "eigen3/Eigen/Geometry"
#include "../common/math.h"
#include "../common/port.h"
#include "../mapping/probability_values.h"
#include "../mapping/trajectory_node.h"
#include "../mapping_2d/probability_grid.h"
#include "cartographer/mapping/proto/submaps.pb.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

// Converts the given probability to log odds.
//把概率转换成log odd
//方便进行地图的更新
inline float Logit(float probability)
{
  return std::log(probability / (1.f - probability));
}

//最大最小值的宏定义
const float kMaxLogOdds = Logit(kMaxProbability);
const float kMinLogOdds = Logit(kMinProbability);

// Converts a probability to a log odds integer. 0 means unknown, [kMinLogOdds,
// kMaxLogOdds] is mapped to [1, 255].
// 把一个概率值转换成整型的log-odd。
inline uint8 ProbabilityToLogOddsInteger(const float probability)
{
  const int value = common::RoundToInt((Logit(probability) - kMinLogOdds) *
                                       254.f / (kMaxLogOdds - kMinLogOdds)) +
                    1;
  CHECK_LE(1, value);
  CHECK_GE(255, value);
  return value;
}

// An individual submap, which has an initial position 'origin', keeps track of
// which laser fans where inserted into it, and sets the
// 'finished_probability_grid' to be used for loop closing once the map no
// longer changes.
// 独立的Submap的基类。保存submap中的数据。
// 包括起点 激光数据 对应的栅格地图等
struct Submap
{
  Submap(const Eigen::Vector3f& origin, int begin_laser_fan_index)
      : origin(origin),
        begin_laser_fan_index(begin_laser_fan_index),
        end_laser_fan_index(begin_laser_fan_index) {}

  // 返回submap的起点
  transform::Rigid3d local_pose() const
  {
    return transform::Rigid3d::Translation(origin.cast<double>());
  }

  // Origin of this submap.
  // submap的起点
  Eigen::Vector3f origin;

  // This Submap contains LaserFans with indices in the range
  // ['begin_laser_fan_index', 'end_laser_fan_index').
  // 位于这个submap中的激光数据的下表
  int begin_laser_fan_index;
  int end_laser_fan_index;

  // The 'finished_probability_grid' when this submap is finished and will not
  // change anymore. Otherwise, this is nullptr and the next call to
  // InsertLaserFan() will change the submap.
  // submap对应的覆盖栅格地图
  // 一旦submap构建完成，则这个栅格地图不会再改变
  const mapping_2d::ProbabilityGrid* finished_probability_grid = nullptr;
};

// A container of Submaps.
/**
 * @brief The Submaps class
 * 这个类是2d::mapping 和 3d::mapping中对应的Submaps的基类。
 * 上面的结构体Submap的容器。
 * 里面很多需要被子类实现的虚函数
 *
 * 相邻的两个submap之间是有50%的重合激光数据的。
 * 因此每次要插入激光的时候，都需要在两个submap里面进行插入。
 *
 *
 *
 */
class Submaps
{
 public:
  static constexpr uint8 kUnknownLogOdds = 0;

  Submaps();
  virtual ~Submaps();

  Submaps(const Submaps&) = delete;
  Submaps& operator=(const Submaps&) = delete;

  // Returns the index of the newest initialized Submap which can be
  // used for scan-to-map matching.
  // 返回最新的能被用来scan-match的submap。一般来说是倒数第二个submap。
  // 因为倒数第二个submap至少已经构建完成超过50%了。
  int matching_index() const;

  // Returns the indices of the Submap into which point clouds will
  // be inserted.
  // 返回在进行激光插入的时候，哪些地图是需要被插入的。
  // 会返回两个submap 因为相邻的两个submap之间只有50%的重合的
  std::vector<int> insertion_indices() const;

  // Returns the Submap with the given 'index'. The same 'index' will always
  // return the same pointer, so that Submaps can be identified by it.
  virtual const Submap* Get(int index) const = 0;

  // Returns the number of Submaps.
  virtual int size() const = 0;

  // Fills data about the Submap with 'index' into the 'response'.
  virtual void SubmapToProto(
      int index, const std::vector<mapping::TrajectoryNode>& trajectory_nodes,
      const transform::Rigid3d& global_submap_pose,
      proto::SubmapQuery::Response* response) = 0;

 protected:
  static void AddProbabilityGridToResponse(
      const transform::Rigid3d& local_submap_pose,
      const mapping_2d::ProbabilityGrid& probability_grid,
      proto::SubmapQuery::Response* response);
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_SUBMAPS_H_
