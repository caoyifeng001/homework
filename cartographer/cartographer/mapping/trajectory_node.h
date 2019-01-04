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

#ifndef CARTOGRAPHER_MAPPING_TRAJECTORY_NODE_H_
#define CARTOGRAPHER_MAPPING_TRAJECTORY_NODE_H_

#include <deque>
#include <vector>

#include "eigen3/Eigen/Core"
#include "../common/time.h"
#include "../sensor/laser.h"
#include "../transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

struct Submaps;

//纯粹机器人运动轨迹上的一个节点
//这个节点的意思就是一个坐标点以及在这个坐标点上的一些传感器数据,主要就是激光雷达
struct TrajectoryNode
{
  //这个轨迹节点上不会变化的一些数据
  struct ConstantData
  {
    //这个节点的时间
    common::Time time;

    // LaserFan in 'pose' frame. Only used in the 2D case.
    // 这个节点上的激光数据  2D情况使用
    // 对于2D的情况 这里的数据是已经经过下面的tracking_to_pose这个矩阵进行转换过了的，
    // 被投影到平面的2D激光数据
    sensor::LaserFan laser_fan;

    // LaserFan in 'pose' frame. Only used in the 3D case.
    // 这个节点上的激光数据  3D情况使用
    sensor::CompressedLaserFan3D laser_fan_3d;

    // Trajectory this node belongs to.
    // TODO(jmason): The naming here is confusing because 'trajectory' doesn't
    // seem like a good name for a Submaps*. Sort this out.
    // 这个节点属于哪一条轨迹  轨迹就是一系列的submap
    const Submaps* trajectory;

    // Transform from the 3D 'tracking' frame to the 'pose' frame of the
    // laser, which contains roll, pitch and height for 2D. In 3D this is
    // always identity.
    // 从3d tracking坐标系 转换到 pose坐标系的转换矩阵
    // 注意这个矩阵只对于2D情况有用 在2D的情况下 把激光雷达从3D空间转换到2D平面的转换矩阵
    // 对于3D的情况，这个矩阵永远都等于I，因为本身就是三维的，不需要转换。
    transform::Rigid3d tracking_to_pose;
  };

  common::Time time() const { return constant_data->time; }

  //存储这个节点的ConstantData
  const ConstantData* constant_data;

  //这个节点自己的位姿
  transform::Rigid3d pose;
};

// Users will only be interested in 'trajectory_nodes'. But 'constant_data'
// is referenced by 'trajectory_nodes'. This struct guarantees that their
// lifetimes are bound.
// 只需要考虑trajectory_nodes就可以了。这个存储机器人的一整条轨迹。
// 这里的轨迹表示各种位姿点 不是submap
struct TrajectoryNodes
{
  std::deque<mapping::TrajectoryNode::ConstantData> constant_data;
  std::vector<mapping::TrajectoryNode> trajectory_nodes;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_TRAJECTORY_NODE_H_
