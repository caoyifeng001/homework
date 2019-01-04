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

#ifndef CARTOGRAPHER_MAPPING_2D_GLOBAL_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_2D_GLOBAL_TRAJECTORY_BUILDER_H_

#include "../mapping/global_trajectory_builder_interface.h"
#include "../mapping_2d/local_trajectory_builder.h"
#include "../mapping_2d/sparse_pose_graph.h"

namespace cartographer {
namespace mapping_2d {

/*
 * 总的类，在这里面会调用其他文件中定义的东西．
 * 在ros里面使用的时候，也只需要定义这个类的实例就可以
 * 这里面有local_trajectory_builder来进行submap的构建．
 * 以及sparse_pose_graph来进行闭环检测和全局优化
*/
class GlobalTrajectoryBuilder
    : public mapping::GlobalTrajectoryBuilderInterface
{
 public:
  GlobalTrajectoryBuilder(const proto::LocalTrajectoryBuilderOptions& options,
                          SparsePoseGraph* sparse_pose_graph);
  ~GlobalTrajectoryBuilder() override;

  GlobalTrajectoryBuilder(const GlobalTrajectoryBuilder&) = delete;
  GlobalTrajectoryBuilder& operator=(const GlobalTrajectoryBuilder&) = delete;

  const Submaps* submaps() const override;
  Submaps* submaps() override;
  kalman_filter::PoseTracker* pose_tracker() const override;
  const mapping::GlobalTrajectoryBuilderInterface::PoseEstimate& pose_estimate()
      const override;

  //调用local_trajectory_builder的相应函数来进行激光帧的插入。
  //插入成功之后会调用sparse_pose_graph的AddScan()来进行回环检测和后端优化
  void AddHorizontalLaserFan(common::Time time,
                             const sensor::LaserFan3D& laser_fan) override;

  //里面主要调用local_trajectory_builder的对应的函数来进行滤波器的更新
  void AddImuData(common::Time time, const Eigen::Vector3d& linear_acceleration,
                  const Eigen::Vector3d& angular_velocity) override;

  void AddOdometerPose(
      common::Time time, const transform::Rigid3d& pose,
      const kalman_filter::PoseCovariance& covariance) override;

  void AddLaserFan3D(common::Time, const sensor::LaserFan3D&) override {
    LOG(FATAL) << "Not implemented.";
  };

 private:
  const proto::LocalTrajectoryBuilderOptions options_;
  SparsePoseGraph* const sparse_pose_graph_;
  LocalTrajectoryBuilder local_trajectory_builder_;
};

}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_GLOBAL_TRAJECTORY_BUILDER_H_
