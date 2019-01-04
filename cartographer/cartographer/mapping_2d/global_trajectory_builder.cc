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

#include "../mapping_2d/global_trajectory_builder.h"

namespace cartographer
{
namespace mapping_2d
{

GlobalTrajectoryBuilder::GlobalTrajectoryBuilder(
    const proto::LocalTrajectoryBuilderOptions& options,
    SparsePoseGraph* sparse_pose_graph)
    : options_(options),
      sparse_pose_graph_(sparse_pose_graph),
      local_trajectory_builder_(options) {}

GlobalTrajectoryBuilder::~GlobalTrajectoryBuilder() {}

const Submaps* GlobalTrajectoryBuilder::submaps() const
{
  return local_trajectory_builder_.submaps();
}

Submaps* GlobalTrajectoryBuilder::submaps()
{
  return local_trajectory_builder_.submaps();
}

kalman_filter::PoseTracker* GlobalTrajectoryBuilder::pose_tracker() const
{
  return local_trajectory_builder_.pose_tracker();
}

/**
 * @brief GlobalTrajectoryBuilder::AddHorizontalLaserFan
 * 增加一帧激光雷达的数据
 * 这里面进行两个操作：
 * 1.调用局部地图构建器local_trajectory_builder的AddHorizonLaserFan()函数
 * 2.如果1调用成功，则调用后端优化器sparse_pose_graph来进行闭环检测和后端优化
 * @param time
 * @param laser_fan
 */
void GlobalTrajectoryBuilder::AddHorizontalLaserFan(
    const common::Time time, const sensor::LaserFan3D& laser_fan)
{
  //进行激光帧插入
  std::unique_ptr<LocalTrajectoryBuilder::InsertionResult> insertion_result =
      local_trajectory_builder_.AddHorizontalLaserFan(time, laser_fan);

  //插入成功则进行回环检测和后端优化
  if (insertion_result != nullptr)
  {
    sparse_pose_graph_->AddScan(
        insertion_result->time,                                             //激光帧的时间
        insertion_result->tracking_to_tracking_2d,                          //把激光数据转换到平面转换矩阵
        insertion_result->laser_fan_in_tracking_2d,                         //平面坐标系中的激光数据
        insertion_result->pose_estimate_2d,                                 //滤波器估计出来的机器人最新位姿
        kalman_filter::Project2D(insertion_result->covariance_estimate),    //滤波器估计出来的机器人位姿的方差
        insertion_result->submaps,                                          //所有的submap
        insertion_result->matching_submap,                                  //本次用来进行scan-match的submap
        insertion_result->insertion_submaps);                               //插入了激光数据的submap 就是submap(size-1) 和 submap(size-2)
  }
}

/**
 * @brief GlobalTrajectoryBuilder::AddImuData
 * 调用局部地图构建器local_trajectory_builder的AddImuData()数据来进行滤波器的更新
 * @param time                  IMU数据对应的时间
 * @param linear_acceleration   IMU的加速度数据
 * @param angular_velocity      IMU的角速度数据
 */
void GlobalTrajectoryBuilder::AddImuData(
    const common::Time time, const Eigen::Vector3d& linear_acceleration,
    const Eigen::Vector3d& angular_velocity)
{
  local_trajectory_builder_.AddImuData(time, linear_acceleration,
                                       angular_velocity);
}

/**
 * @brief GlobalTrajectoryBuilder::AddOdometerPose
 * 调用局部地图构建器local_trajectory_builder的AddOdometerPose()数据来进行滤波器的更新
 * @param time              POSE数据对应的时间
 * @param pose              POSE数据对应的位姿
 * @param covariance        POSE数据对应的方差
 */
void GlobalTrajectoryBuilder::AddOdometerPose(
    const common::Time time, const transform::Rigid3d& pose,
    const kalman_filter::PoseCovariance& covariance)
{
  local_trajectory_builder_.AddOdometerPose(time, pose, covariance);
}

/**
 * @brief GlobalTrajectoryBuilder::pose_estimate
 * 得到滤波器对机器人位姿的估计值
 * @return
 */
const mapping::GlobalTrajectoryBuilderInterface::PoseEstimate&
GlobalTrajectoryBuilder::pose_estimate() const
{
  return local_trajectory_builder_.pose_estimate();
}

}  // namespace mapping_2d
}  // namespace cartographer
