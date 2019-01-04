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

#ifndef CARTOGRAPHER_MAPPING_2D_LOCAL_TRAJECTORY_BUILDER_H_
#define CARTOGRAPHER_MAPPING_2D_LOCAL_TRAJECTORY_BUILDER_H_

#include <memory>

#include "../common/lua_parameter_dictionary.h"
#include "../common/time.h"
#include "../kalman_filter/pose_tracker.h"
#include "../mapping/global_trajectory_builder_interface.h"
#include "../mapping_2d/scan_matching/ceres_scan_matcher.h"
#include "../mapping_2d/scan_matching/real_time_correlative_scan_matcher.h"
#include "../mapping_2d/submaps.h"
#include "../mapping_3d/motion_filter.h"
#include "../sensor/configuration.h"
#include "../sensor/voxel_filter.h"
#include "../transform/rigid_transform.h"

#include "cartographer/mapping_2d/proto/local_trajectory_builder_options.pb.h"


/*
 * localTrajectoryBuilder　主要实现了局部的submap的构建．
*/
namespace cartographer {
namespace mapping_2d {

proto::LocalTrajectoryBuilderOptions CreateLocalTrajectoryBuilderOptions(
    common::LuaParameterDictionary* parameter_dictionary);

// Wires up the local SLAM stack (i.e. UKF, scan matching, etc.) without loop
// closure.
/*
 * 这里LocalTrajectoryBuilder相当于局部SLAM．
 * 这里面把ukf-scanmatching之类的都联系起来了．
 * 这里主要的功能是构建submap，因此相当于局部slam
 *
 * 主要函数有四个
 * AddImuData():
 * 把imu数据送给pose_tracker来进行位姿跟踪
 *
 * AddOdometerPose():
 * 把里程计数据送给pose_tracker来进行位姿跟踪
 *
 * AddHorizontalLaserFan():
 * 加入一帧激光雷达的观测数据来更新里程计
 * 1.得到pose_tracker的预测值　＆　把对应的激光雷达数据投影到2d平面
 * 2.调用scan-match来得到准确的机器人坐标　＆ 用这个坐标来进行滤波器更新
 * 3.得到更新过后的滤波器估计的位姿　＆　转换到2d平面上
 * 4.把对应的平面激光雷达数据插入到submap中(这一步目前还不清除具体怎么实现)
 *
 * ScanMatch():
 * 被AddHorizontalLaserFan()函数调用
 * 1.把滤波器给出的初始估计位姿转换为2d平面的位姿 & 对平面激光雷达数据进行滤波
 * 2.调用real-time csm来进行位姿的优化
 * 3.在real-time csm优化的基础上　调用ceres-scan-match进行进一步优化
 * 4.用ceres_scan_match的结果来作为滤波器的观测值进行更新
 *
*/
class LocalTrajectoryBuilder
{
 public:
  struct InsertionResult
  {
    common::Time time;
    const mapping::Submaps* submaps;
    const mapping::Submap* matching_submap;
    std::vector<const mapping::Submap*> insertion_submaps;
    transform::Rigid3d tracking_to_tracking_2d;
    transform::Rigid3d tracking_2d_to_map;
    sensor::LaserFan laser_fan_in_tracking_2d;
    transform::Rigid2d pose_estimate_2d;
    kalman_filter::PoseCovariance covariance_estimate;
  };

  explicit LocalTrajectoryBuilder(
      const proto::LocalTrajectoryBuilderOptions& options);
  ~LocalTrajectoryBuilder();

  LocalTrajectoryBuilder(const LocalTrajectoryBuilder&) = delete;
  LocalTrajectoryBuilder& operator=(const LocalTrajectoryBuilder&) = delete;

  const mapping::GlobalTrajectoryBuilderInterface::PoseEstimate& pose_estimate()
      const;

  //增加水平的激光束　这里面通过ukf进行初始位姿，调用scanmatch进行位姿优化，用来构建局部地图
  std::unique_ptr<InsertionResult> AddHorizontalLaserFan(
      common::Time, const sensor::LaserFan3D& laser_fan);

  //接受imu的数据，送给ukf来进行位姿跟踪
  void AddImuData(common::Time time, const Eigen::Vector3d& linear_acceleration,
                  const Eigen::Vector3d& angular_velocity);

  //接受里程计的数据　送给ukf来进行位姿跟踪
  void AddOdometerPose(common::Time time, const transform::Rigid3d& pose,
                       const kalman_filter::PoseCovariance& covariance);

  const Submaps* submaps() const;
  Submaps* submaps();
  kalman_filter::PoseTracker* pose_tracker() const;

 private:
  // Transforms 'laser_scan', projects it onto the ground plane,
  // crops and voxel filters.
  //把激光雷达的数据投影到水平面上，同时对其进行一定的滤波
  sensor::LaserFan BuildProjectedLaserFan(
      const transform::Rigid3f& tracking_to_tracking_2d,
      const sensor::LaserFan3D& laser_fan) const;

  // Scan match 'laser_fan_in_tracking_2d' and fill in the
  // 'pose_observation' and 'covariance_observation' with the result.
  // 以ukf估计的位姿为初始位姿，在submap中来进行scanmatch操作
  // 这里的scan-match算法使用的是realtime-CorrelariveScanMatcher
  void ScanMatch(common::Time time, const transform::Rigid3d& pose_prediction,
                 const transform::Rigid3d& tracking_to_tracking_2d,
                 const sensor::LaserFan& laser_fan_in_tracking_2d,
                 transform::Rigid3d* pose_observation,
                 kalman_filter::PoseCovariance* covariance_observation);

  // Lazily constructs a PoseTracker.
  // 初始化PoseTracker的位姿
  void InitializePoseTracker(common::Time time);

  //一些配置的参数
  const proto::LocalTrajectoryBuilderOptions options_;

  //submap的容器
  Submaps submaps_;

  //滤波器估计的上一次的位姿
  mapping::GlobalTrajectoryBuilderInterface::PoseEstimate last_pose_estimate_;

  // Pose of the last computed scan match.
  // 最近的滤波器加入了scan-match之后估计出来的位姿
  transform::Rigid3d scan_matcher_pose_estimate_;

  //运动滤波器　不知道是什么
  mapping_3d::MotionFilter motion_filter_;

  //用来进行real_time_correclative_scan_matcher的变量
  scan_matching::RealTimeCorrelativeScanMatcher
      real_time_correlative_scan_matcher_;

  //ceres_scan_matcher求解器
  scan_matching::CeresScanMatcher ceres_scan_matcher_;

  //一个pose_tracker
  std::unique_ptr<kalman_filter::PoseTracker> pose_tracker_;
};

}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_LOCAL_TRAJECTORY_BUILDER_H_
