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

#ifndef CARTOGRAPHER_KALMAN_FILTER_POSE_TRACKER_H_
#define CARTOGRAPHER_KALMAN_FILTER_POSE_TRACKER_H_

#include <deque>
#include <memory>

#include "eigen3/Eigen/Cholesky"
#include "eigen3/Eigen/Core"
#include "../common/lua_parameter_dictionary.h"
#include "../common/port.h"
#include "../common/time.h"
#include "./gaussian_distribution.h"
#include "./odometry_state_tracker.h"
#include "./unscented_kalman_filter.h"
#include "../transform/transform.h"

#include "cartographer/kalman_filter/proto/pose_tracker_options.pb.h"
#include "cartographer/sensor/proto/sensor.pb.h"

namespace cartographer {
namespace kalman_filter {

typedef Eigen::Matrix3d Pose2DCovariance;
typedef Eigen::Matrix<double, 6, 6> PoseCovariance;

//带协方差的位姿
struct PoseAndCovariance
{
  transform::Rigid3d pose;
  PoseCovariance covariance;
};

/*
 * 运算符*　的重载
 * 对这个位姿做转换
*/
PoseAndCovariance operator*(const transform::Rigid3d& transform,
                            const PoseAndCovariance& pose_and_covariance);

// Projects a 3D pose covariance into a 2D pose covariance.
/*
 * 把3D位姿的协方差矩阵转换为2D位姿的协方差矩阵
 * (x,y,z,alpha,beta,theta)->(x,y,theta)
*/
Pose2DCovariance Project2D(const PoseCovariance& embedded_covariance);

// Embeds a 2D pose covariance into a 3D pose covariance.
PoseCovariance Embed3D(const Pose2DCovariance& embedded_covariance,
                       double position_variance, double orientation_variance);

// Deserializes the 'proto_matrix' into a PoseCovariance.
PoseCovariance PoseCovarianceFromProtoMatrix(
    const sensor::proto::Matrix& proto_matrix);

proto::PoseTrackerOptions CreatePoseTrackerOptions(
    common::LuaParameterDictionary* parameter_dictionary);

// Keeps track of the orientation using angular velocities and linear
// accelerations from an IMU. Because averaged linear acceleration (assuming
// slow movement) is a direct measurement of gravity, roll/pitch does not drift,
// though yaw does.
// imutracker这个imu_tracker是PoseTracker的一部分．主要用来计算机器人的姿态．
// 注意这里的计算机器人的姿态　在整个PoseTracker里面　相当于对机器人位姿的预测.
// imu_tracker的主要功能就是对imu进行姿态结算，同时还能对姿态进行一些预测(匀角速度预测)
// 也就是说如果我们有一个直接能输出角度和角速度的imu的话，那么这里的imu_tracker其实作用是不大的．
// 因此这里的imu的预测和观测　其实是相对于姿态结算来说的，并不是说相对于整个pose_tracker来说的．
class ImuTracker
{
 public:
  ImuTracker(const proto::PoseTrackerOptions& options, common::Time time);

  // Updates the orientation to reflect the given 'time'.
  // 预测时刻time的机器人的位姿　这里预测是假设机器人是以imu_angular_velocity_匀角速度运动
  void Predict(common::Time time);

  // Updates from an IMU reading (in the IMU frame).
  // 增加线性加速度观测到IMU跟踪器中
  /*
   * 增加一个加速度计读数的观测
   * 通过预测的重力矢量　和　观测的　重力矢量　来对重力矢量进行更新．
   * 融合方式为互补滤波
   * 预测的重力矢量为假设机器人是以imu_angular_velocity_匀角速度运行的来进行积分的
   * 观测的重力矢量即为imu的读数
   *
   * 在重力矢量更新完成之后，重新调整机器人的角度使得角度和重力矢量匹配
  */
  void AddImuLinearAccelerationObservation(
      common::Time time, const Eigen::Vector3d& imu_linear_acceleration);

  // 增加角速度观测到跟踪器中
  void AddImuAngularVelocityObservation(
      common::Time time, const Eigen::Vector3d& imu_angular_velocity);

  // Query the current orientation estimate.
  // 返回当前估计的角度
  Eigen::Quaterniond orientation() { return orientation_; }

 private:
  const proto::PoseTrackerOptions options_;
  common::Time time_;                           //当期时间
  common::Time last_linear_acceleration_time_;  //上一次更新线性加速度计的时间
  Eigen::Quaterniond orientation_;              //四元数，当前角度
  Eigen::Vector3d gravity_direction_;           //重力方向　－－imu加速度
  Eigen::Vector3d imu_angular_velocity_;        //imu角速度
};

// A Kalman filter for a 3D state of position and orientation.
// Includes functions to update from IMU and laser scan matches.
// 位姿跟踪器　该跟踪器利用ukf来对位姿进行跟踪
// 通过融合imu和激光雷达scan-match的过程来不断的迭代ukf
class PoseTracker
{
 public:
  //这个枚举类型表示系统的　状态变量的下标　和　对应的状态变量的位数
  //从这个枚举变量我们可以看到，整个系统变量的维数为９维．
  //分别为：xyz的位置　roll\pitch\yaw三个欧拉角　xyz三轴的速度
  enum {
    kMapPositionX = 0,
    kMapPositionY,
    kMapPositionZ,
    kMapOrientationX,
    kMapOrientationY,
    kMapOrientationZ,
    kMapVelocityX,
    kMapVelocityY,
    kMapVelocityZ,
    kDimension  // We terminate loops with this.
  };

  //kalman滤波器的预测方程的选择　2d预测方程　还是 3d预测方程
  //如果车子在平面运动的话，那么只需要２d的预测方程就可以了，但是如果要
  //在整个空间中运动，就需要3d的预测方程
  enum class ModelFunction { k2D, k3D };

  //定义一个kalman滤波器
  using KalmanFilter = UnscentedKalmanFilter<double, kDimension>;
  //定义状态变量
  using State = KalmanFilter::StateType;
  //定义协方差矩阵
  using StateCovariance = Eigen::Matrix<double, kDimension, kDimension>;
  //定义高斯分布
  using Distribution = GaussianDistribution<double, kDimension>;

  // Create a new Kalman filter starting at the origin with a standard normal
  // distribution at 'time'.
  PoseTracker(const proto::PoseTrackerOptions& options,
              const ModelFunction& model_function, common::Time time);
  virtual ~PoseTracker();

  // Sets 'pose' and 'covariance' to the mean and covariance of the belief at
  // 'time' which must be >= to the current time. Must not be nullptr.
  // 得到t时刻估计的机器人位姿和方差
  void GetPoseEstimateMeanAndCovariance(common::Time time,
                                        transform::Rigid3d* pose,
                                        PoseCovariance* covariance);

  // Updates from an IMU reading (in the IMU frame).
  // 增加一个imu的加速度的观测
  // 这里面调用imu_tracker的对应的函数,然后预测系统time时刻的状态
  void AddImuLinearAccelerationObservation(
      common::Time time, const Eigen::Vector3d& imu_linear_acceleration);

  // 增加一个imu的角速度的观测
  // 这里面调用imu_tracker里面的对应的函数，然后预测系统time时刻的状态
  void AddImuAngularVelocityObservation(
      common::Time time, const Eigen::Vector3d& imu_angular_velocity);

  // Updates from a pose estimate in the map frame.
  // 增加一个位姿观测
  // 这个真正的ukf　pose_tracker的观测方程．前面的imu的观测对于整个滤波器来说，都只是预测
  //
  void AddPoseObservation(common::Time time, const transform::Rigid3d& pose,
                          const PoseCovariance& covariance);

  // Updates from a pose estimate in the odometer's map-like frame.
  // 增加一个里程计位姿的观测
  // 这里面会调用上面的函数来对滤波器进行更新，说明把里程计的数据也当做观测数据．
  // 这样一半来说要里程计数据的精度比较高才行．
  void AddOdometerPoseObservation(common::Time time,
                                  const transform::Rigid3d& pose,
                                  const PoseCovariance& covariance);

  common::Time time() const { return time_; }

  // Returns the belief at the 'time' which must be >= to the current time.
  // 得到时刻t的位姿的分布　得到的位姿分布，也就得到了位姿
  Distribution GetBelief(common::Time time);

  const OdometryStateTracker::OdometryStates& odometry_states() const;

 private:
  // Returns the distribution required to initialize the KalmanFilter.
  //　初始化一个kalman滤波器
  static Distribution KalmanFilterInit();

  // Build a model noise distribution (zero mean) for the given time delta.
  // 给定时间t设置０均值的噪声模型
  const Distribution BuildModelNoise(double delta_t) const;

  // Predict the state forward in time. This is a no-op if 'time' has not moved
  // forward.
  // 滤波器按照预测方程进行预测 功能是imu_tracker的差不多
  // 实际上这里面会调用imu_tracker里面的predict
  void Predict(common::Time time);

  // Computes a pose combining the given 'state' with the 'imu_tracker_'
  // orientation.
  // 从state中计算出来一个刚体变换
  transform::Rigid3d RigidFromState(const PoseTracker::State& state);

  const proto::PoseTrackerOptions options_;     //一些配置参数　各个方差之类的
  const ModelFunction model_function_;          //滤波器预测函数 2D or 3D
  common::Time time_;                           //上一次更新的时刻
  KalmanFilter kalman_filter_;                  //kalman滤波器
  ImuTracker imu_tracker_;                      //imu_tracker
  OdometryStateTracker odometry_state_tracker_; //odometrystate跟踪器(缓冲区)
};

}  // namespace kalman_filter
}  // namespace cartographer

#endif  // CARTOGRAPHER_KALMAN_FILTER_POSE_TRACKER_H_
