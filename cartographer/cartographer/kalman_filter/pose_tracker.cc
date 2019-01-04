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

#include "../kalman_filter/pose_tracker.h"

#include <cmath>
#include <limits>
#include <utility>

#include "eigen3/Eigen/Geometry"
#include "../common/lua_parameter_dictionary.h"
#include "../common/math.h"
#include "../common/time.h"
#include "../kalman_filter/gaussian_distribution.h"
#include "../kalman_filter/unscented_kalman_filter.h"
#include "../transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace kalman_filter {

namespace {

PoseTracker::State AddDelta(const PoseTracker::State& state,
                            const PoseTracker::State& delta)
{
  PoseTracker::State new_state = state + delta;

  const Eigen::Quaterniond orientation =
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(state[PoseTracker::kMapOrientationX],
                          state[PoseTracker::kMapOrientationY],
                          state[PoseTracker::kMapOrientationZ]));

  const Eigen::Vector3d rotation_vector(delta[PoseTracker::kMapOrientationX],
                                        delta[PoseTracker::kMapOrientationY],
                                        delta[PoseTracker::kMapOrientationZ]);

  CHECK_LT(rotation_vector.norm(), M_PI / 2.)
      << "Sigma point is far from the mean, recovered delta may be incorrect.";
  const Eigen::Quaterniond rotation =
      transform::AngleAxisVectorToRotationQuaternion(rotation_vector);

  const Eigen::Vector3d new_orientation =
      transform::RotationQuaternionToAngleAxisVector(orientation * rotation);

  new_state[PoseTracker::kMapOrientationX] = new_orientation.x();
  new_state[PoseTracker::kMapOrientationY] = new_orientation.y();
  new_state[PoseTracker::kMapOrientationZ] = new_orientation.z();
  return new_state;
}

PoseTracker::State ComputeDelta(const PoseTracker::State& origin,
                                const PoseTracker::State& target)
{
  PoseTracker::State delta = target - origin;
  const Eigen::Quaterniond origin_orientation =
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(origin[PoseTracker::kMapOrientationX],
                          origin[PoseTracker::kMapOrientationY],
                          origin[PoseTracker::kMapOrientationZ]));

  const Eigen::Quaterniond target_orientation =
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(target[PoseTracker::kMapOrientationX],
                          target[PoseTracker::kMapOrientationY],
                          target[PoseTracker::kMapOrientationZ]));

  const Eigen::Vector3d rotation =
      transform::RotationQuaternionToAngleAxisVector(
          origin_orientation.inverse() * target_orientation);

  delta[PoseTracker::kMapOrientationX] = rotation.x();
  delta[PoseTracker::kMapOrientationY] = rotation.y();
  delta[PoseTracker::kMapOrientationZ] = rotation.z();
  return delta;
}

// Build a model matrix for the given time delta.
/**
 * @brief ModelFunction3D
 * 三维系统的预测函数 该函数被PoseTracker的Predict()函数调用
 * 主要功能为根据系统当前时刻的状态state　预测delta_t时刻后系统的状态
 * @param state     系统当前时刻的状态
 * @param delta_t   对应的delta_t
 * @return
 */
PoseTracker::State ModelFunction3D(const PoseTracker::State& state,
                                   const double delta_t)
{
  CHECK_GT(delta_t, 0.);

  PoseTracker::State new_state;
  new_state[PoseTracker::kMapPositionX] =
      state[PoseTracker::kMapPositionX] +
      delta_t * state[PoseTracker::kMapVelocityX];
  new_state[PoseTracker::kMapPositionY] =
      state[PoseTracker::kMapPositionY] +
      delta_t * state[PoseTracker::kMapVelocityY];
  new_state[PoseTracker::kMapPositionZ] =
      state[PoseTracker::kMapPositionZ] +
      delta_t * state[PoseTracker::kMapVelocityZ];

  new_state[PoseTracker::kMapOrientationX] =
      state[PoseTracker::kMapOrientationX];
  new_state[PoseTracker::kMapOrientationY] =
      state[PoseTracker::kMapOrientationY];
  new_state[PoseTracker::kMapOrientationZ] =
      state[PoseTracker::kMapOrientationZ];

  new_state[PoseTracker::kMapVelocityX] = state[PoseTracker::kMapVelocityX];
  new_state[PoseTracker::kMapVelocityY] = state[PoseTracker::kMapVelocityY];
  new_state[PoseTracker::kMapVelocityZ] = state[PoseTracker::kMapVelocityZ];

  return new_state;
}

// A specialization of ModelFunction3D that limits the z-component of position
// and velocity to 0.
/**
 * @brief ModelFunction2D
 * 二维平面系统的预测函数　该函数被PoseTracker的Predict()函数调用
 * 主要功能为根据系统当前时刻的状态state　预测delta_t时刻后系统的状态
 * 与上面不同的是，这是二维平面的．
 * @param state     系统当前时刻的状态
 * @param delta_t   对应的delta_t
 * @return
 */
PoseTracker::State ModelFunction2D(const PoseTracker::State& state,
                                   const double delta_t)
{
  auto new_state = ModelFunction3D(state, delta_t);
  new_state[PoseTracker::kMapPositionZ] = 0.;
  new_state[PoseTracker::kMapVelocityZ] = 0.;
  new_state[PoseTracker::kMapOrientationX] = 0.;
  new_state[PoseTracker::kMapOrientationY] = 0.;
  return new_state;
}

}  // namespace


//下面是imutrack的函数
ImuTracker::ImuTracker(const proto::PoseTrackerOptions& options,
                       const common::Time time)
    : options_(options),
      time_(time),
      last_linear_acceleration_time_(common::Time::min()),
      orientation_(Eigen::Quaterniond::Identity()),
      gravity_direction_(Eigen::Vector3d::UnitZ()),
      imu_angular_velocity_(Eigen::Vector3d::Zero()) {}

/*
 * IMUTracker预测时间time时候的姿态
 * 假设在时间time_ 到 time　这段时间机器人以imu_angular_velocity_的角速度匀速运动
 * 计算出来在time时刻机器人的角度应该运动到什么角度
 */
void ImuTracker::Predict(const common::Time time)
{
  CHECK_LE(time_, time);

  //时间增量
  const double delta_t = common::ToSeconds(time - time_);

  //按照当前的角速度imu_angular_velocity_运行时间delta_t　得到delta_t时间内的角度的增量　并转换为四元数
  const Eigen::Quaterniond rotation =
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(imu_angular_velocity_ * delta_t));

  //更新机器人的当前角度 \ 重力矢量
  //角度更新了　那么对应的重力矢量应该也更新
  orientation_ = (orientation_ * rotation).normalized();
  gravity_direction_ = rotation.inverse() * gravity_direction_;
  time_ = time;
}

/*
 * 增加一个加速度计读数的观测
 * 通过预测的重力矢量　和　观测的　重力矢量　来对重力矢量进行更新．
 * 融合方式为互补滤波
 *
 * 在重力矢量更新完成之后，重新调整机器人的角度使得角度和重力矢量匹配
*/
void ImuTracker::AddImuLinearAccelerationObservation(
    const common::Time time, const Eigen::Vector3d& imu_linear_acceleration)
{
  //预测time时刻 机器人的角度　因此也会更新预测的加速度矢量
  Predict(time);

  // Update the 'gravity_direction_' with an exponential moving average using
  // the 'imu_gravity_time_constant'.
  // 用重力时间常数来进行指数移动平均
  // 计算时间增量
  const double delta_t =
      last_linear_acceleration_time_ > common::Time::min()
          ? common::ToSeconds(time - last_linear_acceleration_time_)
          : std::numeric_limits<double>::infinity();

  last_linear_acceleration_time_ = time;

  const double alpha =
      1. - std::exp(-delta_t / options_.imu_gravity_time_constant());

  //根据预测和观测来更新重力矢量
  gravity_direction_ =
      (1. - alpha) * gravity_direction_ + alpha * imu_linear_acceleration;

  // Change the 'orientation_' so that it agrees with the current
  // 'gravity_direction_'.
  // 修改机器人的角度　使得由它的角度计算出来的重力矢量　和　更新后的重力矢量重合
  const Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(
      gravity_direction_, orientation_.inverse() * Eigen::Vector3d::UnitZ());

  orientation_ = (orientation_ * rotation).normalized();

  CHECK_GT((orientation_.inverse() * Eigen::Vector3d::UnitZ())
               .dot(gravity_direction_),
           0.);
}

/*
 * 增加一个角速度观测
*/
void ImuTracker::AddImuAngularVelocityObservation(
    const common::Time time, const Eigen::Vector3d& imu_angular_velocity)
{
  //预测t时刻机器人的姿态
  Predict(time);

  //更新机器人的角速度
  imu_angular_velocity_ = imu_angular_velocity;
}

/*
 * PoseAndCovariance的*号重载符
 * 功能为求得一个带方差的位姿，经过一个线性变化之后，得到的新的位姿和方差
*/
PoseAndCovariance operator*(const transform::Rigid3d& transform,
                            const PoseAndCovariance& pose_and_covariance)
{
  /*用方差生成一个0均值的高斯分布*/
  GaussianDistribution<double, 6> distribution(
      Eigen::Matrix<double, 6, 1>::Zero(), pose_and_covariance.covariance);

  Eigen::Matrix<double, 6, 6> linear_transform;

  linear_transform << transform.rotation().matrix(), Eigen::Matrix3d::Zero(),
      Eigen::Matrix3d::Zero(), transform.rotation().matrix();

  return {transform * pose_and_covariance.pose,
          (linear_transform * distribution).GetCovariance()};
}

proto::PoseTrackerOptions CreatePoseTrackerOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::PoseTrackerOptions options;
  options.set_position_model_variance(
      parameter_dictionary->GetDouble("position_model_variance"));
  options.set_orientation_model_variance(
      parameter_dictionary->GetDouble("orientation_model_variance"));
  options.set_velocity_model_variance(
      parameter_dictionary->GetDouble("velocity_model_variance"));
  options.set_imu_gravity_time_constant(
      parameter_dictionary->GetDouble("imu_gravity_time_constant"));
  options.set_imu_gravity_variance(
      parameter_dictionary->GetDouble("imu_gravity_variance"));
  options.set_num_odometry_states(
      parameter_dictionary->GetNonNegativeInt("num_odometry_states"));
  CHECK_GT(options.num_odometry_states(), 0);
  return options;
}

/**
 * @brief PoseTracker::KalmanFilterInit
 * kalman滤波器的初始化　返回一个均值为０方差为无穷小高斯分布
 * @return
 */
PoseTracker::Distribution PoseTracker::KalmanFilterInit()
{
  //状态变量初始为0
  State initial_state = State::Zero();
  // We are certain about the complete state at the beginning. We define the
  // initial pose to be at the origin and axis aligned. Additionally, we claim
  // that we are not moving.
  StateCovariance initial_covariance = 1e-9 * StateCovariance::Identity();
  return Distribution(initial_state, initial_covariance);
}

PoseTracker::PoseTracker(const proto::PoseTrackerOptions& options,
                         const ModelFunction& model_function,
                         const common::Time time)
    : options_(options),
      model_function_(model_function),
      time_(time),
      kalman_filter_(KalmanFilterInit(), AddDelta, ComputeDelta),
      imu_tracker_(options, time),
      odometry_state_tracker_(options.num_odometry_states()) {}

PoseTracker::~PoseTracker() {}

/**
 * @brief PoseTracker::GetBelief
 * 返回时刻为t的位姿的分布
 * t要大于等于当前时间
 * @param time  对应的时刻
 * @return
 */
PoseTracker::Distribution PoseTracker::GetBelief(const common::Time time)
{
  Predict(time);
  return kalman_filter_.GetBelief();
}

/**
 * @brief PoseTracker::GetPoseEstimateMeanAndCovariance
 * 得到t时刻，ukf滤波器估计的机器人的位姿和方差
 * @param time          对应的时间
 * @param pose          返回的机器人位姿
 * @param covariance    返回的机器人方差
 */
void PoseTracker::GetPoseEstimateMeanAndCovariance(const common::Time time,
                                                   transform::Rigid3d* pose,
                                                   PoseCovariance* covariance)
{
  const Distribution belief = GetBelief(time);
  *pose = RigidFromState(belief.GetMean());
  static_assert(kMapPositionX == 0, "Cannot extract PoseCovariance.");
  static_assert(kMapPositionY == 1, "Cannot extract PoseCovariance.");
  static_assert(kMapPositionZ == 2, "Cannot extract PoseCovariance.");
  static_assert(kMapOrientationX == 3, "Cannot extract PoseCovariance.");
  static_assert(kMapOrientationY == 4, "Cannot extract PoseCovariance.");
  static_assert(kMapOrientationZ == 5, "Cannot extract PoseCovariance.");
  *covariance = belief.GetCovariance().block<6, 6>(0, 0);
  covariance->block<2, 2>(3, 3) +=
      options_.imu_gravity_variance() * Eigen::Matrix2d::Identity();
}

/**
 * @brief PoseTracker::BuildModelNoise
 * 构建间隔时间为t的噪声模型
 * 该噪声模型的噪声随着时间间隔的增大而增大
 * @param delta_t   对应的时间间隔
 * @return
 */
const PoseTracker::Distribution PoseTracker::BuildModelNoise(
    const double delta_t) const
{
  // Position is constant, but orientation changes.
  StateCovariance model_noise = StateCovariance::Zero();

  model_noise.diagonal() <<
      // Position in map.
      options_.position_model_variance() * delta_t,
      options_.position_model_variance() * delta_t,
      options_.position_model_variance() * delta_t,

      // Orientation in map.
      options_.orientation_model_variance() * delta_t,
      options_.orientation_model_variance() * delta_t,
      options_.orientation_model_variance() * delta_t,

      // Linear velocities in map.
      options_.velocity_model_variance() * delta_t,
      options_.velocity_model_variance() * delta_t,
      options_.velocity_model_variance() * delta_t;

  return Distribution(State::Zero(), model_noise);
}

/**
 * @brief PoseTracker::Predict
 * 预测time时刻的系统的状态
 * 滤波器预测函数　因此这这个系统中　imu中的数据都是用来预测系统位置的．
 * 因此每次imu数据的更新都需要调用这个函数来对系统状态进行新的预测
 * 这个预测函数会根据系统指定的model_function来自动选择系统的预测方程
 * @param time  对应的时刻
 */
void PoseTracker::Predict(const common::Time time)
{
  imu_tracker_.Predict(time);
  CHECK_LE(time_, time);
  const double delta_t = common::ToSeconds(time - time_);
  if (delta_t == 0.)
  {
    return;
  }
  kalman_filter_.Predict(
      [this, delta_t](const State& state) -> State {
        switch (model_function_)
        {
          case ModelFunction::k2D:
            return ModelFunction2D(state, delta_t);
          case ModelFunction::k3D:
            return ModelFunction3D(state, delta_t);
          default:
            LOG(FATAL);
        }
      },
      BuildModelNoise(delta_t));
  time_ = time;
}

/**
 * @brief PoseTracker::AddImuLinearAccelerationObservation
 * 增加一个imu的加速度读数
 * @param time                      加速度计读数对应的时间
 * @param imu_linear_acceleration　　加速度计读数
 */
void PoseTracker::AddImuLinearAccelerationObservation(
    const common::Time time, const Eigen::Vector3d& imu_linear_acceleration)
{
  //更新imu＿tracker对应的linearAcc
  imu_tracker_.AddImuLinearAccelerationObservation(time,
                                                   imu_linear_acceleration);
  //更新系统状态(机器人位姿变量)
  Predict(time);
}

/**
 * @brief PoseTracker::AddImuAngularVelocityObservation
 * 增加一个imu的角速度观测
 * @param time                  角速度观测对应的时间
 * @param imu_angular_velocity  角速度观测
 */
void PoseTracker::AddImuAngularVelocityObservation(
    const common::Time time, const Eigen::Vector3d& imu_angular_velocity)
{
  //更新imu_tracker对应的angularVel
  imu_tracker_.AddImuAngularVelocityObservation(time, imu_angular_velocity);
  //预测时刻time的系统状态
  Predict(time);
}

/**
 * @brief PoseTracker::AddPoseObservation
 * 增加一个地图坐标系的位姿观测　一般这个观测来自scan-match　或者　里程计读数
 * 这个才真正称得上ukf滤波器的观测值．前面的imu读数是用来更新ukf滤波器的预测值的
 * @param time          对应的时刻t
 * @param pose          对应的位姿
 * @param covariance    对应的协方差矩阵
 */
void PoseTracker::AddPoseObservation(const common::Time time,
                                     const transform::Rigid3d& pose,
                                     const PoseCovariance& covariance)
{
  //ukf滤波器进行预测
  Predict(time);

  // Noise covariance is taken directly from the input values.
  // 构建噪声分布
  const GaussianDistribution<double, 6> delta(
      Eigen::Matrix<double, 6, 1>::Zero(), covariance);

  //ukf融合观测进行滤波器的更新
  kalman_filter_.Observe<6>(
      [this, &pose](const State& state) -> Eigen::Matrix<double, 6, 1> {
        const transform::Rigid3d state_pose = RigidFromState(state);
        const Eigen::Vector3d delta_orientation =
            transform::RotationQuaternionToAngleAxisVector(
                pose.rotation().inverse() * state_pose.rotation());
        const Eigen::Vector3d delta_translation =
            state_pose.translation() - pose.translation();
        Eigen::Matrix<double, 6, 1> return_value;
        return_value << delta_translation, delta_orientation;
        return return_value;
      },
      delta);
}

// Updates from the odometer are in the odometer's map-like frame, called the
// 'odometry' frame. The odometer_pose converts data from the map frame
// into the odometry frame.
/**
 * @brief PoseTracker::AddOdometerPoseObservation
 * 增加一个里程计坐标系的位姿观测
 * 这里面会调用上面的滤波器更新函数
 * @param time              对应的时刻
 * @param odometer_pose     里程计位姿
 * @param covariance        里程计位姿对应的协方差
 */
void PoseTracker::AddOdometerPoseObservation(
    const common::Time time, const transform::Rigid3d& odometer_pose,
    const PoseCovariance& covariance)
{
  /*如果里程计缓冲区里面有数据*/
  if (!odometry_state_tracker_.empty())
  {
    /*计算里程计的增量*/
    const auto& previous_odometry_state = odometry_state_tracker_.newest();
    const transform::Rigid3d delta =
        previous_odometry_state.odometer_pose.inverse() * odometer_pose;

    //根据里程计得到机器人的位姿 PS:下面是state_pose　不是odometer_pose
    //state_pose表示机器人的状态
    //odometer_pose表示机器人的里程计状态
    const transform::Rigid3d new_pose =
        previous_odometry_state.state_pose * delta;
    AddPoseObservation(time, new_pose, covariance);
  }

  //计算t时刻的分布　相当于计算ukf滤波器time时刻估计的位姿
  const Distribution belief = GetBelief(time);

  /*往里程计缓冲区中加入一个里程计观测　加入了里程计观测　和　ukf滤波器预测*/
  odometry_state_tracker_.AddOdometryState(
      {time, odometer_pose, RigidFromState(belief.GetMean())});
}

const OdometryStateTracker::OdometryStates& PoseTracker::odometry_states()
    const
{
  return odometry_state_tracker_.odometry_states();
}

/**
 * @brief PoseTracker::RigidFromState
 * 从机器人的位姿生成一个转换矩阵
 * @param state 机器人的位姿
 * @return
 */
transform::Rigid3d PoseTracker::RigidFromState(
    const PoseTracker::State& state)
{
  return transform::Rigid3d(
      Eigen::Vector3d(state[PoseTracker::kMapPositionX],
                      state[PoseTracker::kMapPositionY],
                      state[PoseTracker::kMapPositionZ]),
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(state[PoseTracker::kMapOrientationX],
                          state[PoseTracker::kMapOrientationY],
                          state[PoseTracker::kMapOrientationZ])) *
          imu_tracker_.orientation());
}

/**
 * @brief Project2D
 * 把机器人3d位姿的方差转换成机器人2d位姿的方差
 * 即提取(x,y,yaw)的方差
 * @param covariance    机器人3d位姿的方差
 * @return
 */
Pose2DCovariance Project2D(const PoseCovariance& covariance)
{
  Pose2DCovariance projected_covariance;
  projected_covariance.block<2, 2>(0, 0) = covariance.block<2, 2>(0, 0);
  projected_covariance.block<2, 1>(0, 2) = covariance.block<2, 1>(0, 5);
  projected_covariance.block<1, 2>(2, 0) = covariance.block<1, 2>(5, 0);
  projected_covariance(2, 2) = covariance(5, 5);
  return projected_covariance;
}

/**
 * @brief Embed3D
 * 用机器人的2d位姿方差　和　位置方差　＆　朝向方差　生成一个3d位姿方差
 * @param embedded_covariance       2d位姿方差
 * @param position_variance         位姿方差
 * @param orientation_variance      朝向方差
 * @return
 */
PoseCovariance Embed3D(const Pose2DCovariance& embedded_covariance,
                       const double position_variance,
                       const double orientation_variance)
{
  PoseCovariance covariance;
  covariance.setZero();
  covariance.block<2, 2>(0, 0) = embedded_covariance.block<2, 2>(0, 0);
  covariance.block<2, 1>(0, 5) = embedded_covariance.block<2, 1>(0, 2);
  covariance.block<1, 2>(5, 0) = embedded_covariance.block<1, 2>(2, 0);
  covariance(5, 5) = embedded_covariance(2, 2);

  covariance(2, 2) = position_variance;
  covariance(3, 3) = orientation_variance;
  covariance(4, 4) = orientation_variance;
  return covariance;
}

/**
 * @brief PoseCovarianceFromProtoMatrix
 * 从proto_matrix中生成一个3d位姿的方差
 * @param proto_matrix
 * @return
 */
PoseCovariance PoseCovarianceFromProtoMatrix(
    const sensor::proto::Matrix& proto_matrix)
{
  PoseCovariance covariance;
  CHECK_EQ(proto_matrix.rows(), 6);
  CHECK_EQ(proto_matrix.cols(), 6);
  for (int i = 0; i < 6; ++i)
  {
    for (int j = 0; j < 6; ++j)
    {
      covariance(i, j) = proto_matrix.data(i * 6 + j);
    }
  }
  return covariance;
}

}  // namespace kalman_filter
}  // namespace cartographer
