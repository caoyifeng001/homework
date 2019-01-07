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

#include "../mapping_2d/local_trajectory_builder.h"

#include <limits>

#include "../common/make_unique.h"
#include "../sensor/laser.h"

namespace cartographer {
namespace mapping_2d {

/**
 * @brief CreateLocalTrajectoryBuilderOptions
 * 构造函数，主要用来进行参数的配置
 * @param parameter_dictionary
 * @return
 */
proto::LocalTrajectoryBuilderOptions CreateLocalTrajectoryBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary)
{
  proto::LocalTrajectoryBuilderOptions options;
  options.set_horizontal_laser_min_z(
      parameter_dictionary->GetDouble("horizontal_laser_min_z"));
  options.set_horizontal_laser_max_z(
      parameter_dictionary->GetDouble("horizontal_laser_max_z"));
  options.set_horizontal_laser_voxel_filter_size(
      parameter_dictionary->GetDouble("horizontal_laser_voxel_filter_size"));
  options.set_use_online_correlative_scan_matching(
      parameter_dictionary->GetBool("use_online_correlative_scan_matching"));
  *options.mutable_adaptive_voxel_filter_options() =
      sensor::CreateAdaptiveVoxelFilterOptions(
          parameter_dictionary->GetDictionary("adaptive_voxel_filter").get());
  *options.mutable_real_time_correlative_scan_matcher_options() =
      scan_matching::CreateRealTimeCorrelativeScanMatcherOptions(
          parameter_dictionary
              ->GetDictionary("real_time_correlative_scan_matcher")
              .get());
  *options.mutable_ceres_scan_matcher_options() =
      scan_matching::CreateCeresScanMatcherOptions(
          parameter_dictionary->GetDictionary("ceres_scan_matcher").get());
  *options.mutable_motion_filter_options() =
      mapping_3d::CreateMotionFilterOptions(
          parameter_dictionary->GetDictionary("motion_filter").get());
  *options.mutable_pose_tracker_options() =
      kalman_filter::CreatePoseTrackerOptions(
          parameter_dictionary->GetDictionary("pose_tracker").get());
  *options.mutable_submaps_options() = CreateSubmapsOptions(
      parameter_dictionary->GetDictionary("submaps").get());
  options.set_use_imu_data(parameter_dictionary->GetBool("use_imu_data"));
  return options;
}

LocalTrajectoryBuilder::LocalTrajectoryBuilder(
    const proto::LocalTrajectoryBuilderOptions& options)
    : options_(options),
      submaps_(options.submaps_options()),
      scan_matcher_pose_estimate_(transform::Rigid3d::Identity()),
      motion_filter_(options_.motion_filter_options()),
      real_time_correlative_scan_matcher_(
          options_.real_time_correlative_scan_matcher_options()),
      ceres_scan_matcher_(options_.ceres_scan_matcher_options()) {}

LocalTrajectoryBuilder::~LocalTrajectoryBuilder() {}

//返回所有的submap
const Submaps* LocalTrajectoryBuilder::submaps() const { return &submaps_; }

//返回所有的submap
Submaps* LocalTrajectoryBuilder::submaps() { return &submaps_; }

//返回位姿跟踪器
kalman_filter::PoseTracker* LocalTrajectoryBuilder::pose_tracker() const
{
  return pose_tracker_.get();
}

/**
 * @brief LocalTrajectoryBuilder::BuildProjectedLaserFan
 * 把激光雷达的数据投影到水平面上，同时对其进行一定的滤波
 * 滤波的参数由配置文件指定．
 * 这个函数被AddHorizontalLaserFan()函数调用
 * @param tracking_to_tracking_2d
 * @param laser_fan
 * @return
 */
sensor::LaserFan LocalTrajectoryBuilder::BuildProjectedLaserFan(
    const transform::Rigid3f& tracking_to_tracking_2d,
    const sensor::LaserFan3D& laser_fan) const
{
  const sensor::LaserFan projected_fan = sensor::ProjectCroppedLaserFan(
      sensor::TransformLaserFan3D(laser_fan, tracking_to_tracking_2d),
      Eigen::Vector3f(-std::numeric_limits<float>::infinity(),
                      -std::numeric_limits<float>::infinity(),
                      options_.horizontal_laser_min_z()),
      Eigen::Vector3f(std::numeric_limits<float>::infinity(),
                      std::numeric_limits<float>::infinity(),
                      options_.horizontal_laser_max_z()));

  return sensor::LaserFan{
      projected_fan.origin,
      sensor::VoxelFiltered(projected_fan.point_cloud,
                            options_.horizontal_laser_voxel_filter_size()),
      sensor::VoxelFiltered(projected_fan.missing_echo_point_cloud,
                            options_.horizontal_laser_voxel_filter_size())};
}

/**
 * @brief LocalTrajectoryBuilder::ScanMatch
 * scan-match操作，这里函数会调用real_time_correlative_scan-match来进行scan-match
 * 这个函数里面通过pose_prediction作为初始位姿给real_time_scm进行结算，用csm得到的姿态作为
 * ceres_scanmatch的初始位姿，然后通过ceres-scan_matching来进行scan-to-map的结算来得到最终的位姿．
 * 然后用这个最终的位姿来更新滤波器
 * 1.滤波器给出初始位姿
 * 2.rt_csm在滤波器给出初始位姿的情况下进行优化得到一个新的位姿
 * 3.ceres_sm在rt_csm的位姿下进行进一步的优化，来得到最终的位姿
 * 这个函数被下面的AddHorizontalLaserFan()调用．
 *
 * @param time                          对应的时间
 * @param pose_prediction               滤波器预测的机器人的位姿
 * @param tracking_to_tracking_2d       tracking坐标系到tracking_2d坐标系的转换矩阵
 * @param laser_fan_in_tracking_2d      tracking_2d坐标系中的激光数据 tracking_2d坐标系表示平面机器人坐标系
 * @param pose_observation              返回的机器人位姿
 * @param covariance_observation        返回的机器人位姿的方差
 */
void LocalTrajectoryBuilder::ScanMatch(
    common::Time time,
    const transform::Rigid3d& pose_prediction,
    const transform::Rigid3d& tracking_to_tracking_2d,
    const sensor::LaserFan& laser_fan_in_tracking_2d,
    transform::Rigid3d* pose_observation,
    kalman_filter::PoseCovariance* covariance_observation)
{
  //用来进行scan-match对应的submap的概率栅格地图
  const ProbabilityGrid& probability_grid =
      submaps_.Get(submaps_.matching_index())->probability_grid;

  //计算出来预测的2d位姿　预测的位姿是3d的，因此必须把它旋转到2d平面
  //因为这里是2d-slam所以要把预测的位姿旋转到2d平面
  transform::Rigid2d pose_prediction_2d =
      transform::Project2D(pose_prediction * tracking_to_tracking_2d.inverse());

  // The online correlative scan matcher will refine the initial estimate for
  // the Ceres scan matcher.
  // csm用滤波器提供的初始化进行优化，然后提高一个更好的初始值给ceres-scan-match
  transform::Rigid2d initial_ceres_pose = pose_prediction_2d;

  //定义一个滤波器
  sensor::AdaptiveVoxelFilter adaptive_voxel_filter(
      options_.adaptive_voxel_filter_options());

  //对激光雷达数据进行滤波 & 转换成点云数据  这里的点云数据是在平面机器人坐标系中
  const sensor::PointCloud2D filtered_point_cloud_in_tracking_2d =
      adaptive_voxel_filter.Filter(laser_fan_in_tracking_2d.point_cloud);

  //配置文件中是否需要用csm来优化ceres-scan-match的初始解
  if (options_.use_online_correlative_scan_matching())
  {
    //通过csm和滤波器过后的2d平面的　激光雷达数据来进行位姿优化
    //传入预测的初始位姿＼激光雷达数据＼栅格地图
    //返回一个更好的值initial_ceres_pose
    real_time_correlative_scan_matcher_.Match(
        pose_prediction_2d, filtered_point_cloud_in_tracking_2d,
        probability_grid, &initial_ceres_pose);
  }

  /*最终通过ceres_scan_match来得到最终的位姿*/
  /*这里得到的位姿是tracking_2d坐标系到map坐标系的转换*/
  transform::Rigid2d tracking_2d_to_map;
  kalman_filter::Pose2DCovariance covariance_observation_2d;
  ceres::Solver::Summary summary;
  ceres_scan_matcher_.Match(
      transform::Project2D(scan_matcher_pose_estimate_ *
                           tracking_to_tracking_2d.inverse()),//表示上一个周期的平面位姿
      initial_ceres_pose,                   //这一次的初始估计位姿
      filtered_point_cloud_in_tracking_2d,  //对应的2维激光点云
      probability_grid,                     //概率栅格地图
      &tracking_2d_to_map, &covariance_observation_2d, &summary);

  CHECK(pose_tracker_ != nullptr);

  /*
   * 把得到的2d位姿的方差转换为3d的位姿和方差
   * 因为滤波器要三维位姿来进行更新
　　*/
  *pose_observation = transform::Embed3D(tracking_2d_to_map);
  // This covariance is used for non-yaw rotation and the fake height of 0.
  constexpr double kFakePositionCovariance = 1.;
  constexpr double kFakeOrientationCovariance = 1.;
  *covariance_observation =
      kalman_filter::Embed3D(covariance_observation_2d, kFakePositionCovariance,
                             kFakeOrientationCovariance);

  /*用ceres-scan-match得到的位姿来更新滤波器*/
  pose_tracker_->AddPoseObservation(
      time, (*pose_observation) * tracking_to_tracking_2d,
      *covariance_observation);
}

/**
 * @brief LocalTrajectoryBuilder::AddHorizontalLaserFan
 * 增加一帧激光雷达数据．
 * 这个函数是主要的函数　在这里把激光进行scan-match操作，得到的位姿来更新滤波器
 * 同时也会把激光数据插入到submap中
 * @param time
 * @param laser_fan
 * @return
 */
std::unique_ptr<LocalTrajectoryBuilder::InsertionResult>
LocalTrajectoryBuilder::AddHorizontalLaserFan(
    const common::Time time, const sensor::LaserFan3D& laser_fan)
{
  // Initialize pose tracker now if we do not ever use an IMU.
  if (!options_.use_imu_data())
  {
    InitializePoseTracker(time);
  }

  if (pose_tracker_ == nullptr)
  {
    // Until we've initialized the UKF with our first IMU message, we cannot
    // compute the orientation of the laser scanner.
    LOG(INFO) << "PoseTracker not yet initialized.";
    return nullptr;
  }

  //得到ukf的预测的位姿和协方差　相当于滤波器中的预测位姿
  transform::Rigid3d pose_prediction;
  kalman_filter::PoseCovariance covariance_prediction;
  pose_tracker_->GetPoseEstimateMeanAndCovariance(time, &pose_prediction,
                                                  &covariance_prediction);

  // Computes the rotation without yaw, as defined by GetYaw().
  // 计算出没有yaw轴的旋转矩阵　或者说yaw角度为０的旋转矩阵
  // 这个旋转矩阵的功能为：从当前坐标系旋转到水平面的旋转矩阵
  // 实现过程:定义一个按照z轴旋转-yaw度的四元数，然后和pose_prediction相乘．
  // 这样相当于pose_prediction把yaw设置为０了．
  // yaw轴为0的旋转矩阵，可以把激光投影到二维平面上．
  // tracking_to_tracking_2d表示把激光雷达目前的坐标系转换到平面坐标系的转换矩阵
  // tracking表示目前激光雷达所在的坐标系
  // tracking_2d表示原点与tracking坐标系重合的平面坐标系

  // 目前的旋转向量是-yaw，平移向量是0,0,0
  const transform::Rigid3d tracking_to_tracking_2d =
      transform::Rigid3d::Rotation(
        //绕z轴旋转-yaw度
        Eigen::Quaterniond(Eigen::AngleAxisd(
              -transform::GetYaw(pose_prediction), Eigen::Vector3d::UnitZ())) *
          pose_prediction.rotation());


  //通过上面计算出来的没有yaw轴的旋转矩阵，把激光雷达投影到2d平面
  //laser_fan_in_tracking_2d表示原始激光雷达投影到2d平面之后的激光数据
  //这里的数据还是在机器人坐标系中
  const sensor::LaserFan laser_fan_in_tracking_2d =
      BuildProjectedLaserFan(tracking_to_tracking_2d.cast<float>(), laser_fan);


  //如果里面没有激光点　则直接返回
  if (laser_fan_in_tracking_2d.point_cloud.empty())
  {
    LOG(WARNING) << "Dropped empty horizontal laser point cloud.";
    return nullptr;
  }

  //在ukf的预测位姿的基础上，通过scanmatch提升得到滤波器的观测位姿
  //ukf通过IMU计算的位姿称为预测位姿．
  //在这里通过scanmatch匹配的位姿叫做观测位姿
  //这ScanMatch()函数内部就会更新滤波器
  transform::Rigid3d pose_observation;
  kalman_filter::PoseCovariance covariance_observation;
  ScanMatch(time, pose_prediction, tracking_to_tracking_2d,
            laser_fan_in_tracking_2d, &pose_observation,
            &covariance_observation);

  //滤波器更新完毕之后，得到机器人的最新的估计的位姿和方差
  kalman_filter::PoseCovariance covariance_estimate;
  pose_tracker_->GetPoseEstimateMeanAndCovariance(
      time, &scan_matcher_pose_estimate_, &covariance_estimate);

  // Remove the untracked z-component which floats around 0 in the UKF.
  //　ukf得到的是三维空间中的位姿　转换为平面坐标系的位姿－－　去除掉z轴的位置
  const auto translation = scan_matcher_pose_estimate_.translation();
  scan_matcher_pose_estimate_ = transform::Rigid3d(
      transform::Rigid3d::Vector(translation.x(), translation.y(), 0.),
      scan_matcher_pose_estimate_.rotation());

  // 得到tracking_2d坐标系到map坐标系的转换　把上面去掉z轴数据的旋转坐标系再旋转到平面上．
  // 即得到tracking_2d到map的转换关系　这个转换关系也就是机器人的位姿－－因为机器人是在2d平面的
  const transform::Rigid3d tracking_2d_to_map =
      scan_matcher_pose_estimate_ * tracking_to_tracking_2d.inverse();

  //更新位姿估计
  last_pose_estimate_ = {
      time,
      {pose_prediction, covariance_prediction},
      {pose_observation, covariance_observation},
      {scan_matcher_pose_estimate_, covariance_estimate},
      scan_matcher_pose_estimate_,
      sensor::TransformPointCloud(
          sensor::ToPointCloud(laser_fan_in_tracking_2d.point_cloud),
          tracking_2d_to_map.cast<float>())};

  //得到滤波器更新之后，估计出来的2d位姿　把Rigid3d赋给一个Rigid2d
  const transform::Rigid2d pose_estimate_2d =
      transform::Project2D(tracking_2d_to_map);

  //运动滤波器器
  if (motion_filter_.IsSimilar(time, transform::Embed3D(pose_estimate_2d)))
  {
    return nullptr;
  }

  /*得到和激光匹配的submap 即submap(size-2)*/
  const mapping::Submap* const matching_submap =
      submaps_.Get(submaps_.matching_index());

  //这里得到的要进行插入的submap 这里的进行插入的submap一般由两个
  //即size()-1和size()-2
  std::vector<const mapping::Submap*> insertion_submaps;
  for (int insertion_index : submaps_.insertion_indices())
  {
    insertion_submaps.push_back(submaps_.Get(insertion_index));
  }

  //把激光插入到submaps中．
  submaps_.InsertLaserFan(TransformLaserFan(laser_fan_in_tracking_2d,
                                            pose_estimate_2d.cast<float>()));

  return common::make_unique<InsertionResult>(InsertionResult{
      time, &submaps_, matching_submap, insertion_submaps,
      tracking_to_tracking_2d, tracking_2d_to_map, laser_fan_in_tracking_2d,
      pose_estimate_2d, covariance_estimate});
}

const mapping::GlobalTrajectoryBuilderInterface::PoseEstimate&
LocalTrajectoryBuilder::pose_estimate() const
{
  return last_pose_estimate_;
}

/**
 * @brief LocalTrajectoryBuilder::AddImuData
 * 给滤波器增加一次imu的数据
 * 即使上就是调用pose_tracker的对应的函数，来更新imu的状态．
 * 实现姿态结算 ，
 * @param time                  imu数据对应的时间
 * @param linear_acceleration   imu数据的线性加速度
 * @param angular_velocity      imu数据的角速度
 */
void LocalTrajectoryBuilder::AddImuData(
    const common::Time time, const Eigen::Vector3d& linear_acceleration,
    const Eigen::Vector3d& angular_velocity)
{
  CHECK(options_.use_imu_data()) << "An unexpected IMU packet was added.";

  InitializePoseTracker(time);
  pose_tracker_->AddImuLinearAccelerationObservation(time, linear_acceleration);
  pose_tracker_->AddImuAngularVelocityObservation(time, angular_velocity);

  //这后面的代码都是背包应用的特殊情况　用来计算背包的倾斜角度是否超过了20度
  //超过了这个角度就报警
  transform::Rigid3d pose_estimate;
  kalman_filter::PoseCovariance unused_covariance_estimate;
  pose_tracker_->GetPoseEstimateMeanAndCovariance(time, &pose_estimate,
                                                  &unused_covariance_estimate);

  // Log a warning if the backpack inclination exceeds 20 degrees. In these
  // cases, it's very likely that 2D SLAM will fail.
  // 计算
  const Eigen::Vector3d gravity_direction =
      Eigen::Quaterniond(pose_estimate.rotation()) * Eigen::Vector3d::UnitZ();

  const double inclination = std::acos(gravity_direction.z());

  constexpr double kMaxInclination = common::DegToRad(20.);

  LOG_IF_EVERY_N(WARNING, inclination > kMaxInclination, 1000)
      << "Max inclination exceeded: " << common::RadToDeg(inclination) << " > "
      << common::RadToDeg(kMaxInclination);
}

/**
 * @brief LocalTrajectoryBuilder::AddOdometerPose
 * 增加一个里程计观测数据
 * 这里面就是调用pose_tracker里面的对应的函数，来对滤波器进行更新
 * @param time          里程计数据的时刻
 * @param pose          里程计数据的位姿
 * @param covariance    里程计数据的方差
 */
void LocalTrajectoryBuilder::AddOdometerPose(
    const common::Time time, const transform::Rigid3d& pose,
    const kalman_filter::PoseCovariance& covariance)
{
  if (pose_tracker_ == nullptr)
  {
    // Until we've initialized the UKF with our first IMU message, we cannot
    // process odometry poses.
    LOG_EVERY_N(INFO, 100) << "PoseTracker not yet initialized.";
  }
  else
  {
    pose_tracker_->AddOdometerPoseObservation(time, pose, covariance);
  }
}

/**
 * @brief LocalTrajectoryBuilder::InitializePoseTracker
 * 初始化位姿追踪器，选择2D的预测函数
 * @param time
 */
void LocalTrajectoryBuilder::InitializePoseTracker(const common::Time time)
{
  if (pose_tracker_ == nullptr)
  {
        pose_tracker_ = common::make_unique<kalman_filter::PoseTracker>(
        options_.pose_tracker_options(),
        kalman_filter::PoseTracker::ModelFunction::k2D, time);
  }
}

}  // namespace mapping_2d
}  // namespace cartographer
