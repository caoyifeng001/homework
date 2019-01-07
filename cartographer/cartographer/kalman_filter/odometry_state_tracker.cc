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

#include "../kalman_filter/odometry_state_tracker.h"

#include "../transform/rigid_transform.h"

namespace cartographer {
namespace kalman_filter {

OdometryState::OdometryState(const common::Time time,
                             const transform::Rigid3d& odometer_pose,
                             const transform::Rigid3d& state_pose)
    : time(time), odometer_pose(odometer_pose), state_pose(state_pose) {}

OdometryStateTracker::OdometryStateTracker(const int window_size)
    : window_size_(window_size)
{
  CHECK_GT(window_size, 0);
}

//返回整个里程计队列
const OdometryStateTracker::OdometryStates&
OdometryStateTracker::odometry_states() const
{
  return odometry_states_;
}

/*增加里程计数据到队列中　如果超过了window_size_，则会把过时的里程计状态去除掉*/
void OdometryStateTracker::AddOdometryState(
    const OdometryState& odometry_state)
{
  odometry_states_.push_back(odometry_state);
  while (odometry_states_.size() > window_size_)
  {
    odometry_states_.pop_front();
  }
}

bool OdometryStateTracker::empty() const { return odometry_states_.empty(); }

const OdometryState& OdometryStateTracker::newest() const
{
  return odometry_states_.back();
}

}  // namespace kalman_filter
}  // namespace cartographer
