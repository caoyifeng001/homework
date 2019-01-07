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

#ifndef CARTOGRAPHER_SENSOR_LASER_H_
#define CARTOGRAPHER_SENSOR_LASER_H_

#include "../common/port.h"
#include "../sensor/compressed_point_cloud.h"
#include "../sensor/point_cloud.h"
#include "cartographer/sensor/proto/sensor.pb.h"

namespace cartographer {
namespace sensor {

// Builds a LaserFan from 'proto' and separates any beams with ranges outside
// the range ['min_range', 'max_range']. Beams beyond 'max_range' inserted into
// the 'missing_echo_point_cloud' with length 'missing_echo_ray_length'. The
// points in both clouds are stored in scan order.
// 2d激光帧信息
struct LaserFan
{
  Eigen::Vector2f origin;               //这帧激光的原点
  PointCloud2D point_cloud;             //正常范围内的激光点
  PointCloud2D missing_echo_point_cloud;//测量值大于max_range的激光点
};

LaserFan ToLaserFan(const proto::LaserScan& proto, float min_range,
                    float max_range, float missing_echo_ray_length);

// Transforms 'laser_fan' according to 'transform'.
// 把laserscan通过转换矩阵transform进行转换
LaserFan TransformLaserFan(const LaserFan& laser_fan,
                           const transform::Rigid2f& transform);

// A 3D variant of LaserFan. Rays begin at 'origin'. 'returns' are the points
// where laser returns were detected. 'misses' are points in the direction of
// rays for which no return was detected, and were inserted at a configured
// distance. It is assumed that between the 'origin' and 'misses' is free space.
// 3d激光帧信息　注意如果激光没有探测到任何物体，那么则认为都是空白区域
// 即对应与ros中的lasercan use inf
struct LaserFan3D
{
  Eigen::Vector3f origin;           //这帧激光的原点
  PointCloud returns;               //正常返回的测量值的激光点
  PointCloud misses;                //没有测量值返回的激光点 即超过了range_max的激光点

  // Reflectivity value of returns.
  std::vector<uint8> reflectivities;//激光返回值的反射率
};

// Like LaserFan3D but with compressed point clouds. The point order changes
// when converting from LaserFan3D.
struct CompressedLaserFan3D
{
  Eigen::Vector3f origin;
  CompressedPointCloud returns;
  CompressedPointCloud misses;

  // Reflectivity value of returns.
  std::vector<uint8> reflectivities;
};

LaserFan3D Decompress(const CompressedLaserFan3D& compressed_laser_fan);

CompressedLaserFan3D Compress(const LaserFan3D& laser_fan);

// Converts 3D 'laser_fan' to a proto::LaserFan3D.
proto::LaserFan3D ToProto(const LaserFan3D& laser_fan);

// Converts 'proto' to a LaserFan3D.
LaserFan3D FromProto(const proto::LaserFan3D& proto);

LaserFan3D ToLaserFan3D(const LaserFan& laser_fan);

LaserFan3D TransformLaserFan3D(const LaserFan3D& laser_fan,
                               const transform::Rigid3f& transform);

// Projects 'laser_fan' into 2D and crops it according to the cuboid defined by
// 'min' and 'max'.
//　把激光雷达投影到2d平面　并且　根据min & max定义的立方体来对激光数据进行裁剪
LaserFan ProjectCroppedLaserFan(const LaserFan3D& laser_fan,
                                const Eigen::Vector3f& min,
                                const Eigen::Vector3f& max);

// Filter a 'laser_fan', retaining only the returns that have no more than
// 'max_range' distance from the laser origin. Removes misses and reflectivity
// information.
// 对完整的激光雷达数据进行滤波，只保留范围不超过max_range的激光点．
// misses激光点　和　反射率信息都被去除掉了．
// 经过这个滤波　激光信息里面就只剩下合法的距离信息了
LaserFan3D FilterLaserFanByMaxRange(const LaserFan3D& laser_fan,
                                    float max_range);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_LASER_H_
