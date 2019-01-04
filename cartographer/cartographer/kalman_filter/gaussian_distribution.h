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

#ifndef CARTOGRAPHER_KALMAN_FILTER_GAUSSIAN_DISTRIBUTION_H_
#define CARTOGRAPHER_KALMAN_FILTER_GAUSSIAN_DISTRIBUTION_H_

#include "eigen3/Eigen/Cholesky"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

namespace cartographer {
namespace kalman_filter {

/*
 * 定义高斯分布类，T表示数据类型 N表示状态变量的维数
 * 这里的数据类型一半都是int float double之类的．
*/
template <typename T, int N>
class GaussianDistribution
{
 public:

  //构造函数　需要指定高斯分布的均值和协方差
  GaussianDistribution(const Eigen::Matrix<T, N, 1>& mean,
                       const Eigen::Matrix<T, N, N>& covariance)
      : mean_(mean), covariance_(covariance) {}

  //返回均值
  const Eigen::Matrix<T, N, 1>& GetMean() const { return mean_; }

  //放回方差
  const Eigen::Matrix<T, N, N>& GetCovariance() const { return covariance_; }

 private:
  Eigen::Matrix<T, N, 1> mean_;
  Eigen::Matrix<T, N, N> covariance_;
};

//定义的静态方法：两个高斯分布相加
//这里的加号定义为：对应的均值相加　对应的方差相加
template <typename T, int N>
GaussianDistribution<T, N> operator+(const GaussianDistribution<T, N>& lhs,
                                     const GaussianDistribution<T, N>& rhs) {
  return GaussianDistribution<T, N>(lhs.GetMean() + rhs.GetMean(),
                                    lhs.GetCovariance() + rhs.GetCovariance());
}

/*
 * 定义的静态方法:高斯分布乘以一个数／数组(或者说高斯分布通过一个线性变换之后生成的新的高斯分布)
 * 数组的位数由N.M指定
*/
template <typename T, int N, int M>
GaussianDistribution<T, N> operator*(const Eigen::Matrix<T, N, M>& lhs,
                                     const GaussianDistribution<T, M>& rhs) {
  return GaussianDistribution<T, N>(
      lhs * rhs.GetMean(), lhs * rhs.GetCovariance() * lhs.transpose());
}

}  // namespace kalman_filter
}  // namespace cartographer

#endif  // CARTOGRAPHER_KALMAN_FILTER_GAUSSIAN_DISTRIBUTION_H_
