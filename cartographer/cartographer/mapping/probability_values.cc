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

#include "../mapping/probability_values.h"

namespace cartographer {
namespace mapping {

namespace {

// 0 is unknown, [1, 32767] maps to [kMinProbability, kMaxProbability].
//　把[1,32767]映射到[kMinProbability, kMaxProbability]
//　传入一个整数　返回一个概率
//  也就是这个整数对应的概率
//　这个函数被下面的PrecomputeValueToProbability()函数调用
//　这里的value是[1,32767]
float SlowValueToProbability(const uint16 value)
{
  CHECK_GE(value, 0);
  CHECK_LE(value, 32767);
  if (value == kUnknownProbabilityValue)
  {
    // Unknown cells have kMinProbability.
    return kMinProbability;
  }
  const float kScale = (kMaxProbability - kMinProbability) / 32766.f;
  return value * kScale + (kMinProbability - kScale);
}

//计算所有的value[1,32768]到probability的转换
const std::vector<float>* PrecomputeValueToProbability()
{
  std::vector<float>* result = new std::vector<float>;
  // Repeat two times, so that both values with and without the update marker
  // can be converted to a probability.
  //　这里计算两遍，因此无论加不加kUpdateMarker都可以进行转换
  //  kUpdateMarker＝32768
  for (int repeat = 0; repeat != 2; ++repeat)
  {
    for (int value = 0; value != 32768; ++value)
    {
      result->push_back(SlowValueToProbability(value));
    }
  }
  return result;
}

}  // namespace

//实现计算好的value到probability的转换的数组
const std::vector<float>* const kValueToProbability =
    PrecomputeValueToProbability();

//计算查询表
//为了使用odds而计算的查询表
//这个函数传入的参数只有两种情况:
//1.hit_probability　对应的 odds
//2.miss_probability 对应的 odds
std::vector<uint16> ComputeLookupTableToApplyOdds(const float odds)
{
  std::vector<uint16> result;
  result.push_back(ProbabilityToValue(ProbabilityFromOdds(odds)) +
                   kUpdateMarker);

  for (int cell = 1; cell != 32768; ++cell)
  {
    //这里的步骤为：
    //1.把cell转换为概率
    //2.把概率转换为odd　然后乘以传入的参数odds
    //3.步骤２的得到的odds，转换为概率
    //4.把概率转换为cell　存入数组中
    result.push_back(ProbabilityToValue(ProbabilityFromOdds(
                         odds * Odds((*kValueToProbability)[cell]))) +
                     kUpdateMarker);
  }
  return result;
}

}  // namespace mapping
}  // namespace cartographer
