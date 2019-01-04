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

#ifndef CARTOGRAPHER_MAPPING_SPARSE_POSE_GRAPH_H_
#define CARTOGRAPHER_MAPPING_SPARSE_POSE_GRAPH_H_

#include <vector>

#include "../common/lua_parameter_dictionary.h"
#include "../mapping/submaps.h"
#include "../mapping/trajectory_node.h"
#include "../transform/rigid_transform.h"

#include "../mapping/proto/scan_matching_progress.pb.h"
#include "../mapping/proto/sparse_pose_graph_options.pb.h"

namespace cartographer
{
namespace mapping
{

proto::SparsePoseGraphOptions CreateSparsePoseGraphOptions(
        common::LuaParameterDictionary* const parameter_dictionary);

// Splits TrajectoryNodes by ID.
std::vector<std::vector<TrajectoryNode>> SplitTrajectoryNodes(
                                             const std::vector<TrajectoryNode>& trajectory_nodes);

/**
 * @brief The SparsePoseGraph class
 * 这个SparsePoseGraph类是一个优化的基类。
 * 在2d slam和3d slam中都需要实现这个基类中的一些方法
 */
class SparsePoseGraph
{
public:
    // A "constraint" as in the paper by Konolige, Kurt, et al. "Efficient sparse
    // pose adjustment for 2d mapping." Intelligent Robots and Systems (IROS),
    // 2010 IEEE/RSJ International Conference on (pp. 22--29). IEEE, 2010.
    // 约束的具体结构可以参考SPA的公式2

    //定义2d约束的结构体
    struct Constraint2D
    {
        struct Pose
        {
            transform::Rigid2d zbar_ij;                     //ij位姿的观测值
            Eigen::Matrix<double, 3, 3> sqrt_Lambda_ij;     //观测值的方差
        };

        // Submap index.
        // submap的下标
        int i;

        // Scan index.
        // 激光的下标
        int j;

        // Pose of the scan 'j' relative to submap 'i'.
        // 激光j在submap i中的位姿
        Pose pose;

        // Differentiates between intra-submap (where scan 'j' was inserted into
        // submap 'i') and inter-submap constraints (where scan 'j' was not inserted
        // into submap 'i').
        // 这个tag指示scan j是否位于submap i内。
        // INTRA_SUBMAP表示scan j位于submap i内。
        // INTER_SUBMAP表示scan j不位于submap i内。
        enum Tag { INTRA_SUBMAP, INTER_SUBMAP } tag;
    };

    //定义3d约束的结构体
    struct Constraint3D
    {
        struct Pose
        {
            transform::Rigid3d zbar_ij;
            Eigen::Matrix<double, 6, 6> sqrt_Lambda_ij;
        };

        // Submap index.
        int i;

        // Scan index.
        int j;

        // Pose of the scan 'j' relative to submap 'i'.
        Pose pose;

        // Differentiates between intra-submap (where scan 'j' was inserted into
        // submap 'i') and inter-submap constraints (where scan 'j' was not inserted
        // into submap 'i').
        enum Tag { INTRA_SUBMAP, INTER_SUBMAP } tag;
    };

    SparsePoseGraph() {}
    virtual ~SparsePoseGraph() {}

    SparsePoseGraph(const SparsePoseGraph&) = delete;
    SparsePoseGraph& operator=(const SparsePoseGraph&) = delete;


    /*下位的函数都是虚函数 需要子类实现它们 具体就是需要2d slam和3d slam中的sparse_pose_graph类都实现这些函数*/

    // Computes optimized poses.
    virtual void RunFinalOptimization() = 0;

    // Will once return true whenever new optimized poses are available.
    virtual bool HasNewOptimizedPoses() = 0;

    // Returns the scan matching progress.
    virtual proto::ScanMatchingProgress GetScanMatchingProgress() = 0;

    // Get the current trajectory clusters.
    virtual std::vector<std::vector<const Submaps*>>
                                                     GetConnectedTrajectories() = 0;

    // Returns the current optimized transforms for the given 'submaps'.
    virtual std::vector<transform::Rigid3d> GetSubmapTransforms(
            const Submaps& submaps) = 0;

    // Returns the transform converting data in the local map frame (i.e. the
    // continuous, non-loop-closed frame) into the global map frame (i.e. the
    // discontinuous, loop-closed frame).
    virtual transform::Rigid3d GetLocalToGlobalTransform(
            const mapping::Submaps& submaps) = 0;

    // Returns the current optimized trajectory.
    virtual std::vector<TrajectoryNode> GetTrajectoryNodes() = 0;

    // Returns the collection of 2D constraints.
    virtual std::vector<Constraint2D> constraints_2d() = 0;

    // Returns the collection of 3D constraints.
    virtual std::vector<Constraint3D> constraints_3d() = 0;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_SPARSE_POSE_GRAPH_H_
