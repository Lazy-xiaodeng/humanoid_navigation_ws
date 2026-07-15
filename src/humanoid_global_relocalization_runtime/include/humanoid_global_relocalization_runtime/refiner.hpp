#pragma once

/*
 * refiner.hpp
 *
 * 文件作用：
 *   1. 定义 ICP/GICP/NDT 精配准的统一接口。
 *   2. 接收 3D-BBS 输出的 top-K 粗候选，在局部范围内做 scan-to-map 精配准。
 *   3. 离线 evaluator 和在线  节点共用本模块，保证同一组参数下的精配准行为一致。
 *
 * 设计说明：
 *   - source=当前 scan，target=预建地图，initial guess=3D-BBS 的 map->base_footprint 粗位姿。
 *   - refine_candidates 会按 fitness score 在候选中选择最优结果。
 *   - 如果所有候选精配准都不收敛，会回退到 BBS top-1，避免未初始化位姿进入后续逻辑。
 */

#include <vector>

#include <Eigen/Dense>

#include "humanoid_global_relocalization_runtime/config.hpp"
#include "humanoid_global_relocalization_runtime/point_cloud_adapter.hpp"
#include "humanoid_global_relocalization_runtime/simple_bbs3d.hpp"

namespace humanoid_global_relocalization
{

struct RefineOutput
{
  Eigen::Matrix4d pose{Eigen::Matrix4d::Identity()};
  int candidate_rank{0};
  double fitness_score{0.0};
  double selection_score{0.0};
  double elapsed_ms{0.0};
  bool converged{false};
};

RefineOutput refine_single_candidate(
  const CloudPtr & map_cloud,
  const CloudPtr & scan_cloud,
  const BbsCandidate & candidate,
  int rank,
  const RefineConfig & config);

RefineOutput refine_candidates(
  const CloudPtr & map_cloud,
  const CloudPtr & scan_cloud,
  const std::vector<BbsCandidate> & candidates,
  const RefineConfig & config);

}  // namespace humanoid_global_relocalization
