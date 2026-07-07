#pragma once

/*
 * temporal_consistency.hpp
 *
 * 文件作用：
 *   1. 定义多帧全局重定位候选一致性分析的数据结构和入口函数。
 *   2. 将每帧 3D-BBS 输出的 map->base_footprint 候选，与同帧 odom->base_footprint 组合成隐含 map->odom。
 *   3. 在连续帧窗口内统计同一个 map->odom 假设被多少帧 top-K 候选共同支持。
 *   4. 为后续在线重定位状态机提供可复用的“候选是否跨帧稳定”判断基础。
 *
 * 设计说明：
 *   - 这个模块不直接读取 bag、不做点云匹配，只处理已经得到的候选位姿和 odom 位姿。
 *   - 离线 evaluator 会用它生成 CSV；在线节点后续也可以用同一套接口维护滑动窗口。
 *   - 它不会单独决定是否发布 map->odom，只输出支持度指标，供状态机和候选拒绝策略使用。
 */

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "humanoid_global_relocalization_runtime/config.hpp"

namespace humanoid_global_relocalization
{

struct TemporalCandidateInput
{
  int rank{0};
  int score{0};
  double score_ratio{0.0};
  Eigen::Matrix4d map_to_base{Eigen::Matrix4d::Identity()};
};

struct TemporalFrameInput
{
  std::string map_path;
  std::string bag_path;
  double stamp_sec{0.0};
  RefineMethod refine_method{RefineMethod::Gicp};
  InputMode input_mode{InputMode::RegisteredWorld};
  bool localized{false};
  bool success{false};
  double translation_error_m{-1.0};
  double yaw_error_deg{-1.0};
  bool has_reference_pose{false};
  Eigen::Matrix4d reference_pose{Eigen::Matrix4d::Identity()};
  int selected_candidate_rank{0};
  bool has_odom_pose{false};
  Eigen::Matrix4d odom_to_base{Eigen::Matrix4d::Identity()};
  std::vector<TemporalCandidateInput> candidates;
};

struct TemporalConsistencyResult
{
  std::string map_path;
  std::string bag_path;
  double stamp_sec{0.0};
  RefineMethod refine_method{RefineMethod::Gicp};
  InputMode input_mode{InputMode::RegisteredWorld};
  bool success{false};
  double translation_error_m{-1.0};
  double yaw_error_deg{-1.0};
  int selected_rank{0};
  int selected_support_frames{0};
  int selected_support_count{0};
  double selected_median_rank{-1.0};
  int best_seed_rank{0};
  int best_support_frames{0};
  int best_support_count{0};
  double best_median_rank{-1.0};
  double selected_map_odom_x{0.0};
  double selected_map_odom_y{0.0};
  double selected_map_odom_yaw_deg{0.0};
  double selected_candidate_translation_error_m{-1.0};
  double selected_candidate_yaw_error_deg{-1.0};
  double best_seed_translation_error_m{-1.0};
  double best_seed_yaw_error_deg{-1.0};
};

// 将 Fast-LIO raw 轴 odom 位姿转换到和候选一致的 base_footprint 标准轴表达。
Eigen::Matrix4d raw_odom_pose_to_base_axis_pose(const Eigen::Matrix4d & raw_odom_pose);

std::vector<TemporalConsistencyResult> analyze_temporal_consistency(
  const std::vector<TemporalFrameInput> & frames,
  const TemporalConsistencyConfig & config);

}  // namespace humanoid_global_relocalization
