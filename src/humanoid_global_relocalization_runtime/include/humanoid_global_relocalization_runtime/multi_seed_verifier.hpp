#pragma once

/*
 * multi_seed_verifier.hpp
 *
 * 文件作用：
 *   1. 从同一失败帧的 BBS/SOLiD 精配候选中建立多个独立定位轨迹。
 *   2. 每条轨迹使用 odom 传播 map->odom，并在后续 scan 上做连续细粒度 GICP。
 *   3. 在滚动窗口内检查修正量、fitness、RMSE、map->odom 稳定性及双轨迹一致性。
 *   4. 只输出恢复候选，不直接发布 ROS 话题或 TF。
 */

#include <deque>
#include <string>
#include <vector>

#include "humanoid_global_relocalization_runtime/config.hpp"
#include "humanoid_global_relocalization_runtime/point_cloud_adapter.hpp"
#include "humanoid_global_relocalization_runtime/refiner.hpp"
#include "humanoid_global_relocalization_runtime/simple_bbs3d.hpp"

namespace humanoid_global_relocalization
{

struct MultiSeedTrackSample
{
  bool valid{false};
  Eigen::Matrix4d pose{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d map_to_odom{Eigen::Matrix4d::Identity()};
  double fitness{0.0};
  double rmse_m{0.0};
  double correction_xy_m{0.0};
  double correction_yaw_deg{0.0};
};

struct MultiSeedTrack
{
  int track_id{0};
  int cluster_id{0};
  int candidate_rank{0};
  std::string source;
  Eigen::Matrix4d anchor_map_to_odom{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d tracked_map_to_odom{Eigen::Matrix4d::Identity()};
  bool has_tracked_map_to_odom{false};
  std::deque<MultiSeedTrackSample> samples;
};

struct MultiSeedInitializationInput
{
  CloudPtr map_cloud;
  CloudPtr scan_cloud;
  Eigen::Matrix4d odom_to_base{Eigen::Matrix4d::Identity()};
  std::vector<BbsCandidate> candidates;
  RefineConfig refine;
};

struct MultiSeedInitializationOutput
{
  std::string reason{"not_run"};
  std::vector<MultiSeedTrack> tracks;
  int refined_candidates{0};
  int retained_clusters{0};
  bool timed_out{false};
  double elapsed_ms{0.0};
};

struct MultiSeedTrackingInput
{
  CloudPtr map_cloud;
  CloudPtr scan_cloud;
  Eigen::Matrix4d odom_to_base{Eigen::Matrix4d::Identity()};
  std::vector<MultiSeedTrack> tracks;
};

struct MultiSeedTrackingOutput
{
  std::string reason{"not_run"};
  std::vector<MultiSeedTrack> tracks;
  int completed_tracks{0};
  bool timed_out{false};
  double elapsed_ms{0.0};
};

struct MultiSeedEvaluationOutput
{
  bool accepted{false};
  std::string reason{"not_ready"};
  Eigen::Matrix4d pose{Eigen::Matrix4d::Identity()};
  int candidate_rank{0};
  int cluster_id{0};
  int qualified_tracks{0};
  int valid_frames{0};
  double fitness_score{0.0};
  double rmse_m{0.0};
  double map_odom_spread_m{0.0};
  double elapsed_ms{0.0};
};

MultiSeedInitializationOutput initialize_multi_seed_tracks(
  const MultiSeedInitializationInput & input,
  const MultiSeedRecoveryConfig & config);

MultiSeedTrackingOutput update_multi_seed_tracks(
  const MultiSeedTrackingInput & input,
  const MultiSeedRecoveryConfig & config);

MultiSeedEvaluationOutput evaluate_multi_seed_tracks(
  const std::vector<MultiSeedTrack> & tracks,
  const Eigen::Matrix4d & current_odom_to_base,
  const MultiSeedRecoveryConfig & config);

}  // namespace humanoid_global_relocalization
