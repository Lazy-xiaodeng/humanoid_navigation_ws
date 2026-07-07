/*
 * temporal_consistency.cpp
 *
 * 文件作用：
 *   1. 实现多帧候选一致性分析。
 *   2. 将每个 3D-BBS 粗候选 map->base_footprint 转成隐含 map->odom。
 *   3. 在同一个 bag、同一张地图、同一种 refine 方法下，按时间窗口统计候选跨帧支持度。
 *   4. 输出“当前 GICP/ICP/NDT 选中的候选是否稳定”和“当前帧 top-K 中最稳定候选是谁”。
 */

#include "humanoid_global_relocalization_runtime/temporal_consistency.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <tuple>

#include "humanoid_global_relocalization_runtime/point_cloud_adapter.hpp"

namespace humanoid_global_relocalization
{
namespace
{

struct CandidateWithMapOdom
{
  TemporalCandidateInput candidate;
  Eigen::Matrix4d map_to_odom{Eigen::Matrix4d::Identity()};
  double map_odom_x{0.0};
  double map_odom_y{0.0};
  double map_odom_yaw_deg{0.0};
};

struct FrameWithMapOdom
{
  TemporalFrameInput frame;
  std::vector<CandidateWithMapOdom> candidates;
};

struct SupportStats
{
  int support_frames{0};
  int support_count{0};
  double median_rank{-1.0};
};

using GroupKey = std::tuple<std::string, std::string, RefineMethod, InputMode>;

double normalize_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double rad_to_deg(double radians)
{
  return radians * 180.0 / M_PI;
}

double yaw_diff_deg(double a_rad, double b_rad)
{
  return std::abs(normalize_angle(a_rad - b_rad)) * 180.0 / M_PI;
}

double median(std::vector<double> values)
{
  if (values.empty()) {
    return -1.0;
  }
  std::sort(values.begin(), values.end());
  const std::size_t mid = values.size() / 2;
  if (values.size() % 2 == 1) {
    return values[mid];
  }
  return 0.5 * (values[mid - 1] + values[mid]);
}

double planar_distance(const CandidateWithMapOdom & a, const CandidateWithMapOdom & b)
{
  const double dx = a.map_odom_x - b.map_odom_x;
  const double dy = a.map_odom_y - b.map_odom_y;
  return std::hypot(dx, dy);
}

std::pair<double, double> candidate_error_against_reference(
  const CandidateWithMapOdom * candidate,
  const TemporalFrameInput & frame)
{
  if (!candidate || !frame.has_reference_pose) {
    return {-1.0, -1.0};
  }

  // 这里评估的是 3D-BBS 粗候选自身的 x/y/yaw 误差，不包含后续 GICP refine。
  // 它用于判断“时序一致性选出的 best_seed 是否位于正确区域”，而不是替代最终精配准误差。
  const double dx = candidate->candidate.map_to_base(0, 3) - frame.reference_pose(0, 3);
  const double dy = candidate->candidate.map_to_base(1, 3) - frame.reference_pose(1, 3);
  const double trans = std::hypot(dx, dy);
  const double yaw = yaw_diff_deg(
    yaw_from_matrix(candidate->candidate.map_to_base),
    yaw_from_matrix(frame.reference_pose));
  return {trans, yaw};
}

SupportStats support_for_seed(
  const std::vector<FrameWithMapOdom> & frames,
  int frame_index,
  const CandidateWithMapOdom & seed,
  const TemporalConsistencyConfig & config)
{
  // 以当前帧某个候选作为 seed，检查前后窗口中每一帧 top-K 是否存在相近的 map->odom。
  // support_frames 关心“有多少帧支持这个假设”，support_count 当前等同于支持帧数，
  // 后续如果允许每帧多个簇内候选，也可以扩展为候选总数。
  const int begin = std::max(0, frame_index - std::max(0, config.window_before));
  const int end = std::min<int>(
    static_cast<int>(frames.size()),
    frame_index + std::max(0, config.window_after) + 1);

  std::vector<double> supporting_best_ranks;
  int support_frames = 0;
  int support_count = 0;
  for (int i = begin; i < end; ++i) {
    int best_rank = std::numeric_limits<int>::max();
    int matched_in_frame = 0;
    for (const auto & candidate : frames[static_cast<std::size_t>(i)].candidates) {
      const double xy = planar_distance(seed, candidate);
      const double yaw = yaw_diff_deg(
        yaw_from_matrix(seed.map_to_odom),
        yaw_from_matrix(candidate.map_to_odom));
      if (xy <= config.xy_gate_m && yaw <= config.yaw_gate_deg) {
        best_rank = std::min(best_rank, candidate.candidate.rank);
        ++matched_in_frame;
      }
    }
    if (matched_in_frame > 0) {
      ++support_frames;
      ++support_count;
      supporting_best_ranks.push_back(static_cast<double>(best_rank));
    }
  }

  SupportStats stats;
  stats.support_frames = support_frames;
  stats.support_count = support_count;
  stats.median_rank = median(supporting_best_ranks);
  return stats;
}

const CandidateWithMapOdom * find_candidate_by_rank(
  const FrameWithMapOdom & frame,
  int rank)
{
  for (const auto & candidate : frame.candidates) {
    if (candidate.candidate.rank == rank) {
      return &candidate;
    }
  }
  return nullptr;
}

std::pair<const CandidateWithMapOdom *, SupportStats> best_supported_seed(
  const std::vector<FrameWithMapOdom> & frames,
  int frame_index,
  const TemporalConsistencyConfig & config)
{
  const CandidateWithMapOdom * best_candidate = nullptr;
  SupportStats best_stats;

  for (const auto & candidate : frames[static_cast<std::size_t>(frame_index)].candidates) {
    const SupportStats stats = support_for_seed(frames, frame_index, candidate, config);
    const auto current_key = std::make_tuple(
      stats.support_frames,
      stats.support_count,
      stats.median_rank >= 0.0 ? -stats.median_rank : -9999.0,
      -candidate.candidate.rank);
    const auto best_key = std::make_tuple(
      best_stats.support_frames,
      best_stats.support_count,
      best_stats.median_rank >= 0.0 ? -best_stats.median_rank : -9999.0,
      best_candidate ? -best_candidate->candidate.rank : -9999);
    if (!best_candidate || current_key > best_key) {
      best_candidate = &candidate;
      best_stats = stats;
    }
  }

  return {best_candidate, best_stats};
}

FrameWithMapOdom build_frame_with_map_odom(const TemporalFrameInput & frame)
{
  FrameWithMapOdom output;
  output.frame = frame;
  if (!frame.has_odom_pose) {
    return output;
  }

  const Eigen::Matrix4d odom_to_base_inv = frame.odom_to_base.inverse();
  output.candidates.reserve(frame.candidates.size());
  for (const auto & candidate : frame.candidates) {
    CandidateWithMapOdom converted;
    converted.candidate = candidate;
    converted.map_to_odom = candidate.map_to_base * odom_to_base_inv;
    converted.map_odom_x = converted.map_to_odom(0, 3);
    converted.map_odom_y = converted.map_to_odom(1, 3);
    converted.map_odom_yaw_deg = rad_to_deg(yaw_from_matrix(converted.map_to_odom));
    output.candidates.push_back(converted);
  }
  return output;
}

GroupKey group_key(const TemporalFrameInput & frame)
{
  return GroupKey{frame.map_path, frame.bag_path, frame.refine_method, frame.input_mode};
}

}  // namespace

Eigen::Matrix4d raw_odom_pose_to_base_axis_pose(const Eigen::Matrix4d & raw_odom_pose)
{
  // Fast-LIO raw 轴和 ROS base_footprint 标准轴关系：
  //   base.x = -raw.z
  //   base.y =  raw.x
  //   base.z = -raw.y
  //
  // 这里处理的是 odom 位姿表达，不处理点云外参平移；目的是把 raw world/body 的轴表达
  // 变换到和 BBS 候选相同的标准轴语义。这个函数与 Python 离线分析脚本保持同一数学约定。
  Eigen::Matrix3d raw_to_base = Eigen::Matrix3d::Zero();
  raw_to_base(0, 2) = -1.0;
  raw_to_base(1, 0) = 1.0;
  raw_to_base(2, 1) = -1.0;

  Eigen::Matrix4d converted = Eigen::Matrix4d::Identity();
  converted.block<3, 3>(0, 0) =
    raw_to_base * raw_odom_pose.block<3, 3>(0, 0) * raw_to_base.transpose();
  converted.block<3, 1>(0, 3) =
    raw_to_base * raw_odom_pose.block<3, 1>(0, 3);
  return converted;
}

std::vector<TemporalConsistencyResult> analyze_temporal_consistency(
  const std::vector<TemporalFrameInput> & frames,
  const TemporalConsistencyConfig & config)
{
  std::vector<TemporalConsistencyResult> results;
  if (!config.enable) {
    return results;
  }

  std::map<GroupKey, std::vector<FrameWithMapOdom>> grouped_frames;
  for (const auto & frame : frames) {
    if (!frame.localized || !frame.has_odom_pose || frame.candidates.empty()) {
      continue;
    }
    grouped_frames[group_key(frame)].push_back(build_frame_with_map_odom(frame));
  }

  for (auto & [key, group] : grouped_frames) {
    (void)key;
    std::sort(
      group.begin(),
      group.end(),
      [](const FrameWithMapOdom & a, const FrameWithMapOdom & b) {
        return a.frame.stamp_sec < b.frame.stamp_sec;
      });

    for (int i = 0; i < static_cast<int>(group.size()); ++i) {
      const auto & frame = group[static_cast<std::size_t>(i)];
      const CandidateWithMapOdom * selected =
        find_candidate_by_rank(frame, frame.frame.selected_candidate_rank);
      if (!selected) {
        continue;
      }

      const SupportStats selected_stats = support_for_seed(group, i, *selected, config);
      const auto [best_candidate, best_stats] = best_supported_seed(group, i, config);

      TemporalConsistencyResult result;
      result.map_path = frame.frame.map_path;
      result.bag_path = frame.frame.bag_path;
      result.stamp_sec = frame.frame.stamp_sec;
      result.refine_method = frame.frame.refine_method;
      result.input_mode = frame.frame.input_mode;
      result.success = frame.frame.success;
      result.translation_error_m = frame.frame.translation_error_m;
      result.yaw_error_deg = frame.frame.yaw_error_deg;
      result.selected_rank = selected->candidate.rank;
      result.selected_support_frames = selected_stats.support_frames;
      result.selected_support_count = selected_stats.support_count;
      result.selected_median_rank = selected_stats.median_rank;
      result.best_seed_rank = best_candidate ? best_candidate->candidate.rank : 0;
      result.best_support_frames = best_stats.support_frames;
      result.best_support_count = best_stats.support_count;
      result.best_median_rank = best_stats.median_rank;
      result.selected_map_odom_x = selected->map_odom_x;
      result.selected_map_odom_y = selected->map_odom_y;
      result.selected_map_odom_yaw_deg = selected->map_odom_yaw_deg;
      const auto selected_error = candidate_error_against_reference(selected, frame.frame);
      const auto best_error = candidate_error_against_reference(best_candidate, frame.frame);
      result.selected_candidate_translation_error_m = selected_error.first;
      result.selected_candidate_yaw_error_deg = selected_error.second;
      result.best_seed_translation_error_m = best_error.first;
      result.best_seed_yaw_error_deg = best_error.second;
      results.push_back(result);
    }
  }

  return results;
}

}  // namespace humanoid_global_relocalization
