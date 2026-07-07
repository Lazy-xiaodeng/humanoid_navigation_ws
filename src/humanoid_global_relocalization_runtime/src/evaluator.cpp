/*
 * evaluator.cpp
 *
 * 文件作用：
 *   1. 实现单帧 PCD 离线评估闭环。
 *   2. 串联地图/scan 加载、预处理、3D-BBS、GICP、资源统计、CSV 输出和对齐 PCD 保存。
 *   3. 支持单帧 PCD 和多 bag 抽帧验证，并保持相同的输出指标和数据结构。
 */

#include "humanoid_global_relocalization_runtime/evaluator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "humanoid_global_relocalization_runtime/bbs2d_search.hpp"
#include "humanoid_global_relocalization_runtime/point_cloud_adapter.hpp"
#include "humanoid_global_relocalization_runtime/refiner.hpp"
#include "humanoid_global_relocalization_runtime/scan_context.hpp"
#include "humanoid_global_relocalization_runtime/temporal_consistency.hpp"

namespace humanoid_global_relocalization
{
namespace
{

double elapsed_ms(const std::chrono::steady_clock::time_point & start, const std::chrono::steady_clock::time_point & end)
{
  return std::chrono::duration<double, std::milli>(end - start).count();
}

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

double stamp_to_sec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

double bag_time_to_sec(std::int64_t nanoseconds)
{
  return static_cast<double>(nanoseconds) * 1e-9;
}

Eigen::Matrix4d pose_to_matrix(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  Eigen::Quaterniond q(
    pose.orientation.w,
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z);
  if (q.norm() < 1e-12) {
    q = Eigen::Quaterniond::Identity();
  } else {
    q.normalize();
  }
  matrix.block<3, 3>(0, 0) = q.toRotationMatrix();
  matrix(0, 3) = pose.position.x;
  matrix(1, 3) = pose.position.y;
  matrix(2, 3) = pose.position.z;
  return matrix;
}

template <typename MessageT>
MessageT deserialize_bag_message(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & bag_message)
{
  // rosbag2_cpp::Reader 返回序列化消息。这里使用 rclcpp::Serialization 按目标消息类型反序列化。
  // 该函数只在已经按 topic 确认类型的地方调用，避免把点云误解成 odom。
  MessageT message;
  rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
  rclcpp::Serialization<MessageT> serialization;
  serialization.deserialize_message(&serialized, &message);
  return message;
}

CloudPtr pointcloud2_to_xyz_cloud(const sensor_msgs::msg::PointCloud2 & msg)
{
  CloudPtr cloud(new Cloud);
  pcl::fromROSMsg(msg, *cloud);
  return cloud;
}

struct BagCloudSample
{
  std::string bag_path;
  int frame_index{-1};
  double stamp_sec{0.0};
  double bag_time_sec{0.0};
  sensor_msgs::msg::PointCloud2 cloud_msg;
};

struct BagReferenceSample
{
  double header_stamp_sec{0.0};
  double bag_time_sec{0.0};
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
};

struct BagData
{
  std::vector<nav_msgs::msg::Odometry> odoms;
  std::vector<BagReferenceSample> reference_poses;
  std::vector<BagCloudSample> clouds;
  std::vector<BagCloudSample> scan_context_keyframes;
};

struct LoadedBag
{
  std::string path;
  bool ok{false};
  std::string error;
  BagData data;
  std::vector<ScanContextEntry> scan_context_database;
};

struct EvaluationRecord
{
  EvaluationSummary summary;
  CloudPtr map_cloud;
  CloudPtr scan_cloud;
  RefineConfig refine;
  FrameConfig frames;
  EvaluationConfig evaluation;
  InputMode input_mode{InputMode::RegisteredWorld};
};

void append_evaluation_csvs(const RuntimeConfig & config, const EvaluationSummary & summary);

struct VoxelKey
{
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    // 三个大质数混合体素坐标。这里不要求加密散列，只要让相邻体素尽量分散到哈希桶即可。
    const std::size_t hx = static_cast<std::size_t>(key.x * 73856093);
    const std::size_t hy = static_cast<std::size_t>(key.y * 19349663);
    const std::size_t hz = static_cast<std::size_t>(key.z * 83492791);
    return hx ^ hy ^ hz;
  }
};

using OccupancySet = std::unordered_set<VoxelKey, VoxelKeyHash>;

VoxelKey voxel_key_from_point(double x, double y, double z, double voxel_size)
{
  return VoxelKey{
    static_cast<int>(std::floor(x / voxel_size)),
    static_cast<int>(std::floor(y / voxel_size)),
    static_cast<int>(std::floor(z / voxel_size))};
}

OccupancySet build_occupancy_set(const CloudPtr & map_cloud, double voxel_size)
{
  // 轨迹 likelihood 只需要判断 scan 点落到地图占据附近的比例，因此把预处理后的地图点云离散成体素集合。
  // 使用 unordered_set 可以让每个点的邻域查询保持近似 O(1)，避免为每个候选做 KdTree 最近邻查询。
  OccupancySet occupied;
  if (!map_cloud || voxel_size <= 1e-6) {
    return occupied;
  }
  occupied.reserve(map_cloud->size() * 2U);
  for (const auto & point : map_cloud->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    occupied.insert(voxel_key_from_point(point.x, point.y, point.z, voxel_size));
  }
  return occupied;
}

bool has_occupied_neighbor(
  const OccupancySet & occupied,
  const VoxelKey & center,
  int neighbor_radius)
{
  // neighbor_radius=0 表示必须落在同一个体素；=1 表示允许 3x3x3 邻域，
  // 可以容忍 scan 噪声、体素边界和小量 odom 误差，但半径过大也会让重复结构更难区分。
  const int radius = std::max(0, neighbor_radius);
  for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dz = -radius; dz <= radius; ++dz) {
        if (occupied.find(VoxelKey{center.x + dx, center.y + dy, center.z + dz}) != occupied.end()) {
          return true;
        }
      }
    }
  }
  return false;
}

double overlap_ratio_for_pose(
  const CloudPtr & scan_cloud,
  const Eigen::Matrix4d & map_to_base,
  const OccupancySet & occupied,
  double voxel_size,
  int neighbor_radius)
{
  // 将一帧局部 scan 通过候选位姿投到 map 下，然后统计与地图占据体素的重合比例。
  // 这个分数不做最近邻距离优化，目的是低成本地判断“这个候选能否解释历史轨迹上的多帧观测”。
  if (!scan_cloud || scan_cloud->empty() || occupied.empty() || voxel_size <= 1e-6) {
    return 0.0;
  }

  int valid_points = 0;
  int hit_points = 0;
  for (const auto & point : scan_cloud->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    const Eigen::Vector4d local(point.x, point.y, point.z, 1.0);
    const Eigen::Vector4d mapped = map_to_base * local;
    const VoxelKey key = voxel_key_from_point(mapped.x(), mapped.y(), mapped.z(), voxel_size);
    ++valid_points;
    if (has_occupied_neighbor(occupied, key, neighbor_radius)) {
      ++hit_points;
    }
  }

  return valid_points > 0 ? static_cast<double>(hit_points) / static_cast<double>(valid_points) : 0.0;
}

std::set<int> explicit_sample_indices(const InputConfig & input)
{
  // bag_sample_frame_indices 用于随机选点和复现问题帧。这里过滤负数并去重，
  // 避免 YAML 中重复或非法序号导致同一帧重复评估。
  std::set<int> indices;
  for (const int index : input.bag_sample_frame_indices) {
    if (index >= 0) {
      indices.insert(index);
    }
  }
  return indices;
}

bool should_sample_cloud_frame(
  int cloud_seen,
  int sampled_count,
  const InputConfig & input,
  const std::set<int> & explicit_indices)
{
  // 显式序号列表优先级最高，用于随机覆盖任意启动位置。此时 max_bag_frames 仍作为安全上限，
  // 防止误把几万个 frame index 写进 YAML 后一次离线验证跑太久。
  if (!explicit_indices.empty()) {
    return explicit_indices.count(cloud_seen) > 0 &&
      sampled_count < std::max(1, input.max_bag_frames);
  }

  // 没有显式序号时保持旧逻辑：跳过开头若干帧，然后按固定 stride 抽样。
  const int sample_index = cloud_seen - std::max(0, input.bag_start_frame_skip);
  return sample_index >= 0 &&
    sample_index % std::max(1, input.bag_frame_stride) == 0 &&
    sampled_count < std::max(1, input.max_bag_frames);
}

void finalize_resource_usage(const ResourceSnapshot & start_resources, EvaluationSummary & summary)
{
  // ResourceSnapshot 中的 CPU 时间来自 getrusage，是进程启动以来的累计值。
  // 为了让每一帧 CSV 能直接比较“本次全局重定位消耗了多少 CPU”，这里额外记录评估前后的差值。
  summary.resources = sample_resource_snapshot();
  summary.delta_user_cpu_ms = std::max(0.0, summary.resources.user_cpu_ms - start_resources.user_cpu_ms);
  summary.delta_system_cpu_ms = std::max(0.0, summary.resources.system_cpu_ms - start_resources.system_cpu_ms);
}

BagData read_bag_data(const RuntimeConfig & config, const std::string & bag_path)
{
  BagData data;
  rosbag2_cpp::Reader reader;
  reader.open(bag_path);

  const std::string cloud_topic = config.input.mode == InputMode::RegisteredWorld ?
    config.input.registered_world_topic :
    config.input.body_topic;

  rosbag2_storage::StorageFilter filter;
  filter.topics = {cloud_topic, config.input.odom_topic};
  if (config.evaluation.use_bag_reference_pose && !config.evaluation.reference_pose_topic.empty()) {
    filter.topics.push_back(config.evaluation.reference_pose_topic);
  }
  reader.set_filter(filter);

  const auto sample_indices = explicit_sample_indices(config.input);
  int cloud_seen = 0;
  while (reader.has_next()) {
    const auto bag_message = reader.read_next();
    if (bag_message->topic_name == config.input.odom_topic) {
      data.odoms.push_back(deserialize_bag_message<nav_msgs::msg::Odometry>(bag_message));
      continue;
    }
    if (bag_message->topic_name == config.evaluation.reference_pose_topic) {
      auto pose_msg = deserialize_bag_message<geometry_msgs::msg::PoseWithCovarianceStamped>(bag_message);
      data.reference_poses.push_back(
        BagReferenceSample{
          stamp_to_sec(pose_msg.header.stamp),
          bag_time_to_sec(bag_message->recv_timestamp),
          std::move(pose_msg)});
      continue;
    }
    if (bag_message->topic_name != cloud_topic) {
      continue;
    }

    const bool sample_cloud = should_sample_cloud_frame(
      cloud_seen,
      static_cast<int>(data.clouds.size()),
      config.input,
      sample_indices);
    const bool scan_context_keyframe =
      config.scan_context.enable &&
      config.scan_context.database_path.empty() &&
      config.scan_context.keyframe_stride > 0 &&
      cloud_seen % std::max(1, config.scan_context.keyframe_stride) == 0;

    // 抽帧在读取阶段执行，避免把大型点云全部留在内存里。既支持旧的 skip/stride 顺序抽样，
    // 也支持 bag_sample_frame_indices 指定的随机帧序号，用于更真实地覆盖任意启动位置。
    if (sample_cloud || scan_context_keyframe)
    {
      auto cloud_msg = deserialize_bag_message<sensor_msgs::msg::PointCloud2>(bag_message);
      const double stamp_sec = stamp_to_sec(cloud_msg.header.stamp);
      const double bag_time_sec = bag_time_to_sec(bag_message->recv_timestamp);
      if (sample_cloud) {
        data.clouds.push_back(
          BagCloudSample{
            bag_path,
            cloud_seen,
            stamp_sec,
            bag_time_sec,
            cloud_msg});
      }
      if (scan_context_keyframe) {
        data.scan_context_keyframes.push_back(
          BagCloudSample{
            bag_path,
            cloud_seen,
            stamp_sec,
            bag_time_sec,
            std::move(cloud_msg)});
      }
    }
    ++cloud_seen;
  }

  return data;
}

std::optional<nav_msgs::msg::Odometry> nearest_odom(
  const std::vector<nav_msgs::msg::Odometry> & odoms,
  double stamp_sec,
  double tolerance_sec)
{
  if (odoms.empty()) {
    return std::nullopt;
  }

  const nav_msgs::msg::Odometry * best = nullptr;
  double best_dt = std::numeric_limits<double>::max();
  for (const auto & odom : odoms) {
    const double dt = std::abs(stamp_to_sec(odom.header.stamp) - stamp_sec);
    if (dt < best_dt) {
      best = &odom;
      best_dt = dt;
    }
  }
  if (!best || best_dt > tolerance_sec) {
    return std::nullopt;
  }
  return *best;
}

std::optional<BagReferenceSample> nearest_reference_pose(
  const std::vector<BagReferenceSample> & poses,
  double stamp_sec,
  double tolerance_sec)
{
  if (poses.empty()) {
    return std::nullopt;
  }

  const BagReferenceSample * best = nullptr;
  double best_dt = std::numeric_limits<double>::max();
  for (const auto & pose : poses) {
    const double dt = std::abs(pose.bag_time_sec - stamp_sec);
    if (dt < best_dt) {
      best = &pose;
      best_dt = dt;
    }
  }
  if (!best || best_dt > tolerance_sec) {
    return std::nullopt;
  }
  return *best;
}

Eigen::Matrix4d force_2d_pose_if_needed(const Eigen::Matrix4d & pose, const FrameConfig & config)
{
  if (!config.force_2d_output) {
    return pose;
  }

  const double yaw = yaw_from_matrix(pose);
  Eigen::Matrix4d constrained = Eigen::Matrix4d::Identity();
  constrained(0, 0) = std::cos(yaw);
  constrained(0, 1) = -std::sin(yaw);
  constrained(1, 0) = std::sin(yaw);
  constrained(1, 1) = std::cos(yaw);
  constrained(0, 3) = pose(0, 3);
  constrained(1, 3) = pose(1, 3);
  return constrained;
}

std::optional<std::string> reference_sanity_reject_reason(
  const RuntimeConfig & config,
  const Eigen::Matrix4d & reference_pose)
{
  // bag 中的参考位姿是评估成功率的依据，但它本身也可能因为 frame 错乱或记录异常跳到地图外。
  // 这里只做非常保守的范围检查：默认室内地图搜索范围是 ±35m，参考阈值给到 ±100m。
  // 被拒绝的参考不会参与 success_rate，但 CSV 仍保留 reference_* 和 rejected reason 供排查。
  if (!config.evaluation.enable_reference_sanity_check) {
    return std::nullopt;
  }

  const double x = reference_pose(0, 3);
  const double y = reference_pose(1, 3);
  const double z = reference_pose(2, 3);
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    return std::string("reference_pose_has_nan_or_inf");
  }

  if (std::abs(x) > config.evaluation.reference_max_abs_xy_m ||
    std::abs(y) > config.evaluation.reference_max_abs_xy_m)
  {
    std::ostringstream reason;
    reason << "reference_xy_out_of_range(abs_limit=" << config.evaluation.reference_max_abs_xy_m << ")";
    return reason.str();
  }

  if (std::abs(z) > config.evaluation.reference_max_abs_z_m) {
    std::ostringstream reason;
    reason << "reference_z_out_of_range(abs_limit=" << config.evaluation.reference_max_abs_z_m << ")";
    return reason.str();
  }

  return std::nullopt;
}

void evaluate_against_reference(
  const RuntimeConfig & config,
  const Eigen::Matrix4d & reference_pose,
  const std::string & reference_source,
  EvaluationSummary & summary)
{
  // 误差只按室内导航真正关心的平面 x/y/yaw 统计。z/roll/pitch 保留在位姿里，
  // 但回归成功率不让它们主导判断，避免地面/传感器高度差干扰全局重定位结论。
  summary.reference_source = reference_source;
  summary.reference_pose = reference_pose;
  if (const auto reject_reason = reference_sanity_reject_reason(config, reference_pose)) {
    summary.has_reference_pose = false;
    summary.reference_rejected_reason = *reject_reason;
    summary.translation_error_m = -1.0;
    summary.yaw_error_deg = -1.0;
    summary.success = false;
    return;
  }

  summary.has_reference_pose = true;
  const Eigen::Vector2d delta_xy(
    summary.final_pose(0, 3) - reference_pose(0, 3),
    summary.final_pose(1, 3) - reference_pose(1, 3));
  summary.translation_error_m = delta_xy.norm();
  summary.yaw_error_deg =
    std::abs(normalize_angle(yaw_from_matrix(summary.final_pose) - yaw_from_matrix(reference_pose))) * 180.0 / M_PI;
  summary.success =
    summary.translation_error_m <= config.evaluation.success_translation_thresh &&
    summary.yaw_error_deg <= config.evaluation.success_yaw_thresh_deg;
}

CloudPtr registered_world_to_base_scan(
  const sensor_msgs::msg::PointCloud2 & cloud_msg,
  const nav_msgs::msg::Odometry & odom,
  const RuntimeConfig & config)
{
  CloudPtr world_cloud = pointcloud2_to_xyz_cloud(cloud_msg);
  const Eigen::Matrix4d t_world_body = pose_to_matrix(odom.pose.pose);
  CloudPtr raw_body_cloud = transform_cloud(world_cloud, t_world_body.inverse());
  return convert_raw_body_to_base_cloud(raw_body_cloud, config.frames);
}

CloudPtr body_msg_to_base_scan(
  const sensor_msgs::msg::PointCloud2 & cloud_msg,
  const RuntimeConfig & config)
{
  CloudPtr raw_body_cloud = pointcloud2_to_xyz_cloud(cloud_msg);
  return convert_raw_body_to_base_cloud(raw_body_cloud, config.frames);
}

std::vector<ScanContextEntry> build_scan_context_database(
  const RuntimeConfig & config,
  const BagData & data)
{
  if (!config.scan_context.database_path.empty()) {
    return load_scan_context_database(config.scan_context.database_path);
  }

  // 离线验证阶段用 bag 中抽样 keyframe 模拟“建图/巡航阶段预生成的描述子数据库”。
  // keyframe 的 map 位姿来自 /robot_realpose；真正上线后应替换为建图轨迹或离线生成的 map keyframe 位姿。
  std::vector<ScanContextEntry> database;
  if (!config.scan_context.enable || data.scan_context_keyframes.empty()) {
    return database;
  }
  database.reserve(data.scan_context_keyframes.size());

  for (const auto & sample : data.scan_context_keyframes) {
    std::optional<nav_msgs::msg::Odometry> odom;
    CloudPtr base_scan;
    if (config.input.mode == InputMode::RegisteredWorld) {
      odom = nearest_odom(data.odoms, sample.stamp_sec, config.input.odom_time_tolerance_sec);
      if (!odom) {
        continue;
      }
      base_scan = registered_world_to_base_scan(sample.cloud_msg, *odom, config);
    } else {
      base_scan = body_msg_to_base_scan(sample.cloud_msg, config);
    }

    const auto reference = nearest_reference_pose(
      data.reference_poses,
      sample.bag_time_sec,
      config.evaluation.reference_time_tolerance_sec);
    if (!reference) {
      continue;
    }

    CloudPtr scan_cloud = preprocess_scan_cloud(base_scan, config.preprocess);
    ScanContextEntry entry;
    entry.cloud_index = sample.frame_index;
    entry.map_to_base = force_2d_pose_if_needed(pose_to_matrix(reference->pose_msg.pose.pose), config.frames);
    entry.descriptor = compute_scan_context_descriptor(scan_cloud, config.scan_context);
    database.push_back(std::move(entry));
  }

  if (!config.scan_context.save_database_path.empty()) {
    save_scan_context_database(config.scan_context.save_database_path, database);
  }

  return database;
}

double pose_xy_distance(const Eigen::Matrix4d & lhs, const Eigen::Matrix4d & rhs)
{
  return std::hypot(lhs(0, 3) - rhs(0, 3), lhs(1, 3) - rhs(1, 3));
}

double pose_yaw_distance_deg(const Eigen::Matrix4d & lhs, const Eigen::Matrix4d & rhs)
{
  return std::abs(normalize_angle(yaw_from_matrix(lhs) - yaw_from_matrix(rhs))) * 180.0 / M_PI;
}

bool is_duplicate_candidate(
  const std::vector<BbsCandidate> & candidates,
  const Eigen::Matrix4d & pose,
  const ScanContextConfig & config)
{
  // Scan Context 可能召回与 BBS 非常接近的 seed；去重可以避免 GICP 重复算同一个盆地。
  for (const auto & candidate : candidates) {
    if (pose_xy_distance(candidate.pose, pose) <= config.duplicate_xy_gate_m &&
      pose_yaw_distance_deg(candidate.pose, pose) <= config.duplicate_yaw_gate_deg)
    {
      return true;
    }
  }
  return false;
}

bool is_duplicate_bbs2d_candidate(
  const std::vector<BbsCandidate> & candidates,
  const Eigen::Matrix4d & pose,
  const Bbs2dConfig & config)
{
  // 2.5D 深搜通常会在同一位置附近给出多个相似 yaw 或相邻格点。
  // 融合前用较小的去重阈值剔除与 3D-BBS/Scan Context 已有 seed 几乎相同的候选，
  // 让精配准预算尽量花在不同区域的多假设上。
  for (const auto & candidate : candidates) {
    if (pose_xy_distance(candidate.pose, pose) <= config.duplicate_xy_gate_m &&
      pose_yaw_distance_deg(candidate.pose, pose) <= config.duplicate_yaw_gate_deg)
    {
      return true;
    }
  }
  return false;
}

void fuse_bbs2d_candidates(
  const RuntimeConfig & config,
  const std::vector<Eigen::Vector3d> & scan_points,
  const Bbs2dSearch * bbs2d,
  BbsResult & result)
{
  if (!config.bbs2d.enable || !bbs2d || scan_points.empty()) {
    return;
  }

  const BbsResult deep_result = bbs2d->localize(scan_points, config.bbs);
  if (!deep_result.localized || deep_result.candidates.empty()) {
    return;
  }

  std::vector<BbsCandidate> fused;
  fused.reserve(result.candidates.size() + deep_result.candidates.size());
  const int prefix = std::min<int>(
    std::max(0, config.bbs2d.fuse_prefix_3d_candidates),
    static_cast<int>(result.candidates.size()));
  for (int i = 0; i < prefix; ++i) {
    fused.push_back(result.candidates[static_cast<std::size_t>(i)]);
  }

  for (const auto & candidate : deep_result.candidates) {
    if (!is_duplicate_bbs2d_candidate(fused, candidate.pose, config.bbs2d)) {
      fused.push_back(candidate);
    }
  }
  for (int i = prefix; i < static_cast<int>(result.candidates.size()); ++i) {
    if (!is_duplicate_bbs2d_candidate(fused, result.candidates[static_cast<std::size_t>(i)].pose, config.bbs2d)) {
      fused.push_back(result.candidates[static_cast<std::size_t>(i)]);
    }
  }

  if (!fused.empty()) {
    result.candidates = std::move(fused);
    result.localized = true;
    result.search_ms += deep_result.search_ms;
  }
}

void fuse_scan_context_candidates(
  const RuntimeConfig & config,
  const CloudPtr & scan_cloud,
  int current_cloud_index,
  const std::vector<ScanContextEntry> * database,
  BbsResult & result)
{
  if (!config.scan_context.enable || !database || database->empty() || !scan_cloud || scan_cloud->empty()) {
    return;
  }

  const ScanContextDescriptor query =
    compute_scan_context_descriptor(scan_cloud, config.scan_context);
  const auto matches =
    query_scan_context_database(query, *database, current_cloud_index, config.scan_context);
  if (matches.empty()) {
    return;
  }

  std::vector<BbsCandidate> fused;
  fused.reserve(
    result.candidates.size() +
    matches.size() * std::max<std::size_t>(1U, config.scan_context.yaw_offsets_deg.size()));

  // 先保留一小段 BBS 最高分候选，避免描述子召回质量差时完全压掉原有全局搜索结果。
  const int refine_budget = std::max(1, config.refine.max_refine_candidates);
  const int bbs_prefix = std::min<int>(
    static_cast<int>(result.candidates.size()),
    config.scan_context.enable ? std::max(1, refine_budget / 3) : refine_budget);
  for (int i = 0; i < bbs_prefix; ++i) {
    fused.push_back(result.candidates[static_cast<std::size_t>(i)]);
  }

  for (const auto & match : matches) {
    for (const double yaw_offset_deg : config.scan_context.yaw_offsets_deg) {
      const Eigen::Matrix4d seed_pose =
        apply_scan_context_yaw_shift(match.map_to_base, match.shift, yaw_offset_deg, config.scan_context);
      if (is_duplicate_candidate(fused, seed_pose, config.scan_context)) {
        continue;
      }
      BbsCandidate candidate;
      candidate.pose = seed_pose;
      candidate.score = -match.rank;
      candidate.score_ratio = std::max(0.0, 1.0 - match.distance);
      fused.push_back(candidate);
    }
  }

  for (std::size_t i = static_cast<std::size_t>(bbs_prefix); i < result.candidates.size(); ++i) {
    if (!is_duplicate_candidate(fused, result.candidates[i].pose, config.scan_context)) {
      fused.push_back(result.candidates[i]);
    }
  }

  if (!fused.empty()) {
    result.candidates = std::move(fused);
    result.localized = true;
  }
}

EvaluationSummary evaluate_scan_with_built_map(
  const RuntimeConfig & config,
  const CloudPtr & map_cloud,
  SimpleBbs3d & bbs,
  const Bbs2dSearch * bbs2d,
  const CloudPtr & base_scan,
  const std::string & message_prefix,
  double build_index_ms,
  const std::string & map_path,
  const std::string & bag_path,
  int bag_frame_index,
  double stamp_sec,
  const std::optional<Eigen::Matrix4d> & reference_pose,
  const std::string & reference_source,
  const std::vector<ScanContextEntry> * scan_context_database = nullptr,
  CloudPtr * preprocessed_scan_out = nullptr)
{
  EvaluationSummary summary;
  const auto total_start = std::chrono::steady_clock::now();
  const auto resource_start = sample_resource_snapshot();
  summary.map_path = map_path;
  summary.bag_path = bag_path;
  summary.bag_frame_index = bag_frame_index;
  summary.stamp_sec = stamp_sec;
  summary.refine_method = config.refine.method;
  summary.map_points = static_cast<int>(map_cloud->size());

  CloudPtr scan_cloud = preprocess_scan_cloud(base_scan, config.preprocess);
  if (preprocessed_scan_out) {
    *preprocessed_scan_out = scan_cloud;
  }
  summary.scan_points = static_cast<int>(scan_cloud->size());
  const auto scan_points = cloud_to_eigen_points(scan_cloud);

  summary.bbs_result = bbs.localize(scan_points);
  summary.bbs_result.build_index_ms = build_index_ms;
  fuse_bbs2d_candidates(config, scan_points, bbs2d, summary.bbs_result);
  if (!summary.bbs_result.localized) {
    summary.message = message_prefix + (summary.bbs_result.timed_out ? " 3D-BBS timed out" : " 3D-BBS did not find candidate");
    summary.total_ms = elapsed_ms(total_start, std::chrono::steady_clock::now());
    finalize_resource_usage(resource_start, summary);
    append_evaluation_csvs(config, summary);
    return summary;
  }

  RefineOutput refined;
  if (config.scan_context.enable && scan_context_database && !scan_context_database->empty()) {
    // 两阶段策略：
    //   1. 先只精配准 BBS 候选。如果第一阶段已经达到极高置信，就不让描述子候选覆盖它。
    //   2. 只有 BBS/GICP 不够确定时，才把 Scan Context seed 融入候选池做二阶段深搜。
    // 这样能避免“描述子召回了一个低 fitness 但错误的重复结构”把原本很准的 BBS 结果顶掉。
    RefineOutput primary_refined =
      refine_candidates(map_cloud, scan_cloud, summary.bbs_result.candidates, config.refine);
    if (primary_refined.converged &&
      primary_refined.fitness_score <= config.temporal.single_frame_high_confidence_max_fitness)
    {
      refined = primary_refined;
    } else {
      const double primary_refine_ms = primary_refined.elapsed_ms;
      fuse_scan_context_candidates(
        config,
        scan_cloud,
        bag_frame_index,
        scan_context_database,
        summary.bbs_result);
      refined = refine_candidates(map_cloud, scan_cloud, summary.bbs_result.candidates, config.refine);
      refined.elapsed_ms += primary_refine_ms;
    }
  } else {
    refined = refine_candidates(map_cloud, scan_cloud, summary.bbs_result.candidates, config.refine);
  }
  summary.final_pose = force_2d_pose_if_needed(refined.pose, config.frames);
  summary.refined_candidate_rank = refined.candidate_rank;
  summary.refine_fitness_score = refined.fitness_score;
  summary.refine_ms = refined.elapsed_ms;

  if (reference_pose) {
    evaluate_against_reference(config, *reference_pose, reference_source, summary);
  } else if (config.evaluation.reference_pose) {
    evaluate_against_reference(config, *config.evaluation.reference_pose, "config_reference_pose", summary);
  }

  if (config.evaluation.save_aligned_cloud) {
    static int save_index = 0;
    CloudPtr aligned = transform_cloud(scan_cloud, summary.final_pose);
    const std::filesystem::path aligned_path =
      std::filesystem::path(config.input.output_dir) /
      ("aligned_scan_" + to_string(config.refine.method) + "_" + std::to_string(save_index++) + ".pcd");
    save_pcd_xyz(aligned_path.string(), *aligned);
  }

  summary.ok = true;
  summary.message = message_prefix + " ok";
  summary.total_ms = elapsed_ms(total_start, std::chrono::steady_clock::now());
  finalize_resource_usage(resource_start, summary);
  append_evaluation_csvs(config, summary);
  return summary;
}

double rad_to_deg(double radians)
{
  return radians * 180.0 / M_PI;
}

std::vector<std::string> effective_map_paths(const RuntimeConfig & config)
{
  // map_candidates 用于批量对比多张 PCD；如果没有配置，就退回单张 map_path。
  std::vector<std::string> maps = config.input.map_candidates;
  if (maps.empty() && !config.input.map_path.empty()) {
    maps.push_back(config.input.map_path);
  }
  return maps;
}

void append_metrics_csv(
  const RuntimeConfig & config,
  const EvaluationSummary & summary,
  const EvaluationScenario & scenario)
{
  std::filesystem::create_directories(config.input.output_dir);
  const std::filesystem::path csv_path =
    std::filesystem::path(config.input.output_dir) / config.evaluation.metrics_csv_name;
  const bool write_header = !std::filesystem::exists(csv_path);

  std::ofstream out(csv_path, std::ios::app);
  if (!out) {
    throw std::runtime_error("failed to open metrics csv: " + csv_path.string());
  }

  if (write_header) {
    out
      << "scenario_name,map_path,bag_path,bag_frame_index,stamp_sec,refine_method,refined_candidate_rank,refine_fitness_score,"
      << "reference_source,has_reference,reference_rejected_reason,"
      << "simulated_prior_dx_m,simulated_prior_dy_m,simulated_prior_dz_m,"
      << "simulated_prior_dyaw_deg,input_mode,localized,success,score,score_ratio,"
      << "final_x_m,final_y_m,final_z_m,final_yaw_deg,"
      << "reference_x_m,reference_y_m,reference_z_m,reference_yaw_deg,"
      << "translation_error_m,yaw_error_deg,"
      << "map_points,scan_points,candidate_count,build_index_ms,search_ms,refine_ms,total_ms,"
      << "user_cpu_ms,system_cpu_ms,delta_user_cpu_ms,delta_system_cpu_ms,"
      << "rss_mb,peak_rss_mb,virtual_mem_mb,thread_count,message\n";
  }

  const int score = summary.bbs_result.candidates.empty() ? 0 : summary.bbs_result.candidates.front().score;
  const double score_ratio =
    summary.bbs_result.candidates.empty() ? 0.0 : summary.bbs_result.candidates.front().score_ratio;
  const bool has_reference_sample =
    !summary.reference_source.empty() || !summary.reference_rejected_reason.empty() || summary.has_reference_pose;

  out << scenario.name << ","
      << '"' << summary.map_path << '"' << ","
      << '"' << summary.bag_path << '"' << ","
      << summary.bag_frame_index << ","
      << std::fixed << std::setprecision(6) << summary.stamp_sec << ","
      << to_string(summary.refine_method) << ","
      << summary.refined_candidate_rank << ","
      << summary.refine_fitness_score << ","
      << '"' << summary.reference_source << '"' << ","
      << (summary.has_reference_pose ? 1 : 0) << ","
      << '"' << summary.reference_rejected_reason << '"' << ","
      << std::fixed << std::setprecision(6)
      << scenario.prior_xyz_offset.x() << ","
      << scenario.prior_xyz_offset.y() << ","
      << scenario.prior_xyz_offset.z() << ","
      << rad_to_deg(scenario.prior_rpy_offset.z()) << ","
      << to_string(config.input.mode) << ","
      << (summary.bbs_result.localized ? 1 : 0) << ","
      << (summary.success ? 1 : 0) << ","
      << score << ","
      << score_ratio << ","
      << summary.final_pose(0, 3) << ","
      << summary.final_pose(1, 3) << ","
      << summary.final_pose(2, 3) << ","
      << rad_to_deg(yaw_from_matrix(summary.final_pose)) << ","
      << (has_reference_sample ? summary.reference_pose(0, 3) : -1.0) << ","
      << (has_reference_sample ? summary.reference_pose(1, 3) : -1.0) << ","
      << (has_reference_sample ? summary.reference_pose(2, 3) : -1.0) << ","
      << (has_reference_sample ? rad_to_deg(yaw_from_matrix(summary.reference_pose)) : -1.0) << ","
      << summary.translation_error_m << ","
      << summary.yaw_error_deg << ","
      << summary.map_points << ","
      << summary.scan_points << ","
      << summary.bbs_result.candidates.size() << ","
      << summary.bbs_result.build_index_ms << ","
      << summary.bbs_result.search_ms << ","
      << summary.refine_ms << ","
      << summary.total_ms << ","
      << summary.resources.user_cpu_ms << ","
      << summary.resources.system_cpu_ms << ","
      << summary.delta_user_cpu_ms << ","
      << summary.delta_system_cpu_ms << ","
      << summary.resources.rss_mb << ","
      << summary.resources.peak_rss_mb << ","
      << summary.resources.virtual_mem_mb << ","
      << summary.resources.thread_count << ","
      << '"' << summary.message << '"' << "\n";
}

void append_candidates_csv(
  const RuntimeConfig & config,
  const EvaluationSummary & summary,
  const EvaluationScenario & scenario)
{
  // 候选 CSV 专门记录 3D-BBS 粗搜索输出。它不只看最终 GICP 位姿，而是保留 top-K 排名，
  // 方便在重复走廊、对称区域里判断算法是否“候选集中有正确解，但排序/精配准还需优化”。
  std::filesystem::create_directories(config.input.output_dir);
  const std::filesystem::path csv_path =
    std::filesystem::path(config.input.output_dir) / config.evaluation.candidates_csv_name;
  const bool write_header = !std::filesystem::exists(csv_path);

  std::ofstream out(csv_path, std::ios::app);
  if (!out) {
    throw std::runtime_error("failed to open candidates csv: " + csv_path.string());
  }

  if (write_header) {
    out
      << "scenario_name,map_path,bag_path,bag_frame_index,stamp_sec,refine_method,input_mode,rank,score,score_ratio,"
      << "candidate_x_m,candidate_y_m,candidate_z_m,candidate_yaw_deg,localized,message\n";
  }

  for (std::size_t i = 0; i < summary.bbs_result.candidates.size(); ++i) {
    const auto & candidate = summary.bbs_result.candidates[i];
    out << scenario.name << ","
        << '"' << summary.map_path << '"' << ","
        << '"' << summary.bag_path << '"' << ","
        << summary.bag_frame_index << ","
        << std::fixed << std::setprecision(6) << summary.stamp_sec << ","
        << to_string(summary.refine_method) << ","
        << to_string(config.input.mode) << ","
        << i + 1 << ","
        << candidate.score << ","
        << std::fixed << std::setprecision(6) << candidate.score_ratio << ","
        << candidate.pose(0, 3) << ","
        << candidate.pose(1, 3) << ","
        << candidate.pose(2, 3) << ","
        << rad_to_deg(yaw_from_matrix(candidate.pose)) << ","
        << (summary.bbs_result.localized ? 1 : 0) << ","
        << '"' << summary.message << '"' << "\n";
  }
}

void append_evaluation_csvs(const RuntimeConfig & config, const EvaluationSummary & summary)
{
  // 每个 scan 的搜索只执行一次；场景只代表“假设外部定位先验错成什么样”。
  // 因为 3D-BBS 不吃初始位姿，同一结果可以对应多个大跳/任意启动场景，用于后处理统计。
  for (const auto & scenario : config.scenarios) {
    append_metrics_csv(config, summary, scenario);
    append_candidates_csv(config, summary, scenario);
  }
}

TemporalFrameInput to_temporal_frame_input(const EvaluationSummary & summary)
{
  TemporalFrameInput frame;
  frame.map_path = summary.map_path;
  frame.bag_path = summary.bag_path;
  frame.stamp_sec = summary.stamp_sec;
  frame.refine_method = summary.refine_method;
  frame.localized = summary.bbs_result.localized;
  frame.success = summary.success;
  frame.translation_error_m = summary.translation_error_m;
  frame.yaw_error_deg = summary.yaw_error_deg;
  frame.has_reference_pose = summary.has_reference_pose;
  frame.reference_pose = summary.reference_pose;
  frame.selected_candidate_rank = summary.refined_candidate_rank;
  frame.has_odom_pose = summary.has_odom_pose;
  frame.odom_to_base = summary.odom_to_base_pose;

  frame.candidates.reserve(summary.bbs_result.candidates.size());
  for (std::size_t i = 0; i < summary.bbs_result.candidates.size(); ++i) {
    const auto & candidate = summary.bbs_result.candidates[i];
    TemporalCandidateInput temporal_candidate;
    temporal_candidate.rank = static_cast<int>(i) + 1;
    temporal_candidate.score = candidate.score;
    temporal_candidate.score_ratio = candidate.score_ratio;
    temporal_candidate.map_to_base = candidate.pose;
    frame.candidates.push_back(temporal_candidate);
  }
  return frame;
}

void append_temporal_consistency_csv(
  const RuntimeConfig & config,
  const std::vector<EvaluationSummary> & summaries)
{
  if (!config.temporal.enable || summaries.empty()) {
    return;
  }

  std::vector<TemporalFrameInput> temporal_frames;
  temporal_frames.reserve(summaries.size());
  for (const auto & summary : summaries) {
    TemporalFrameInput frame = to_temporal_frame_input(summary);
    frame.input_mode = config.input.mode;
    temporal_frames.push_back(frame);
  }

  const auto results = analyze_temporal_consistency(temporal_frames, config.temporal);
  if (results.empty()) {
    return;
  }

  std::filesystem::create_directories(config.input.output_dir);
  const std::filesystem::path csv_path =
    std::filesystem::path(config.input.output_dir) / config.temporal.csv_name;
  const bool write_header = !std::filesystem::exists(csv_path);

  std::ofstream out(csv_path, std::ios::app);
  if (!out) {
    throw std::runtime_error("failed to open temporal consistency csv: " + csv_path.string());
  }

  if (write_header) {
    out
      << "scenario_name,map_path,bag_path,stamp_sec,refine_method,input_mode,"
      << "success,translation_error_m,yaw_error_deg,"
      << "selected_rank,selected_support_frames,selected_support_count,selected_median_rank,"
      << "best_seed_rank,best_support_frames,best_support_count,best_median_rank,"
      << "selected_map_odom_x,selected_map_odom_y,selected_map_odom_yaw_deg,"
      << "selected_candidate_translation_error_m,selected_candidate_yaw_error_deg,"
      << "best_seed_translation_error_m,best_seed_yaw_error_deg\n";
  }

  for (const auto & result : results) {
    out << config.temporal.scenario_name << ","
        << '"' << result.map_path << '"' << ","
        << '"' << result.bag_path << '"' << ","
        << std::fixed << std::setprecision(6) << result.stamp_sec << ","
        << to_string(result.refine_method) << ","
        << to_string(result.input_mode) << ","
        << (result.success ? 1 : 0) << ","
        << result.translation_error_m << ","
        << result.yaw_error_deg << ","
        << result.selected_rank << ","
        << result.selected_support_frames << ","
        << result.selected_support_count << ","
        << result.selected_median_rank << ","
        << result.best_seed_rank << ","
        << result.best_support_frames << ","
        << result.best_support_count << ","
        << result.best_median_rank << ","
        << result.selected_map_odom_x << ","
        << result.selected_map_odom_y << ","
        << result.selected_map_odom_yaw_deg << ","
        << result.selected_candidate_translation_error_m << ","
        << result.selected_candidate_yaw_error_deg << ","
        << result.best_seed_translation_error_m << ","
        << result.best_seed_yaw_error_deg << "\n";
  }
}

const TemporalConsistencyResult * find_temporal_result_for_record(
  const std::vector<TemporalConsistencyResult> & results,
  const EvaluationRecord & record)
{
  for (const auto & result : results) {
    if (result.map_path == record.summary.map_path &&
      result.bag_path == record.summary.bag_path &&
      result.refine_method == record.summary.refine_method &&
      result.input_mode == record.input_mode &&
      std::abs(result.stamp_sec - record.summary.stamp_sec) < 1e-6)
    {
      return &result;
    }
  }
  return nullptr;
}

std::optional<BbsCandidate> candidate_by_rank(const EvaluationSummary & summary, int rank)
{
  if (rank <= 0 || rank > static_cast<int>(summary.bbs_result.candidates.size())) {
    return std::nullopt;
  }
  return summary.bbs_result.candidates[static_cast<std::size_t>(rank - 1)];
}

std::pair<double, double> pose_error_against_reference(
  const Eigen::Matrix4d & pose,
  const Eigen::Matrix4d & reference)
{
  const double dx = pose(0, 3) - reference(0, 3);
  const double dy = pose(1, 3) - reference(1, 3);
  const double trans = std::hypot(dx, dy);
  const double yaw =
    std::abs(normalize_angle(yaw_from_matrix(pose) - yaw_from_matrix(reference))) * 180.0 / M_PI;
  return {trans, yaw};
}

void append_temporal_decision_csv(
  const RuntimeConfig & config,
  const std::vector<EvaluationRecord> & records)
{
  if (!config.temporal.enable || records.empty()) {
    return;
  }

  std::vector<TemporalFrameInput> temporal_frames;
  temporal_frames.reserve(records.size());
  for (const auto & record : records) {
    TemporalFrameInput frame = to_temporal_frame_input(record.summary);
    frame.input_mode = record.input_mode;
    temporal_frames.push_back(frame);
  }

  const auto temporal_results = analyze_temporal_consistency(temporal_frames, config.temporal);
  if (temporal_results.empty()) {
    return;
  }

  std::filesystem::create_directories(config.input.output_dir);
  const std::filesystem::path csv_path =
    std::filesystem::path(config.input.output_dir) / config.temporal.decision_csv_name;
  const bool write_header = !std::filesystem::exists(csv_path);

  std::ofstream out(csv_path, std::ios::app);
  if (!out) {
    throw std::runtime_error("failed to open temporal decision csv: " + csv_path.string());
  }

  if (write_header) {
    out
      << "scenario_name,map_path,bag_path,stamp_sec,refine_method,input_mode,"
      << "decision,decision_reason,min_support_frames,best_seed_rank,best_support_frames,"
      << "selected_rank,selected_support_frames,"
      << "best_seed_translation_error_m,best_seed_yaw_error_deg,"
      << "refined_converged,refined_fitness,refined_ms,refined_x_m,refined_y_m,refined_z_m,refined_yaw_deg,"
      << "has_reference,refined_translation_error_m,refined_yaw_error_deg,refined_success\n";
  }

  for (const auto & record : records) {
    const auto * temporal = find_temporal_result_for_record(temporal_results, record);
    if (!temporal) {
      continue;
    }

    const int min_support = std::max(1, config.temporal.online_min_support_frames);
    const bool support_ok = temporal->selected_support_frames >= min_support;
    const auto selected_candidate = candidate_by_rank(record.summary, temporal->selected_rank);

    std::string decision = "reject";
    std::string reason = "selected_support_below_threshold";
    RefineOutput refined;
    Eigen::Matrix4d refined_pose = Eigen::Matrix4d::Identity();
    bool refined_success = false;
    double refined_trans_error = -1.0;
    double refined_yaw_error = -1.0;

    if (support_ok && selected_candidate && record.map_cloud && record.scan_cloud) {
      decision = "accept";
      reason = "selected_support_ok";
      refined = refine_single_candidate(
        record.map_cloud,
        record.scan_cloud,
        *selected_candidate,
        temporal->selected_rank,
        record.refine);
      refined_pose = force_2d_pose_if_needed(
        refined.converged ? refined.pose : selected_candidate->pose,
        record.frames);
      if (record.summary.has_reference_pose) {
        const auto error = pose_error_against_reference(refined_pose, record.summary.reference_pose);
        refined_trans_error = error.first;
        refined_yaw_error = error.second;
        refined_success =
          refined_trans_error <= record.evaluation.success_translation_thresh &&
          refined_yaw_error <= record.evaluation.success_yaw_thresh_deg;
      }
      if (!refined.converged) {
        decision = "reject";
        reason = "refine_not_converged";
      } else if (refined.fitness_score > config.temporal.online_max_refine_fitness) {
        decision = "reject";
        reason = "refine_fitness_above_threshold";
      }
    } else if (
      !support_ok &&
      config.temporal.single_frame_high_confidence_fallback_enable &&
      record.summary.bbs_result.localized &&
      record.summary.refined_candidate_rank > 0 &&
      record.summary.refine_fitness_score >= 0.0 &&
      record.summary.refine_fitness_score <= config.temporal.single_frame_high_confidence_max_fitness)
    {
      // 这个分支只接受“单帧已经非常贴图”的结果，用来补救 support=1 的过度保守拒绝。
      // 它不放宽重复结构里的普通单帧结果：fitness 稍高时仍会继续交给 trajectory/active-view。
      decision = "accept";
      reason = "single_frame_high_confidence";
      refined.converged = true;
      refined.fitness_score = record.summary.refine_fitness_score;
      refined.elapsed_ms = record.summary.refine_ms;
      refined.candidate_rank = record.summary.refined_candidate_rank;
      refined.pose = record.summary.final_pose;
      refined_pose = force_2d_pose_if_needed(record.summary.final_pose, record.frames);
      if (record.summary.has_reference_pose) {
        const auto error = pose_error_against_reference(refined_pose, record.summary.reference_pose);
        refined_trans_error = error.first;
        refined_yaw_error = error.second;
        refined_success =
          refined_trans_error <= record.evaluation.success_translation_thresh &&
          refined_yaw_error <= record.evaluation.success_yaw_thresh_deg;
      }
    } else if (support_ok && !selected_candidate) {
      reason = "selected_rank_not_found";
    }

    out << config.temporal.scenario_name << ","
        << '"' << record.summary.map_path << '"' << ","
        << '"' << record.summary.bag_path << '"' << ","
        << std::fixed << std::setprecision(6) << record.summary.stamp_sec << ","
        << to_string(record.summary.refine_method) << ","
        << to_string(record.input_mode) << ","
        << decision << ","
        << reason << ","
        << min_support << ","
        << temporal->best_seed_rank << ","
        << temporal->best_support_frames << ","
        << temporal->selected_rank << ","
        << temporal->selected_support_frames << ","
        << temporal->best_seed_translation_error_m << ","
        << temporal->best_seed_yaw_error_deg << ","
        << (refined.converged ? 1 : 0) << ","
        << refined.fitness_score << ","
        << refined.elapsed_ms << ","
        << refined_pose(0, 3) << ","
        << refined_pose(1, 3) << ","
        << refined_pose(2, 3) << ","
        << rad_to_deg(yaw_from_matrix(refined_pose)) << ","
        << (record.summary.has_reference_pose ? 1 : 0) << ","
        << refined_trans_error << ","
        << refined_yaw_error << ","
        << (refined_success ? 1 : 0) << "\n";
  }
}

using RecordGroupKey = std::tuple<std::string, std::string, RefineMethod, InputMode>;

RecordGroupKey record_group_key(const EvaluationRecord & record)
{
  return RecordGroupKey{
    record.summary.map_path,
    record.summary.bag_path,
    record.summary.refine_method,
    record.input_mode};
}

void append_trajectory_likelihood_csv(
  const RuntimeConfig & config,
  const std::vector<EvaluationRecord> & records)
{
  if (!config.temporal.enable || !config.temporal.enable_trajectory_likelihood || records.empty()) {
    return;
  }

  std::filesystem::create_directories(config.input.output_dir);
  const std::filesystem::path csv_path =
    std::filesystem::path(config.input.output_dir) / config.temporal.trajectory_likelihood_csv_name;
  const bool write_header = !std::filesystem::exists(csv_path);

  std::ofstream out(csv_path, std::ios::app);
  if (!out) {
    throw std::runtime_error("failed to open trajectory likelihood csv: " + csv_path.string());
  }

  if (write_header) {
    out
      << "scenario_name,map_path,bag_path,stamp_sec,refine_method,input_mode,"
      << "candidate_rank,selected_by_single_frame,selected_by_trajectory,trajectory_decision,"
      << "support_frames,window_frames,average_overlap,best_average_overlap,second_average_overlap,margin,"
      << "single_agreement_xy_m,single_agreement_yaw_deg,single_agreement_fallback,"
      << "candidate_score,candidate_score_ratio,candidate_x_m,candidate_y_m,candidate_z_m,candidate_yaw_deg,"
      << "candidate_translation_error_m,candidate_yaw_error_deg,"
      << "selected_by_refine,refined_converged,refined_fitness,refined_fitness_margin,refined_ms,"
      << "refined_x_m,refined_y_m,refined_z_m,refined_yaw_deg,"
      << "refined_translation_error_m,refined_yaw_error_deg,"
      << "voxel_size,neighbor_radius,min_overlap_ratio,min_average_overlap,min_margin,message\n";
  }

  std::set<int> target_center_indices;
  for (const int index : config.temporal.trajectory_center_frame_indices) {
    if (index >= 0) {
      target_center_indices.insert(index);
    }
  }

  std::map<RecordGroupKey, std::vector<std::size_t>> grouped_indices;
  for (std::size_t i = 0; i < records.size(); ++i) {
    if (!records[i].summary.bbs_result.localized || !records[i].summary.has_odom_pose ||
      !records[i].scan_cloud || !records[i].map_cloud)
    {
      continue;
    }
    grouped_indices[record_group_key(records[i])].push_back(i);
  }

  for (auto & [key, indices] : grouped_indices) {
    (void)key;
    std::sort(
      indices.begin(),
      indices.end(),
      [&records](std::size_t lhs, std::size_t rhs) {
        return records[lhs].summary.stamp_sec < records[rhs].summary.stamp_sec;
      });
    if (indices.empty()) {
      continue;
    }

    const auto & first_record = records[indices.front()];
    const double voxel_size = std::max(0.05, config.temporal.trajectory_voxel_size);
    const int neighbor_radius = std::max(0, config.temporal.trajectory_neighbor_radius);
    const OccupancySet occupied = build_occupancy_set(first_record.map_cloud, voxel_size);
    if (occupied.empty()) {
      continue;
    }

    for (int center_pos = 0; center_pos < static_cast<int>(indices.size()); ++center_pos) {
      const auto & center = records[indices[static_cast<std::size_t>(center_pos)]];
      if (!target_center_indices.empty() &&
        target_center_indices.count(center.summary.bag_frame_index) == 0)
      {
        continue;
      }
      const int candidate_count = std::min<int>(
        std::max(1, config.temporal.trajectory_max_candidates),
        static_cast<int>(center.summary.bbs_result.candidates.size()));
      if (candidate_count <= 0) {
        continue;
      }

      const int begin = std::max(0, center_pos - std::max(0, config.temporal.window_before));
      const int end = std::min<int>(
        static_cast<int>(indices.size()),
        center_pos + std::max(0, config.temporal.window_after) + 1);
      const int window_frames = std::max(0, end - begin);

      struct CandidateLikelihood
      {
        int source_frame_offset{0};
        double source_stamp_sec{0.0};
        int source_candidate_rank{0};
        int seed_rank{0};
        int score{0};
        double score_ratio{0.0};
        Eigen::Matrix4d current_map_to_base{Eigen::Matrix4d::Identity()};
        double average_overlap{0.0};
        int support_frames{0};
      };
      std::vector<CandidateLikelihood> likelihoods;
      likelihoods.reserve(static_cast<std::size_t>(candidate_count * std::max(1, window_frames)));

      int seed_rank = 0;
      struct AcceptedSeedPose
      {
        Eigen::Matrix4d map_to_odom{Eigen::Matrix4d::Identity()};
      };
      std::vector<AcceptedSeedPose> accepted_seed_poses;
      for (int source_pos = begin; source_pos < end; ++source_pos) {
        const auto & source = records[indices[static_cast<std::size_t>(source_pos)]];
        if (!source.summary.has_odom_pose || !source.summary.bbs_result.localized) {
          continue;
        }
        const int source_candidate_count = std::min<int>(
          candidate_count,
          static_cast<int>(source.summary.bbs_result.candidates.size()));
        for (int candidate_index = 0; candidate_index < source_candidate_count; ++candidate_index) {
          const auto & candidate =
            source.summary.bbs_result.candidates[static_cast<std::size_t>(candidate_index)];
          const Eigen::Matrix4d map_to_odom =
            candidate.pose * source.summary.odom_to_base_pose.inverse();
          bool duplicate_seed = false;
          for (const auto & accepted : accepted_seed_poses) {
            const double dx = map_to_odom(0, 3) - accepted.map_to_odom(0, 3);
            const double dy = map_to_odom(1, 3) - accepted.map_to_odom(1, 3);
            const double xy = std::hypot(dx, dy);
            const double yaw_deg =
              std::abs(normalize_angle(yaw_from_matrix(map_to_odom) - yaw_from_matrix(accepted.map_to_odom))) *
              180.0 / M_PI;
            if (xy <= 0.25 * config.temporal.xy_gate_m && yaw_deg <= 0.25 * config.temporal.yaw_gate_deg) {
              duplicate_seed = true;
              break;
            }
          }
          if (duplicate_seed) {
            continue;
          }
          accepted_seed_poses.push_back(AcceptedSeedPose{map_to_odom});
          double overlap_sum = 0.0;
          int support_frames = 0;
          int scored_frames = 0;

          for (int frame_pos = begin; frame_pos < end; ++frame_pos) {
            const auto & frame = records[indices[static_cast<std::size_t>(frame_pos)]];
            if (!frame.summary.has_odom_pose || !frame.scan_cloud) {
              continue;
            }
            const Eigen::Matrix4d predicted_map_to_base =
              force_2d_pose_if_needed(map_to_odom * frame.summary.odom_to_base_pose, frame.frames);
            const double overlap = overlap_ratio_for_pose(
              frame.scan_cloud,
              predicted_map_to_base,
              occupied,
              voxel_size,
              neighbor_radius);
            overlap_sum += overlap;
            ++scored_frames;
            if (overlap >= config.temporal.trajectory_min_overlap_ratio) {
              ++support_frames;
            }
          }

          CandidateLikelihood item;
          item.source_frame_offset = source_pos - center_pos;
          item.source_stamp_sec = source.summary.stamp_sec;
          item.source_candidate_rank = candidate_index + 1;
          item.seed_rank = ++seed_rank;
          item.score = candidate.score;
          item.score_ratio = candidate.score_ratio;
          item.current_map_to_base =
            force_2d_pose_if_needed(map_to_odom * center.summary.odom_to_base_pose, center.frames);
          item.average_overlap = scored_frames > 0 ? overlap_sum / static_cast<double>(scored_frames) : 0.0;
          item.support_frames = support_frames;
          likelihoods.push_back(item);
        }
      }

      std::sort(
        likelihoods.begin(),
        likelihoods.end(),
        [](const CandidateLikelihood & lhs, const CandidateLikelihood & rhs) {
          const auto left_key = std::make_tuple(
            lhs.support_frames,
            lhs.average_overlap,
            -std::abs(lhs.source_frame_offset),
            -lhs.source_candidate_rank,
            -lhs.seed_rank);
          const auto right_key = std::make_tuple(
            rhs.support_frames,
            rhs.average_overlap,
            -std::abs(rhs.source_frame_offset),
            -rhs.source_candidate_rank,
            -rhs.seed_rank);
          return left_key > right_key;
        });

      const double best_overlap = likelihoods.empty() ? 0.0 : likelihoods.front().average_overlap;
      double second_overlap = 0.0;
      if (!likelihoods.empty()) {
        // top-N 里经常会出现同一个正确位姿簇的多个重复 seed。如果直接拿排序第二名算 margin，
        // margin 会被重复候选压成 0，导致“正确且稳定”的轨迹假设被误判为歧义。因此这里寻找
        // 第一个与 best 位姿相差超过 temporal gate 的不同簇，再计算真正有意义的竞争 margin。
        const Eigen::Matrix4d & best_pose = likelihoods.front().current_map_to_base;
        for (std::size_t i = 1; i < likelihoods.size(); ++i) {
          const Eigen::Matrix4d & other_pose = likelihoods[i].current_map_to_base;
          const double dx = best_pose(0, 3) - other_pose(0, 3);
          const double dy = best_pose(1, 3) - other_pose(1, 3);
          const double xy = std::hypot(dx, dy);
          const double yaw_deg =
            std::abs(normalize_angle(yaw_from_matrix(best_pose) - yaw_from_matrix(other_pose))) * 180.0 / M_PI;
          if (xy > config.temporal.xy_gate_m || yaw_deg > config.temporal.yaw_gate_deg) {
            second_overlap = likelihoods[i].average_overlap;
            break;
          }
        }
      }
      const double margin = best_overlap - second_overlap;
      const int best_seed_rank = likelihoods.empty() ? 0 : likelihoods.front().seed_rank;
      const bool trajectory_accept =
        !likelihoods.empty() &&
        likelihoods.front().support_frames >= std::max(1, config.temporal.online_min_support_frames) &&
        best_overlap >= config.temporal.trajectory_min_overlap_ratio &&
        best_overlap >= config.temporal.trajectory_min_average_overlap &&
        margin >= config.temporal.trajectory_min_margin;
      double single_agreement_xy = std::numeric_limits<double>::infinity();
      double single_agreement_yaw_deg = std::numeric_limits<double>::infinity();
      if (!likelihoods.empty()) {
        const Eigen::Matrix4d & best_pose = likelihoods.front().current_map_to_base;
        const double dx = best_pose(0, 3) - center.summary.final_pose(0, 3);
        const double dy = best_pose(1, 3) - center.summary.final_pose(1, 3);
        single_agreement_xy = std::hypot(dx, dy);
        single_agreement_yaw_deg =
          std::abs(normalize_angle(yaw_from_matrix(best_pose) - yaw_from_matrix(center.summary.final_pose))) *
          180.0 / M_PI;
      }
      const bool single_agreement_accept =
        config.temporal.trajectory_single_agreement_fallback_enable &&
        !trajectory_accept &&
        !likelihoods.empty() &&
        likelihoods.front().support_frames >= std::max(1, config.temporal.online_min_support_frames) &&
        best_overlap >= config.temporal.trajectory_single_agreement_min_overlap &&
        margin >= config.temporal.trajectory_single_agreement_min_margin &&
        center.summary.refine_fitness_score <= config.temporal.trajectory_single_agreement_max_fitness &&
        single_agreement_xy <= config.temporal.trajectory_single_agreement_max_xy_m &&
        single_agreement_yaw_deg <= config.temporal.trajectory_single_agreement_max_yaw_deg;

      struct RefinedLikelihood
      {
        int seed_rank{0};
        RefineOutput output;
      };
      std::vector<RefinedLikelihood> refined_likelihoods;
      if (config.temporal.trajectory_refine_enable && center.map_cloud && center.scan_cloud) {
        const int refine_count = std::min<int>(
          std::max(0, config.temporal.trajectory_refine_top_n),
          static_cast<int>(likelihoods.size()));
        refined_likelihoods.reserve(static_cast<std::size_t>(refine_count));
        for (int i = 0; i < refine_count; ++i) {
          const auto & likelihood = likelihoods[static_cast<std::size_t>(i)];
          BbsCandidate seed_candidate;
          seed_candidate.pose = likelihood.current_map_to_base;
          seed_candidate.score = likelihood.score;
          seed_candidate.score_ratio = likelihood.score_ratio;
          RefineOutput refined = refine_single_candidate(
            center.map_cloud,
            center.scan_cloud,
            seed_candidate,
            likelihood.seed_rank,
            center.refine);
          refined.pose = force_2d_pose_if_needed(
            refined.converged ? refined.pose : likelihood.current_map_to_base,
            center.frames);
          refined_likelihoods.push_back(RefinedLikelihood{likelihood.seed_rank, refined});
        }
      }

      std::sort(
        refined_likelihoods.begin(),
        refined_likelihoods.end(),
        [](const RefinedLikelihood & lhs, const RefinedLikelihood & rhs) {
          const double left_fitness = lhs.output.converged ?
            lhs.output.fitness_score :
            std::numeric_limits<double>::infinity();
          const double right_fitness = rhs.output.converged ?
            rhs.output.fitness_score :
            std::numeric_limits<double>::infinity();
          return std::make_tuple(-left_fitness, -lhs.seed_rank) >
                 std::make_tuple(-right_fitness, -rhs.seed_rank);
        });
      const int refined_best_seed_rank = refined_likelihoods.empty() ? 0 : refined_likelihoods.front().seed_rank;
      const double refined_best_fitness =
        refined_likelihoods.empty() ? std::numeric_limits<double>::infinity() :
        refined_likelihoods.front().output.fitness_score;
      const double refined_second_fitness =
        refined_likelihoods.size() > 1 ?
        refined_likelihoods[1].output.fitness_score :
        std::numeric_limits<double>::infinity();
      const double refined_fitness_margin =
        std::isfinite(refined_second_fitness) && std::isfinite(refined_best_fitness) ?
        refined_second_fitness - refined_best_fitness :
        std::numeric_limits<double>::infinity();
      const bool refined_accept =
        !refined_likelihoods.empty() &&
        refined_likelihoods.front().output.converged &&
        refined_best_fitness <= config.temporal.trajectory_refine_max_fitness &&
        refined_fitness_margin >= config.temporal.trajectory_refine_min_fitness_margin;

      for (const auto & likelihood : likelihoods) {
        const bool selected_by_single_frame =
          likelihood.source_frame_offset == 0 &&
          likelihood.source_candidate_rank == center.summary.refined_candidate_rank;
        const bool selected_by_trajectory = likelihood.seed_rank == best_seed_rank;
        const bool selected_by_refine = likelihood.seed_rank == refined_best_seed_rank;
        const RefineOutput * refined_output = nullptr;
        for (const auto & refined : refined_likelihoods) {
          if (refined.seed_rank == likelihood.seed_rank) {
            refined_output = &refined.output;
            break;
          }
        }
        const auto error = center.summary.has_reference_pose ?
          pose_error_against_reference(likelihood.current_map_to_base, center.summary.reference_pose) :
          std::pair<double, double>{-1.0, -1.0};
        const Eigen::Matrix4d refined_pose =
          refined_output ? refined_output->pose : Eigen::Matrix4d::Identity();
        const auto refined_error =
          refined_output && center.summary.has_reference_pose ?
          pose_error_against_reference(refined_pose, center.summary.reference_pose) :
          std::pair<double, double>{-1.0, -1.0};
        const bool final_accept =
          ((trajectory_accept || single_agreement_accept) && selected_by_trajectory) ||
          (refined_accept && selected_by_refine);
        out << config.temporal.scenario_name << ","
            << '"' << center.summary.map_path << '"' << ","
            << '"' << center.summary.bag_path << '"' << ","
            << std::fixed << std::setprecision(6) << center.summary.stamp_sec << ","
            << to_string(center.summary.refine_method) << ","
            << to_string(center.input_mode) << ","
            << likelihood.source_candidate_rank << ","
            << (selected_by_single_frame ? 1 : 0) << ","
            << (selected_by_trajectory ? 1 : 0) << ","
            << (final_accept ? "accept" : "candidate") << ","
            << likelihood.support_frames << ","
            << window_frames << ","
            << likelihood.average_overlap << ","
            << best_overlap << ","
            << second_overlap << ","
            << margin << ","
            << single_agreement_xy << ","
            << single_agreement_yaw_deg << ","
            << (single_agreement_accept ? 1 : 0) << ","
            << likelihood.score << ","
            << likelihood.score_ratio << ","
            << likelihood.current_map_to_base(0, 3) << ","
            << likelihood.current_map_to_base(1, 3) << ","
            << likelihood.current_map_to_base(2, 3) << ","
            << rad_to_deg(yaw_from_matrix(likelihood.current_map_to_base)) << ","
            << error.first << ","
            << error.second << ","
            << (selected_by_refine ? 1 : 0) << ","
            << (refined_output && refined_output->converged ? 1 : 0) << ","
            << (refined_output ? refined_output->fitness_score : -1.0) << ","
            << refined_fitness_margin << ","
            << (refined_output ? refined_output->elapsed_ms : 0.0) << ","
            << (refined_output ? refined_pose(0, 3) : -1.0) << ","
            << (refined_output ? refined_pose(1, 3) : -1.0) << ","
            << (refined_output ? refined_pose(2, 3) : -1.0) << ","
            << (refined_output ? rad_to_deg(yaw_from_matrix(refined_pose)) : -1.0) << ","
            << refined_error.first << ","
            << refined_error.second << ","
            << voxel_size << ","
            << neighbor_radius << ","
            << config.temporal.trajectory_min_overlap_ratio << ","
            << config.temporal.trajectory_min_average_overlap << ","
            << config.temporal.trajectory_min_margin << ","
            << '"' << "trajectory_likelihood_explains_history_window" << '"' << "\n";
      }
    }
  }
}

}  // namespace

EvaluationSummary run_single_pcd_evaluation(const RuntimeConfig & config)
{
  EvaluationSummary summary;

  if (config.input.single_scan_pcd_path.empty()) {
    const auto total_start = std::chrono::steady_clock::now();
    summary.message = "single_scan_pcd_path is empty and bag_paths is empty; no evaluation input configured";
    summary.resources = sample_resource_snapshot();
    summary.total_ms = elapsed_ms(total_start, std::chrono::steady_clock::now());
    return summary;
  }

  std::filesystem::create_directories(config.input.output_dir);

  const auto maps = effective_map_paths(config);
  if (maps.empty()) {
    throw std::runtime_error("map_path/map_candidates is empty");
  }

  RuntimeConfig run_config = config;
  run_config.input.map_path = maps.front();
  run_config.refine.method = config.refine.method;

  // 加载并预处理地图。地图进入 BBS 前会按 map_leaf_size 降采样，避免全量 PCD 直接构图导致资源飙升。
  CloudPtr raw_map = load_pcd_xyz(run_config.input.map_path);
  CloudPtr map_cloud = preprocess_map_cloud(raw_map, config.preprocess);

  // PCD 验证默认认为 single_scan_pcd_path 已经是所选输入模式下的局部 scan。
  // 如果 input_mode=body，会做 raw body -> base；如果后续 PCD 来自 registered_world，则应先由 bag/odom adapter 转局部。
  CloudPtr raw_scan = load_pcd_xyz(config.input.single_scan_pcd_path);
  CloudPtr base_scan = config.input.mode == InputMode::Body ?
    convert_raw_body_to_base_cloud(raw_scan, config.frames) :
    raw_scan;

  const auto map_points = cloud_to_eigen_points(map_cloud);
  SimpleBbs3d bbs(run_config.bbs);
  Bbs2dSearch bbs2d(run_config.bbs2d);
  const auto build_start = std::chrono::steady_clock::now();
  bbs.build_map_index(map_points);
  bbs2d.build_map_index(map_points, run_config.bbs);
  const double build_index_ms = elapsed_ms(build_start, std::chrono::steady_clock::now());

  return evaluate_scan_with_built_map(
    run_config,
    map_cloud,
    bbs,
    run_config.bbs2d.enable ? &bbs2d : nullptr,
    base_scan,
    "single_pcd=" + config.input.single_scan_pcd_path,
    build_index_ms,
    run_config.input.map_path,
    "",
    -1,
    0.0,
    config.evaluation.reference_pose,
    config.evaluation.reference_pose ? "config_reference_pose" : "",
    nullptr);
}

std::vector<EvaluationSummary> run_bag_evaluation(const RuntimeConfig & config)
{
  std::vector<EvaluationSummary> summaries;
  std::vector<EvaluationRecord> records;
  if (config.input.bag_paths.empty()) {
    EvaluationSummary summary;
    summary.message = "bag_paths is empty";
    summary.resources = sample_resource_snapshot();
    summaries.push_back(summary);
    return summaries;
  }

  std::filesystem::create_directories(config.input.output_dir);

  std::vector<LoadedBag> loaded_bags;
  loaded_bags.reserve(config.input.bag_paths.size());
  for (const auto & bag_path : config.input.bag_paths) {
    LoadedBag loaded;
    loaded.path = bag_path;
    try {
      loaded.data = read_bag_data(config, bag_path);
      loaded.scan_context_database = build_scan_context_database(config, loaded.data);
      loaded.ok = true;
    } catch (const std::exception & exc) {
      loaded.ok = false;
      loaded.error = exc.what();
      EvaluationSummary summary;
      summary.message = "failed to read bag " + bag_path + ": " + exc.what();
      summary.bag_path = bag_path;
      summary.resources = sample_resource_snapshot();
      append_evaluation_csvs(config, summary);
      summaries.push_back(summary);
    }
    loaded_bags.push_back(std::move(loaded));
  }

  const auto map_paths = effective_map_paths(config);
  if (map_paths.empty()) {
    throw std::runtime_error("map_path/map_candidates is empty");
  }

  const auto refine_methods = config.refine.methods_for_sweep.empty() ?
    std::vector<RefineMethod>{config.refine.method} :
    config.refine.methods_for_sweep;

  for (const auto & map_path : map_paths) {
    RuntimeConfig map_config = config;
    map_config.input.map_path = map_path;

    // 地图和 3D-BBS voxel index 对同一张地图只构建一次。不同 refine 方法共享同一个 BBS 粗搜索索引。
    CloudPtr raw_map = load_pcd_xyz(map_path);
    CloudPtr map_cloud = preprocess_map_cloud(raw_map, map_config.preprocess);
    const auto map_points = cloud_to_eigen_points(map_cloud);

    SimpleBbs3d bbs(map_config.bbs);
    Bbs2dSearch bbs2d(map_config.bbs2d);
    const auto build_start = std::chrono::steady_clock::now();
    bbs.build_map_index(map_points);
    bbs2d.build_map_index(map_points, map_config.bbs);
    const double build_index_ms = elapsed_ms(build_start, std::chrono::steady_clock::now());

    for (const auto method : refine_methods) {
      RuntimeConfig method_config = map_config;
      method_config.refine.method = method;

      for (const auto & loaded : loaded_bags) {
        if (!loaded.ok) {
          continue;
        }
        if (loaded.data.clouds.empty()) {
          EvaluationSummary summary;
          summary.map_path = map_path;
          summary.bag_path = loaded.path;
          summary.refine_method = method;
          summary.message = "bag has no sampled clouds for topic/mode";
          summary.resources = sample_resource_snapshot();
          append_evaluation_csvs(method_config, summary);
          summaries.push_back(summary);
          continue;
        }

        for (const auto & sample : loaded.data.clouds) {
          CloudPtr base_scan;
          std::optional<nav_msgs::msg::Odometry> synced_odom;
          if (method_config.input.mode == InputMode::RegisteredWorld) {
            synced_odom = nearest_odom(
              loaded.data.odoms, sample.stamp_sec, method_config.input.odom_time_tolerance_sec);
            if (!synced_odom) {
              EvaluationSummary summary;
              summary.map_path = map_path;
              summary.bag_path = sample.bag_path;
              summary.stamp_sec = sample.stamp_sec;
              summary.refine_method = method;
              summary.message = "skip " + sample.bag_path + " stamp=" + std::to_string(sample.stamp_sec) +
                " because synced odom was not found";
              summary.resources = sample_resource_snapshot();
              append_evaluation_csvs(method_config, summary);
              summaries.push_back(summary);
              continue;
            }
            base_scan = registered_world_to_base_scan(sample.cloud_msg, *synced_odom, method_config);
          } else {
            // body 模式不需要 odom 来反变换点云，但多帧 map->odom 一致性仍然需要同帧 odom。
            // 如果当前 bag 没有 /odom，单帧重定位照常评估，时序一致性 CSV 会自动跳过该帧。
            synced_odom = nearest_odom(
              loaded.data.odoms, sample.stamp_sec, method_config.input.odom_time_tolerance_sec);
            base_scan = body_msg_to_base_scan(sample.cloud_msg, method_config);
          }

          std::optional<Eigen::Matrix4d> reference_pose;
          std::string reference_source;
          if (method_config.evaluation.use_bag_reference_pose) {
            const auto reference = nearest_reference_pose(
              loaded.data.reference_poses, sample.bag_time_sec, method_config.evaluation.reference_time_tolerance_sec);
            if (reference) {
              reference_pose = pose_to_matrix(reference->pose_msg.pose.pose);
              reference_source = method_config.evaluation.reference_pose_topic + "(bag_time)";
            }
          }

          const std::string prefix =
            "map=" + map_path + " refine=" + to_string(method) +
            " bag=" + sample.bag_path + " stamp=" + std::to_string(sample.stamp_sec);
          CloudPtr preprocessed_scan;
          auto summary = evaluate_scan_with_built_map(
            method_config,
            map_cloud,
            bbs,
            method_config.bbs2d.enable ? &bbs2d : nullptr,
            base_scan,
            prefix,
            build_index_ms,
            map_path,
            sample.bag_path,
            sample.frame_index,
            sample.stamp_sec,
            reference_pose,
            reference_source,
            &loaded.scan_context_database,
            &preprocessed_scan);
          if (synced_odom) {
            summary.has_odom_pose = true;
            summary.odom_to_base_pose =
              raw_odom_pose_to_base_axis_pose(pose_to_matrix(synced_odom->pose.pose));
          }
          EvaluationRecord record;
          record.summary = summary;
          record.map_cloud = map_cloud;
          record.scan_cloud = preprocessed_scan;
          record.refine = method_config.refine;
          record.frames = method_config.frames;
          record.evaluation = method_config.evaluation;
          record.input_mode = method_config.input.mode;
          records.push_back(record);
          summaries.push_back(summary);
        }
      }
    }
  }

  append_temporal_consistency_csv(config, summaries);
  append_temporal_decision_csv(config, records);
  append_trajectory_likelihood_csv(config, records);
  return summaries;
}

}  // namespace humanoid_global_relocalization
