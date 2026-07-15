/*
 * global_relocalization_node.cpp
 *
 * 文件作用：
 *   1. 提供全局重定位运行层 ROS Node 入口。
 *   2. 按 YAML 的 input_mode 在 /fast_lio/cloud_registered+/odom 与 /cloud_registered_body 两种链路间切换。
 *   3. 把输入点云统一转换成 base_footprint 标准轴 scan，执行 3D-BBS 全局搜索，并发布 top-K 候选。
 *   4. 维护 scan/odom/candidate 滑动窗口；第一层 selected 支持不足或精配质量不足时，
 *      尝试使用长历史 trajectory likelihood 和单帧一致性补救规则发布更可信的恢复候选。
 *
 * 重要边界：
 *   - 本节点只发布 /global_relocalization/* 运行态话题，不直接发布 TF 或 initialpose。
 *   - recovery_map_to_odom 是给上层恢复状态机审核的 PoseStamped 候选。
 *   - enable_relocalization=false 时只加载配置并保持运行，用于确认 launch/参数，不消耗重定位搜索 CPU。
 *
 * 代码块顺序：
 *   1. 头文件和匿名命名空间工具函数。
 *   2. 节点初始化：参数加载、发布器、订阅器、地图索引。
 *   3. 输入链路：odom 缓存、点云同步、点云预处理和全局搜索。
 *   4. 恢复决策：多帧一致性、单帧高置信、轨迹 likelihood。
 *   5. 输出链路：恢复候选、恢复状态、候选可视化。
 *   6. main 入口。
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_set>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"

#include "humanoid_global_relocalization_runtime/bbs2d_search.hpp"
#include "humanoid_global_relocalization_runtime/config.hpp"
#include "humanoid_global_relocalization_runtime/point_cloud_adapter.hpp"
#include "humanoid_global_relocalization_runtime/refiner.hpp"
#include "humanoid_global_relocalization_runtime/scan_context.hpp"
#include "humanoid_global_relocalization_runtime/solid.hpp"
#include "humanoid_global_relocalization_runtime/simple_bbs3d.hpp"
#include "humanoid_global_relocalization_runtime/temporal_consistency.hpp"

namespace humanoid_global_relocalization
{
namespace
{

std::string json_escape(const std::string & value)
{
  std::ostringstream escaped;
  for (const unsigned char c : value) {
    switch (c) {
      case '"': escaped << "\\\""; break;
      case '\\': escaped << "\\\\"; break;
      case '\b': escaped << "\\b"; break;
      case '\f': escaped << "\\f"; break;
      case '\n': escaped << "\\n"; break;
      case '\r': escaped << "\\r"; break;
      case '\t': escaped << "\\t"; break;
      default:
        if (c < 0x20) {
          escaped << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                  << static_cast<int>(c) << std::dec << std::setfill(' ');
        } else {
          escaped << static_cast<char>(c);
        }
    }
  }
  return escaped.str();
}

std::string json_number(double value)
{
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream text;
  text << std::setprecision(12) << value;
  return text.str();
}

double stamp_to_sec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

Eigen::Matrix4d pose_to_matrix(const geometry_msgs::msg::Pose & pose)
{
  // Fast-LIO /odom 的 pose 表示 camera_init(raw world) -> body(raw body)。
  // registered_world 模式会用它的逆把点云从 raw world 变回 raw body。
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
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

geometry_msgs::msg::Pose matrix_to_pose(const Eigen::Matrix4d & matrix)
{
  geometry_msgs::msg::Pose pose;
  const Eigen::Quaterniond q(matrix.block<3, 3>(0, 0));
  pose.position.x = matrix(0, 3);
  pose.position.y = matrix(1, 3);
  pose.position.z = matrix(2, 3);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

Eigen::Matrix4d constrain_to_2d_if_needed(const Eigen::Matrix4d & pose, const FrameConfig & config)
{
  if (!config.force_2d_output) {
    return pose;
  }

  // 与评估工具保持同一位姿约束：室内导航只输出 x/y/yaw，z/roll/pitch 归零。
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

double normalized_angle(double angle)
{
  // 统一把角度差压到 [-pi, pi]，用于比较两个候选朝向是否属于同一个位姿簇。
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double pose_xy_distance(const Eigen::Matrix4d & lhs, const Eigen::Matrix4d & rhs)
{
  return std::hypot(lhs(0, 3) - rhs(0, 3), lhs(1, 3) - rhs(1, 3));
}

double pose_yaw_distance_deg(const Eigen::Matrix4d & lhs, const Eigen::Matrix4d & rhs)
{
  return std::abs(normalized_angle(yaw_from_matrix(lhs) - yaw_from_matrix(rhs))) * 180.0 / M_PI;
}

bool is_duplicate_candidate(
  const std::vector<BbsCandidate> & candidates,
  const Eigen::Matrix4d & pose,
  const ScanContextConfig & config)
{
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
  // Bbs2dSearch 内部已经做过大范围 NMS；这里用较小阈值只剔除与已有候选几乎重合的 seed。
  // 这样可以保留不同区域的多假设，又不会让 GICP 和轨迹验证重复处理同一个位置。
  for (const auto & candidate : candidates) {
    if (pose_xy_distance(candidate.pose, pose) <= config.duplicate_xy_gate_m &&
      pose_yaw_distance_deg(candidate.pose, pose) <= config.duplicate_yaw_gate_deg)
    {
      return true;
    }
  }
  return false;
}

bool is_duplicate_solid_candidate(
  const std::vector<BbsCandidate> & candidates,
  const Eigen::Matrix4d & pose,
  const SolidConfig & config)
{
  for (const auto & candidate : candidates) {
    if (pose_xy_distance(candidate.pose, pose) <= config.duplicate_xy_gate_m &&
      pose_yaw_distance_deg(candidate.pose, pose) <= config.duplicate_yaw_gate_deg)
    {
      return true;
    }
  }
  return false;
}

void move_candidate_to_front(std::vector<BbsCandidate> & candidates, int one_based_rank)
{
  if (one_based_rank <= 1 || one_based_rank > static_cast<int>(candidates.size())) {
    return;
  }
  auto best = candidates[static_cast<std::size_t>(one_based_rank - 1)];
  candidates.erase(candidates.begin() + (one_based_rank - 1));
  candidates.insert(candidates.begin(), best);
}

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
    // 用三个大质数混合体素坐标，降低相邻体素落到同一桶的概率。
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

OccupancySet build_occupancy_set_for_online(const CloudPtr & map_cloud, double voxel_size)
{
  // 在线 trajectory likelihood 不做 KdTree 最近邻，只把地图离散成占据体素集合。
  // 这样每个候选在多帧 scan 上的 overlap 查询成本可控，适合 recovery 触发时使用。
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
  // neighbor_radius 越大越能容忍点云噪声，也越容易让重复结构候选得高分。
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

double overlap_ratio_for_online_pose(
  const CloudPtr & scan_cloud,
  const Eigen::Matrix4d & map_to_base,
  const OccupancySet & occupied,
  double voxel_size,
  int neighbor_radius)
{
  // 将局部 scan 投到 map，再统计有多少点落在地图占据体素邻域内。
  // 这个值用于判断某个隐含 map->odom 假设是否能解释一段历史 scan。
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
    const auto key = voxel_key_from_point(mapped.x(), mapped.y(), mapped.z(), voxel_size);
    ++valid_points;
    if (has_occupied_neighbor(occupied, key, neighbor_radius)) {
      ++hit_points;
    }
  }
  return valid_points > 0 ? static_cast<double>(hit_points) / static_cast<double>(valid_points) : 0.0;
}

CloudPtr pointcloud2_to_xyz_cloud(const sensor_msgs::msg::PointCloud2 & msg)
{
  CloudPtr cloud(new Cloud);
  pcl::fromROSMsg(msg, *cloud);
  return cloud;
}

}  // namespace

class GlobalRelocalizationNode : public rclcpp::Node
{
public:
  GlobalRelocalizationNode()
  : Node("global_relocalization_node")
  {
    config_file_ = declare_parameter<std::string>(
      "config_file",
      "src/humanoid_global_relocalization_runtime/config/relocalization_runtime.yaml");

    try {
      config_ = load_config_file(config_file_);
      RCLCPP_INFO(
        get_logger(),
        "loaded config=%s input_mode=%s enable_relocalization=%s",
        config_file_.c_str(),
        to_string(config_.input.mode).c_str(),
        config_.output.enable_relocalization ? "true" : "false");
    } catch (const std::exception & exc) {
      RCLCPP_ERROR(get_logger(), "failed to load global relocalization config: %s", exc.what());
      return;
    }

    // 运行态话题无论是否启用实时搜索都先创建，方便上层状态机和运维工具固定订阅。
    candidate_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      config_.output.candidate_pose_array_topic, rclcpp::QoS(1).transient_local());
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      config_.output.candidate_marker_topic, rclcpp::QoS(1).transient_local());
    aligned_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      config_.output.aligned_cloud_topic, rclcpp::QoS(1));
    recovery_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      config_.output.recovery_pose_topic, rclcpp::QoS(1).transient_local());
    recovery_map_odom_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      config_.output.recovery_map_odom_topic, rclcpp::QoS(1).transient_local());
    recovery_status_pub_ = create_publisher<std_msgs::msg::String>(
      config_.output.recovery_status_topic, rclcpp::QoS(1).transient_local());
    request_sub_ = create_subscription<std_msgs::msg::String>(
      config_.output.request_topic,
      rclcpp::QoS(10),
      [this](std_msgs::msg::String::SharedPtr msg) {handle_relocalization_request(msg->data);});

    if (!config_.output.enable_relocalization) {
      RCLCPP_INFO(
        get_logger(),
        "relocalization search is disabled; this node will not publish recovery candidates");
      return;
    }

    if (!build_map_index()) {
      return;
    }

    search_active_ = !config_.output.on_demand_mode;
    publish_control_status(search_active_ ? "searching" : "idle", "node_ready");

    // registered_world 模式需要缓存 /odom，用点云时间戳找最近位姿做 raw world -> raw body。
    if (config_.input.mode == InputMode::RegisteredWorld || config_.temporal.enable) {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        config_.input.odom_topic,
        rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::SharedPtr msg) { handle_odom(std::move(msg)); });
    }

    const std::string cloud_topic = config_.input.mode == InputMode::RegisteredWorld ?
      config_.input.registered_world_topic :
      config_.input.body_topic;
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic,
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { handle_cloud(std::move(msg)); });

    RCLCPP_INFO(
      get_logger(),
      "relocalization subscribed cloud=%s odom=%s; outputs candidates=%s markers=%s aligned=%s recovery_pose=%s recovery_map_odom=%s status=%s",
      cloud_topic.c_str(),
      config_.input.odom_topic.c_str(),
      config_.output.candidate_pose_array_topic.c_str(),
      config_.output.candidate_marker_topic.c_str(),
      config_.output.aligned_cloud_topic.c_str(),
      config_.output.recovery_pose_topic.c_str(),
      config_.output.recovery_map_odom_topic.c_str(),
      config_.output.recovery_status_topic.c_str());
  }

private:
  void reset_attempt_state()
  {
    temporal_window_.clear();
    scan_window_.clear();
    default_reject_streak_ = 0;
    precision_attempts_remaining_ = 0;
    last_search_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_precision_layer_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }

  void publish_control_status(const std::string & state, const std::string & reason)
  {
    if (!recovery_status_pub_) {
      return;
    }
    std_msgs::msg::String status;
    std::ostringstream text;
    text << "{\"protocol_version\":1"
         << ",\"event_type\":\"global_relocalization_control\""
         << ",\"attempt_id\":\"" << json_escape(active_attempt_id_) << "\""
         << ",\"map_id\":\"" << json_escape(active_map_id_) << "\""
         << ",\"stamp_sec\":" << json_number(now().seconds())
         << ",\"state\":\"" << json_escape(state) << "\""
         << ",\"reason\":\"" << json_escape(reason) << "\""
         << ",\"search_active\":" << (search_active_ ? "true" : "false")
         << ",\"map_path\":\"" << json_escape(config_.input.map_path) << "\""
         << ",\"scan_context_entries\":" << scan_context_database_.size()
         << ",\"solid_entries\":" << solid_database_.size()
         << ",\"precision_scan_context_entries\":" << precision_scan_context_database_.size()
         << ",\"degraded_recall\":"
         << ((scan_context_database_.empty() && solid_database_.empty()) ? "true" : "false")
         << "}";
    status.data = text.str();
    recovery_status_pub_->publish(status);
  }

  void handle_relocalization_request(const std::string & data)
  {
    try {
      const YAML::Node request = YAML::Load(data);
      if (!request.IsMap()) {
        publish_control_status("rejected", "request_not_object");
        return;
      }
      const std::string command = request["command"] ? request["command"].as<std::string>() : "";
      const std::string attempt_id = request["attempt_id"] ? request["attempt_id"].as<std::string>() : "";
      const std::string map_id = request["map_id"] ? request["map_id"].as<std::string>() : "";
      if (command == "start") {
        if (attempt_id.empty() || map_id.empty()) {
          publish_control_status("rejected", "missing_attempt_id_or_map_id");
          return;
        }
        const std::string previous_attempt_id = active_attempt_id_;
        const std::string previous_map_id = active_map_id_;
        active_attempt_id_ = attempt_id;
        search_active_ = false;
        std::string asset_error;
        if (!activate_requested_map_assets(request, map_id, asset_error)) {
          publish_control_status("rejected", asset_error);
          active_attempt_id_ = previous_attempt_id;
          active_map_id_ = previous_map_id;
          return;
        }
        active_map_id_ = map_id;
        reset_attempt_state();
        search_active_ = true;
        publish_control_status("searching", "request_started");
        return;
      }
      if (command == "cancel") {
        if (!attempt_id.empty() && active_attempt_id_ != attempt_id) {
          publish_control_status("rejected", "cancel_attempt_mismatch");
          return;
        }
        search_active_ = !config_.output.on_demand_mode;
        reset_attempt_state();
        publish_control_status("cancelled", "request_cancelled");
        if (config_.output.on_demand_mode) {
          active_attempt_id_.clear();
        }
        return;
      }
      publish_control_status("rejected", "unknown_command");
    } catch (const std::exception & exc) {
      publish_control_status("rejected", std::string("request_parse_error:") + exc.what());
    }
  }

  bool activate_requested_map_assets(
    const YAML::Node & request, const std::string & map_id, std::string & error)
  {
    const std::string requested_map_path =
      request["map_path"] ? request["map_path"].as<std::string>() : "";
    const std::string requested_sc_path = request["scan_context_database_path"] ?
      request["scan_context_database_path"].as<std::string>() : "";
    const std::string requested_solid_path = request["solid_database_path"] ?
      request["solid_database_path"].as<std::string>() : "";

    if (requested_map_path.empty()) {
      if (active_map_id_ != "unassigned" && active_map_id_ != map_id) {
        error = "map_assets_missing_for_requested_map";
        return false;
      }
      return true;
    }

    const bool assets_changed =
      requested_map_path != config_.input.map_path ||
      requested_sc_path != config_.scan_context.database_path ||
      requested_solid_path != config_.solid.database_path;
    if (!assets_changed) {
      return true;
    }

    const std::string old_map_path = config_.input.map_path;
    const std::string old_sc_path = config_.scan_context.database_path;
    const std::string old_precision_sc_path = config_.precision_recovery.scan_context_database_path;
    const std::string old_solid_path = config_.solid.database_path;
    const bool old_sc_enable = config_.scan_context.enable;
    const bool old_solid_enable = config_.solid.enable;

    config_.input.map_path = requested_map_path;
    config_.scan_context.database_path = requested_sc_path;
    config_.scan_context.enable = !requested_sc_path.empty();
    config_.precision_recovery.scan_context_database_path = requested_sc_path;
    config_.solid.database_path = requested_solid_path;
    config_.solid.enable = !requested_solid_path.empty();
    if (build_map_index()) {
      return true;
    }

    config_.input.map_path = old_map_path;
    config_.scan_context.database_path = old_sc_path;
    config_.scan_context.enable = old_sc_enable;
    config_.precision_recovery.scan_context_database_path = old_precision_sc_path;
    config_.solid.database_path = old_solid_path;
    config_.solid.enable = old_solid_enable;
    (void)build_map_index();
    error = "map_assets_load_failed";
    return false;
  }

  bool build_map_index()
  {
    try {
      scan_context_database_.clear();
      solid_database_.clear();
      precision_scan_context_database_.clear();
      // 运行态与评估工具使用同一套地图预处理和 3D-BBS 参数，保证参数语义一致。
      CloudPtr raw_map = load_pcd_xyz(config_.input.map_path);
      map_cloud_ = preprocess_map_cloud(raw_map, config_.preprocess);
      const auto map_points = cloud_to_eigen_points(map_cloud_);
      bbs_ = std::make_unique<SimpleBbs3d>(config_.bbs);
      bbs2d_ = std::make_unique<Bbs2dSearch>(config_.bbs2d);

      const auto start = std::chrono::steady_clock::now();
      bbs_->build_map_index(map_points);
      bbs2d_->build_map_index(map_points, config_.bbs);
      const double voxel_size = std::max(0.05, config_.temporal.trajectory_voxel_size);
      map_occupancy_ = build_occupancy_set_for_online(map_cloud_, voxel_size);
      if (config_.scan_context.enable && !config_.scan_context.database_path.empty()) {
        scan_context_database_ = load_scan_context_database(config_.scan_context.database_path);
      }
      if (config_.solid.enable && !config_.solid.database_path.empty()) {
        solid_database_ = load_solid_database(config_.solid.database_path);
      }
      if (config_.precision_recovery.enable &&
        config_.precision_recovery.enable_scan_context_recall &&
        !config_.precision_recovery.scan_context_database_path.empty())
      {
        precision_scan_context_database_ =
          load_scan_context_database(config_.precision_recovery.scan_context_database_path);
      }
      const auto build_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();

      RCLCPP_INFO(
        get_logger(),
        "built global relocalization map index: map_points=%zu occupancy_voxels=%zu scan_context_entries=%zu solid_entries=%zu precision_scan_context_entries=%zu bbs2d=%s precision_layer=%s build_ms=%.3f",
        map_cloud_->size(),
        map_occupancy_.size(),
        scan_context_database_.size(),
        solid_database_.size(),
        precision_scan_context_database_.size(),
        config_.bbs2d.enable ? "enabled" : "disabled",
        config_.precision_recovery.enable ? "enabled" : "disabled",
        build_ms);
      return true;
    } catch (const std::exception & exc) {
      RCLCPP_ERROR(get_logger(), "failed to build 3D-BBS map index: %s", exc.what());
      return false;
    }
  }

  void handle_odom(nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // 只保留最近一小段 odom，避免长时间运行时内存无界增长。
    odom_buffer_.push_back(*msg);
    while (odom_buffer_.size() > 300) {
      odom_buffer_.pop_front();
    }
  }

  std::optional<nav_msgs::msg::Odometry> nearest_odom(double stamp_sec) const
  {
    const nav_msgs::msg::Odometry * best = nullptr;
    double best_dt = std::numeric_limits<double>::max();
    for (const auto & odom : odom_buffer_) {
      const double dt = std::abs(stamp_to_sec(odom.header.stamp) - stamp_sec);
      if (dt < best_dt) {
        best = &odom;
        best_dt = dt;
      }
    }
    if (!best || best_dt > config_.input.odom_time_tolerance_sec) {
      return std::nullopt;
    }
    return *best;
  }

  void fuse_bbs2d_candidates_for_online(
    const std::vector<Eigen::Vector3d> & scan_points,
    BbsResult & result)
  {
    if (!config_.bbs2d.enable || !bbs2d_ || scan_points.empty()) {
      return;
    }

    const BbsResult deep_result = bbs2d_->localize(scan_points, config_.bbs);
    if (!deep_result.localized || deep_result.candidates.empty()) {
      return;
    }

    std::vector<BbsCandidate> fused;
    fused.reserve(result.candidates.size() + deep_result.candidates.size());
    const int prefix = std::min<int>(
      std::max(0, config_.bbs2d.fuse_prefix_3d_candidates),
      static_cast<int>(result.candidates.size()));
    for (int i = 0; i < prefix; ++i) {
      fused.push_back(result.candidates[static_cast<std::size_t>(i)]);
    }

    // 2.5D BBS 在随机 300 点压测中更适合做候选池召回，而不是单帧 top1 发布。
    // 因此在线融合时把它追加到候选池，让后续 trajectory likelihood 使用，而不是强行挤到第一名。
    for (const auto & candidate : deep_result.candidates) {
      if (!is_duplicate_bbs2d_candidate(fused, candidate.pose, config_.bbs2d)) {
        fused.push_back(candidate);
      }
    }
    for (int i = prefix; i < static_cast<int>(result.candidates.size()); ++i) {
      const auto & candidate = result.candidates[static_cast<std::size_t>(i)];
      if (!is_duplicate_bbs2d_candidate(fused, candidate.pose, config_.bbs2d)) {
        fused.push_back(candidate);
      }
    }
    if (fused.empty()) {
      return;
    }

    result.candidates = std::move(fused);
    result.localized = true;
    result.search_ms += deep_result.search_ms;
    RCLCPP_INFO(
      get_logger(),
      "2.5D BBS recall fused candidates=%zu deep_candidates=%zu deep_ms=%.3f",
      result.candidates.size(),
      deep_result.candidates.size(),
      deep_result.search_ms);
  }

  void fuse_scan_context_candidates_for_online(const CloudPtr & scan_cloud, BbsResult & result)
  {
    if (!config_.scan_context.enable || scan_context_database_.empty() || !scan_cloud || scan_cloud->empty()) {
      return;
    }
    if (!result.localized || result.candidates.empty()) {
      return;
    }

    CloudPtr scan_ptr(new Cloud(*scan_cloud));
    RefineOutput primary_refined =
      refine_candidates(map_cloud_, scan_ptr, result.candidates, config_.refine);
    if (primary_refined.converged &&
      primary_refined.fitness_score <= config_.temporal.single_frame_high_confidence_max_fitness)
    {
      move_candidate_to_front(result.candidates, primary_refined.candidate_rank);
      return;
    }

    const ScanContextDescriptor query =
      compute_scan_context_descriptor(scan_cloud, config_.scan_context);
    const auto matches =
      query_scan_context_database(query, scan_context_database_, -1, config_.scan_context);
    if (matches.empty()) {
      move_candidate_to_front(result.candidates, primary_refined.candidate_rank);
      return;
    }

    std::vector<BbsCandidate> fused;
    const int refine_budget = std::max(1, config_.refine.max_refine_candidates);
    const int bbs_prefix = std::min<int>(
      static_cast<int>(result.candidates.size()),
      std::max(1, refine_budget / 3));
    for (int i = 0; i < bbs_prefix; ++i) {
      fused.push_back(result.candidates[static_cast<std::size_t>(i)]);
    }
    for (const auto & match : matches) {
      for (const double yaw_offset_deg : config_.scan_context.yaw_offsets_deg) {
        const Eigen::Matrix4d seed_pose =
          apply_scan_context_yaw_shift(match.map_to_base, match.shift, yaw_offset_deg, config_.scan_context);
        if (is_duplicate_candidate(fused, seed_pose, config_.scan_context)) {
          continue;
        }
        BbsCandidate candidate;
        candidate.pose = seed_pose;
        candidate.score = -match.rank;
        candidate.score_ratio = std::max(0.0, 1.0 - match.distance);
        fused.push_back(candidate);
      }
    }
    for (int i = bbs_prefix; i < static_cast<int>(result.candidates.size()); ++i) {
      if (!is_duplicate_candidate(fused, result.candidates[static_cast<std::size_t>(i)].pose, config_.scan_context)) {
        fused.push_back(result.candidates[static_cast<std::size_t>(i)]);
      }
    }
    if (fused.empty()) {
      return;
    }

    RefineOutput fused_refined = refine_candidates(map_cloud_, scan_ptr, fused, config_.refine);
    move_candidate_to_front(fused, fused_refined.candidate_rank);
    result.candidates = std::move(fused);
    RCLCPP_INFO(
      get_logger(),
      "scan context recall fused candidates=%zu primary_fitness=%.6f fused_fitness=%.6f selected_rank=%d",
      result.candidates.size(),
      primary_refined.fitness_score,
      fused_refined.fitness_score,
      fused_refined.candidate_rank);
  }

  void fuse_solid_candidates_for_online(const CloudPtr & scan_cloud, BbsResult & result)
  {
    if (!config_.solid.enable || solid_database_.empty() || !scan_cloud || scan_cloud->empty()) {
      return;
    }
    const auto descriptor = compute_solid_descriptor(scan_cloud, config_.solid);
    const auto matches = query_solid_database(descriptor, solid_database_, -1, config_.solid);
    const CloudPtr registration_scan = prepare_solid_registration_cloud(scan_cloud, config_.solid);
    RefineConfig solid_refine = config_.refine;
    solid_refine.max_iterations = config_.solid.registration_max_iterations;
    solid_refine.max_correspondence_distance = config_.solid.registration_max_correspondence_m;
    if (matches.empty()) {
      return;
    }
    std::vector<BbsCandidate> fused;
    fused.reserve(result.candidates.size() + matches.size());
    const int prefix = std::min<int>(
      std::max(0, config_.solid.bbs_prefix_candidates), static_cast<int>(result.candidates.size()));
    for (int i = 0; i < prefix; ++i) {
      fused.push_back(result.candidates[static_cast<std::size_t>(i)]);
    }
    std::vector<std::pair<double, BbsCandidate>> solid_candidates;
    solid_candidates.reserve(matches.size());
    for (const auto & match : matches) {
      const auto entry = std::find_if(
        solid_database_.begin(), solid_database_.end(), [&](const SolidEntry & value) {
          return value.cloud_index == match.cloud_index && value.source_id == match.source_id;
        });
      if (entry == solid_database_.end() || !entry->cloud || entry->cloud->empty()) {
        continue;
      }
      BbsCandidate relative_seed;
      relative_seed.pose = apply_solid_heading(Eigen::Matrix4d::Identity(), match.relative_yaw_deg);
      const RefineOutput relative = refine_single_candidate(
        entry->cloud, registration_scan, relative_seed, match.rank, solid_refine);
      result.search_ms += relative.elapsed_ms;
      if (!relative.converged) {
        continue;
      }
      BbsCandidate candidate;
      candidate.pose = entry->map_to_base * relative.pose;
      candidate.score = -match.rank;
      candidate.score_ratio = match.similarity;
      candidate.pre_refined = true;
      candidate.refinement_fitness = relative.fitness_score;
      candidate.selection_score = relative.fitness_score;
      const double official_score =
        (1.0 - match.similarity) + 0.25 * std::sqrt(std::max(0.0, relative.fitness_score));
      solid_candidates.emplace_back(official_score, std::move(candidate));
    }
    std::sort(solid_candidates.begin(), solid_candidates.end(), [](const auto & lhs, const auto & rhs) {
      return lhs.first < rhs.first;
    });
    bool first_solid_candidate = true;
    for (const auto & [score, source_candidate] : solid_candidates) {
      (void)score;
      BbsCandidate candidate = source_candidate;
      candidate.solid_primary = first_solid_candidate;
      if (!is_duplicate_solid_candidate(fused, candidate.pose, config_.solid)) {
        fused.push_back(candidate);
        first_solid_candidate = false;
      }
    }
    for (std::size_t i = static_cast<std::size_t>(prefix); i < result.candidates.size(); ++i) {
      if (!is_duplicate_solid_candidate(fused, result.candidates[i].pose, config_.solid)) {
        fused.push_back(result.candidates[i]);
      }
    }
    if (!fused.empty()) {
      result.candidates = std::move(fused);
      result.localized = true;
    }
    RCLCPP_INFO(
      get_logger(), "SOLiD recall fused candidates=%zu matches=%zu", result.candidates.size(), matches.size());
  }

  bool precision_layer_available() const
  {
    return config_.precision_recovery.enable &&
           config_.precision_recovery.enable_scan_context_recall &&
           !precision_scan_context_database_.empty();
  }

  bool should_run_precision_layer(const rclcpp::Time & now) const
  {
    if (!precision_layer_available()) {
      return false;
    }
    if (precision_attempts_remaining_ <= 0) {
      return false;
    }
    if (last_precision_layer_time_.nanoseconds() != 0) {
      const double dt = (now - last_precision_layer_time_).seconds();
      if (dt < std::max(0.0, config_.precision_recovery.cooldown_sec)) {
        return false;
      }
    }
    return true;
  }

  void update_precision_layer_state(bool recovery_published, bool precision_layer_used)
  {
    if (!precision_layer_available()) {
      return;
    }
    if (recovery_published) {
      default_reject_streak_ = 0;
      precision_attempts_remaining_ = 0;
      return;
    }

    ++default_reject_streak_;
    if (precision_layer_used) {
      precision_attempts_remaining_ = std::max(0, precision_attempts_remaining_ - 1);
      return;
    }

    if (!config_.precision_recovery.trigger_on_default_reject) {
      return;
    }

    const int trigger_rejects = std::max(1, config_.precision_recovery.min_default_reject_frames);
    if (default_reject_streak_ >= trigger_rejects) {
      precision_attempts_remaining_ =
        std::max(precision_attempts_remaining_, std::max(1, config_.precision_recovery.attempt_frames));
    }
  }

  void arm_precision_layer_attempts()
  {
    if (!precision_layer_available()) {
      return;
    }
    precision_attempts_remaining_ =
      std::max(precision_attempts_remaining_, std::max(1, config_.precision_recovery.attempt_frames));
  }

  void fuse_precision_layer_candidates_for_online(const CloudPtr & base_scan, BbsResult & result)
  {
    if (!precision_layer_available() || !base_scan || base_scan->empty()) {
      return;
    }

    PreprocessConfig precision_preprocess = config_.preprocess;
    precision_preprocess.scan_leaf_size = config_.precision_recovery.scan_leaf_size;
    CloudPtr precision_scan_cloud = preprocess_scan_cloud(base_scan, precision_preprocess);
    if (!precision_scan_cloud || precision_scan_cloud->empty()) {
      return;
    }

    BbsResult precision_result = bbs_->localize(cloud_to_eigen_points(precision_scan_cloud));
    if (!precision_result.localized || precision_result.candidates.empty()) {
      return;
    }

    RefineConfig precision_refine = config_.refine;
    precision_refine.max_refine_candidates =
      std::max(1, config_.precision_recovery.max_refine_candidates);
    ScanContextConfig precision_scan_context = config_.scan_context;
    precision_scan_context.enable = true;
    precision_scan_context.database_path = config_.precision_recovery.scan_context_database_path;

    CloudPtr scan_ptr(new Cloud(*precision_scan_cloud));
    RefineOutput primary_refined =
      refine_candidates(map_cloud_, scan_ptr, precision_result.candidates, precision_refine);
    if (!primary_refined.converged ||
      primary_refined.fitness_score > config_.temporal.single_frame_high_confidence_max_fitness)
    {
      const ScanContextDescriptor query =
        compute_scan_context_descriptor(precision_scan_cloud, precision_scan_context);
      const auto matches =
        query_scan_context_database(query, precision_scan_context_database_, -1, precision_scan_context);

      std::vector<BbsCandidate> precision_fused;
      const int refine_budget = std::max(1, precision_refine.max_refine_candidates);
      const int bbs_prefix = std::min<int>(
        static_cast<int>(precision_result.candidates.size()),
        std::max(1, refine_budget / 3));
      for (int i = 0; i < bbs_prefix; ++i) {
        precision_fused.push_back(precision_result.candidates[static_cast<std::size_t>(i)]);
      }
      for (const auto & match : matches) {
        for (const double yaw_offset_deg : precision_scan_context.yaw_offsets_deg) {
          const Eigen::Matrix4d seed_pose =
            apply_scan_context_yaw_shift(
            match.map_to_base, match.shift, yaw_offset_deg, precision_scan_context);
          if (is_duplicate_candidate(precision_fused, seed_pose, precision_scan_context)) {
            continue;
          }
          BbsCandidate candidate;
          candidate.pose = seed_pose;
          candidate.score = -match.rank;
          candidate.score_ratio = std::max(0.0, 1.0 - match.distance);
          precision_fused.push_back(candidate);
        }
      }
      for (int i = bbs_prefix; i < static_cast<int>(precision_result.candidates.size()); ++i) {
        const auto & candidate = precision_result.candidates[static_cast<std::size_t>(i)];
        if (!is_duplicate_candidate(precision_fused, candidate.pose, precision_scan_context)) {
          precision_fused.push_back(candidate);
        }
      }
      if (!precision_fused.empty()) {
        RefineOutput fused_refined =
          refine_candidates(map_cloud_, scan_ptr, precision_fused, precision_refine);
        move_candidate_to_front(precision_fused, fused_refined.candidate_rank);
        primary_refined = fused_refined;
        precision_result.candidates = std::move(precision_fused);
      }
    } else {
      move_candidate_to_front(precision_result.candidates, primary_refined.candidate_rank);
    }

    if (config_.precision_recovery.enable_bbs2d_recall && bbs2d_) {
      const BbsResult deep_result =
        bbs2d_->localize(cloud_to_eigen_points(precision_scan_cloud), config_.bbs);
      for (const auto & candidate : deep_result.candidates) {
        if (!is_duplicate_bbs2d_candidate(precision_result.candidates, candidate.pose, config_.bbs2d)) {
          precision_result.candidates.push_back(candidate);
        }
      }
      precision_result.search_ms += deep_result.search_ms;
    }

    std::vector<BbsCandidate> merged;
    merged.reserve(result.candidates.size() + precision_result.candidates.size());
    for (const auto & candidate : result.candidates) {
      if (!is_duplicate_candidate(merged, candidate.pose, precision_scan_context)) {
        merged.push_back(candidate);
      }
    }
    for (const auto & candidate : precision_result.candidates) {
      if (!is_duplicate_candidate(merged, candidate.pose, precision_scan_context)) {
        merged.push_back(candidate);
      }
    }
    if (merged.empty()) {
      return;
    }

    RefineOutput merged_refined = refine_candidates(map_cloud_, scan_ptr, merged, precision_refine);
    move_candidate_to_front(merged, merged_refined.candidate_rank);
    result.candidates = std::move(merged);
    result.localized = true;
    result.search_ms += precision_result.search_ms;
    last_precision_layer_time_ = this->now();
    RCLCPP_INFO(
      get_logger(),
      "precision recovery layer fused candidates=%zu precision_candidates=%zu fitness=%.6f selected_rank=%d remaining_attempts=%d reject_streak=%d",
      result.candidates.size(),
      precision_result.candidates.size(),
      merged_refined.fitness_score,
      merged_refined.candidate_rank,
      precision_attempts_remaining_,
      default_reject_streak_);
  }

  void handle_cloud(sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!bbs_ || !search_active_) {
      return;
    }

    // 实时搜索默认节流，避免每帧都做全图 BBS 搜索把 CPU 打满。
    const auto now = this->now();
    if (last_search_time_.nanoseconds() != 0) {
      const double dt = (now - last_search_time_).seconds();
      if (dt < config_.output.relocalization_min_period_sec) {
        return;
      }
    }
    last_search_time_ = now;

    CloudPtr base_scan;
    std::optional<nav_msgs::msg::Odometry> synced_odom;
    if (config_.input.mode == InputMode::RegisteredWorld) {
      synced_odom = nearest_odom(stamp_to_sec(msg->header.stamp));
      if (!synced_odom) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "skip registered_world cloud because synced odom was not found");
        return;
      }
      CloudPtr world_cloud = pointcloud2_to_xyz_cloud(*msg);
      CloudPtr raw_body_cloud = transform_cloud(world_cloud, pose_to_matrix(synced_odom->pose.pose).inverse());
      base_scan = convert_raw_body_to_base_cloud(raw_body_cloud, config_.frames);
    } else {
      synced_odom = nearest_odom(stamp_to_sec(msg->header.stamp));
      base_scan = convert_raw_body_to_base_cloud(pointcloud2_to_xyz_cloud(*msg), config_.frames);
    }

    CloudPtr scan_cloud = preprocess_scan_cloud(base_scan, config_.preprocess);
    const auto scan_points = cloud_to_eigen_points(scan_cloud);
    auto result = bbs_->localize(scan_points);
    fuse_solid_candidates_for_online(base_scan, result);
    fuse_scan_context_candidates_for_online(scan_cloud, result);
    fuse_bbs2d_candidates_for_online(scan_points, result);
    const bool precision_layer_used = should_run_precision_layer(now);
    if (precision_layer_used) {
      fuse_precision_layer_candidates_for_online(base_scan, result);
    }

    publish_runtime_outputs(*msg, *scan_cloud, result);
    const bool recovery_published =
      update_temporal_verifier(*msg, *scan_cloud, result, synced_odom, precision_layer_used);
    update_precision_layer_state(recovery_published, precision_layer_used);
    RCLCPP_INFO(
      get_logger(),
      "relocalization localized=%s candidates=%zu search_ms=%.3f scan_points=%zu precision_layer=%s reject_streak=%d precision_attempts_remaining=%d",
      result.localized ? "true" : "false",
      result.candidates.size(),
      result.search_ms,
      scan_cloud->size(),
      precision_layer_used ? "used" : "idle",
      default_reject_streak_,
      precision_attempts_remaining_);
  }

  TemporalFrameInput make_temporal_frame(
    const sensor_msgs::msg::PointCloud2 & source_msg,
    const BbsResult & result,
    const nav_msgs::msg::Odometry & odom) const
  {
    // 在线窗口只使用当前和历史帧，因此 selected_candidate_rank 暂时表示“单帧 top-1”。
    // recovery pose 只允许发布 selected/top-1 自身也稳定的候选；best_seed_rank 只作为审计字段。
    TemporalFrameInput frame;
    frame.map_path = config_.input.map_path;
    frame.bag_path = "runtime";
    frame.stamp_sec = stamp_to_sec(source_msg.header.stamp);
    frame.refine_method = RefineMethod::None;
    frame.input_mode = config_.input.mode;
    frame.localized = result.localized;
    frame.selected_candidate_rank = result.candidates.empty() ? 0 : 1;
    frame.has_odom_pose = true;
    frame.odom_to_base = raw_odom_pose_to_base_axis_pose(pose_to_matrix(odom.pose.pose));
    frame.candidates.reserve(result.candidates.size());
    for (std::size_t i = 0; i < result.candidates.size(); ++i) {
      TemporalCandidateInput candidate;
      candidate.rank = static_cast<int>(i) + 1;
      candidate.score = result.candidates[i].score;
      candidate.score_ratio = result.candidates[i].score_ratio;
      candidate.map_to_base = result.candidates[i].pose;
      frame.candidates.push_back(candidate);
    }
    return frame;
  }

  std::optional<Eigen::Matrix4d> candidate_pose_by_rank(const BbsResult & result, int rank) const
  {
    if (rank <= 0 || rank > static_cast<int>(result.candidates.size())) {
      return std::nullopt;
    }
    return result.candidates[static_cast<std::size_t>(rank - 1)].pose;
  }

  int precision_acceptance_risk_score(
    const TemporalConsistencyResult & current,
    const RefineOutput & refined,
    const BbsResult & result) const
  {
    const auto & risk = config_.precision_recovery;
    int score = 0;

    if (current.selected_support_frames <= risk.bad_support_frames) {
      score += 2;
    } else if (current.selected_support_frames <= risk.weak_support_frames) {
      score += 1;
    }

    if (current.selected_rank >= risk.bad_selected_rank) {
      score += 2;
    } else if (current.selected_rank >= risk.weak_selected_rank) {
      score += 1;
    }

    if (refined.fitness_score >= risk.bad_refine_fitness) {
      score += 2;
    } else if (refined.fitness_score >= risk.weak_refine_fitness) {
      score += 1;
    }

    const auto selected_pose = candidate_pose_by_rank(result, current.selected_rank);
    const auto best_seed_pose = candidate_pose_by_rank(result, current.best_seed_rank);
    if (selected_pose && best_seed_pose) {
      const double seed_xy = pose_xy_distance(*selected_pose, *best_seed_pose);
      const double seed_yaw = pose_yaw_distance_deg(*selected_pose, *best_seed_pose);
      if (seed_xy >= risk.bad_seed_disagreement_xy_m) {
        score += 2;
      } else if (seed_xy >= risk.weak_seed_disagreement_xy_m) {
        score += 1;
      }
      if (seed_yaw >= risk.bad_seed_disagreement_yaw_deg) {
        score += 2;
      } else if (seed_yaw >= risk.weak_seed_disagreement_yaw_deg) {
        score += 1;
      }
    }

    return score;
  }

  bool should_review_with_precision_layer(
    const TemporalConsistencyResult & current,
    const RefineOutput & refined,
    const BbsResult & result,
    bool precision_layer_used,
    int & risk_score) const
  {
    risk_score = precision_acceptance_risk_score(current, refined, result);
    return precision_layer_available() &&
           config_.precision_recovery.trigger_on_weak_accept &&
           !precision_layer_used &&
           risk_score >= std::max(1, config_.precision_recovery.trigger_risk_score);
  }

  bool update_temporal_verifier(
    const sensor_msgs::msg::PointCloud2 & source_msg,
    const Cloud & scan_cloud,
    const BbsResult & result,
    const std::optional<nav_msgs::msg::Odometry> & synced_odom,
    bool precision_layer_used)
  {
    if (!config_.temporal.enable || !result.localized || result.candidates.empty()) {
      return false;
    }
    if (!synced_odom) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "temporal verifier skipped because synced odom is unavailable");
      return false;
    }

    temporal_window_.push_back(make_temporal_frame(source_msg, result, *synced_odom));
    scan_window_.push_back(CloudPtr(new Cloud(scan_cloud)));
    const int max_history = std::max(1, config_.temporal.online_max_history_frames);
    while (static_cast<int>(temporal_window_.size()) > max_history) {
      temporal_window_.pop_front();
      scan_window_.pop_front();
    }

    std::vector<TemporalFrameInput> frames(temporal_window_.begin(), temporal_window_.end());
    TemporalConsistencyConfig online_config = config_.temporal;
    online_config.window_after = 0;
    const auto results = analyze_temporal_consistency(frames, online_config);
    if (results.empty()) {
      return false;
    }

    const auto current_stamp = stamp_to_sec(source_msg.header.stamp);
    const auto current = std::find_if(
      results.rbegin(),
      results.rend(),
      [current_stamp](const TemporalConsistencyResult & item) {
        return std::abs(item.stamp_sec - current_stamp) < 1e-6;
      });
    if (current == results.rend()) {
      return false;
    }

    if (current->selected_support_frames < std::max(1, config_.temporal.online_min_support_frames)) {
      const auto trajectory_attempt =
        try_publish_trajectory_recovery(source_msg, result, *synced_odom, std::nullopt);
      if (trajectory_attempt.published) {
        return true;
      }
      const bool single_frame_published =
        try_publish_single_frame_high_confidence(source_msg, scan_cloud, result, *synced_odom, *current);
      if (single_frame_published) {
        return true;
      }
      publish_recovery_status(
        source_msg.header.stamp,
        trajectory_attempt.attempted ? trajectory_attempt.reject_state : "reject_selected_support_below_threshold",
        current->best_seed_rank,
        current->best_support_frames,
        current->selected_rank,
        current->selected_support_frames,
        0.0,
        0.0,
        0.0,
        false,
        0.0,
        &trajectory_attempt);
      RCLCPP_INFO(
        get_logger(),
        "temporal selected candidate not accepted yet: best_rank=%d best_support=%d selected_rank=%d selected_support=%d/%d",
        current->best_seed_rank,
        current->best_support_frames,
        current->selected_rank,
        current->selected_support_frames,
        config_.temporal.online_min_support_frames);
      return false;
    }

    const auto coarse_pose = candidate_pose_by_rank(result, current->selected_rank);
    if (!coarse_pose) {
      return false;
    }

    BbsCandidate seed_candidate;
    seed_candidate.pose = *coarse_pose;
    seed_candidate.score = result.candidates[static_cast<std::size_t>(current->selected_rank - 1)].score;
    seed_candidate.score_ratio = result.candidates[static_cast<std::size_t>(current->selected_rank - 1)].score_ratio;
    CloudPtr scan_ptr(new Cloud(scan_cloud));
    const RefineOutput refined =
      refine_single_candidate(map_cloud_, scan_ptr, seed_candidate, current->selected_rank, config_.refine);

    const auto & selected_source =
      result.candidates[static_cast<std::size_t>(current->selected_rank - 1)];
    const double fitness_limit =
      config_.temporal.solid_primary_relaxed_gate_enable && selected_source.solid_primary ?
      config_.temporal.solid_primary_max_refine_fitness :
      config_.temporal.online_max_refine_fitness;

    if (!refined.converged || refined.fitness_score > fitness_limit) {
      const auto trajectory_attempt =
        try_publish_trajectory_recovery(source_msg, result, *synced_odom, refined);
      if (trajectory_attempt.published) {
        return true;
      }
      publish_recovery_status(
        source_msg.header.stamp,
        trajectory_attempt.attempted ? trajectory_attempt.reject_state :
        (refined.converged ? "reject_refine_fitness_above_threshold" : "reject_refine_not_converged"),
        current->best_seed_rank,
        current->best_support_frames,
        current->selected_rank,
        current->selected_support_frames,
        0.0,
        0.0,
        0.0,
        refined.converged,
        refined.fitness_score,
        &trajectory_attempt);
      RCLCPP_INFO(
        get_logger(),
        "temporal selected candidate rejected by refine quality: selected_rank=%d selected_support=%d fitness=%.6f threshold=%.6f converged=%s",
        current->selected_rank,
        current->selected_support_frames,
        refined.fitness_score,
        fitness_limit,
        refined.converged ? "true" : "false");
      return false;
    }

    const Eigen::Matrix4d log_publish_pose =
      constrain_to_2d_if_needed(refined.converged ? refined.pose : *coarse_pose, config_.frames);
    const Eigen::Matrix4d log_odom_to_base =
      raw_odom_pose_to_base_axis_pose(pose_to_matrix(synced_odom->pose.pose));
    const Eigen::Matrix4d log_map_to_odom =
      constrain_to_2d_if_needed(log_publish_pose * log_odom_to_base.inverse(), config_.frames);

    int acceptance_risk_score = 0;
    if (should_review_with_precision_layer(
        *current, refined, result, precision_layer_used, acceptance_risk_score))
    {
      arm_precision_layer_attempts();
      publish_recovery_status(
        source_msg.header.stamp,
        "reject_precision_layer_risk_review_required",
        current->best_seed_rank,
        current->best_support_frames,
        current->selected_rank,
        current->selected_support_frames,
        log_map_to_odom(0, 3),
        log_map_to_odom(1, 3),
        yaw_from_matrix(log_map_to_odom) * 180.0 / M_PI,
        refined.converged,
        refined.fitness_score);
      RCLCPP_INFO(
        get_logger(),
        "temporal selected candidate deferred for precision review: risk=%d threshold=%d selected_rank=%d selected_support=%d best_rank=%d best_support=%d fitness=%.6f",
        acceptance_risk_score,
        std::max(1, config_.precision_recovery.trigger_risk_score),
        current->selected_rank,
        current->selected_support_frames,
        current->best_seed_rank,
        current->best_support_frames,
        refined.fitness_score);
      return false;
    }

    publish_recovery_outputs(
      source_msg,
      *synced_odom,
      "accepted",
      current->best_seed_rank,
      current->best_support_frames,
      current->selected_rank,
      current->selected_support_frames,
      log_publish_pose,
      refined);

    RCLCPP_INFO(
      get_logger(),
      "temporal selected candidate accepted: selected_rank=%d selected_support=%d best_rank=%d best_support=%d map_odom=(%.3f, %.3f, %.2fdeg) refine=%s converged=%s fitness=%.6f refine_ms=%.3f",
      current->selected_rank,
      current->selected_support_frames,
      current->best_seed_rank,
      current->best_support_frames,
      log_map_to_odom(0, 3),
      log_map_to_odom(1, 3),
      yaw_from_matrix(log_map_to_odom) * 180.0 / M_PI,
      to_string(config_.refine.method).c_str(),
      refined.converged ? "true" : "false",
      refined.fitness_score,
      refined.elapsed_ms);
    return true;
  }

  bool try_publish_single_frame_high_confidence(
    const sensor_msgs::msg::PointCloud2 & source_msg,
    const Cloud & scan_cloud,
    const BbsResult & result,
    const nav_msgs::msg::Odometry & synced_odom,
    const TemporalConsistencyResult & current)
  {
    if (!config_.temporal.single_frame_high_confidence_fallback_enable || !result.localized ||
      result.candidates.empty())
    {
      return false;
    }

    const auto coarse_pose = candidate_pose_by_rank(result, current.selected_rank);
    if (!coarse_pose) {
      return false;
    }

    BbsCandidate seed_candidate;
    seed_candidate.pose = *coarse_pose;
    seed_candidate.score = result.candidates[static_cast<std::size_t>(current.selected_rank - 1)].score;
    seed_candidate.score_ratio = result.candidates[static_cast<std::size_t>(current.selected_rank - 1)].score_ratio;
    CloudPtr scan_ptr(new Cloud(scan_cloud));
    RefineOutput refined =
      refine_single_candidate(map_cloud_, scan_ptr, seed_candidate, current.selected_rank, config_.refine);
    const bool high_confidence_ok =
      config_.temporal.single_frame_high_confidence_fallback_enable && refined.converged &&
      refined.fitness_score <= config_.temporal.single_frame_high_confidence_max_fitness;
    if (!high_confidence_ok)
    {
      return false;
    }

    const Eigen::Matrix4d publish_pose =
      constrain_to_2d_if_needed(refined.pose, config_.frames);
    publish_recovery_outputs(
      source_msg,
      synced_odom,
      "accepted_single_frame_high_confidence",
      current.best_seed_rank,
      current.best_support_frames,
      current.selected_rank,
      current.selected_support_frames,
      publish_pose,
      refined);

    RCLCPP_INFO(
      get_logger(),
      "single-frame high-confidence fallback accepted: selected_rank=%d selected_support=%d/%d fitness=%.6f threshold=%.6f",
      current.selected_rank,
      current.selected_support_frames,
      config_.temporal.online_min_support_frames,
      refined.fitness_score,
      config_.temporal.single_frame_high_confidence_max_fitness);
    return true;
  }

  struct OnlineLikelihood
  {
    int source_frame_offset{0};
    int source_candidate_rank{0};
    int seed_rank{0};
    int score{0};
    double score_ratio{0.0};
    Eigen::Matrix4d current_map_to_base{Eigen::Matrix4d::Identity()};
    double average_overlap{0.0};
    int support_frames{0};
  };

  struct TrajectoryRecoveryAttempt
  {
    bool attempted{false};
    bool published{false};
    std::string reject_state{"trajectory_not_attempted"};
    std::string hint{"none"};
    int seed_rank{0};
    int candidate_rank{0};
    int support_frames{0};
    int min_support_frames{0};
    double average_overlap{0.0};
    double margin{0.0};
    double single_agreement_xy{std::numeric_limits<double>::infinity()};
    double single_agreement_yaw{std::numeric_limits<double>::infinity()};
    bool single_refined{false};
    bool single_refine_converged{false};
    double single_refine_fitness{0.0};
  };

  TrajectoryRecoveryAttempt make_trajectory_reject_attempt(
    const OnlineLikelihood & best,
    double margin,
    double single_agreement_xy,
    double single_agreement_yaw,
    const std::optional<RefineOutput> & single_refine,
    const std::string & state,
    const std::string & hint,
    int min_support) const
  {
    // 将 trajectory recovery 的拒绝原因整理成结构化字段，后面统一写入 recovery_status。
    // 这样上层状态机可以区分“再等几帧历史”和“需要主动采集新视角”。
    TrajectoryRecoveryAttempt attempt;
    attempt.attempted = true;
    attempt.published = false;
    attempt.reject_state = state;
    attempt.hint = hint;
    attempt.seed_rank = best.seed_rank;
    attempt.candidate_rank = best.source_candidate_rank;
    attempt.support_frames = best.support_frames;
    attempt.min_support_frames = min_support;
    attempt.average_overlap = best.average_overlap;
    attempt.margin = margin;
    attempt.single_agreement_xy = single_agreement_xy;
    attempt.single_agreement_yaw = single_agreement_yaw;
    attempt.single_refined = static_cast<bool>(single_refine);
    attempt.single_refine_converged = single_refine && single_refine->converged;
    attempt.single_refine_fitness = single_refine ? single_refine->fitness_score : 0.0;
    return attempt;
  }

  TrajectoryRecoveryAttempt try_publish_trajectory_recovery(
    const sensor_msgs::msg::PointCloud2 & source_msg,
    const BbsResult & current_result,
    const nav_msgs::msg::Odometry & synced_odom,
    const std::optional<RefineOutput> & existing_single_refine)
  {
    if (!config_.temporal.enable_trajectory_likelihood || map_occupancy_.empty() ||
      temporal_window_.empty() || scan_window_.size() != temporal_window_.size())
    {
      return TrajectoryRecoveryAttempt{};
    }

    const auto likelihoods = compute_online_trajectory_likelihoods();
    if (likelihoods.empty()) {
      TrajectoryRecoveryAttempt attempt;
      attempt.attempted = true;
      attempt.reject_state = "reject_trajectory_no_candidate";
      attempt.hint = "wait_more_history";
      attempt.min_support_frames = std::max(1, config_.temporal.online_min_support_frames);
      return attempt;
    }

    const auto & best = likelihoods.front();
    const double best_overlap = best.average_overlap;
    double second_overlap = 0.0;
    for (std::size_t i = 1; i < likelihoods.size(); ++i) {
      const auto & other = likelihoods[i];
      const double dx = best.current_map_to_base(0, 3) - other.current_map_to_base(0, 3);
      const double dy = best.current_map_to_base(1, 3) - other.current_map_to_base(1, 3);
      const double xy = std::hypot(dx, dy);
      const double yaw_deg =
        std::abs(normalized_angle(yaw_from_matrix(best.current_map_to_base) - yaw_from_matrix(other.current_map_to_base))) *
        180.0 / M_PI;
      if (xy > config_.temporal.xy_gate_m || yaw_deg > config_.temporal.yaw_gate_deg) {
        second_overlap = other.average_overlap;
        break;
      }
    }
    const double margin = best_overlap - second_overlap;
    const int min_support = std::max(1, config_.temporal.online_min_support_frames);
    const bool strict_accept =
      best.support_frames >= min_support &&
      best_overlap >= config_.temporal.trajectory_min_overlap_ratio &&
      best_overlap >= config_.temporal.trajectory_min_average_overlap &&
      margin >= config_.temporal.trajectory_min_margin;

    std::optional<RefineOutput> single_refine = existing_single_refine;
    if (!single_refine && current_result.localized && !current_result.candidates.empty()) {
      CloudPtr current_scan = scan_window_.empty() ? CloudPtr() : scan_window_.back();
      BbsCandidate top1;
      top1.pose = current_result.candidates.front().pose;
      top1.score = current_result.candidates.front().score;
      top1.score_ratio = current_result.candidates.front().score_ratio;
      single_refine = refine_single_candidate(map_cloud_, current_scan, top1, 1, config_.refine);
    }

    double single_agreement_xy = std::numeric_limits<double>::infinity();
    double single_agreement_yaw = std::numeric_limits<double>::infinity();
    Eigen::Matrix4d single_pose = Eigen::Matrix4d::Identity();
    if (single_refine && single_refine->converged) {
      single_pose = constrain_to_2d_if_needed(single_refine->pose, config_.frames);
      const double dx = best.current_map_to_base(0, 3) - single_pose(0, 3);
      const double dy = best.current_map_to_base(1, 3) - single_pose(1, 3);
      single_agreement_xy = std::hypot(dx, dy);
      single_agreement_yaw =
        std::abs(normalized_angle(yaw_from_matrix(best.current_map_to_base) - yaw_from_matrix(single_pose))) *
        180.0 / M_PI;
    }

    const bool single_agreement_accept =
      config_.temporal.trajectory_single_agreement_fallback_enable &&
      !strict_accept &&
      single_refine &&
      single_refine->converged &&
      single_refine->fitness_score <= config_.temporal.trajectory_single_agreement_max_fitness &&
      best.support_frames >= min_support &&
      best_overlap >= config_.temporal.trajectory_single_agreement_min_overlap &&
      margin >= config_.temporal.trajectory_single_agreement_min_margin &&
      single_agreement_xy <= config_.temporal.trajectory_single_agreement_max_xy_m &&
      single_agreement_yaw <= config_.temporal.trajectory_single_agreement_max_yaw_deg;

    if (!strict_accept && !single_agreement_accept) {
      std::string reject_state = "need_active_view_single_agreement_failed";
      std::string hint = "active_view";
      if (best.support_frames < min_support) {
        reject_state = "reject_trajectory_support_below_threshold";
        hint = "wait_more_history";
      } else if (best_overlap < config_.temporal.trajectory_single_agreement_min_overlap) {
        reject_state = "need_active_view_overlap_below_threshold";
        hint = "active_view";
      } else if (margin < config_.temporal.trajectory_single_agreement_min_margin) {
        reject_state = "need_active_view_margin_below_threshold";
        hint = "active_view";
      } else if (!single_refine || !single_refine->converged) {
        reject_state = "need_active_view_single_refine_unavailable";
        hint = "active_view";
      } else if (single_refine->fitness_score > config_.temporal.trajectory_single_agreement_max_fitness) {
        reject_state = "need_active_view_single_fitness_above_threshold";
        hint = "active_view";
      }
      RCLCPP_INFO(
        get_logger(),
        "trajectory recovery candidate rejected: state=%s support=%d/%d overlap=%.6f margin=%.6f single_agree=(%.3fm, %.3fdeg)",
        reject_state.c_str(),
        best.support_frames,
        min_support,
        best_overlap,
        margin,
        single_agreement_xy,
        single_agreement_yaw);
      return make_trajectory_reject_attempt(
        best,
        margin,
        single_agreement_xy,
        single_agreement_yaw,
        single_refine,
        reject_state,
        hint,
        min_support);
    }

    RefineOutput publish_refine;
    Eigen::Matrix4d publish_pose = best.current_map_to_base;
    std::string state = "accepted_trajectory";
    if (single_agreement_accept && single_refine) {
      publish_refine = *single_refine;
      publish_pose = single_refine->pose;
      state = "accepted_trajectory_single_agreement";
    } else {
      BbsCandidate seed;
      seed.pose = best.current_map_to_base;
      seed.score = best.score;
      seed.score_ratio = best.score_ratio;
      publish_refine = refine_single_candidate(map_cloud_, scan_window_.back(), seed, best.source_candidate_rank, config_.refine);
      publish_pose = publish_refine.converged ? publish_refine.pose : best.current_map_to_base;
      if (!publish_refine.converged || publish_refine.fitness_score > config_.temporal.online_max_refine_fitness) {
        RCLCPP_INFO(
          get_logger(),
          "trajectory recovery rejected by refine quality: converged=%s fitness=%.6f threshold=%.6f",
          publish_refine.converged ? "true" : "false",
          publish_refine.fitness_score,
          config_.temporal.online_max_refine_fitness);
        return make_trajectory_reject_attempt(
          best,
          margin,
          single_agreement_xy,
          single_agreement_yaw,
          single_refine,
          publish_refine.converged ? "need_active_view_trajectory_refine_fitness_above_threshold" :
          "need_active_view_trajectory_refine_not_converged",
          "active_view",
          min_support);
      }
    }

    publish_recovery_outputs(
      source_msg,
      synced_odom,
      state,
      best.seed_rank,
      best.support_frames,
      best.source_candidate_rank,
      best.support_frames,
      publish_pose,
      publish_refine);

    RCLCPP_INFO(
      get_logger(),
      "trajectory recovery accepted: state=%s seed=%d rank=%d support=%d overlap=%.6f margin=%.6f single_agree=(%.3fm, %.3fdeg)",
      state.c_str(),
      best.seed_rank,
      best.source_candidate_rank,
      best.support_frames,
      best_overlap,
      margin,
      single_agreement_xy,
      single_agreement_yaw);
    TrajectoryRecoveryAttempt attempt;
    attempt.attempted = true;
    attempt.published = true;
    attempt.reject_state = state;
    attempt.hint = "accepted";
    attempt.seed_rank = best.seed_rank;
    attempt.candidate_rank = best.source_candidate_rank;
    attempt.support_frames = best.support_frames;
    attempt.min_support_frames = min_support;
    attempt.average_overlap = best_overlap;
    attempt.margin = margin;
    attempt.single_agreement_xy = single_agreement_xy;
    attempt.single_agreement_yaw = single_agreement_yaw;
    attempt.single_refined = static_cast<bool>(single_refine);
    attempt.single_refine_converged = single_refine && single_refine->converged;
    attempt.single_refine_fitness = single_refine ? single_refine->fitness_score : publish_refine.fitness_score;
    return attempt;
  }

  std::vector<OnlineLikelihood> compute_online_trajectory_likelihoods() const
  {
    std::vector<OnlineLikelihood> likelihoods;
    if (temporal_window_.empty() || scan_window_.size() != temporal_window_.size()) {
      return likelihoods;
    }

    const int center_pos = static_cast<int>(temporal_window_.size()) - 1;
    const int begin = std::max(0, center_pos - std::max(0, config_.temporal.window_before));
    const int end = center_pos + 1;
    const auto & center = temporal_window_.back();
    const int candidate_count = center.candidates.empty() ? 0 : std::min<int>(
      std::max(1, config_.temporal.trajectory_max_candidates),
      static_cast<int>(center.candidates.size()));
    if (candidate_count <= 0) {
      return likelihoods;
    }

    struct AcceptedSeed
    {
      Eigen::Matrix4d map_to_odom{Eigen::Matrix4d::Identity()};
    };
    std::vector<AcceptedSeed> accepted_seeds;
    int seed_rank = 0;
    for (int source_pos = begin; source_pos < end; ++source_pos) {
      const auto & source = temporal_window_[static_cast<std::size_t>(source_pos)];
      const int source_candidate_count = std::min<int>(
        candidate_count,
        static_cast<int>(source.candidates.size()));
      for (int candidate_index = 0; candidate_index < source_candidate_count; ++candidate_index) {
        const auto & candidate = source.candidates[static_cast<std::size_t>(candidate_index)];
        const Eigen::Matrix4d map_to_odom = candidate.map_to_base * source.odom_to_base.inverse();
        bool duplicate = false;
        for (const auto & accepted : accepted_seeds) {
          const double dx = map_to_odom(0, 3) - accepted.map_to_odom(0, 3);
          const double dy = map_to_odom(1, 3) - accepted.map_to_odom(1, 3);
          const double xy = std::hypot(dx, dy);
          const double yaw_deg =
            std::abs(normalized_angle(yaw_from_matrix(map_to_odom) - yaw_from_matrix(accepted.map_to_odom))) *
            180.0 / M_PI;
          if (xy <= 0.25 * config_.temporal.xy_gate_m && yaw_deg <= 0.25 * config_.temporal.yaw_gate_deg) {
            duplicate = true;
            break;
          }
        }
        if (duplicate) {
          continue;
        }
        accepted_seeds.push_back(AcceptedSeed{map_to_odom});

        double overlap_sum = 0.0;
        int support_frames = 0;
        int scored_frames = 0;
        for (int frame_pos = begin; frame_pos < end; ++frame_pos) {
          const auto & frame = temporal_window_[static_cast<std::size_t>(frame_pos)];
          const Eigen::Matrix4d predicted_map_to_base =
            constrain_to_2d_if_needed(map_to_odom * frame.odom_to_base, config_.frames);
          const double overlap = overlap_ratio_for_online_pose(
            scan_window_[static_cast<std::size_t>(frame_pos)],
            predicted_map_to_base,
            map_occupancy_,
            std::max(0.05, config_.temporal.trajectory_voxel_size),
            config_.temporal.trajectory_neighbor_radius);
          overlap_sum += overlap;
          ++scored_frames;
          if (overlap >= config_.temporal.trajectory_min_overlap_ratio) {
            ++support_frames;
          }
        }

        OnlineLikelihood item;
        item.source_frame_offset = source_pos - center_pos;
        item.source_candidate_rank = candidate.rank;
        item.seed_rank = ++seed_rank;
        item.score = candidate.score;
        item.score_ratio = candidate.score_ratio;
        item.current_map_to_base =
          constrain_to_2d_if_needed(map_to_odom * center.odom_to_base, config_.frames);
        item.average_overlap = scored_frames > 0 ? overlap_sum / static_cast<double>(scored_frames) : 0.0;
        item.support_frames = support_frames;
        likelihoods.push_back(item);
      }
    }

    std::sort(
      likelihoods.begin(),
      likelihoods.end(),
      [](const OnlineLikelihood & lhs, const OnlineLikelihood & rhs) {
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
    return likelihoods;
  }

  void publish_recovery_outputs(
    const sensor_msgs::msg::PointCloud2 & source_msg,
    const nav_msgs::msg::Odometry & synced_odom,
    const std::string & state,
    int best_rank,
    int best_support,
    int selected_rank,
    int selected_support,
    const Eigen::Matrix4d & map_to_base_pose,
    const RefineOutput & refined)
  {
    const Eigen::Matrix4d publish_pose = constrain_to_2d_if_needed(map_to_base_pose, config_.frames);
    const Eigen::Matrix4d odom_to_base =
      raw_odom_pose_to_base_axis_pose(pose_to_matrix(synced_odom.pose.pose));
    const Eigen::Matrix4d map_to_odom =
      constrain_to_2d_if_needed(publish_pose * odom_to_base.inverse(), config_.frames);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = source_msg.header.stamp;
    pose_msg.header.frame_id = "map";
    pose_msg.pose = matrix_to_pose(publish_pose);
    recovery_pose_pub_->publish(pose_msg);

    geometry_msgs::msg::PoseStamped map_odom_msg;
    map_odom_msg.header.stamp = source_msg.header.stamp;
    map_odom_msg.header.frame_id = "map";
    map_odom_msg.pose = matrix_to_pose(map_to_odom);
    recovery_map_odom_pub_->publish(map_odom_msg);

    publish_recovery_status(
      source_msg.header.stamp,
      state,
      best_rank,
      best_support,
      selected_rank,
      selected_support,
      map_to_odom(0, 3),
      map_to_odom(1, 3),
      yaw_from_matrix(map_to_odom) * 180.0 / M_PI,
      refined.converged,
      refined.fitness_score);
  }

  void publish_recovery_status(
    const builtin_interfaces::msg::Time & stamp,
    const std::string & state,
    int best_rank,
    int best_support,
    int selected_rank,
    int selected_support,
    double map_odom_x,
    double map_odom_y,
    double map_odom_yaw_deg,
    bool refined_converged,
    double refined_fitness,
    const TrajectoryRecoveryAttempt * trajectory_attempt = nullptr)
  {
    // 所有恢复状态使用同一 JSON 契约；位姿应用仍由上层恢复协调器负责。
    std_msgs::msg::String status;
    std::ostringstream text;
    text << "{\"protocol_version\":1"
         << ",\"event_type\":\"global_relocalization_result\""
         << ",\"attempt_id\":\"" << json_escape(active_attempt_id_) << "\""
         << ",\"map_id\":\"" << json_escape(active_map_id_) << "\""
         << ",\"stamp_sec\":" << json_number(stamp_to_sec(stamp))
         << ",\"state\":\"" << json_escape(state) << "\""
         << ",\"input_mode\":\"" << json_escape(to_string(config_.input.mode)) << "\""
         << ",\"best_rank\":" << best_rank
         << ",\"best_support\":" << best_support
         << ",\"selected_rank\":" << selected_rank
         << ",\"selected_support\":" << selected_support
         << ",\"min_support\":" << config_.temporal.online_min_support_frames
         << ",\"map_odom_x\":" << json_number(map_odom_x)
         << ",\"map_odom_y\":" << json_number(map_odom_y)
         << ",\"map_odom_yaw_deg\":" << json_number(map_odom_yaw_deg)
         << ",\"refined_converged\":" << (refined_converged ? "true" : "false")
         << ",\"refined_fitness\":" << json_number(refined_fitness)
         << ",\"precision_layer_available\":" << (precision_layer_available() ? "true" : "false")
         << ",\"default_reject_streak\":" << default_reject_streak_
         << ",\"precision_attempts_remaining\":" << precision_attempts_remaining_;
    if (trajectory_attempt && trajectory_attempt->attempted) {
      text << ",\"trajectory_attempted\":true"
           << ",\"recovery_hint\":\"" << json_escape(trajectory_attempt->hint) << "\""
           << ",\"trajectory_seed\":" << trajectory_attempt->seed_rank
           << ",\"trajectory_rank\":" << trajectory_attempt->candidate_rank
           << ",\"trajectory_support\":" << trajectory_attempt->support_frames
           << ",\"trajectory_min_support\":" << trajectory_attempt->min_support_frames
           << ",\"trajectory_average_overlap\":" << json_number(trajectory_attempt->average_overlap)
           << ",\"trajectory_margin\":" << json_number(trajectory_attempt->margin)
           << ",\"trajectory_single_agree_xy\":" << json_number(trajectory_attempt->single_agreement_xy)
           << ",\"trajectory_single_agree_yaw\":" << json_number(trajectory_attempt->single_agreement_yaw)
           << ",\"trajectory_single_refined\":" << (trajectory_attempt->single_refined ? "true" : "false")
           << ",\"trajectory_single_converged\":" << (trajectory_attempt->single_refine_converged ? "true" : "false")
           << ",\"trajectory_single_fitness\":" << json_number(trajectory_attempt->single_refine_fitness);
    } else {
      text << ",\"trajectory_attempted\":false,\"recovery_hint\":\"none\"";
    }
    text << "}";
    status.data = text.str();
    recovery_status_pub_->publish(status);
  }

  void publish_runtime_outputs(
    const sensor_msgs::msg::PointCloud2 & source_msg,
    const Cloud & scan_cloud,
    const BbsResult & result)
  {
    // PoseArray 适合 RViz 中直接显示坐标轴；MarkerArray 额外显示 rank/score 文本。
    geometry_msgs::msg::PoseArray poses;
    poses.header.stamp = source_msg.header.stamp;
    poses.header.frame_id = "map";

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header = poses.header;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);

    for (std::size_t i = 0; i < result.candidates.size(); ++i) {
      const auto pose_matrix = constrain_to_2d_if_needed(result.candidates[i].pose, config_.frames);
      const auto pose_msg = matrix_to_pose(pose_matrix);
      poses.poses.push_back(pose_msg);

      visualization_msgs::msg::Marker arrow;
      arrow.header = poses.header;
      arrow.ns = "bbs_candidates";
      arrow.id = static_cast<int>(i);
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;
      arrow.pose = pose_msg;
      arrow.scale.x = 0.8;
      arrow.scale.y = 0.08;
      arrow.scale.z = 0.08;
      arrow.color.a = 0.9;
      arrow.color.r = i == 0 ? 0.1 : 1.0;
      arrow.color.g = i == 0 ? 0.9 : 0.7;
      arrow.color.b = i == 0 ? 0.2 : 0.1;
      markers.markers.push_back(arrow);

      std::ostringstream label;
      label << "#" << (i + 1) << " score=" << result.candidates[i].score;
      visualization_msgs::msg::Marker text;
      text.header = poses.header;
      text.ns = "bbs_candidate_scores";
      text.id = static_cast<int>(1000 + i);
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose = pose_msg;
      text.pose.position.z += 0.6;
      text.scale.z = 0.35;
      text.color.a = 0.95;
      text.color.r = 1.0;
      text.color.g = 1.0;
      text.color.b = 1.0;
      text.text = label.str();
      markers.markers.push_back(text);
    }

    candidate_pub_->publish(poses);
    marker_pub_->publish(markers);

    if (result.localized && !result.candidates.empty()) {
      const auto best_pose = constrain_to_2d_if_needed(result.candidates.front().pose, config_.frames);
      CloudPtr scan_ptr(new Cloud(scan_cloud));
      CloudPtr aligned = transform_cloud(scan_ptr, best_pose);
      sensor_msgs::msg::PointCloud2 aligned_msg;
      pcl::toROSMsg(*aligned, aligned_msg);
      aligned_msg.header = poses.header;
      aligned_cloud_pub_->publish(aligned_msg);
    }
  }

  std::string config_file_;
  RuntimeConfig config_;
  CloudPtr map_cloud_;
  OccupancySet map_occupancy_;
  std::vector<ScanContextEntry> scan_context_database_;
  std::vector<SolidEntry> solid_database_;
  std::vector<ScanContextEntry> precision_scan_context_database_;
  std::unique_ptr<SimpleBbs3d> bbs_;
  std::unique_ptr<Bbs2dSearch> bbs2d_;
  rclcpp::Time last_search_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_precision_layer_time_{0, 0, RCL_ROS_TIME};
  int default_reject_streak_{0};
  int precision_attempts_remaining_{0};
  bool search_active_{false};
  std::string active_attempt_id_{"continuous"};
  std::string active_map_id_{"unassigned"};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr request_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr candidate_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr recovery_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr recovery_map_odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr recovery_status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_cloud_pub_;
  std::deque<nav_msgs::msg::Odometry> odom_buffer_;
  std::deque<TemporalFrameInput> temporal_window_;
  std::deque<CloudPtr> scan_window_;
};

}  // namespace humanoid_global_relocalization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<humanoid_global_relocalization::GlobalRelocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
