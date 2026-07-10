/*
 * route_runtime_types.hpp
 *
 * 文件用途：
 * 1. 集中定义路线任务运行层的通用配置、枚举和运行态数据结构。
 * 2. 让 ROS 节点外壳、启动门控、路线任务状态机、障碍等待模块共用同一套字段含义。
 * 3. 避免把大量业务状态散落在 navigation_state_manager.cpp 里，后续排查 CPU/内存和功能问题时更容易定位。
 *
 * 代码块顺序：
 * 1. 通用小工具：时间、字符串、频率保护和速度死区。
 * 2. 导航状态枚举：对外发布 current_state 时使用。
 * 3. 参数结构：所有 YAML 可调参数在这里集中声明默认值。
 * 4. 运行态结构：按数据链路拆成机器人、定位、地图、点位库、路线任务和环境输入状态。
 */

#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace humanoid_route_runtime
{

inline double now_seconds()
{
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

inline std::string trim_copy(const std::string & input)
{
  const auto begin = input.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = input.find_last_not_of(" \t\r\n");
  return input.substr(begin, end - begin + 1);
}

inline double safe_rate(const double rate_hz)
{
  return rate_hz > 0.01 ? rate_hz : 1.0;
}

inline std::chrono::milliseconds to_duration(
  const double seconds,
  const std::chrono::milliseconds fallback)
{
  const auto millis = static_cast<int64_t>(seconds * 1000.0);
  return millis > 0 ? std::chrono::milliseconds(millis) : fallback;
}

inline double deadzone(const double value, const double threshold = 0.01)
{
  return std::abs(value) < threshold ? 0.0 : value;
}

enum class NavigationState
{
  Idle,
  Planning,
  Executing,
  Paused,
  ReachedWaypoint,
  Completed,
  Failed,
  Cancelled,
};

inline std::string navigation_state_to_string(const NavigationState state)
{
  switch (state) {
    case NavigationState::Idle:
      return "idle";
    case NavigationState::Planning:
      return "planning";
    case NavigationState::Executing:
      return "executing";
    case NavigationState::Paused:
      return "paused";
    case NavigationState::ReachedWaypoint:
      return "reached_waypoint";
    case NavigationState::Completed:
      return "completed";
    case NavigationState::Failed:
      return "failed";
    case NavigationState::Cancelled:
      return "cancelled";
  }
  return "idle";
}

struct RouteRuntimeConfig
{
  // 基础到点和状态发布参数。
  double position_tolerance{0.15};
  double orientation_tolerance{0.3};
  double status_publish_rate{1.0};
  std::string default_frame_id{"map"};
  std::string map_frame{"map"};
  std::string base_frame{"base_footprint"};
  double pose_tf_timeout_sec{0.05};

  // 障碍阻塞和恢复参数。
  double obstacle_block_timeout{4.0};
  double velocity_threshold{0.10};
  double blockage_pose_delta_deadzone{0.10};
  double blockage_recovery_velocity_threshold{0.15};
  double blockage_recovery_confirm_sec{1.0};
  bool obstacle_wait_enable{true};
  double obstacle_wait_push_interval_sec{4.0};
  int obstacle_clear_required_frames{5};
  double obstacle_clear_check_rate_hz{5.0};
  int obstacle_clear_cost_threshold{100};
  double obstacle_clear_front_min_x_m{0.15};
  double obstacle_clear_front_max_x_m{0.80};
  double obstacle_clear_half_width_m{0.30};
  double obstacle_min_wait_before_resume_sec{2.0};
  double obstacle_clear_required_duration_sec{2.5};
  double obstacle_clear_required_duration_after_false_resume_sec{4.0};
  double obstacle_false_resume_window_sec{3.0};
  bool obstacle_resume_use_roi{true};
  std::string obstacle_roi_has_obstacle_topic{"/front_obstacle/has_obstacle"};
  double obstacle_roi_timeout_sec{1.0};
  int obstacle_roi_required_clear_frames{3};
  std::string local_costmap_topic{"/local_costmap/costmap"};
  bool obstacle_costmap_analyze_only_when_waiting{true};
  bool obstacle_costmap_window_bounded_scan{true};

  // 机器人、定位、地图和路线任务参数。
  bool require_walk_mode_for_navigation{true};
  double robot_status_timeout{2.0};
  double pending_navigation_timeout{90.0};
  double obstacle_block_near_goal_distance{0.7};
  int localization_resume_stable_frames{3};
  std::string localization_health_status_topic{"/localization/trust_status"};
  std::string map_status_topic{"/map/status"};
  double localization_health_timeout_sec{3.0};
  bool localization_allow_start_with_last_good_tf{false};
  double localization_last_good_tf_max_age_sec{0.0};
  double route_task_first_task_reached_tolerance_m{0.4};
  double route_task_transit_passed_tolerance_m{0.5};
  bool route_task_transit_projection_passed_enabled{true};
  double route_task_nav2_feedback_timeout_sec{3.0};
  double route_task_goal_cancel_timeout_sec{2.0};
  double route_task_goal_reject_retry_timeout_sec{8.0};
  bool route_task_default_interrupt_broadcast{true};
  bool route_task_nav2_execution_enable{true};
  std::string reverse_navigation_bt_xml{
    "/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_navigation2/config/behavior_tree/"
    "navigate_reverse_xy_then_yaw.xml"};

  // ROS 话题和 action 名称。
  std::string navigation_requests_topic{"/navigation/requests"};
  std::string navigation_waypoints_data_topic{"/navigation/waypoints_data"};
  std::string navigation_status_topic{"/navigation/status"};
  std::string navigation_acknowledgments_topic{"/navigation/acknowledgments"};
  std::string navigation_goal_topic{"/goal_pose"};
  std::string cmd_vel_topic{"/cmd_vel"};
  std::string odom_topic{"/odom"};
  std::string robot_status_topic{"/robot_status_raw"};
  std::string behavior_tree_log_topic{"/behavior_tree_log"};
  std::string navigate_to_pose_action{"navigate_to_pose"};
  std::string navigate_through_poses_action{"navigate_through_poses"};
};

struct RobotRuntimeState
{
  std::string control_state{"Unknown"};
  bool motion_busy{false};
  bool ready_for_navigation{false};
  std::string current_motion;
  double last_update{0.0};
};

struct LocalizationRuntimeState
{
  bool healthy{false};
  bool has_last_good_tf{false};
  int resume_stable_count{0};
  double last_good_tf_time{0.0};
  double last_status_time{0.0};
  std::string state{"UNKNOWN"};
  std::string text;
};

struct MapRuntimeState
{
  std::string active_map_id;
  std::string map_state{"unknown"};
  std::string localization_state{"unknown"};
  double last_update{0.0};
};

struct StoredWaypoint
{
  std::string waypoint_id;
  std::string waypoint_name;
  std::string type;
  std::string map_id;
  std::string frame_id;
  bool has_position{false};
  bool has_orientation{false};
  std::array<double, 3> position{0.0, 0.0, 0.0};
  std::array<double, 4> orientation{0.0, 0.0, 0.0, 1.0};
  std::string waypoint_role;
  bool need_broadcast{false};
  std::string broadcast_id;
  std::string broadcast_text;
  bool broadcast_blocking{true};
  bool stop_and_align{true};
  std::string walk_direction{"forward"};
};

struct WaypointCacheState
{
  std::string map_id;
  std::string revision;
  std::map<std::string, std::string> revisions_by_map;
  std::map<std::string, StoredWaypoint> waypoints;
  std::map<std::string, std::map<std::string, StoredWaypoint>> waypoints_by_map;
  std::size_t count{0};
  double last_update{0.0};
};

struct RouteWaypoint
{
  // route task 内部只使用归一化后的 waypoint_id，避免数字、空格、None 等输入造成比较不一致。
  std::string waypoint_id;
  std::string map_id;
  std::string waypoint_name;
  std::string waypoint_role;
  std::string frame_id;
  std::array<double, 3> position{0.0, 0.0, 0.0};
  std::array<double, 4> orientation{0.0, 0.0, 0.0, 1.0};
  int source_index{0};

  // 只有 task 点拥有播报/停靠/朝向收尾语义；transit 点会被归一化为全部关闭。
  bool need_broadcast{false};
  std::string broadcast_id;
  std::string broadcast_text;
  bool broadcast_blocking{false};
  bool stop_and_align{false};
  std::string walk_direction{"forward"};
};

struct ActiveRouteSegment
{
  std::string segment_id;
  std::string segment_direction{"forward"};
  std::string segment_start_task_id;
  std::string segment_target_task_id;
  int segment_start_source_index{-1};
  int segment_target_source_index{-1};
  std::vector<std::string> transit_waypoint_ids;
  std::vector<std::string> execution_waypoint_ids;
  std::vector<std::string> passed_transit_waypoint_ids;
  int current_segment_progress_index{0};
  int64_t segment_goal_generation{0};
};

struct RouteTaskRuntimeState
{
  NavigationState current_state{NavigationState::Idle};
  std::string detailed_state{"IDLE"};
  std::string navigation_mode;
  int current_waypoint_index{0};
  int total_waypoints{0};
  double navigation_start_time{0.0};
  bool pending_navigation_active{false};
  double pending_navigation_created_at{0.0};
  std::string pending_navigation_reason;
  std::string pending_navigation_request_json;
  int64_t event_counter{0};

  double pause_time{0.0};
  double pause_duration_limit{0.0};
  std::string pause_source;
  std::string pause_reason;
  std::string resume_mode;

  // route task 专属运行态。它们只服务新路线任务，不再兼容旧单点/多点 APP 命令。
  bool active{false};
  std::string task_session_id;
  std::string route_id;
  std::string map_id;
  std::string request_message_id;
  std::string route_waypoint_source;
  std::string waypoints_revision;
  double started_at{0.0};
  int64_t route_task_version{0};
  int64_t active_goal_generation{0};
  int64_t current_goal_generation{0};

  std::vector<RouteWaypoint> route_waypoints;
  std::vector<std::string> master_route_task_ids;
  std::vector<std::string> completed_task_ids;
  std::vector<std::string> skipped_task_ids;
  std::string current_anchor_task_id;
  int current_anchor_task_index{-1};
  std::string current_target_task_id;
  int current_target_task_index{-1};
  std::optional<ActiveRouteSegment> active_segment;

  bool awaiting_broadcast{false};
  std::string waiting_broadcast_waypoint_id;
  std::string waiting_broadcast_id;
  bool jump_interrupts_broadcast{false};
  std::string last_completed_task_id;
  std::string last_completed_broadcast_task_session_id;
  std::string last_completed_broadcast_waypoint_id;
  std::string last_completed_broadcast_id;
};

struct EnvironmentRuntimeState
{
  double last_pose_update{0.0};
  bool has_current_pose{false};
  std::array<double, 3> current_position{0.0, 0.0, 0.0};
  std::array<double, 4> current_orientation{0.0, 0.0, 0.0, 1.0};
  double velocity_linear_x{0.0};
  double velocity_linear_y{0.0};
  double velocity_angular_z{0.0};
  bool has_pose_derived_speed{false};
  double pose_derived_speed{0.0};
  double last_motion_pose_time{0.0};
  std::array<double, 3> last_motion_position{0.0, 0.0, 0.0};
  double latest_costmap_stamp{0.0};
  std::string latest_costmap_frame_id;
  uint32_t latest_costmap_width{0};
  uint32_t latest_costmap_height{0};
  double latest_costmap_resolution{0.0};
  double latest_costmap_origin_x{0.0};
  double latest_costmap_origin_y{0.0};
  std::vector<int8_t> latest_costmap_data;
  bool latest_roi_has_obstacle{false};
  double latest_roi_stamp{0.0};
  int roi_clear_confirm_count{0};
  std::size_t latest_bt_event_count{0};
};

}  // namespace humanoid_route_runtime
