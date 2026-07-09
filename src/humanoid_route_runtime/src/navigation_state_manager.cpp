/*
 * navigation_state_manager.cpp
 *
 * 文件用途：
 * 1. 承接 Test 工作区单地图导航状态管理逻辑，接收 /navigation/requests 中的 APP 导航命令。
 * 2. 维护导航状态机：空闲、规划、执行、暂停、完成、失败可恢复、取消。
 * 3. 调用 Nav2 NavigateToPose action 执行单点、多点、展厅点位导航。
 * 4. 监听机器人底层状态、prior-map 定位健康、局部代价地图、ROI 障碍、Nav2 行为树日志。
 * 5. 对外发布 /navigation/status 与 /navigation/acknowledgments，字段结构保持 Python 版语义。
 *
 * 上游节点：
 * - dynamic_waypoints_manager_cpp 或 dynamic_waypoints_manager：发布 /navigation/requests 和 /navigation/waypoints_data。
 * - prior_map_odom_bridge：发布 /localization/prior_map_odom_bridge_status。
 * - fast_lio/Nav2/底层网关：发布 /odom、/local_costmap/costmap、/robot_status_raw、/behavior_tree_log。
 *
 * 下游节点：
 * - Nav2 bt_navigator：接收 navigate_to_pose action goal。
 * - APP/数据整合链路：订阅 /navigation/status 与 /navigation/acknowledgments。
 * - 底盘控制链路：必要时通过 /cmd_vel 发布零速度保护。
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/msg/behavior_tree_log.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

namespace
{
using json = nlohmann::json;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

double now_seconds()
{
  using Clock = std::chrono::system_clock;
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

double yaw_from_pose(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

bool as_bool(const json & value)
{
  if (value.is_boolean()) {
    return value.get<bool>();
  }
  if (value.is_number()) {
    return value.get<double>() != 0.0;
  }
  if (value.is_string()) {
    auto text = value.get<std::string>();
    std::transform(text.begin(), text.end(), text.begin(), ::tolower);
    return text == "1" || text == "true" || text == "yes" || text == "y" || text == "on";
  }
  return false;
}

std::string json_string(const json & value)
{
  return value.dump(-1, ' ', false, nlohmann::json::error_handler_t::replace);
}

std::vector<double> json_number_array(const json & data, const std::string & key, const std::vector<double> & fallback)
{
  if (!data.contains(key) || !data.at(key).is_array()) {
    return fallback;
  }
  std::vector<double> values;
  for (const auto & item : data.at(key)) {
    if (item.is_number()) {
      values.push_back(item.get<double>());
    }
  }
  return values.empty() ? fallback : values;
}

std::string state_to_detail_default(const std::string & state)
{
  std::string text = state;
  std::transform(text.begin(), text.end(), text.begin(), ::toupper);
  return text;
}
}  // namespace

class NavigationStateManagerCpp : public rclcpp::Node
{
public:
  NavigationStateManagerCpp()
  : Node("navigation_state_manager"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameters();
    load_parameters();
    setup_communication();
    setup_timers();
    RCLCPP_INFO(get_logger(), "导航状态管理器 C++ 版启动完成 - Test 单地图协议");
  }

private:
  // ========================= 参数与状态初始化 =========================
  void declare_parameters()
  {
    declare_parameter<double>("position_tolerance", 0.15);
    declare_parameter<double>("orientation_tolerance", 0.3);
    declare_parameter<double>("waypoint_timeout", 300.0);
    declare_parameter<double>("status_publish_rate", 2.0);
    declare_parameter<std::string>("default_frame_id", "map");
    declare_parameter<double>("obstacle_block_timeout", 4.0);
    declare_parameter<double>("velocity_threshold", 0.10);
    declare_parameter<double>("blockage_pose_delta_deadzone", 0.10);
    declare_parameter<double>("blockage_recovery_velocity_threshold", 0.15);
    declare_parameter<double>("blockage_recovery_confirm_sec", 1.0);
    declare_parameter<bool>("obstacle_wait_enable", true);
    declare_parameter<double>("obstacle_wait_push_interval_sec", 4.0);
    declare_parameter<int>("obstacle_clear_required_frames", 5);
    declare_parameter<double>("obstacle_clear_check_rate_hz", 5.0);
    declare_parameter<int>("obstacle_clear_cost_threshold", 100);
    declare_parameter<double>("obstacle_clear_front_min_x_m", 0.15);
    declare_parameter<double>("obstacle_clear_front_max_x_m", 0.80);
    declare_parameter<double>("obstacle_clear_half_width_m", 0.30);
    declare_parameter<double>("obstacle_min_wait_before_resume_sec", 2.0);
    declare_parameter<double>("obstacle_clear_required_duration_sec", 3.0);
    declare_parameter<double>("obstacle_clear_required_duration_after_false_resume_sec", 4.0);
    declare_parameter<double>("obstacle_false_resume_window_sec", 3.0);
    declare_parameter<bool>("obstacle_resume_use_roi", true);
    declare_parameter<std::string>("obstacle_roi_has_obstacle_topic", "/front_obstacle/has_obstacle");
    declare_parameter<double>("obstacle_roi_timeout_sec", 1.0);
    declare_parameter<int>("obstacle_roi_required_clear_frames", 3);
    declare_parameter<std::string>("local_costmap_topic", "/local_costmap/costmap");
    declare_parameter<bool>("require_walk_mode_for_navigation", true);
    declare_parameter<double>("robot_status_timeout", 2.0);
    declare_parameter<double>("pending_navigation_timeout", 90.0);
    declare_parameter<double>("obstacle_block_near_goal_distance", 0.7);
    declare_parameter<std::string>("navigation_failure_policy", "pause_on_failed");
    declare_parameter<bool>("auto_pause_on_localization_recovery", false);
    declare_parameter<double>("localization_stop_hold_sec", 2.0);
    declare_parameter<double>("localization_resume_settle_sec", 1.0);
    declare_parameter<bool>("localization_auto_resume_require_recovery_done", true);
    declare_parameter<int>("localization_resume_stable_frames", 3);
    declare_parameter<std::string>("localization_health_status_topic", "/localization/prior_map_odom_bridge_status");
    declare_parameter<double>("localization_health_timeout_sec", 3.0);
    declare_parameter<bool>("localization_allow_start_with_last_good_tf", true);
    declare_parameter<double>("localization_last_good_tf_max_age_sec", 0.0);
    declare_parameter<std::string>("localization_recovery_status_topic", "/localization/recovery_status");
    declare_parameter<std::string>("localization_recovery_request_topic", "/localization/recovery_requests");
    declare_parameter<bool>("request_localization_recovery_on_nav_failure", false);
    declare_parameter<bool>("request_navigation_context_recovery_on_localization_failure", false);
    declare_parameter<double>("localization_recovery_request_cooldown_sec", 20.0);
    declare_parameter<double>("localization_recovery_prior_radius_m", 10.0);
    declare_parameter<double>("localization_context_recovery_request_cooldown_sec", 4.0);
    declare_parameter<double>("localization_context_prior_radius_m", 5.0);
    declare_parameter<double>("localization_context_prior_max_previous_age_sec", 300.0);
    declare_parameter<double>("localization_context_prior_min_segment_length_m", 0.2);
    declare_parameter<bool>("localization_resume_reverse_enabled", true);
    declare_parameter<double>("localization_resume_reverse_max_distance_m", 2.0);
    declare_parameter<double>("localization_resume_reverse_rear_angle_deg", 70.0);
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("base_frame", "base_footprint");
    declare_parameter<double>("pose_tf_timeout_sec", 0.05);
    declare_parameter<std::string>("reverse_navigation_bt_xml", "");
  }

  void load_parameters()
  {
    position_tolerance_ = get_parameter("position_tolerance").as_double();
    orientation_tolerance_ = get_parameter("orientation_tolerance").as_double();
    waypoint_timeout_ = get_parameter("waypoint_timeout").as_double();
    status_publish_rate_ = std::max(0.1, get_parameter("status_publish_rate").as_double());
    default_frame_id_ = get_parameter("default_frame_id").as_string();
    obstacle_block_timeout_ = get_parameter("obstacle_block_timeout").as_double();
    velocity_threshold_ = get_parameter("velocity_threshold").as_double();
    blockage_pose_delta_deadzone_ = get_parameter("blockage_pose_delta_deadzone").as_double();
    blockage_recovery_velocity_threshold_ = get_parameter("blockage_recovery_velocity_threshold").as_double();
    blockage_recovery_confirm_sec_ = get_parameter("blockage_recovery_confirm_sec").as_double();
    obstacle_wait_enable_ = get_parameter("obstacle_wait_enable").as_bool();
    obstacle_wait_push_interval_sec_ = get_parameter("obstacle_wait_push_interval_sec").as_double();
    obstacle_clear_required_frames_ = std::max(1, static_cast<int>(get_parameter("obstacle_clear_required_frames").as_int()));
    obstacle_clear_check_rate_hz_ = std::max(1.0, get_parameter("obstacle_clear_check_rate_hz").as_double());
    obstacle_clear_cost_threshold_ = static_cast<int>(get_parameter("obstacle_clear_cost_threshold").as_int());
    obstacle_clear_front_min_x_m_ = get_parameter("obstacle_clear_front_min_x_m").as_double();
    obstacle_clear_front_max_x_m_ = get_parameter("obstacle_clear_front_max_x_m").as_double();
    obstacle_clear_half_width_m_ = get_parameter("obstacle_clear_half_width_m").as_double();
    obstacle_min_wait_before_resume_sec_ = get_parameter("obstacle_min_wait_before_resume_sec").as_double();
    obstacle_clear_required_duration_sec_ = get_parameter("obstacle_clear_required_duration_sec").as_double();
    obstacle_clear_required_duration_after_false_resume_sec_ =
      get_parameter("obstacle_clear_required_duration_after_false_resume_sec").as_double();
    obstacle_false_resume_window_sec_ = get_parameter("obstacle_false_resume_window_sec").as_double();
    obstacle_resume_use_roi_ = get_parameter("obstacle_resume_use_roi").as_bool();
    obstacle_roi_has_obstacle_topic_ = get_parameter("obstacle_roi_has_obstacle_topic").as_string();
    obstacle_roi_timeout_sec_ = get_parameter("obstacle_roi_timeout_sec").as_double();
    obstacle_roi_required_clear_frames_ = std::max(1, static_cast<int>(get_parameter("obstacle_roi_required_clear_frames").as_int()));
    local_costmap_topic_ = get_parameter("local_costmap_topic").as_string();
    require_walk_mode_for_navigation_ = get_parameter("require_walk_mode_for_navigation").as_bool();
    robot_status_timeout_ = get_parameter("robot_status_timeout").as_double();
    pending_navigation_timeout_ = get_parameter("pending_navigation_timeout").as_double();
    obstacle_block_near_goal_distance_ = get_parameter("obstacle_block_near_goal_distance").as_double();
    navigation_failure_policy_ = get_parameter("navigation_failure_policy").as_string();
    auto_pause_on_localization_recovery_ = get_parameter("auto_pause_on_localization_recovery").as_bool();
    localization_stop_hold_sec_ = get_parameter("localization_stop_hold_sec").as_double();
    localization_resume_settle_sec_ = get_parameter("localization_resume_settle_sec").as_double();
    localization_auto_resume_require_recovery_done_ =
      get_parameter("localization_auto_resume_require_recovery_done").as_bool();
    localization_resume_stable_frames_ = static_cast<int>(get_parameter("localization_resume_stable_frames").as_int());
    localization_health_status_topic_ = get_parameter("localization_health_status_topic").as_string();
    localization_health_timeout_sec_ = get_parameter("localization_health_timeout_sec").as_double();
    localization_allow_start_with_last_good_tf_ =
      get_parameter("localization_allow_start_with_last_good_tf").as_bool();
    localization_last_good_tf_max_age_sec_ = get_parameter("localization_last_good_tf_max_age_sec").as_double();
    localization_recovery_status_topic_ = get_parameter("localization_recovery_status_topic").as_string();
    localization_recovery_request_topic_ = get_parameter("localization_recovery_request_topic").as_string();
    request_localization_recovery_on_nav_failure_ =
      get_parameter("request_localization_recovery_on_nav_failure").as_bool();
    request_navigation_context_recovery_on_localization_failure_ =
      get_parameter("request_navigation_context_recovery_on_localization_failure").as_bool();
    localization_recovery_request_cooldown_sec_ =
      get_parameter("localization_recovery_request_cooldown_sec").as_double();
    localization_recovery_prior_radius_m_ = get_parameter("localization_recovery_prior_radius_m").as_double();
    localization_context_recovery_request_cooldown_sec_ =
      get_parameter("localization_context_recovery_request_cooldown_sec").as_double();
    localization_context_prior_radius_m_ = get_parameter("localization_context_prior_radius_m").as_double();
    localization_context_prior_max_previous_age_sec_ =
      get_parameter("localization_context_prior_max_previous_age_sec").as_double();
    localization_context_prior_min_segment_length_m_ =
      get_parameter("localization_context_prior_min_segment_length_m").as_double();
    localization_resume_reverse_enabled_ = get_parameter("localization_resume_reverse_enabled").as_bool();
    localization_resume_reverse_max_distance_m_ =
      get_parameter("localization_resume_reverse_max_distance_m").as_double();
    localization_resume_reverse_rear_angle_rad_ =
      get_parameter("localization_resume_reverse_rear_angle_deg").as_double() * M_PI / 180.0;
    map_frame_ = get_parameter("map_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    pose_tf_timeout_sec_ = get_parameter("pose_tf_timeout_sec").as_double();
    reverse_navigation_bt_xml_ = get_parameter("reverse_navigation_bt_xml").as_string();
  }

  // ========================= ROS 通信与定时器 =========================
  void setup_communication()
  {
    navigation_status_pub_ = create_publisher<std_msgs::msg::String>("/navigation/status", 10);
    navigation_ack_pub_ = create_publisher<std_msgs::msg::String>("/navigation/acknowledgments", 10);
    navigation_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    navigation_path_pub_ = create_publisher<nav_msgs::msg::Path>("/navigation/current_path", 10);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    localization_recovery_request_pub_ =
      create_publisher<std_msgs::msg::String>(localization_recovery_request_topic_, 10);

    navigation_request_sub_ = create_subscription<std_msgs::msg::String>(
      "/navigation/requests", 10,
      [this](std_msgs::msg::String::SharedPtr msg) { navigation_request_callback(msg->data); });
    waypoints_data_sub_ = create_subscription<std_msgs::msg::String>(
      "/navigation/waypoints_data", 10,
      [this](std_msgs::msg::String::SharedPtr msg) { waypoints_data_callback(msg->data); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) { odom_callback(*msg); });
    local_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      local_costmap_topic_, 10,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) { local_costmap_callback(*msg); });
    roi_obstacle_sub_ = create_subscription<std_msgs::msg::Bool>(
      obstacle_roi_has_obstacle_topic_, 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) { roi_obstacle_callback(*msg); });
    robot_status_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot_status_raw", 10,
      [this](std_msgs::msg::String::SharedPtr msg) { robot_status_callback(msg->data); });
    nav2_behavior_log_sub_ = create_subscription<nav2_msgs::msg::BehaviorTreeLog>(
      "/behavior_tree_log", 10,
      [this](nav2_msgs::msg::BehaviorTreeLog::SharedPtr msg) { nav2_log_callback(*msg); });
    localization_recovery_sub_ = create_subscription<std_msgs::msg::String>(
      localization_recovery_status_topic_, 10,
      [this](std_msgs::msg::String::SharedPtr msg) { localization_recovery_status_callback(msg->data); });
    localization_status_sub_ = create_subscription<std_msgs::msg::String>(
      localization_health_status_topic_, 10,
      [this](std_msgs::msg::String::SharedPtr msg) { on_localization_status(msg->data); });

    // 构造函数阶段不能使用 shared_from_this()，否则节点还没被 shared_ptr 接管会触发 bad_weak_ptr。
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
  }

  void setup_timers()
  {
    status_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / status_publish_rate_),
      [this]() { publish_navigation_status(); });
    navigation_check_timer_ = create_wall_timer(500ms, [this]() { check_navigation_status(); });
    timeout_timer_ = create_wall_timer(5s, [this]() { check_timeout(); });
    pending_timer_ = create_wall_timer(200ms, [this]() { try_execute_pending_navigation(); });
    localization_resume_timer_ = create_wall_timer(500ms, [this]() { try_resume_after_localization_recovery(); });
    localization_stop_timer_ = create_wall_timer(33ms, [this]() { enforce_localization_stop(); });
    obstacle_wait_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / obstacle_clear_check_rate_hz_),
      [this]() { process_obstacle_wait_state(); });
    localization_timeout_timer_ = create_wall_timer(1s, [this]() { check_localization_status_timeout(); });
  }

  // ========================= 输入回调：路点、定位、机器人、传感器 =========================
  void navigation_request_callback(const std::string & payload)
  {
    try {
      const auto request = json::parse(payload);
      const std::string request_type = request.value("request_type", "");
      if (request_type == "navigation_command") {
        handle_navigation_command(request);
      } else {
        send_acknowledgment("error", "error", "未知请求类型: " + request_type);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "处理导航请求错误: %s", e.what());
      send_acknowledgment("error", "error", std::string("处理请求失败: ") + e.what());
    }
  }

  void waypoints_data_callback(const std::string & payload)
  {
    try {
      auto message = json::parse(payload);
      json legacy = message;
      if (message.contains("protocol_version")) {
        if (message.value("data_type", "") != "waypoints_data") {
          return;
        }
        legacy = message.value("data", json::object());
      }
      const auto data = legacy.value("data", json::object());
      waypoints_data_ = data.value("waypoints", json::object());
      RCLCPP_INFO(get_logger(), "收到路点数据更新，共 %d 个点位", count_cached_waypoints());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "处理路点数据错误: %s", e.what());
    }
  }

  void on_localization_status(const std::string & text_raw)
  {
    const double now = now_seconds();
    const std::string text = text_raw;
    const auto pos = text.find(' ');
    const std::string kind = text.empty() ? "UNKNOWN" : text.substr(0, pos);
    localization_last_status_time_ = now;
    last_localization_status_summary_ = {
      {"topic", localization_health_status_topic_},
      {"state", kind},
      {"text", text},
      {"stamp_sec", now},
    };

    if (kind == "ACCEPTED") {
      localization_has_last_good_tf_ = true;
      localization_last_good_tf_time_ = now;
      is_localization_healthy_ = true;
      localization_healthy_count_++;
      localization_resume_stable_count_++;
      if (localization_healthy_count_ >= std::max(1, localization_resume_stable_frames_)) {
        handle_localization_healthy();
      }
      return;
    }

    if (localization_can_use_last_good_tf(now)) {
      is_localization_healthy_ = true;
      return;
    }

    is_localization_healthy_ = false;
    localization_healthy_count_ = 0;
    localization_resume_stable_count_ = 0;
  }

  void check_localization_status_timeout()
  {
    if (localization_last_status_time_ <= 0.0) {
      return;
    }
    const double age = now_seconds() - localization_last_status_time_;
    if (age > localization_health_timeout_sec_ && is_localization_healthy_) {
      is_localization_healthy_ = false;
      localization_healthy_count_ = 0;
      localization_resume_stable_count_ = 0;
      RCLCPP_WARN(get_logger(), "[prior_map] 定位状态超时 %.1fs，暂停新的导航启动", age);
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry & msg)
  {
    auto pose = lookup_current_map_pose();
    if (pose) {
      current_pose_ = *pose;
      current_pose_frame_ = map_frame_;
      last_pose_update_ = now_seconds();
    }
    current_velocity_ = convert_fastlio_velocity_to_standard(msg);
    update_pose_derived_speed(msg);
    check_obstacle_blockage();
  }

  void local_costmap_callback(const nav_msgs::msg::OccupancyGrid & msg)
  {
    latest_local_costmap_stamp_ = now_seconds();
    const auto result = analyze_front_obstacle_window(msg);
    latest_front_obstacle_blocked_ = result.first;
    latest_front_obstacle_stats_ = result.second;
  }

  void roi_obstacle_callback(const std_msgs::msg::Bool & msg)
  {
    latest_roi_obstacle_has_obstacle_ = msg.data;
    latest_roi_obstacle_stamp_ = now_seconds();
    if (msg.data) {
      roi_obstacle_clear_confirm_count_ = 0;
    } else {
      roi_obstacle_clear_confirm_count_++;
    }
  }

  void robot_status_callback(const std::string & payload)
  {
    try {
      const auto status = json::parse(payload);
      const auto values = status.value("values", json::object());
      if (!values.is_object()) {
        return;
      }
      robot_control_state_ = values.value("robot_status", values.value("robot_state", robot_control_state_));
      robot_motion_busy_ = values.contains("motion_busy") && as_bool(values.at("motion_busy"));
      robot_current_motion_ = values.value("current_motion", "");
      if (values.contains("control_ready_for_navigation")) {
        robot_ready_for_navigation_ = as_bool(values.at("control_ready_for_navigation"));
      } else {
        robot_ready_for_navigation_ = robot_control_state_ == "Walk" && !robot_motion_busy_;
      }
      last_robot_status_update_ = now_seconds();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "处理机器人控制状态错误: %s", e.what());
    }
  }

  void localization_recovery_status_callback(const std::string & payload)
  {
    try {
      const auto status = json::parse(payload);
      localization_recovery_last_status_ = status;
      const std::string event_type = status.value("event_type", "");
      if (event_type == "localization_recovery_started") {
        handle_localization_recovery_started(status);
      } else if (
        event_type == "localization_recovery_prior_published" ||
        event_type == "localization_relocalize_requested" ||
        event_type == "localization_relocalize_completed" ||
        event_type == "localization_initialpose_published" ||
        event_type == "localization_recovery_waiting" ||
        event_type == "localization_recovery_clearing_buffer" ||
        event_type == "localization_recovery_buffer_cleared")
      {
        handle_localization_recovery_progress(status);
      } else if (event_type == "localization_relocalize_failed") {
        handle_localization_recovery_failed(status);
      } else if (event_type == "localization_manual_initialpose_override") {
        handle_localization_manual_initialpose_override(status);
      } else if (event_type == "localization_recovered") {
        handle_localization_recovered(status);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "处理定位恢复状态错误: %s", e.what());
    }
  }

  // ========================= 启动门控与缓存请求 =========================
  bool localization_can_use_last_good_tf(std::optional<double> now = std::nullopt) const
  {
    if (!localization_allow_start_with_last_good_tf_ || !localization_has_last_good_tf_) {
      return false;
    }
    const double max_age = std::max(0.0, localization_last_good_tf_max_age_sec_);
    if (max_age <= 0.0) {
      return true;
    }
    return (now.value_or(now_seconds()) - localization_last_good_tf_time_) <= max_age;
  }

  std::optional<std::string> get_navigation_start_block_reason() const
  {
    if (!require_walk_mode_for_navigation_) {
      return std::nullopt;
    }
    const double now = now_seconds();
    if (last_robot_status_update_ <= 0.0) {
      return "尚未收到机器人底层状态，暂不启动导航";
    }
    const double age = now - last_robot_status_update_;
    if (age > robot_status_timeout_) {
      return "机器人底层状态超时 " + format_1(age) + "s，暂不启动导航";
    }
    if (robot_ready_for_navigation_) {
      return std::nullopt;
    }
    if (robot_motion_busy_) {
      const std::string motion = robot_current_motion_.empty() ? "" : " (" + robot_current_motion_ + ")";
      return "机器人正在执行动作" + motion + "，尚未回到 Walk，暂不启动导航";
    }
    return "机器人当前状态为 " + robot_control_state_ + "，尚未回到 Walk，暂不启动导航";
  }

  std::optional<std::string> get_localization_start_block_reason() const
  {
    if (is_localization_healthy_) {
      return std::nullopt;
    }
    if (localization_last_status_time_ <= 0.0) {
      return "尚未收到定位状态 " + localization_health_status_topic_ + "，暂不启动导航";
    }
    const double age = now_seconds() - localization_last_status_time_;
    if (age > localization_health_timeout_sec_) {
      return "定位状态超时 " + format_1(age) + "s > " + format_1(localization_health_timeout_sec_) + "s，暂不启动导航";
    }
    if (localization_can_use_last_good_tf()) {
      return std::nullopt;
    }
    if (!localization_has_last_good_tf_) {
      return "prior-map bridge 尚未接受过 map->odom，暂不启动导航";
    }
    const std::string state = last_localization_status_summary_.value("state", "UNKNOWN");
    const std::string text = last_localization_status_summary_.value("text", "");
    if (!localization_allow_start_with_last_good_tf_) {
      return "prior-map定位未接受最新结果: " + state + " " + text;
    }
    const double last_good_age = now_seconds() - localization_last_good_tf_time_;
    return "last good map->odom 已超限 " + format_1(last_good_age) + "s > " +
           format_1(localization_last_good_tf_max_age_sec_) + "s，暂不启动导航";
  }

  bool reject_navigation_start_if_robot_not_ready(const std::string & ack_type)
  {
    const auto reason = get_navigation_start_block_reason();
    if (!reason) {
      return false;
    }
    send_acknowledgment(ack_type, "error", *reason);
    publish_status_update("navigation_start_rejected", {{"reason", *reason}});
    return true;
  }

  bool defer_navigation_start_if_robot_not_ready(const json & request)
  {
    const auto reason = get_navigation_start_block_reason();
    if (!reason) {
      return false;
    }
    const double now = now_seconds();
    const bool fresh = last_robot_status_update_ > 0.0 && now - last_robot_status_update_ <= robot_status_timeout_;
    const bool unavailable = !fresh;
    const bool can_defer = fresh && (robot_motion_busy_ || robot_control_state_ == "Menu");
    if (!(can_defer || unavailable)) {
      send_acknowledgment("navigation_started", "error", *reason);
      publish_status_update("navigation_start_rejected", {{"reason", *reason}});
      return true;
    }

    pending_navigation_request_ = request;
    pending_navigation_created_at_ = now;
    if (unavailable) {
      pending_navigation_reason_ = *reason + "。已缓存导航请求，等待底层状态恢复并确认 Walk 后再开始导航。";
    } else if (robot_motion_busy_) {
      pending_navigation_reason_ = *reason + "。已缓存导航请求，等待动作执行完成并回到 Walk 后再开始导航。";
    } else {
      pending_navigation_reason_ = *reason + "。已缓存导航请求，等待机器人回到 Walk 后再开始导航。";
    }
    const std::string command_type = request.value("command_data", json::object()).value("command_type", "");
    send_acknowledgment("navigation_pending", "pending", pending_navigation_reason_);
    publish_status_update("navigation_pending", {
      {"reason", pending_navigation_reason_},
      {"block_reason", *reason},
      {"command_type", command_type},
      {"timeout_sec", pending_navigation_timeout_},
    });
    return true;
  }

  void cache_navigation_for_recovery(const json & request)
  {
    const bool was_pending = pending_navigation_request_.has_value();
    pending_navigation_request_ = request;
    pending_navigation_created_at_ = now_seconds();
    pending_navigation_reason_ = "等待定位恢复后自动执行";
    send_acknowledgment(
      was_pending ? "navigation_pending_overwritten" : "navigation_pending",
      "pending",
      std::string("定位异常(prior-map不健康)，导航请求已") + (was_pending ? "覆盖" : "缓存") + "，定位恢复后自动执行");
  }

  void try_execute_pending_navigation()
  {
    if (!pending_navigation_request_) {
      return;
    }
    if (current_state_ != "idle") {
      if (current_state_ == "paused" && localization_auto_paused_) {
        return;
      }
      const auto request = *pending_navigation_request_;
      pending_navigation_request_.reset();
      pending_navigation_reason_.clear();
      const auto command_type = request.value("command_data", json::object()).value("command_type", "");
      const std::string message = "已有其他导航任务启动，取消待执行导航";
      send_acknowledgment("navigation_pending_cancelled", "error", message);
      publish_status_update("navigation_pending_cancelled", {{"reason", message}, {"command_type", command_type}});
      return;
    }

    auto reason = get_navigation_start_block_reason();
    if (!reason) {
      reason = get_localization_start_block_reason();
    }
    if (reason) {
      if (now_seconds() - pending_navigation_created_at_ > pending_navigation_timeout_) {
        const auto request = *pending_navigation_request_;
        pending_navigation_request_.reset();
        pending_navigation_reason_.clear();
        const auto command_type = request.value("command_data", json::object()).value("command_type", "");
        send_acknowledgment("navigation_started", "error", "待执行导航超时: " + *reason);
        publish_status_update("navigation_pending_timeout", {{"reason", *reason}, {"command_type", command_type}});
      }
      return;
    }

    const auto request = *pending_navigation_request_;
    pending_navigation_request_.reset();
    pending_navigation_reason_.clear();
    send_acknowledgment("navigation_pending", "success", "机器人状态已就绪，开始执行待启动导航");
    handle_navigation_command(request);
  }

  // ========================= APP 导航命令处理 =========================
  void handle_navigation_command(const json & request)
  {
    const auto command = request.value("command_data", json::object());
    const std::string type = command.value("command_type", "");
    if (type == "start_single_navigation") {
      handle_start_single_navigation(command, request);
    } else if (type == "start_multi_point_navigation") {
      handle_start_multi_point_navigation(command, request);
    } else if (type == "start_exhibition_navigation") {
      handle_start_exhibition_navigation(command, request);
    } else if (type == "stop_navigation") {
      handle_stop_navigation(command);
    } else if (type == "pause_navigation") {
      handle_pause_navigation(command);
    } else if (type == "resume_navigation") {
      handle_resume_navigation(command);
    } else if (type == "retry_failed_waypoint") {
      handle_retry_failed_waypoint(command);
    } else if (type == "skip_failed_waypoint") {
      handle_skip_failed_waypoint(command);
    } else if (type == "abort_failed_navigation") {
      handle_abort_failed_navigation(command);
    } else {
      send_acknowledgment("error", "error", "未知命令: " + type);
    }
  }

  void handle_start_single_navigation(const json & command, const json & request)
  {
    if (current_state_ != "idle") {
      send_acknowledgment("start_single_navigation", "error", "当前正在执行其他导航任务");
      return;
    }
    if (get_localization_start_block_reason()) {
      cache_navigation_for_recovery(request);
      return;
    }
    if (defer_navigation_start_if_robot_not_ready(request)) {
      return;
    }

    const std::string waypoint_id = command.value("waypoint_id", "");
    json waypoint = request.value("waypoint_data", json::object());
    if (waypoint.empty()) {
      auto found = find_waypoint_data_by_id(waypoint_id);
      if (!found) {
        send_acknowledgment("error", "error", "点位 '" + waypoint_id + "' 不存在");
        return;
      }
      waypoint = *found;
    }

    current_state_ = "planning";
    current_navigation_mode_ = "single_point";
    current_sequence_id_ = "single_" + std::to_string(static_cast<int64_t>(now_seconds()));
    waypoint_ids_ = {waypoint_id};
    total_waypoints_ = 1;
    current_waypoint_index_ = 0;
    navigation_start_time_ = now_seconds();
    send_acknowledgment("navigation_started", "success", "开始单点导航到 '" + waypoint.value("name", waypoint_id) + "'");
    navigate_to_waypoint(waypoint);
  }

  void handle_start_multi_point_navigation(const json & command, const json & request)
  {
    if (current_state_ != "idle") {
      send_acknowledgment("start_multi_point_navigation", "error", "当前正在执行其他导航任务");
      return;
    }
    if (get_localization_start_block_reason()) {
      cache_navigation_for_recovery(request);
      return;
    }
    if (defer_navigation_start_if_robot_not_ready(request)) {
      return;
    }
    merge_request_waypoints_data(request);

    if (!command.contains("waypoint_ids") || !command.at("waypoint_ids").is_array() || command.at("waypoint_ids").empty()) {
      send_acknowledgment("error", "error", "点位列表不能为空");
      return;
    }
    waypoint_ids_.clear();
    for (const auto & id_value : command.at("waypoint_ids")) {
      const std::string id = id_value.get<std::string>();
      if (!find_waypoint_data_by_id(id)) {
        send_acknowledgment("error", "error", "点位 '" + id + "' 不存在");
        return;
      }
      waypoint_ids_.push_back(id);
    }
    current_state_ = "planning";
    current_navigation_mode_ = "multi_point";
    current_sequence_id_ = "multi_" + std::to_string(static_cast<int64_t>(now_seconds()));
    total_waypoints_ = static_cast<int>(waypoint_ids_.size());
    current_waypoint_index_ = 0;
    navigation_start_time_ = now_seconds();
    send_acknowledgment("navigation_started", "success", "开始多点导航，共 " + std::to_string(total_waypoints_) + " 个点位");
    start_navigation_sequence();
  }

  void handle_start_exhibition_navigation(const json &, const json & request)
  {
    if (current_state_ != "idle") {
      send_acknowledgment("start_exhibition_navigation", "error", "当前正在执行其他导航任务");
      return;
    }
    if (get_localization_start_block_reason()) {
      cache_navigation_for_recovery(request);
      return;
    }
    if (defer_navigation_start_if_robot_not_ready(request)) {
      return;
    }
    auto exhibition = request.value("waypoints_data", json::object());
    if (!exhibition.empty()) {
      merge_request_waypoints_data(request);
    }
    if (exhibition.empty()) {
      exhibition = waypoints_data_.value("exhibition_point", json::object());
    }
    if (exhibition.empty()) {
      send_acknowledgment("error", "error", "没有可用的展台点位");
      return;
    }
    waypoint_ids_.clear();
    for (auto it = exhibition.begin(); it != exhibition.end(); ++it) {
      waypoint_ids_.push_back(it.key());
    }
    current_state_ = "planning";
    current_navigation_mode_ = "exhibition_tour";
    current_sequence_id_ = "exhibition_" + std::to_string(static_cast<int64_t>(now_seconds()));
    total_waypoints_ = static_cast<int>(waypoint_ids_.size());
    current_waypoint_index_ = 0;
    navigation_start_time_ = now_seconds();
    send_acknowledgment("navigation_started", "success", "开始展台导航，共 " + std::to_string(total_waypoints_) + " 个点位");
    start_navigation_sequence();
  }

  void handle_stop_navigation(const json & command)
  {
    if (current_state_ == "idle" && pending_navigation_request_) {
      pending_navigation_request_.reset();
      pending_navigation_reason_.clear();
      send_acknowledgment("navigation_pending_cancelled", "success", "已取消待执行导航");
      publish_status_update("navigation_pending_cancelled", {{"reason", "user_stop"}});
      return;
    }
    if (current_state_ == "idle") {
      send_acknowledgment("stop_navigation", "error", "当前没有在执行导航");
      return;
    }
    const auto params = command.value("stop_parameters", json::object());
    const bool emergency_stop = params.value("emergency_stop", false);
    const std::string stop_reason = params.value("reason", "user_request");
    cancel_navigation();
    publish_zero_cmd_vel();
    const int completed = current_waypoint_index_;
    const int total = total_waypoints_;
    current_state_ = "cancelled";
    const double duration = navigation_start_time_ > 0.0 ? now_seconds() - navigation_start_time_ : 0.0;
    const double percent = total > 0 ? completed * 100.0 / total : 0.0;
    send_acknowledgment("navigation_stopped", "success", "导航已停止，完成 " + std::to_string(completed) + "/" + std::to_string(total) + " 个点位");
    publish_status_update("navigation_stopped", {
      {"reason", stop_reason},
      {"emergency_stop", emergency_stop},
      {"completed_waypoints", completed},
      {"total_waypoints", total},
      {"navigation_duration", round_1(duration)},
      {"completion_percentage", round_1(percent)},
      {"last_waypoint_reached", completed > 0 && completed - 1 < static_cast<int>(waypoint_ids_.size()) ? json(waypoint_ids_[completed - 1]) : json(nullptr)},
    });
    reset_navigation_state();
  }

  void handle_pause_navigation(const json & command)
  {
    if (current_state_ != "executing") {
      send_acknowledgment("pause_navigation", "error", "当前没有在执行导航");
      return;
    }
    const auto params = command.value("pause_parameters", json::object());
    pause_time_ = now_seconds();
    pause_duration_limit_ = params.value("pause_duration", 0.0);
    current_state_ = "paused";
    current_detailed_state_ = "PAUSED";
    current_pause_source_ = "user_request";
    current_pause_reason_ = "用户手动暂停导航";
    current_resume_mode_ = "manual";
    clear_obstacle_wait_state();
    reset_block_detection();
    cancel_navigation();
    send_acknowledgment("pause_navigation", "success", "导航已暂停");
    publish_status_update("navigation_paused", build_pause_event_data("user_request", current_pause_reason_, "manual"));
  }

  void handle_resume_navigation(const json & command)
  {
    if (current_state_ == "recoverable_failed") {
      send_acknowledgment("navigation_resumed", "error", "导航处于失败可恢复状态，请使用 retry_failed_waypoint 或 skip_failed_waypoint");
      return;
    }
    if (current_state_ != "paused") {
      send_acknowledgment("resume_navigation", "error", "导航未暂停");
      return;
    }
    if (is_localization_resume_blocked()) {
      reject_resume_during_localization_recovery();
      return;
    }
    if (current_waypoint_.empty()) {
      send_acknowledgment("navigation_resumed", "error", "没有可恢复的导航目标");
      reset_navigation_state();
      return;
    }
    if (reject_navigation_start_if_robot_not_ready("navigation_resumed")) {
      return;
    }
    const std::string resume_reason = command.value("reason", "user_request");
    const double pause_elapsed = pause_time_ > 0.0 ? now_seconds() - pause_time_ : 0.0;
    const std::string resume_source = current_pause_source_.empty() ? "user_request" : current_pause_source_;
    if (obstacle_wait_active_) {
      clear_obstacle_wait_state();
    }
    current_state_ = "executing";
    current_detailed_state_ = "EXECUTING";
    current_pause_source_.clear();
    current_pause_reason_.clear();
    current_resume_mode_.clear();
    send_acknowledgment("navigation_resumed", "success", "导航已恢复");
    publish_status_update("navigation_resumed", {
      {"resumed_waypoint_id", current_waypoint_.value("id", "")},
      {"resumed_waypoint_name", current_waypoint_.value("name", "")},
      {"waypoint_index", current_waypoint_index_},
      {"total_waypoints", total_waypoints_},
      {"pause_duration_actual", round_1(pause_elapsed)},
      {"resume_reason", resume_reason},
      {"resume_source", resume_source},
    });
    navigate_to_waypoint(current_waypoint_);
  }

  void handle_retry_failed_waypoint(const json &)
  {
    if (current_state_ != "recoverable_failed") {
      send_acknowledgment("retry_failed_waypoint", "error", "当前没有可重试的失败导航");
      return;
    }
    if (current_waypoint_.empty()) {
      send_acknowledgment("retry_failed_waypoint", "error", "失败点位信息缺失，无法重试");
      reset_navigation_state();
      return;
    }
    if (reject_navigation_start_if_robot_not_ready("retry_failed_waypoint")) {
      return;
    }
    auto context = build_failure_recovery_context();
    current_state_ = "executing";
    current_detailed_state_ = "EXECUTING";
    reset_block_detection();
    send_acknowledgment("retry_failed_waypoint", "success", "重试失败点: " + current_waypoint_.value("name", ""), context);
    publish_status_update("navigation_retry_failed_waypoint", context);
    navigate_to_waypoint(current_waypoint_);
  }

  void handle_skip_failed_waypoint(const json &)
  {
    if (current_state_ != "recoverable_failed") {
      send_acknowledgment("skip_failed_waypoint", "error", "当前没有可跳过的失败导航");
      return;
    }
    auto failed_context = build_failure_recovery_context();
    json skipped = {
      {"waypoint_index", current_waypoint_index_},
      {"waypoint_id", current_waypoint_.value("id", "")},
      {"waypoint_name", current_waypoint_.value("name", "")},
      {"reason", failed_context.value("reason", "")},
    };
    skipped_waypoints_.push_back(skipped);
    current_waypoint_index_++;
    current_goal_handle_.reset();
    reset_block_detection();
    if (current_waypoint_index_ >= total_waypoints_) {
      failed_context["recoverable"] = false;
      failed_context["available_actions"] = json::array();
      failed_context["skipped_waypoint"] = skipped;
      failed_context["skipped_waypoints"] = skipped_waypoints_;
      failed_context["remaining_waypoint_ids"] = json::array();
      send_acknowledgment("skip_failed_waypoint", "success", "已跳过失败点，后续没有更多点位，导航完成", failed_context);
      handle_navigation_completed();
      return;
    }
    const auto next = find_waypoint_data_by_id(waypoint_ids_[current_waypoint_index_]);
    if (!next) {
      send_acknowledgment("skip_failed_waypoint", "error", "下一个路点 '" + waypoint_ids_[current_waypoint_index_] + "' 不存在", failed_context);
      return;
    }
    current_state_ = "planning";
    current_detailed_state_ = "PLANNING";
    failed_context["recoverable"] = false;
    failed_context["available_actions"] = {"pause_navigation", "stop_navigation"};
    failed_context["skipped_waypoint"] = skipped;
    failed_context["skipped_waypoints"] = skipped_waypoints_;
    failed_context["next_waypoint_index"] = current_waypoint_index_;
    failed_context["next_waypoint_id"] = waypoint_ids_[current_waypoint_index_];
    failed_context["next_waypoint_name"] = next->value("name", "");
    send_acknowledgment("skip_failed_waypoint", "success", "已跳过失败点，继续导航到: " + next->value("name", ""), failed_context);
    publish_status_update("navigation_skip_failed_waypoint", failed_context);
    navigate_to_waypoint(*next);
  }

  void handle_abort_failed_navigation(const json &)
  {
    if (current_state_ != "recoverable_failed" && current_state_ != "failed") {
      send_acknowledgment("abort_failed_navigation", "error", "当前没有可终止的失败导航");
      return;
    }
    const auto context = build_failure_recovery_context();
    send_acknowledgment("abort_failed_navigation", "success", "已终止失败导航任务", context);
    publish_status_update("navigation_aborted", context);
    reset_navigation_state();
  }

  // ========================= Nav2 action 与导航序列 =========================
  void start_navigation_sequence()
  {
    if (waypoint_ids_.empty() || current_waypoint_index_ >= static_cast<int>(waypoint_ids_.size())) {
      handle_navigation_failed("导航序列为空或已完成");
      return;
    }
    const auto waypoint = find_waypoint_data_by_id(waypoint_ids_[current_waypoint_index_]);
    if (!waypoint) {
      handle_navigation_failed("路点 '" + waypoint_ids_[current_waypoint_index_] + "' 不存在");
      return;
    }
    navigate_to_waypoint(*waypoint);
  }

  void navigate_to_waypoint(const json & waypoint, const std::optional<std::string> & force_walk_direction = std::nullopt)
  {
    current_waypoint_ = waypoint;
    current_state_ = "executing";
    current_waypoint_start_time_ = now_seconds();
    auto goal = NavigateToPose::Goal();
    goal.pose = waypoint_to_pose_stamped(waypoint);
    const std::string walk_direction = force_walk_direction.value_or(get_waypoint_walk_direction(waypoint));
    if (walk_direction == "backward") {
      goal.behavior_tree = reverse_navigation_bt_xml_;
    }

    if (!nav_client_->wait_for_action_server(5s)) {
      handle_navigation_failed("Nav2服务器不可用");
      return;
    }

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
      [this](const GoalHandleNavigate::SharedPtr & goal_handle) { nav2_goal_response_callback(goal_handle); };
    options.feedback_callback =
      [this](GoalHandleNavigate::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        nav2_feedback_callback(*feedback);
      };
    options.result_callback =
      [this](const GoalHandleNavigate::WrappedResult & result) { nav2_result_callback(result); };
    nav_client_->async_send_goal(goal, options);

    publish_status_update("waypoint_started", {
      {"waypoint_id", waypoint.value("id", "")},
      {"waypoint_name", waypoint.value("name", "")},
      {"waypoint_index", current_waypoint_index_},
      {"total_waypoints", total_waypoints_},
      {"position", waypoint.value("position", json::array())},
      {"walk_direction", walk_direction},
      {"behavior_tree", goal.behavior_tree},
    });
  }

  void nav2_goal_response_callback(const GoalHandleNavigate::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      handle_navigation_failed("Nav2目标被拒绝");
      return;
    }
    current_goal_handle_ = goal_handle;
    if (current_state_ == "paused" && localization_auto_paused_) {
      cancel_navigation();
    }
  }

  void nav2_result_callback(const GoalHandleNavigate::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      handle_nav2_succeeded();
    } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
      handle_nav2_cancelled();
    } else {
      handle_nav2_failed();
    }
  }

  void nav2_feedback_callback(const NavigateToPose::Feedback & feedback)
  {
    distance_remaining_ = feedback.distance_remaining;
    estimated_time_remaining_ =
      feedback.estimated_time_remaining.sec + feedback.estimated_time_remaining.nanosec * 1e-9;
    const double navigation_time =
      feedback.navigation_time.sec + feedback.navigation_time.nanosec * 1e-9;
    publish_status_update("navigation_progress_update", {
      {"current_pose", {{"position", {
        {"x", feedback.current_pose.pose.position.x},
        {"y", feedback.current_pose.pose.position.y},
        {"z", feedback.current_pose.pose.position.z},
      }}}},
      {"distance_remaining", distance_remaining_},
      {"estimated_time_remaining", estimated_time_remaining_},
      {"navigation_time_sec", navigation_time},
    });
  }

  void handle_nav2_succeeded()
  {
    if (current_state_ != "executing" || current_waypoint_.empty()) {
      return;
    }
    publish_status_update("waypoint_reached", {
      {"waypoint_id", current_waypoint_.value("id", "")},
      {"waypoint_name", current_waypoint_.value("name", "")},
      {"waypoint_index", current_waypoint_index_},
      {"total_waypoints", total_waypoints_},
      {"confirmation_source", "nav2"},
    });
    record_last_succeeded_waypoint(current_waypoint_, current_waypoint_index_);
    current_waypoint_index_++;
    if (current_waypoint_index_ >= total_waypoints_) {
      handle_navigation_completed();
      return;
    }
    const auto next = find_waypoint_data_by_id(waypoint_ids_[current_waypoint_index_]);
    if (!next) {
      handle_navigation_failed("下一个路点 '" + waypoint_ids_[current_waypoint_index_] + "' 不存在");
      return;
    }
    next_waypoint_timer_ = create_wall_timer(1s, [this, next]() {
      if (next_waypoint_timer_) {
        next_waypoint_timer_->cancel();
        next_waypoint_timer_.reset();
      }
      navigate_to_waypoint(*next);
    });
  }

  void handle_nav2_failed()
  {
    if (try_enter_obstacle_wait_from_nav_failure("Nav2导航失败")) {
      return;
    }
    handle_navigation_failed("Nav2导航失败");
  }

  void handle_nav2_cancelled()
  {
    if (current_state_ == "paused") {
      return;
    }
    if (current_state_ == "executing") {
      current_state_ = "cancelled";
      publish_status_update("navigation_cancelled", {{"reason", "nav2_cancelled"}});
      reset_navigation_state();
    }
  }

  void cancel_navigation()
  {
    if (!current_goal_handle_) {
      return;
    }
    auto future = nav_client_->async_cancel_goal(current_goal_handle_);
    (void)future;
  }

  // ========================= 障碍等待与阻塞判断 =========================
  void check_obstacle_blockage()
  {
    if (current_state_ != "executing") {
      if (is_blocked_by_obstacle_) {
        reset_block_detection();
      }
      return;
    }
    const auto speed = get_blockage_motion_speed();
    if (!speed) {
      return;
    }
    const auto [value, source] = *speed;
    if (block_reported_) {
      if (has_confirmed_blockage_recovery(value, source)) {
        reset_block_detection();
      }
      return;
    }
    const auto suppression = get_obstacle_blockage_suppression_reason();
    if (suppression) {
      if (is_blocked_by_obstacle_) {
        reset_block_detection();
      }
      return;
    }
    if (is_stopped_for_blockage(value, source)) {
      clear_block_recovery_candidate();
      if (!is_blocked_by_obstacle_) {
        is_blocked_by_obstacle_ = true;
        block_start_time_ = now_seconds();
        current_detailed_state_ = "BLOCKED_BY_OBSTACLE";
      } else if (now_seconds() - block_start_time_ > obstacle_block_timeout_) {
        handle_obstacle_block_timeout(now_seconds() - block_start_time_);
      }
    } else if (is_blocked_by_obstacle_) {
      if (has_confirmed_blockage_recovery(value, source)) {
        reset_block_detection();
      } else if (now_seconds() - block_start_time_ > obstacle_block_timeout_) {
        handle_obstacle_block_timeout(now_seconds() - block_start_time_);
      }
    }
  }

  void handle_obstacle_block_timeout(double block_duration)
  {
    if (block_reported_ || !obstacle_wait_enable_) {
      return;
    }
    enter_obstacle_wait_state(block_duration);
  }

  void enter_obstacle_wait_state(double block_duration)
  {
    if (obstacle_wait_active_) {
      return;
    }
    block_reported_ = true;
    current_state_ = "paused";
    current_detailed_state_ = "OBSTACLE_WAITING";
    pause_time_ = now_seconds();
    pause_duration_limit_ = 0.0;
    current_pause_source_ = "obstacle_wait";
    current_pause_reason_ = "检测到障碍物，前方路径被挡住";
    current_resume_mode_ = "auto";
    obstacle_wait_active_ = true;
    obstacle_wait_started_at_ = pause_time_;
    obstacle_wait_last_push_time_ = 0.0;
    obstacle_clear_confirm_count_ = 0;
    obstacle_clear_started_at_ = 0.0;
    roi_obstacle_clear_confirm_count_ = 0;
    if (last_obstacle_resume_time_ > 0.0 && pause_time_ - last_obstacle_resume_time_ <= obstacle_false_resume_window_sec_) {
      obstacle_recent_false_resume_count_++;
    } else {
      obstacle_recent_false_resume_count_ = 0;
    }
    reset_block_detection();
    cancel_navigation();
    publish_zero_cmd_vel();
    auto event = build_pause_event_data("obstacle_wait", current_pause_reason_, "auto");
    event["block_duration"] = round_1(block_duration);
    event["waiting_for_obstacle_clear"] = true;
    event["clear_confirmed_frames"] = obstacle_clear_confirm_count_;
    event["clear_required_frames"] = obstacle_clear_required_frames_;
    event["clear_required_duration_sec"] = get_current_obstacle_clear_required_duration();
    event["false_resume_count"] = obstacle_recent_false_resume_count_;
    publish_status_update("navigation_paused", event);
    send_acknowledgment("navigation_paused", "success", "导航已因障碍物暂停", event);
    publish_obstacle_blocked_event(block_duration, true);
  }

  void process_obstacle_wait_state()
  {
    if (!obstacle_wait_active_ || current_state_ != "paused") {
      return;
    }
    const double now = now_seconds();
    const double block_duration = obstacle_wait_started_at_ > 0.0 ? now - obstacle_wait_started_at_ : 0.0;
    const double max_costmap_age = std::max(1.0, 2.0 / obstacle_clear_check_rate_hz_);
    if (now - latest_local_costmap_stamp_ > max_costmap_age || latest_front_obstacle_blocked_) {
      obstacle_clear_confirm_count_ = 0;
      obstacle_clear_started_at_ = 0.0;
      return;
    }
    const auto roi = is_roi_obstacle_clear_for_resume(now);
    if (!roi.first) {
      obstacle_clear_confirm_count_ = 0;
      obstacle_clear_started_at_ = 0.0;
      return;
    }
    obstacle_clear_confirm_count_++;
    if (obstacle_clear_confirm_count_ < obstacle_clear_required_frames_) {
      return;
    }
    if (block_duration < obstacle_min_wait_before_resume_sec_) {
      return;
    }
    if (obstacle_clear_started_at_ <= 0.0) {
      obstacle_clear_started_at_ = now;
      return;
    }
    if (now - obstacle_clear_started_at_ < get_current_obstacle_clear_required_duration()) {
      return;
    }
    resume_navigation_from_obstacle_wait();
  }

  void resume_navigation_from_obstacle_wait()
  {
    if (!obstacle_wait_active_ || current_state_ != "paused") {
      return;
    }
    if (current_waypoint_.empty()) {
      clear_obstacle_wait_state();
      reset_navigation_state();
      return;
    }
    const double pause_elapsed = pause_time_ > 0.0 ? now_seconds() - pause_time_ : 0.0;
    clear_obstacle_wait_state();
    current_state_ = "executing";
    current_detailed_state_ = "EXECUTING";
    current_pause_source_.clear();
    current_pause_reason_.clear();
    current_resume_mode_.clear();
    json event = {
      {"resumed_waypoint_id", current_waypoint_.value("id", "")},
      {"resumed_waypoint_name", current_waypoint_.value("name", "")},
      {"waypoint_index", current_waypoint_index_},
      {"total_waypoints", total_waypoints_},
      {"pause_duration_actual", round_1(pause_elapsed)},
      {"resume_reason", "obstacle_cleared_auto_resume"},
      {"resume_source", "obstacle_wait"},
      {"front_obstacle_stats", latest_front_obstacle_stats_},
      {"roi_obstacle_stats", build_roi_obstacle_stats()},
    };
    send_acknowledgment("navigation_resumed", "success", "障碍物已消失，导航自动恢复", event);
    publish_status_update("navigation_resumed", event);
    last_obstacle_resume_time_ = now_seconds();
    navigate_to_waypoint(current_waypoint_);
  }

  bool try_enter_obstacle_wait_from_nav_failure(const std::string & reason)
  {
    if (!obstacle_wait_enable_ || obstacle_wait_active_) {
      return false;
    }
    if (current_state_ != "executing" && current_state_ != "planning") {
      return false;
    }
    if (current_waypoint_.empty() || get_obstacle_blockage_suppression_reason()) {
      return false;
    }
    RCLCPP_WARN(get_logger(), "Nav2/RPP 执行失败，按前方障碍阻塞接管并暂停等待: %s", reason.c_str());
    enter_obstacle_wait_state(0.0);
    return true;
  }

  // ========================= 定位恢复联动 =========================
  void handle_localization_healthy()
  {
    if (!localization_auto_paused_) {
      return;
    }
    if (localization_auto_resume_require_recovery_done_ && !localization_recovery_done_) {
      if (now_seconds() - last_resume_wait_log_time_ > 2.0) {
        last_resume_wait_log_time_ = now_seconds();
        publish_status_update("navigation_localization_resume_waiting", {
          {"reason", "prior-map定位已接受，但定位恢复流程尚未确认完成，暂不恢复导航"},
          {"localization_event", localization_recovery_last_status_.value("event_type", "")},
          {"localization_stable_count", localization_resume_stable_count_},
          {"required_localization_stable_frames", localization_resume_stable_frames_},
          {"localization_status", last_localization_status_summary_},
        });
      }
      return;
    }
    localization_healthy_count_ = 0;
    localization_resume_pending_ = true;
    if (localization_recovered_at_ <= 0.0) {
      localization_recovered_at_ = now_seconds();
    }
    try_resume_after_localization_recovery();
  }

  void handle_localization_recovery_started(const json & status)
  {
    if (!auto_pause_on_localization_recovery_) {
      return;
    }
    localization_recovery_active_ = true;
    localization_recovery_done_ = false;
    localization_resume_stable_count_ = 0;
    localization_recovery_reason_ = status.value("reason", "定位异常，正在重定位");
    localization_recovery_started_at_ = now_seconds();
    localization_recovered_at_ = 0.0;
    if (current_state_ == "executing" || current_state_ == "planning") {
      localization_auto_paused_ = true;
      localization_resume_pending_ = false;
      current_state_ = "paused";
      current_detailed_state_ = "LOCALIZATION_RECOVERY";
      pause_time_ = now_seconds();
      pause_duration_limit_ = 0.0;
      reset_block_detection();
      begin_localization_stop_hold();
      cancel_navigation();
      auto event = build_localization_pause_context(status);
      publish_status_update("navigation_paused", event);
      publish_status_update("navigation_localization_recovery_started", event);
      send_acknowledgment("navigation_auto_paused", "success", "定位异常，已暂停导航并开始自动重定位", event);
      request_navigation_context_recovery_for_localization(localization_recovery_reason_, status);
    }
  }

  void handle_localization_recovery_progress(const json & status)
  {
    if (!localization_recovery_active_) {
      return;
    }
    publish_status_update("navigation_localization_recovery_progress", {
      {"reason", status.value("reason", localization_recovery_reason_)},
      {"localization_event", status.value("event_type", "")},
      {"recovery_count", status.value("recovery_count", 0)},
      {"relocalize_attempts", status.value("relocalize_attempts", 0)},
      {"current_waypoint_id", current_waypoint_.value("id", "")},
      {"current_waypoint_name", current_waypoint_.value("name", "")},
      {"waypoint_index", current_waypoint_index_},
      {"total_waypoints", total_waypoints_},
      {"auto_resume_pending", localization_auto_paused_},
    });
  }

  void handle_localization_recovery_failed(const json & status)
  {
    localization_recovery_active_ = true;
    localization_recovery_done_ = false;
    localization_resume_pending_ = false;
    localization_resume_stable_count_ = 0;
    localization_recovery_reason_ = status.value("reason", localization_recovery_reason_);
    localization_recovered_at_ = 0.0;
    publish_status_update("navigation_localization_recovery_failed", localization_recovery_event_context());
  }

  void handle_localization_manual_initialpose_override(const json & status)
  {
    localization_recovery_active_ = false;
    localization_recovery_done_ = false;
    localization_resume_pending_ = false;
    localization_resume_stable_count_ = 0;
    localization_recovery_reason_ = status.value("reason", "人工重定位已接管");
    auto event = localization_recovery_event_context();
    event["localization_event"] = status.value("event_type", "");
    event["manual_lockout_sec"] = status.value("manual_lockout_sec", 0.0);
    publish_status_update("navigation_localization_manual_override", event);
  }

  void handle_localization_recovered(const json & status)
  {
    localization_recovery_active_ = false;
    localization_recovery_done_ = true;
    localization_resume_stable_count_ = 0;
    localization_recovery_reason_ = status.value("reason", "定位已恢复");
    localization_recovered_at_ = now_seconds();
    auto event = localization_recovery_event_context();
    event["recovery_duration"] =
      localization_recovery_started_at_ > 0.0 ? round_1(now_seconds() - localization_recovery_started_at_) : 0.0;
    publish_status_update("navigation_localization_recovered", event);
    if (localization_auto_paused_) {
      localization_resume_pending_ = true;
      try_resume_after_localization_recovery();
    }
  }

  void try_resume_after_localization_recovery()
  {
    if (!localization_resume_pending_) {
      return;
    }
    if (current_state_ != "paused" || !localization_auto_paused_) {
      localization_resume_pending_ = false;
      return;
    }
    if (localization_auto_resume_require_recovery_done_ &&
      (localization_recovery_active_ || !localization_recovery_done_))
    {
      publish_status_update("navigation_localization_resume_waiting", {
        {"reason", "定位恢复流程尚未确认完成，暂不恢复导航"},
        {"localization_event", localization_recovery_last_status_.value("event_type", "")},
        {"current_waypoint_id", current_waypoint_.value("id", "")},
        {"current_waypoint_name", current_waypoint_.value("name", "")},
        {"waypoint_index", current_waypoint_index_},
        {"total_waypoints", total_waypoints_},
      });
      return;
    }
    if (current_waypoint_.empty()) {
      localization_resume_pending_ = false;
      localization_auto_paused_ = false;
      publish_status_update("navigation_localization_resume_failed", {{"reason", "没有可恢复的导航目标"}});
      reset_navigation_state();
      return;
    }
    if (const auto reason = get_navigation_start_block_reason()) {
      publish_status_update("navigation_localization_resume_waiting", {{"reason", *reason}});
      return;
    }
    if (!is_localization_healthy_ || localization_resume_stable_count_ < std::max(0, localization_resume_stable_frames_)) {
      publish_status_update("navigation_localization_resume_waiting", {
        {"reason", "等待 prior-map 定位稳定帧 " + std::to_string(localization_resume_stable_count_) + "/" +
          std::to_string(std::max(0, localization_resume_stable_frames_))},
        {"localization_stable_count", localization_resume_stable_count_},
        {"required_localization_stable_frames", std::max(0, localization_resume_stable_frames_)},
        {"localization_status", last_localization_status_summary_},
      });
      return;
    }
    const double settle = localization_recovered_at_ + std::max(0.0, localization_resume_settle_sec_) - now_seconds();
    if (settle > 0.0) {
      publish_status_update("navigation_localization_resume_waiting", {
        {"reason", "定位恢复后等待 prior-map/代价地图稳定 " + format_1(settle) + "s"},
        {"settle_remaining_sec", round_2(settle)},
      });
      return;
    }

    const double pause_elapsed = pause_time_ > 0.0 ? now_seconds() - pause_time_ : 0.0;
    auto event = json{
      {"resumed_waypoint_id", current_waypoint_.value("id", "")},
      {"resumed_waypoint_name", current_waypoint_.value("name", "")},
      {"waypoint_index", current_waypoint_index_},
      {"total_waypoints", total_waypoints_},
      {"pause_duration_actual", round_1(pause_elapsed)},
      {"resume_reason", "localization_recovered"},
      {"localization_reason", localization_recovery_reason_},
    };
    const auto reverse = should_reverse_resume_to_current_waypoint();
    event.update(reverse.second);
    localization_resume_pending_ = false;
    localization_auto_paused_ = false;
    localization_stop_until_ = 0.0;
    current_state_ = "executing";
    current_detailed_state_ = "EXECUTING";
    send_acknowledgment("navigation_auto_resumed", "success", "定位已恢复，继续未完成导航", event);
    publish_status_update("navigation_resumed", event);
    navigate_to_waypoint(current_waypoint_, reverse.first ? std::optional<std::string>("backward") : std::nullopt);
  }

  // ========================= 状态输出与辅助工具 =========================
  void send_acknowledgment(
    const std::string & ack_type,
    const std::string & status,
    const std::string & message = "",
    const json & extra = json::object())
  {
    json payload = {
      {"ack_type", ack_type},
      {"status", status},
      {"message", message},
      {"timestamp", now_seconds()},
    };
    if (extra.is_object()) {
      payload.update(extra);
    }
    std_msgs::msg::String msg;
    msg.data = json_string(payload);
    navigation_ack_pub_->publish(msg);
  }

  void publish_status_update(const std::string & event_type, const json & event_data)
  {
    json payload = {
      {"event_type", event_type},
      {"event_data", event_data},
      {"timestamp", now_seconds()},
      {"current_state", current_state_},
      {"navigation_mode", current_navigation_mode_.empty() ? json(nullptr) : json(current_navigation_mode_)},
      {"sequence_id", current_sequence_id_.empty() ? json(nullptr) : json(current_sequence_id_)},
    };
    std_msgs::msg::String msg;
    msg.data = json_string(payload);
    navigation_status_pub_->publish(msg);
  }

  void publish_navigation_status()
  {
    try {
      std_msgs::msg::String msg;
      msg.data = json_string(get_current_status_summary());
      navigation_status_pub_->publish(msg);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "发布导航状态错误: %s", e.what());
    }
  }

  json get_current_status_summary()
  {
    auto suppression = get_obstacle_blockage_suppression_reason();
    json status = {
      {"timestamp", now_seconds()},
      {"current_state", current_state_},
      {"navigation_mode", current_navigation_mode_.empty() ? json(nullptr) : json(current_navigation_mode_)},
      {"sequence_id", current_sequence_id_.empty() ? json(nullptr) : json(current_sequence_id_)},
      {"current_waypoint_index", current_waypoint_index_},
      {"total_waypoints", total_waypoints_},
      {"progress_percentage", calculate_progress_percentage()},
      {"is_active", is_navigation_active()},
      {"navigation_duration", navigation_start_time_ > 0.0 ? now_seconds() - navigation_start_time_ : 0.0},
      {"detailed_state", current_detailed_state_},
      {"recovery_active", current_detailed_state_ == "RECOVERING"},
      {"obstacle_blocked", is_blocked_by_obstacle_},
      {"block_duration", block_start_time_ > 0.0 ? now_seconds() - block_start_time_ : 0.0},
      {"block_reported", block_reported_},
      {"obstacle_block_suppressed", suppression.has_value()},
      {"obstacle_block_suppression_reason", suppression.value_or("")},
      {"pending_navigation", pending_navigation_request_.has_value()},
      {"pending_navigation_age", pending_navigation_request_ ? now_seconds() - pending_navigation_created_at_ : 0.0},
      {"pending_navigation_reason", pending_navigation_reason_},
      {"failure_recoverable", current_state_ == "recoverable_failed"},
      {"failure_context", last_failure_context_},
      {"skipped_waypoints", skipped_waypoints_},
      {"localization_recovery_active", localization_recovery_active_},
      {"localization_auto_paused", localization_auto_paused_},
      {"localization_resume_pending", localization_resume_pending_},
      {"localization_recovery_reason", localization_recovery_reason_},
      {"localization_recovery_status", localization_recovery_last_status_},
      {"pause_source", current_pause_source_},
      {"pause_reason", current_pause_reason_},
      {"resume_mode", current_resume_mode_},
      {"waiting_for_obstacle_clear", obstacle_wait_active_},
      {"obstacle_wait_active", obstacle_wait_active_},
      {"obstacle_wait_duration", obstacle_wait_active_ && obstacle_wait_started_at_ > 0.0 ? now_seconds() - obstacle_wait_started_at_ : 0.0},
      {"obstacle_clear_confirm_count", obstacle_clear_confirm_count_},
      {"obstacle_clear_required_frames", obstacle_clear_required_frames_},
      {"front_obstacle_blocked", latest_front_obstacle_blocked_},
      {"front_obstacle_stats", latest_front_obstacle_stats_},
    };
    if (current_pose_) {
      status["current_pose"] = pose_to_dict(*current_pose_);
      status["current_pose"]["frame_id"] = current_pose_frame_;
    }
    if (!current_waypoint_.empty()) {
      status["current_goal"] = {
        {"waypoint_id", current_waypoint_.value("id", "")},
        {"waypoint_name", current_waypoint_.value("name", "")},
        {"position", current_waypoint_.value("position", json::array())},
        {"waypoint_index", current_waypoint_index_},
      };
      status["distance_to_goal"] = calculate_distance_to_waypoint();
    }
    if (current_velocity_) {
      status["current_velocity"] = {
        {"linear", {
          {"x", current_velocity_->linear.x},
          {"y", current_velocity_->linear.y},
          {"z", current_velocity_->linear.z},
        }},
        {"angular", {
          {"x", current_velocity_->angular.x},
          {"y", current_velocity_->angular.y},
          {"z", current_velocity_->angular.z},
        }},
      };
    }
    return status;
  }

  // ========================= 通用辅助：路点、位姿、速度、costmap =========================
  std::optional<json> find_waypoint_data_by_id(const std::string & id) const
  {
    if (id.empty()) {
      return std::nullopt;
    }
    if (waypoints_data_.contains(id) && waypoints_data_.at(id).is_object()) {
      return std::optional<json>{waypoints_data_.at(id)};
    }
    for (const auto & item : waypoints_data_.items()) {
      if (item.value().is_object() && item.value().contains(id) && item.value().at(id).is_object()) {
        return std::optional<json>{item.value().at(id)};
      }
    }
    for (const auto & item : waypoints_data_.items()) {
      if (!item.value().is_object()) {
        continue;
      }
      for (const auto & sub : item.value().items()) {
        if (sub.value().is_object() && sub.value().contains(id) && sub.value().at(id).is_object()) {
          return std::optional<json>{sub.value().at(id)};
        }
      }
    }
    return std::nullopt;
  }

  void merge_request_waypoints_data(const json & request)
  {
    const auto request_waypoints = request.value("waypoints_data", json::object());
    if (!request_waypoints.is_object()) {
      return;
    }
    for (auto it = request_waypoints.begin(); it != request_waypoints.end(); ++it) {
      if (it.value().is_object()) {
        waypoints_data_[it.key()] = it.value();
      }
    }
  }

  int count_cached_waypoints() const
  {
    int count = 0;
    for (const auto & item : waypoints_data_.items()) {
      if (item.value().is_object() && item.value().value("id", "") == item.key()) {
        count++;
      } else if (item.value().is_object()) {
        for (const auto & nested : item.value().items()) {
          if (nested.value().is_object()) {
            count++;
          }
        }
      }
    }
    return count;
  }

  geometry_msgs::msg::PoseStamped waypoint_to_pose_stamped(const json & waypoint)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = get_clock()->now();
    pose.header.frame_id = waypoint.value("frame_id", default_frame_id_);
    const auto position = json_number_array(waypoint, "position", {0.0, 0.0, 0.0});
    const auto orientation = json_number_array(waypoint, "orientation", {0.0, 0.0, 0.0, 1.0});
    pose.pose.position.x = position.size() > 0 ? position[0] : 0.0;
    pose.pose.position.y = position.size() > 1 ? position[1] : 0.0;
    pose.pose.position.z = position.size() > 2 ? position[2] : 0.0;
    pose.pose.orientation.x = orientation.size() > 0 ? orientation[0] : 0.0;
    pose.pose.orientation.y = orientation.size() > 1 ? orientation[1] : 0.0;
    pose.pose.orientation.z = orientation.size() > 2 ? orientation[2] : 0.0;
    pose.pose.orientation.w = orientation.size() > 3 ? orientation[3] : 1.0;
    return pose;
  }

  std::string get_waypoint_walk_direction(const json & waypoint) const
  {
    const auto properties = waypoint.value("properties", json::object());
    json direction = "forward";
    for (const auto & key : {"walk_direction", "navigation_direction", "drive_direction", "motion_direction"}) {
      if (properties.contains(key)) {
        direction = properties.at(key);
        break;
      }
    }
    if (direction == "forward" && waypoint.contains("walk_direction")) {
      direction = waypoint.at("walk_direction");
    }
    if (direction.is_boolean()) {
      return direction.get<bool>() ? "backward" : "forward";
    }
    std::string text = direction.is_string() ? direction.get<std::string>() : "forward";
    std::transform(text.begin(), text.end(), text.begin(), ::tolower);
    if (text == "backward" || text == "reverse" || text == "back" || text == "倒走" || text == "倒车" || text == "后退") {
      return "backward";
    }
    return "forward";
  }

  std::optional<geometry_msgs::msg::Pose> lookup_pose_in_frame(const std::string & target_frame)
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(
        target_frame, base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(pose_tf_timeout_sec_));
      geometry_msgs::msg::Pose pose;
      pose.position.x = transform.transform.translation.x;
      pose.position.y = transform.transform.translation.y;
      pose.position.z = transform.transform.translation.z;
      pose.orientation = transform.transform.rotation;
      return pose;
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }

  std::optional<geometry_msgs::msg::Pose> lookup_current_map_pose()
  {
    return lookup_pose_in_frame(map_frame_);
  }

  geometry_msgs::msg::Twist convert_fastlio_velocity_to_standard(const nav_msgs::msg::Odometry & msg)
  {
    auto deadzone = [](double v) { return std::abs(v) < 0.01 ? 0.0 : v; };
    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = deadzone(-msg.twist.twist.linear.z);
    velocity.linear.y = deadzone(msg.twist.twist.linear.x);
    velocity.linear.z = deadzone(-msg.twist.twist.linear.y);
    velocity.angular.x = deadzone(-msg.twist.twist.angular.z);
    velocity.angular.y = deadzone(msg.twist.twist.angular.x);
    velocity.angular.z = deadzone(-msg.twist.twist.angular.y);
    return velocity;
  }

  void update_pose_derived_speed(const nav_msgs::msg::Odometry & msg)
  {
    double stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
    if (stamp <= 0.0) {
      stamp = now_seconds();
    }
    const auto & p = msg.pose.pose.position;
    const std::array<double, 3> pose{p.x, p.y, p.z};
    if (!last_motion_pose_) {
      last_motion_pose_ = pose;
      last_motion_pose_time_ = stamp;
      pose_derived_speed_.reset();
      return;
    }
    const double dt = stamp - last_motion_pose_time_;
    if (dt <= 0.0 || dt > 2.0) {
      last_motion_pose_ = pose;
      last_motion_pose_time_ = stamp;
      pose_derived_speed_.reset();
      return;
    }
    const double dx = pose[0] - (*last_motion_pose_)[0];
    const double dz = pose[2] - (*last_motion_pose_)[2];
    pose_derived_speed_ = std::sqrt(dx * dx + dz * dz) / dt;
    last_motion_pose_ = pose;
    last_motion_pose_time_ = stamp;
  }

  std::optional<std::pair<double, std::string>> get_blockage_motion_speed() const
  {
    std::optional<double> twist_speed;
    if (current_velocity_) {
      const double linear = std::hypot(current_velocity_->linear.x, current_velocity_->linear.y);
      const double angular = std::abs(current_velocity_->angular.z);
      twist_speed = std::hypot(linear, angular);
    }
    std::optional<double> pose_speed = pose_derived_speed_;
    if (pose_speed && *pose_speed < blockage_pose_delta_deadzone_) {
      pose_speed = 0.0;
    }
    if (!pose_speed) {
      if (!twist_speed) {
        return std::nullopt;
      }
      return std::make_pair(*twist_speed, "odom_twist");
    }
    if (!twist_speed || *pose_speed >= *twist_speed) {
      return std::make_pair(*pose_speed, "pose_delta");
    }
    return std::make_pair(*twist_speed, "odom_twist");
  }

  std::pair<bool, json> analyze_front_obstacle_window(const nav_msgs::msg::OccupancyGrid & msg)
  {
    const auto robot_pose = lookup_pose_in_frame(msg.header.frame_id);
    if (!robot_pose) {
      return {true, {{"available", false}, {"reason", "missing_tf:" + msg.header.frame_id + "->" + base_frame_}}};
    }
    const double robot_x = robot_pose->position.x;
    const double robot_y = robot_pose->position.y;
    const double yaw = yaw_from_pose(*robot_pose);
    const double resolution = msg.info.resolution;
    const int width = static_cast<int>(msg.info.width);
    const int height = static_cast<int>(msg.info.height);
    const double origin_x = msg.info.origin.position.x;
    const double origin_y = msg.info.origin.position.y;
    int occupied = 0;
    int max_cost = 0;
    int samples = 0;
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    for (int y = 0; y < height; ++y) {
      const double world_y = origin_y + (y + 0.5) * resolution;
      for (int x = 0; x < width; ++x) {
        const double world_x = origin_x + (x + 0.5) * resolution;
        const double dx = world_x - robot_x;
        const double dy = world_y - robot_y;
        const double forward_x = cos_yaw * dx + sin_yaw * dy;
        const double lateral_y = -sin_yaw * dx + cos_yaw * dy;
        if (forward_x < obstacle_clear_front_min_x_m_ || forward_x > obstacle_clear_front_max_x_m_) {
          continue;
        }
        if (std::abs(lateral_y) > obstacle_clear_half_width_m_) {
          continue;
        }
        samples++;
        const int idx = y * width + x;
        if (idx < 0 || idx >= static_cast<int>(msg.data.size())) {
          continue;
        }
        const int cost = static_cast<int>(msg.data[idx]);
        max_cost = std::max(max_cost, cost);
        if (cost >= obstacle_clear_cost_threshold_) {
          occupied++;
        }
      }
    }
    const bool blocked = occupied > 0;
    return {blocked, {
      {"available", true},
      {"frame_id", msg.header.frame_id},
      {"window_front_min_x_m", round_3(obstacle_clear_front_min_x_m_)},
      {"window_front_max_x_m", round_3(obstacle_clear_front_max_x_m_)},
      {"window_half_width_m", round_3(obstacle_clear_half_width_m_)},
      {"sample_cells", samples},
      {"occupied_cells", occupied},
      {"max_cost", max_cost},
      {"blocked", blocked},
    }};
  }

  std::pair<bool, json> is_roi_obstacle_clear_for_resume(double now)
  {
    auto stats = build_roi_obstacle_stats(now);
    if (!obstacle_resume_use_roi_) {
      stats["decision"] = "disabled";
      return {true, stats};
    }
    if (!stats.value("fresh", false)) {
      stats["decision"] = "stale_fallback_to_costmap";
      return {true, stats};
    }
    if (latest_roi_obstacle_has_obstacle_.value_or(false)) {
      stats["decision"] = "roi_blocked";
      return {false, stats};
    }
    if (roi_obstacle_clear_confirm_count_ < obstacle_roi_required_clear_frames_) {
      stats["decision"] = "roi_clear_frames_not_enough";
      return {false, stats};
    }
    stats["decision"] = "roi_clear";
    return {true, stats};
  }

  json build_roi_obstacle_stats(std::optional<double> now = std::nullopt) const
  {
    const double current = now.value_or(now_seconds());
    const bool has_age = latest_roi_obstacle_stamp_ > 0.0;
    const double age = has_age ? current - latest_roi_obstacle_stamp_ : 0.0;
    return {
      {"enabled", obstacle_resume_use_roi_},
      {"topic", obstacle_roi_has_obstacle_topic_},
      {"has_obstacle", latest_roi_obstacle_has_obstacle_ ? json(*latest_roi_obstacle_has_obstacle_) : json(nullptr)},
      {"age_sec", has_age ? json(round_3(age)) : json(nullptr)},
      {"fresh", has_age && age <= obstacle_roi_timeout_sec_},
      {"clear_confirmed_frames", roi_obstacle_clear_confirm_count_},
      {"clear_required_frames", obstacle_roi_required_clear_frames_},
    };
  }

  // ========================= 失败、完成、状态重置 =========================
  void handle_navigation_failed(const std::string & reason)
  {
    if (current_state_ == "paused") {
      return;
    }
    if (navigation_failure_policy_ == "abort_all") {
      current_state_ = "failed";
      send_acknowledgment("navigation_failed", "error", reason);
      publish_status_update("navigation_failed", {
        {"reason", reason},
        {"failed_waypoint_index", current_waypoint_index_},
        {"failed_waypoint_id", current_waypoint_.value("id", "")},
      });
      reset_navigation_state();
      return;
    }
    if (navigation_failure_policy_ == "skip_failed_continue") {
      current_state_ = "recoverable_failed";
      last_failure_context_ = build_failure_recovery_context(reason);
      handle_skip_failed_waypoint(json::object());
      return;
    }
    cancel_navigation();
    current_state_ = "recoverable_failed";
    current_detailed_state_ = "RECOVERABLE_FAILED";
    current_goal_handle_.reset();
    reset_block_detection();
    last_failure_context_ = build_failure_recovery_context(reason);
    send_acknowledgment("navigation_failed", "error", reason, last_failure_context_);
    publish_status_update("navigation_failed", last_failure_context_);
    request_localization_recovery_for_failed_navigation(reason);
  }

  void handle_navigation_completed()
  {
    current_state_ = "completed";
    json context = {
      {"completed_waypoints", total_waypoints_},
      {"total_waypoints", total_waypoints_},
      {"navigation_mode", current_navigation_mode_.empty() ? json(nullptr) : json(current_navigation_mode_)},
      {"skipped_waypoints", skipped_waypoints_},
    };
    send_acknowledgment("navigation_completed", "success", "导航完成，共完成 " + std::to_string(total_waypoints_) + " 个点位", context);
    publish_status_update("navigation_completed", context);
    reset_navigation_state();
  }

  json build_failure_recovery_context(const std::string & reason = "") const
  {
    json remaining = json::array();
    for (int i = current_waypoint_index_; i < static_cast<int>(waypoint_ids_.size()); ++i) {
      remaining.push_back(waypoint_ids_[i]);
    }
    return {
      {"recoverable", true},
      {"reason", reason.empty() ? last_failure_context_.value("reason", "") : reason},
      {"failed_waypoint_index", current_waypoint_index_},
      {"failed_waypoint_id", current_waypoint_.value("id", "")},
      {"failed_waypoint_name", current_waypoint_.value("name", "")},
      {"completed_waypoints", current_waypoint_index_},
      {"total_waypoints", total_waypoints_},
      {"remaining_waypoint_ids", remaining},
      {"current_sequence_id", current_sequence_id_},
      {"navigation_mode", current_navigation_mode_.empty() ? json(nullptr) : json(current_navigation_mode_)},
      {"skipped_waypoints", skipped_waypoints_},
      {"available_actions", {"retry_failed_waypoint", "skip_failed_waypoint", "stop_navigation"}},
      {"resume_navigation_allowed", false},
    };
  }

  void reset_navigation_state()
  {
    current_state_ = "idle";
    current_detailed_state_ = "IDLE";
    current_navigation_mode_.clear();
    current_sequence_id_.clear();
    current_waypoint_index_ = 0;
    total_waypoints_ = 0;
    current_waypoint_ = json::object();
    waypoint_ids_.clear();
    skipped_waypoints_ = json::array();
    last_failure_context_ = json::object();
    navigation_start_time_ = 0.0;
    current_goal_handle_.reset();
    localization_recovery_active_ = false;
    localization_auto_paused_ = false;
    localization_resume_pending_ = false;
    localization_recovery_reason_.clear();
    localization_recovery_started_at_ = 0.0;
    localization_recovery_last_status_ = json::object();
    current_pause_source_.clear();
    current_pause_reason_.clear();
    current_resume_mode_.clear();
    clear_obstacle_wait_state();
    reset_block_detection();
    nav2_blockage_suppression_nodes_.clear();
    distance_remaining_ = std::numeric_limits<double>::infinity();
    estimated_time_remaining_ = 0.0;
  }

  // ========================= 小工具函数 =========================
  void nav2_log_callback(const nav2_msgs::msg::BehaviorTreeLog & msg)
  {
    if (current_state_ != "executing" && current_state_ != "planning") {
      nav2_blockage_suppression_nodes_.clear();
      if (current_detailed_state_ == "RECOVERING" || current_detailed_state_ == "TURNING") {
        current_detailed_state_ = state_to_detail_default(current_state_);
      }
      return;
    }
    for (const auto & event : msg.event_log) {
      if (is_nav2_blockage_suppression_node(event.node_name)) {
        if (event.current_status == "RUNNING") {
          nav2_blockage_suppression_nodes_.insert(event.node_name);
          current_detailed_state_ = event.node_name == "SpinToPose" ? "TURNING" : "RECOVERING";
        } else {
          nav2_blockage_suppression_nodes_.erase(event.node_name);
          if (nav2_blockage_suppression_nodes_.empty() &&
            (current_detailed_state_ == "RECOVERING" || current_detailed_state_ == "TURNING"))
          {
            current_detailed_state_ = current_state_ == "executing" ? "EXECUTING" : state_to_detail_default(current_state_);
          }
        }
      }
      if (event.node_name.find("ComputePathToPose") != std::string::npos && event.current_status == "FAILURE") {
        current_detailed_state_ = "PLANNING_FAILED";
      }
    }
  }

  bool is_nav2_blockage_suppression_node(const std::string & node_name) const
  {
    return node_name == "SpinToPose" || node_name == "Spin" || node_name == "BackUp";
  }

  std::optional<std::string> get_obstacle_blockage_suppression_reason() const
  {
    if (robot_motion_busy_) {
      return robot_current_motion_.empty() ? "机器人动作执行阶段" : "机器人动作执行阶段(" + robot_current_motion_ + ")";
    }
    if (robot_control_state_ == "Menu") {
      return "机器人动作库模式";
    }
    if (!nav2_blockage_suppression_nodes_.empty()) {
      std::string active;
      for (const auto & node : nav2_blockage_suppression_nodes_) {
        active += active.empty() ? node : ", " + node;
      }
      return "Nav2主动转向/后退阶段(" + active + ")";
    }
    if (obstacle_block_near_goal_distance_ <= 0.0) {
      return std::nullopt;
    }
    if (std::isfinite(distance_remaining_) && distance_remaining_ <= obstacle_block_near_goal_distance_) {
      return "接近目标点阶段(剩余路径 " + format_2(distance_remaining_) + "m)";
    }
    const double distance = calculate_distance_to_waypoint();
    if (std::isfinite(distance) && distance <= obstacle_block_near_goal_distance_) {
      return "接近目标点阶段(直线距离 " + format_2(distance) + "m)";
    }
    return std::nullopt;
  }

  double calculate_distance_to_waypoint() const
  {
    if (!current_pose_ || current_waypoint_.empty()) {
      return std::numeric_limits<double>::infinity();
    }
    const auto position = json_number_array(current_waypoint_, "position", {0.0, 0.0, 0.0});
    const double dx = current_pose_->position.x - (position.size() > 0 ? position[0] : 0.0);
    const double dy = current_pose_->position.y - (position.size() > 1 ? position[1] : 0.0);
    const double dz = current_pose_->position.z - (position.size() > 2 ? position[2] : 0.0);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  double calculate_progress_percentage() const
  {
    if (total_waypoints_ == 0) {
      return 0.0;
    }
    if (current_state_ == "completed") {
      return 100.0;
    }
    double progress = (static_cast<double>(current_waypoint_index_) / std::max(total_waypoints_, 1)) * 100.0;
    if (current_state_ == "executing" && current_pose_ && !current_waypoint_.empty()) {
      const double distance = calculate_distance_to_waypoint();
      constexpr double max_distance = 10.0;
      if (distance < max_distance) {
        const double waypoint_progress = (1.0 - std::min(distance / max_distance, 1.0)) * (100.0 / total_waypoints_);
        progress = std::min(progress + waypoint_progress, 100.0);
      }
    }
    return round_1(progress);
  }

  bool is_navigation_active() const
  {
    return current_state_ == "planning" || current_state_ == "executing" ||
           current_state_ == "paused" || current_state_ == "recoverable_failed";
  }

  void check_navigation_status()
  {
    if (current_state_ == "executing" && current_pose_ && !current_waypoint_.empty()) {
      last_known_distance_ = calculate_distance_to_waypoint();
    }
  }

  void check_timeout()
  {
    if (current_state_ == "executing" && !current_waypoint_.empty() &&
      now_seconds() - current_waypoint_start_time_ > waypoint_timeout_)
    {
      handle_navigation_failed("路点导航超时");
    }
  }

  void publish_zero_cmd_vel()
  {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
  }

  void begin_localization_stop_hold()
  {
    localization_stop_until_ = std::max(localization_stop_until_, now_seconds() + std::max(0.0, localization_stop_hold_sec_));
    for (int i = 0; i < 3; ++i) {
      publish_zero_cmd_vel();
    }
  }

  void enforce_localization_stop()
  {
    const bool localization_pause_active =
      localization_auto_paused_ && current_state_ == "paused" && current_detailed_state_ == "LOCALIZATION_RECOVERY";
    if (localization_pause_active || now_seconds() < localization_stop_until_) {
      publish_zero_cmd_vel();
    }
  }

  json build_pause_event_data(const std::string & source, const std::string & reason, const std::string & resume_mode) const
  {
    json location = nullptr;
    if (current_pose_) {
      location = {
        {"x", current_pose_->position.x},
        {"y", current_pose_->position.y},
        {"z", current_pose_->position.z},
      };
    }
    return {
      {"pause_source", source},
      {"reason", reason},
      {"resume_mode", resume_mode},
      {"waiting_for_obstacle_clear", source == "obstacle_wait"},
      {"pause_location", location},
      {"pause_time", pause_time_},
      {"pause_duration", pause_duration_limit_},
      {"current_waypoint_id", current_waypoint_.value("id", "")},
      {"current_waypoint_name", current_waypoint_.value("name", "")},
      {"waypoint_index", current_waypoint_index_},
      {"total_waypoints", total_waypoints_},
    };
  }

  json build_localization_pause_context(const json & status) const
  {
    auto event = build_pause_event_data("localization_recovery", status.value("reason", "定位异常，正在重定位"), "auto");
    event["waiting_for_obstacle_clear"] = false;
    event["localization_event"] = status.value("event_type", "");
    event["recovery_count"] = status.value("recovery_count", 0);
    event["use_prior"] = status.value("use_prior", false);
    return event;
  }

  json localization_recovery_event_context() const
  {
    return {
      {"reason", localization_recovery_reason_},
      {"current_waypoint_id", current_waypoint_.value("id", "")},
      {"current_waypoint_name", current_waypoint_.value("name", "")},
      {"waypoint_index", current_waypoint_index_},
      {"total_waypoints", total_waypoints_},
      {"auto_resume_pending", localization_auto_paused_},
    };
  }

  std::pair<bool, json> should_reverse_resume_to_current_waypoint() const
  {
    json context = {{"reverse_resume_selected", false}, {"reverse_resume_reason", ""}};
    if (!localization_resume_reverse_enabled_) {
      context["reverse_resume_reason"] = "disabled";
      return {false, context};
    }
    if (!current_pose_ || current_waypoint_.empty()) {
      context["reverse_resume_reason"] = "missing current pose or waypoint";
      return {false, context};
    }
    const auto target = json_number_array(current_waypoint_, "position", {});
    if (target.size() < 2) {
      context["reverse_resume_reason"] = "current waypoint has no valid position";
      return {false, context};
    }
    const double dx = target[0] - current_pose_->position.x;
    const double dy = target[1] - current_pose_->position.y;
    const double distance = std::hypot(dx, dy);
    context["reverse_resume_distance_m"] = round_3(distance);
    if (distance <= std::max(position_tolerance_, 0.05)) {
      context["reverse_resume_reason"] = "already within waypoint tolerance";
      return {false, context};
    }
    if (distance > std::max(0.0, localization_resume_reverse_max_distance_m_)) {
      context["reverse_resume_reason"] = "waypoint too far for reverse resume: " + format_2(distance) + "m";
      return {false, context};
    }
    const double robot_yaw = yaw_from_pose(*current_pose_);
    const double target_bearing = std::atan2(dy, dx);
    const double rear_error = std::abs(normalize_angle(target_bearing - robot_yaw - M_PI));
    context["reverse_resume_rear_angle_deg"] = round_1(rear_error * 180.0 / M_PI);
    if (rear_error > std::max(0.0, localization_resume_reverse_rear_angle_rad_)) {
      context["reverse_resume_reason"] = "waypoint is not behind robot: rear_error=" + format_1(rear_error * 180.0 / M_PI) + "deg";
      return {false, context};
    }
    context["reverse_resume_selected"] = true;
    context["reverse_resume_reason"] = "current waypoint is behind robot after localization recovery";
    context["reverse_resume_behavior_tree"] = reverse_navigation_bt_xml_;
    return {true, context};
  }

  void record_last_succeeded_waypoint(const json & waypoint, int index)
  {
    last_succeeded_waypoint_ = waypoint;
    last_succeeded_waypoint_index_ = index;
    last_succeeded_pose_ = current_pose_ ? pose_to_dict(*current_pose_) : json(nullptr);
    last_succeeded_time_ = now_seconds();
  }

  void request_localization_recovery_for_failed_navigation(const std::string & reason)
  {
    if (!request_localization_recovery_on_nav_failure_ || current_waypoint_.empty()) {
      return;
    }
    const double now = now_seconds();
    if (now - last_localization_recovery_request_time_ < localization_recovery_request_cooldown_sec_) {
      return;
    }
    const auto payload = build_navigation_context_recovery_request(
      reason,
      "navigation_failure",
      localization_recovery_prior_radius_m_,
      std::nullopt,
      "navigation_failure_recovery_request");
    if (!payload) {
      return;
    }
    publish_localization_recovery_request(*payload);
    last_localization_recovery_request_time_ = now;
  }

  void request_navigation_context_recovery_for_localization(const std::string & reason, const json & status)
  {
    if (!request_navigation_context_recovery_on_localization_failure_ || current_waypoint_.empty()) {
      return;
    }
    const double now = now_seconds();
    const std::string current_id = current_waypoint_.value("id", "");
    const int recovery_count = status.value("recovery_count", 0);
    const std::string request_key =
      current_sequence_id_ + ":" + std::to_string(current_waypoint_index_) + ":" +
      current_id + ":" + std::to_string(recovery_count);
    if (request_key == last_navigation_context_recovery_key_) {
      return;
    }
    if (now - last_navigation_context_recovery_request_time_ < localization_context_recovery_request_cooldown_sec_) {
      return;
    }
    const auto payload = build_navigation_context_recovery_request(
      reason,
      "localization_failure",
      localization_context_prior_radius_m_,
      std::optional<json>{status},
      "localization_failure_navigation_context_recovery_request");
    if (!payload) {
      return;
    }
    publish_localization_recovery_request(*payload);
    last_navigation_context_recovery_request_time_ = now;
    last_navigation_context_recovery_key_ = request_key;
  }

  void publish_localization_recovery_request(const json & payload)
  {
    std_msgs::msg::String msg;
    msg.data = json_string(payload);
    localization_recovery_request_pub_->publish(msg);
    publish_status_update("navigation_localization_recovery_requested", payload);
  }

  std::optional<json> build_navigation_context_recovery_request(
    const std::string & reason,
    const std::string & trigger_event,
    double radius_m,
    const std::optional<json> & status,
    const std::string & event_type) const
  {
    const auto current_position = waypoint_position_tuple(current_waypoint_);
    if (current_waypoint_.empty() || !current_position) {
      return std::nullopt;
    }

    const double now = now_seconds();
    auto previous = resolve_previous_waypoint_context(now);
    const auto previous_position = waypoint_position_tuple(previous.waypoint);
    auto next = resolve_next_waypoint_context();

    std::string selected_prior_source = "navigation_context_current_goal";
    json selected_prior = {
      {"position", {{"x", (*current_position)[0]}, {"y", (*current_position)[1]}, {"z", (*current_position)[2]}}},
      {"orientation", waypoint_orientation_object(current_waypoint_)},
    };
    json selected_prior_meta = {
      {"method", "current_goal"},
      {"previous_source", previous.source},
    };

    if (previous_position) {
      const double dx = (*current_position)[0] - (*previous_position)[0];
      const double dy = (*current_position)[1] - (*previous_position)[1];
      const double dz = (*current_position)[2] - (*previous_position)[2];
      const double segment_length = std::hypot(dx, dy);
      if (segment_length >= std::max(0.0, localization_context_prior_min_segment_length_m_)) {
        const double reference_x = current_pose_ ? current_pose_->position.x : (*current_position)[0];
        const double reference_y = current_pose_ ? current_pose_->position.y : (*current_position)[1];
        const double projection =
          ((reference_x - (*previous_position)[0]) * dx + (reference_y - (*previous_position)[1]) * dy) /
          std::max(segment_length * segment_length, 1e-6);
        const double projection_clamped = std::max(0.0, std::min(1.0, projection));
        const double prior_x = (*previous_position)[0] + projection_clamped * dx;
        const double prior_y = (*previous_position)[1] + projection_clamped * dy;
        const double prior_z = (*previous_position)[2] + projection_clamped * dz;
        const double prior_yaw = std::atan2(dy, dx);

        selected_prior_source = "navigation_context_segment";
        selected_prior = {
          {"position", {{"x", prior_x}, {"y", prior_y}, {"z", prior_z}}},
          {"orientation", quaternion_from_yaw(prior_yaw)},
        };
        selected_prior_meta = {
          {"method", "projected_previous_to_current_segment"},
          {"previous_source", previous.source},
          {"segment_length_m", round_3(segment_length)},
          {"projection_ratio", round_3(projection_clamped)},
          {"raw_projection_ratio", round_3(projection)},
          {"reference_pose_source", current_pose_ ? "current_map_pose" : "current_goal"},
        };
      }
    }

    radius_m = std::max(0.0, radius_m);
    json payload = {
      {"event_type", event_type},
      {"source", "navigation_state_manager"},
      {"reason", reason},
      {"timestamp", now},
      {"trigger_event", trigger_event},
      {"prior_source", selected_prior_source},
      {"prior_frame_id", current_waypoint_.value("frame_id", default_frame_id_)},
      {"prior_pose", selected_prior},
      {"search_radius_m", radius_m},
      {"prior_max_xy_m", radius_m},
      {"allow_full_global_fallback", true},
      {"failed_waypoint_index", current_waypoint_index_},
      {"failed_waypoint_id", current_waypoint_.value("id", "")},
      {"failed_waypoint_name", current_waypoint_.value("name", "")},
      {"current_pose", current_pose_ ? pose_to_dict(*current_pose_) : json(nullptr)},
      {"current_detailed_state", current_detailed_state_},
      {"navigation_context", {
        {"current_waypoint", waypoint_context_dict(current_waypoint_, current_waypoint_index_)},
        {"previous_waypoint", waypoint_context_dict(previous.waypoint, previous.index)},
        {"next_waypoint", waypoint_context_dict(next.waypoint, next.index)},
        {"selected_prior", selected_prior_meta},
        {"current_sequence_id", current_sequence_id_},
        {"navigation_mode", current_navigation_mode_.empty() ? json(nullptr) : json(current_navigation_mode_)},
        {"total_waypoints", total_waypoints_},
      }},
    };
    if (status && status->is_object()) {
      payload["localization_status"] = {
        {"event_type", status->value("event_type", "")},
        {"recovery_count", status->value("recovery_count", 0)},
        {"relocalize_attempts", status->value("relocalize_attempts", 0)},
        {"prior_reason", status->value("prior_reason", "")},
      };
    }
    return payload;
  }

  struct WaypointContext
  {
    json waypoint = nullptr;
    int index = -1;
    std::string source = "none";
  };

  WaypointContext resolve_previous_waypoint_context(double now) const
  {
    if (!waypoint_ids_.empty() && current_waypoint_index_ > 0) {
      const int previous_index = current_waypoint_index_ - 1;
      const auto previous_waypoint = find_waypoint_data_by_id(waypoint_ids_[previous_index]);
      if (previous_waypoint) {
        return {*previous_waypoint, previous_index, "current_sequence_previous"};
      }
    }
    if (last_succeeded_waypoint_.is_null() || last_succeeded_waypoint_.empty()) {
      return {};
    }
    const std::string current_id = current_waypoint_.value("id", "");
    const std::string previous_id = last_succeeded_waypoint_.value("id", "");
    if (!current_id.empty() && !previous_id.empty() && current_id == previous_id) {
      return {};
    }
    const double age = last_succeeded_time_ > 0.0 ? now - last_succeeded_time_ : std::numeric_limits<double>::infinity();
    if (localization_context_prior_max_previous_age_sec_ > 0.0 &&
      age > localization_context_prior_max_previous_age_sec_)
    {
      return {nullptr, -1, "last_succeeded_too_old"};
    }
    return {last_succeeded_waypoint_, last_succeeded_waypoint_index_, "last_succeeded_waypoint"};
  }

  WaypointContext resolve_next_waypoint_context() const
  {
    const int next_index = current_waypoint_index_ + 1;
    if (waypoint_ids_.empty() || next_index < 0 || next_index >= static_cast<int>(waypoint_ids_.size())) {
      return {};
    }
    const auto next_waypoint = find_waypoint_data_by_id(waypoint_ids_[next_index]);
    if (!next_waypoint) {
      return {};
    }
    return {*next_waypoint, next_index, "current_sequence_next"};
  }

  std::optional<std::array<double, 3>> waypoint_position_tuple(const json & waypoint) const
  {
    if (!waypoint.is_object()) {
      return std::nullopt;
    }
    const auto position = json_number_array(waypoint, "position", {});
    if (position.size() < 2) {
      return std::nullopt;
    }
    return std::array<double, 3>{position[0], position[1], position.size() >= 3 ? position[2] : 0.0};
  }

  json waypoint_context_dict(const json & waypoint, int index) const
  {
    const auto position = waypoint_position_tuple(waypoint);
    if (!waypoint.is_object() || !position) {
      return nullptr;
    }
    return {
      {"id", waypoint.value("id", "")},
      {"name", waypoint.value("name", "")},
      {"index", index},
      {"frame_id", waypoint.value("frame_id", default_frame_id_)},
      {"position", {{"x", (*position)[0]}, {"y", (*position)[1]}, {"z", (*position)[2]}}},
    };
  }

  json quaternion_from_yaw(double yaw) const
  {
    const double half_yaw = yaw * 0.5;
    return {
      {"x", 0.0},
      {"y", 0.0},
      {"z", std::sin(half_yaw)},
      {"w", std::cos(half_yaw)},
    };
  }

  json pose_to_dict(const geometry_msgs::msg::Pose & pose) const
  {
    return {
      {"position", {{"x", pose.position.x}, {"y", pose.position.y}, {"z", pose.position.z}}},
      {"orientation", {
        {"x", pose.orientation.x},
        {"y", pose.orientation.y},
        {"z", pose.orientation.z},
        {"w", pose.orientation.w},
      }},
    };
  }

  json waypoint_position_object(const json & waypoint) const
  {
    const auto position = json_number_array(waypoint, "position", {0.0, 0.0, 0.0});
    return {
      {"x", position.size() > 0 ? position[0] : 0.0},
      {"y", position.size() > 1 ? position[1] : 0.0},
      {"z", position.size() > 2 ? position[2] : 0.0},
    };
  }

  json waypoint_orientation_object(const json & waypoint) const
  {
    const auto orientation = json_number_array(waypoint, "orientation", {0.0, 0.0, 0.0, 1.0});
    return {
      {"x", orientation.size() > 0 ? orientation[0] : 0.0},
      {"y", orientation.size() > 1 ? orientation[1] : 0.0},
      {"z", orientation.size() > 2 ? orientation[2] : 0.0},
      {"w", orientation.size() > 3 ? orientation[3] : 1.0},
    };
  }

  void clear_obstacle_wait_state()
  {
    obstacle_wait_active_ = false;
    obstacle_wait_started_at_ = 0.0;
    obstacle_wait_last_push_time_ = 0.0;
    obstacle_clear_confirm_count_ = 0;
  }

  void reset_block_detection()
  {
    is_blocked_by_obstacle_ = false;
    block_start_time_ = 0.0;
    block_reported_ = false;
    clear_block_recovery_candidate();
    if (current_detailed_state_ == "BLOCKED_BY_OBSTACLE") {
      current_detailed_state_ = "EXECUTING";
    }
  }

  void clear_block_recovery_candidate()
  {
    block_recovery_candidate_start_time_.reset();
    block_recovery_candidate_source_.clear();
  }

  bool has_confirmed_blockage_recovery(double velocity, const std::string & source)
  {
    if (velocity < blockage_recovery_velocity_threshold_) {
      clear_block_recovery_candidate();
      return false;
    }
    const double now = now_seconds();
    if (!block_recovery_candidate_start_time_) {
      block_recovery_candidate_start_time_ = now;
      block_recovery_candidate_source_ = source;
      return blockage_recovery_confirm_sec_ <= 0.0;
    }
    return now - *block_recovery_candidate_start_time_ >= blockage_recovery_confirm_sec_;
  }

  bool is_stopped_for_blockage(double velocity, const std::string & source) const
  {
    if (source == "pose_delta") {
      return velocity < blockage_recovery_velocity_threshold_;
    }
    return velocity < velocity_threshold_;
  }

  double get_current_obstacle_clear_required_duration() const
  {
    if (obstacle_recent_false_resume_count_ > 0) {
      return std::max(obstacle_clear_required_duration_sec_, obstacle_clear_required_duration_after_false_resume_sec_);
    }
    return obstacle_clear_required_duration_sec_;
  }

  void publish_obstacle_blocked_event(double block_duration, bool send_ack)
  {
    json event = {
      {"reason", "检测到障碍物，前方路径被挡住"},
      {"block_duration", round_1(block_duration)},
      {"blocked_waypoint_id", current_waypoint_.value("id", "")},
      {"blocked_waypoint_name", current_waypoint_.value("name", "")},
      {"blocked_waypoint_index", current_waypoint_index_},
      {"total_waypoints", total_waypoints_},
      {"position", current_waypoint_.value("position", json::array())},
      {"waiting_for_obstacle_clear", true},
      {"clear_confirmed_frames", obstacle_clear_confirm_count_},
      {"clear_required_frames", obstacle_clear_required_frames_},
      {"clear_duration_sec", obstacle_clear_started_at_ > 0.0 ? round_1(now_seconds() - obstacle_clear_started_at_) : 0.0},
      {"clear_required_duration_sec", get_current_obstacle_clear_required_duration()},
      {"min_wait_before_resume_sec", obstacle_min_wait_before_resume_sec_},
      {"false_resume_count", obstacle_recent_false_resume_count_},
      {"pause_source", "obstacle_wait"},
      {"front_obstacle_stats", latest_front_obstacle_stats_},
      {"roi_obstacle_stats", build_roi_obstacle_stats()},
    };
    publish_status_update("navigation_obstacle_blocked", event);
    if (send_ack) {
      send_acknowledgment("navigation_obstacle_blocked", "error", "检测到障碍物，前方路径被挡住");
    }
  }

  bool is_localization_resume_blocked() const
  {
    return localization_recovery_active_ || localization_auto_paused_ ||
           localization_resume_pending_ || current_detailed_state_ == "LOCALIZATION_RECOVERY";
  }

  void reject_resume_during_localization_recovery()
  {
    const std::string reason = "定位恢复中，暂不能手动恢复导航；定位准确且 TF 恢复后系统会自动继续未完成导航";
    json event = localization_recovery_event_context();
    event["reason"] = reason;
    event["blocked_command"] = "resume_navigation";
    event["manual_resume_rejected"] = true;
    event["auto_resume_pending"] = localization_auto_paused_ || localization_resume_pending_;
    send_acknowledgment("resume_navigation", "error", reason, event);
    publish_status_update("navigation_localization_resume_waiting", event);
  }

  static double round_1(double value) { return std::round(value * 10.0) / 10.0; }
  static double round_2(double value) { return std::round(value * 100.0) / 100.0; }
  static double round_3(double value) { return std::round(value * 1000.0) / 1000.0; }
  static std::string format_1(double value) { char b[64]; std::snprintf(b, sizeof(b), "%.1f", value); return b; }
  static std::string format_2(double value) { char b[64]; std::snprintf(b, sizeof(b), "%.2f", value); return b; }

  // 参数
  double position_tolerance_{0.15};
  double orientation_tolerance_{0.3};
  double waypoint_timeout_{300.0};
  double status_publish_rate_{2.0};
  std::string default_frame_id_{"map"};
  double obstacle_block_timeout_{4.0};
  double velocity_threshold_{0.10};
  double blockage_pose_delta_deadzone_{0.10};
  double blockage_recovery_velocity_threshold_{0.15};
  double blockage_recovery_confirm_sec_{1.0};
  bool obstacle_wait_enable_{true};
  double obstacle_wait_push_interval_sec_{4.0};
  int obstacle_clear_required_frames_{5};
  double obstacle_clear_check_rate_hz_{5.0};
  int obstacle_clear_cost_threshold_{100};
  double obstacle_clear_front_min_x_m_{0.15};
  double obstacle_clear_front_max_x_m_{0.80};
  double obstacle_clear_half_width_m_{0.30};
  double obstacle_min_wait_before_resume_sec_{2.0};
  double obstacle_clear_required_duration_sec_{3.0};
  double obstacle_clear_required_duration_after_false_resume_sec_{4.0};
  double obstacle_false_resume_window_sec_{3.0};
  bool obstacle_resume_use_roi_{true};
  std::string obstacle_roi_has_obstacle_topic_{"/front_obstacle/has_obstacle"};
  double obstacle_roi_timeout_sec_{1.0};
  int obstacle_roi_required_clear_frames_{3};
  std::string local_costmap_topic_{"/local_costmap/costmap"};
  bool require_walk_mode_for_navigation_{true};
  double robot_status_timeout_{2.0};
  double pending_navigation_timeout_{90.0};
  double obstacle_block_near_goal_distance_{0.7};
  std::string navigation_failure_policy_{"pause_on_failed"};
  bool auto_pause_on_localization_recovery_{false};
  double localization_stop_hold_sec_{2.0};
  double localization_resume_settle_sec_{1.0};
  bool localization_auto_resume_require_recovery_done_{true};
  int localization_resume_stable_frames_{3};
  std::string localization_health_status_topic_{"/localization/prior_map_odom_bridge_status"};
  double localization_health_timeout_sec_{3.0};
  bool localization_allow_start_with_last_good_tf_{true};
  double localization_last_good_tf_max_age_sec_{0.0};
  std::string localization_recovery_status_topic_{"/localization/recovery_status"};
  std::string localization_recovery_request_topic_{"/localization/recovery_requests"};
  bool request_localization_recovery_on_nav_failure_{false};
  bool request_navigation_context_recovery_on_localization_failure_{false};
  double localization_recovery_request_cooldown_sec_{20.0};
  double localization_recovery_prior_radius_m_{10.0};
  double localization_context_recovery_request_cooldown_sec_{4.0};
  double localization_context_prior_radius_m_{5.0};
  double localization_context_prior_max_previous_age_sec_{300.0};
  double localization_context_prior_min_segment_length_m_{0.2};
  bool localization_resume_reverse_enabled_{true};
  double localization_resume_reverse_max_distance_m_{2.0};
  double localization_resume_reverse_rear_angle_rad_{70.0 * M_PI / 180.0};
  std::string map_frame_{"map"};
  std::string base_frame_{"base_footprint"};
  double pose_tf_timeout_sec_{0.05};
  std::string reverse_navigation_bt_xml_;

  // 运行状态
  std::string current_state_{"idle"};
  std::string current_detailed_state_{"IDLE"};
  std::string current_navigation_mode_;
  std::string current_sequence_id_;
  int current_waypoint_index_{0};
  int total_waypoints_{0};
  json current_waypoint_ = json::object();
  std::vector<std::string> waypoint_ids_;
  json skipped_waypoints_ = json::array();
  json last_failure_context_ = json::object();
  json waypoints_data_ = json::object();
  double navigation_start_time_{0.0};
  double current_waypoint_start_time_{0.0};
  double pause_time_{0.0};
  double pause_duration_limit_{0.0};
  std::string current_pause_source_;
  std::string current_pause_reason_;
  std::string current_resume_mode_;

  std::optional<geometry_msgs::msg::Pose> current_pose_;
  std::string current_pose_frame_{"map"};
  std::optional<geometry_msgs::msg::Twist> current_velocity_;
  std::optional<double> pose_derived_speed_;
  std::optional<std::array<double, 3>> last_motion_pose_;
  double last_motion_pose_time_{0.0};
  double last_pose_update_{0.0};
  double last_known_distance_{0.0};
  std::string robot_control_state_{"Unknown"};
  bool robot_motion_busy_{false};
  std::string robot_current_motion_;
  bool robot_ready_for_navigation_{false};
  double last_robot_status_update_{0.0};
  std::optional<json> pending_navigation_request_;
  double pending_navigation_created_at_{0.0};
  std::string pending_navigation_reason_;

  bool is_blocked_by_obstacle_{false};
  double block_start_time_{0.0};
  bool block_reported_{false};
  std::optional<double> block_recovery_candidate_start_time_;
  std::string block_recovery_candidate_source_;
  bool obstacle_wait_active_{false};
  double obstacle_wait_started_at_{0.0};
  double obstacle_wait_last_push_time_{0.0};
  int obstacle_clear_confirm_count_{0};
  double obstacle_clear_started_at_{0.0};
  double last_obstacle_resume_time_{0.0};
  int obstacle_recent_false_resume_count_{0};
  bool latest_front_obstacle_blocked_{false};
  json latest_front_obstacle_stats_ = json::object();
  double latest_local_costmap_stamp_{0.0};
  std::optional<bool> latest_roi_obstacle_has_obstacle_;
  double latest_roi_obstacle_stamp_{0.0};
  int roi_obstacle_clear_confirm_count_{0};
  std::set<std::string> nav2_blockage_suppression_nodes_;
  double distance_remaining_{std::numeric_limits<double>::infinity()};
  double estimated_time_remaining_{0.0};

  bool localization_recovery_active_{false};
  bool localization_auto_paused_{false};
  bool localization_resume_pending_{false};
  std::string localization_recovery_reason_;
  double localization_recovery_started_at_{0.0};
  json localization_recovery_last_status_ = json::object();
  double localization_stop_until_{0.0};
  double localization_recovered_at_{0.0};
  bool localization_recovery_done_{false};
  int localization_resume_stable_count_{0};
  json last_localization_status_summary_ = json::object();
  bool localization_has_last_good_tf_{false};
  double localization_last_good_tf_time_{0.0};
  double last_resume_wait_log_time_{0.0};
  double last_localization_recovery_request_time_{0.0};
  double last_navigation_context_recovery_request_time_{0.0};
  std::string last_navigation_context_recovery_key_;
  json last_succeeded_waypoint_ = json::object();
  int last_succeeded_waypoint_index_{-1};
  json last_succeeded_pose_ = nullptr;
  double last_succeeded_time_{0.0};
  bool is_localization_healthy_{false};
  int localization_healthy_count_{0};
  double localization_last_status_time_{0.0};

  // ROS 对象
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  GoalHandleNavigate::SharedPtr current_goal_handle_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_ack_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr navigation_goal_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr navigation_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr localization_recovery_request_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_request_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr waypoints_data_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr roi_obstacle_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_status_sub_;
  rclcpp::Subscription<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr nav2_behavior_log_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_recovery_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_status_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr navigation_check_timer_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
  rclcpp::TimerBase::SharedPtr pending_timer_;
  rclcpp::TimerBase::SharedPtr localization_resume_timer_;
  rclcpp::TimerBase::SharedPtr localization_stop_timer_;
  rclcpp::TimerBase::SharedPtr obstacle_wait_timer_;
  rclcpp::TimerBase::SharedPtr localization_timeout_timer_;
  rclcpp::TimerBase::SharedPtr next_waypoint_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationStateManagerCpp>());
  rclcpp::shutdown();
  return 0;
}
