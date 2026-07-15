/*
 * navigation_state_manager.cpp
 *
 * 文件用途：
 * 1. 这是路线任务运行层的 ROS Node 外壳，负责参数加载、topic/action/timer 绑定和模块调度。
 * 2. 具体业务被拆到独立模块：
 *    route_task_protocol：JSON 协议解析与组包辅助。
 *    navigation_gatekeeper：机器人/定位/地图启动门控。
 *    route_task_state_machine：路线任务命令状态机边界。
 *    obstacle_wait_manager：障碍等待和恢复判断边界。
 *    nav2_action_controller：Nav2 action 控制器边界。
 * 3. 在 route_task.nav2_execution_enable=true 时下发 Nav2 goal，并用 generation 隔离旧 goal 回调。
 * 4. 在 route_task.nav2_execution_enable=false 时只维护协议、状态和事件，方便离线验证调试。
 *
 * 代码块顺序：
 * 1. 头文件和类型别名。
 * 2. 节点初始化：参数、发布订阅、action client、定时器。
 * 3. 输入链路：导航请求、点位库、定位、地图、机器人、odom、costmap、ROI、BT 日志。
 * 4. 输出链路：ack、状态事件、周期状态、零速度。
 * 5. 运行态工具：事件 ID、待执行超时、活动状态判断。
 * 6. main 入口。
 */

#include <chrono>
#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "action_msgs/msg/goal_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "humanoid_route_runtime/nav2_action_controller.hpp"
#include "humanoid_route_runtime/navigation_gatekeeper.hpp"
#include "humanoid_route_runtime/obstacle_wait_manager.hpp"
#include "humanoid_route_runtime/route_runtime_types.hpp"
#include "humanoid_route_runtime/route_task_protocol.hpp"
#include "humanoid_route_runtime/route_task_state_machine.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/msg/behavior_tree_log.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rapidjson/document.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

namespace humanoid_route_runtime
{

class NavigationStateManagerNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using GoalHandleNavigateThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

  explicit NavigationStateManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("navigation_state_manager_cpp", options),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameters();
    load_parameters();
    nav2_controller_.configure(config_.navigate_to_pose_action, config_.navigate_through_poses_action);
    setup_interfaces();

    RCLCPP_WARN(
      get_logger(),
      "navigation_state_manager_cpp 已启动：route_task.nav2_execution_enable=%s",
      config_.route_task_nav2_execution_enable ? "true" : "false");
  }

  ~NavigationStateManagerNode() override
  {
    try {
      if (!rclcpp::ok()) {
        return;
      }
      if (!is_navigation_active()) {
        return;
      }
      cancel_active_route_goal();
      publish_zero_cmd_vel();
      rapidjson::Document event;
      event.SetObject();
      auto & allocator = event.GetAllocator();
      event.AddMember("reason", "node_shutdown", allocator);
      event.AddMember("route_task", route_.active, allocator);
      event.AddMember(
        "task_session_id", rapidjson::Value(route_.task_session_id.c_str(), allocator).Move(), allocator);
      event.AddMember("route_id", rapidjson::Value(route_.route_id.c_str(), allocator).Move(), allocator);
      publish_status_update("navigation_stopped", event);
    } catch (const std::exception & exc) {
      RCLCPP_WARN(get_logger(), "节点退出清理失败: %s", exc.what());
    }
  }

private:
  // ===========================================================================
  // 2. 节点初始化：参数、发布订阅、action client、定时器
  // ===========================================================================

  void declare_parameters()
  {
    declare_parameter<double>("position_tolerance", config_.position_tolerance);
    declare_parameter<double>("orientation_tolerance", config_.orientation_tolerance);
    declare_parameter<double>("status_publish_rate", config_.status_publish_rate);
    declare_parameter<std::string>("default_frame_id", config_.default_frame_id);
    declare_parameter<std::string>("map_frame", config_.map_frame);
    declare_parameter<std::string>("base_frame", config_.base_frame);
    declare_parameter<double>("pose_tf_timeout_sec", config_.pose_tf_timeout_sec);

    declare_parameter<double>("obstacle_block_timeout", config_.obstacle_block_timeout);
    declare_parameter<double>("velocity_threshold", config_.velocity_threshold);
    declare_parameter<double>("blockage_pose_delta_deadzone", config_.blockage_pose_delta_deadzone);
    declare_parameter<double>(
      "blockage_recovery_velocity_threshold", config_.blockage_recovery_velocity_threshold);
    declare_parameter<double>("blockage_recovery_confirm_sec", config_.blockage_recovery_confirm_sec);
    declare_parameter<bool>("obstacle_wait_enable", config_.obstacle_wait_enable);
    declare_parameter<double>("obstacle_wait_push_interval_sec", config_.obstacle_wait_push_interval_sec);
    declare_parameter<int>("obstacle_clear_required_frames", config_.obstacle_clear_required_frames);
    declare_parameter<double>("obstacle_clear_check_rate_hz", config_.obstacle_clear_check_rate_hz);
    declare_parameter<int>("obstacle_clear_cost_threshold", config_.obstacle_clear_cost_threshold);
    declare_parameter<double>("obstacle_clear_front_min_x_m", config_.obstacle_clear_front_min_x_m);
    declare_parameter<double>("obstacle_clear_front_max_x_m", config_.obstacle_clear_front_max_x_m);
    declare_parameter<double>("obstacle_clear_half_width_m", config_.obstacle_clear_half_width_m);
    declare_parameter<double>(
      "obstacle_min_wait_before_resume_sec", config_.obstacle_min_wait_before_resume_sec);
    declare_parameter<double>(
      "obstacle_clear_required_duration_sec", config_.obstacle_clear_required_duration_sec);
    declare_parameter<double>(
      "obstacle_clear_required_duration_after_false_resume_sec",
      config_.obstacle_clear_required_duration_after_false_resume_sec);
    declare_parameter<double>("obstacle_false_resume_window_sec", config_.obstacle_false_resume_window_sec);
    declare_parameter<bool>("obstacle_resume_use_roi", config_.obstacle_resume_use_roi);
    declare_parameter<std::string>(
      "obstacle_roi_has_obstacle_topic", config_.obstacle_roi_has_obstacle_topic);
    declare_parameter<double>("obstacle_roi_timeout_sec", config_.obstacle_roi_timeout_sec);
    declare_parameter<int>("obstacle_roi_required_clear_frames", config_.obstacle_roi_required_clear_frames);
    declare_parameter<std::string>("local_costmap_topic", config_.local_costmap_topic);
    declare_parameter<bool>(
      "obstacle_costmap_analyze_only_when_waiting", config_.obstacle_costmap_analyze_only_when_waiting);
    declare_parameter<bool>(
      "obstacle_costmap_window_bounded_scan", config_.obstacle_costmap_window_bounded_scan);

    declare_parameter<bool>("require_walk_mode_for_navigation", config_.require_walk_mode_for_navigation);
    declare_parameter<double>("robot_status_timeout", config_.robot_status_timeout);
    declare_parameter<double>("pending_navigation_timeout", config_.pending_navigation_timeout);
    declare_parameter<double>("obstacle_block_near_goal_distance", config_.obstacle_block_near_goal_distance);
    declare_parameter<int>("localization_resume_stable_frames", config_.localization_resume_stable_frames);
    declare_parameter<std::string>(
      "localization_health_status_topic", config_.localization_health_status_topic);
    declare_parameter<std::string>("map_status_topic", config_.map_status_topic);
    declare_parameter<double>("localization_health_timeout_sec", config_.localization_health_timeout_sec);
    declare_parameter<bool>(
      "localization_auto_pause_on_recovery_required",
      config_.localization_auto_pause_on_recovery_required);
    declare_parameter<bool>(
      "localization_allow_start_with_last_good_tf", config_.localization_allow_start_with_last_good_tf);
    declare_parameter<double>(
      "localization_last_good_tf_max_age_sec", config_.localization_last_good_tf_max_age_sec);
    declare_parameter<double>(
      "route_task.first_task_reached_tolerance_m", config_.route_task_first_task_reached_tolerance_m);
    declare_parameter<double>(
      "route_task.transit_passed_tolerance_m", config_.route_task_transit_passed_tolerance_m);
    declare_parameter<bool>(
      "route_task.transit_projection_passed_enabled",
      config_.route_task_transit_projection_passed_enabled);
    declare_parameter<double>(
      "route_task.nav2_feedback_timeout_sec", config_.route_task_nav2_feedback_timeout_sec);
    declare_parameter<double>("route_task.goal_cancel_timeout_sec", config_.route_task_goal_cancel_timeout_sec);
    declare_parameter<double>(
      "route_task.goal_reject_retry_timeout_sec", config_.route_task_goal_reject_retry_timeout_sec);
    declare_parameter<bool>(
      "route_task.default_interrupt_broadcast", config_.route_task_default_interrupt_broadcast);
    declare_parameter<bool>(
      "route_task.nav2_execution_enable", config_.route_task_nav2_execution_enable);
    declare_parameter<std::string>("reverse_navigation_bt_xml", config_.reverse_navigation_bt_xml);

    declare_parameter<std::string>("navigation_requests_topic", config_.navigation_requests_topic);
    declare_parameter<std::string>("navigation_waypoints_data_topic", config_.navigation_waypoints_data_topic);
    declare_parameter<std::string>("navigation_status_topic", config_.navigation_status_topic);
    declare_parameter<std::string>(
      "navigation_acknowledgments_topic", config_.navigation_acknowledgments_topic);
    declare_parameter<std::string>("navigation_goal_topic", config_.navigation_goal_topic);
    declare_parameter<std::string>("cmd_vel_topic", config_.cmd_vel_topic);
    declare_parameter<std::string>("odom_topic", config_.odom_topic);
    declare_parameter<std::string>("robot_status_topic", config_.robot_status_topic);
    declare_parameter<std::string>("behavior_tree_log_topic", config_.behavior_tree_log_topic);
    declare_parameter<std::string>("navigate_to_pose_action", config_.navigate_to_pose_action);
    declare_parameter<std::string>(
      "navigate_through_poses_action", config_.navigate_through_poses_action);
  }

  void load_parameters()
  {
    config_.position_tolerance = get_parameter("position_tolerance").as_double();
    config_.orientation_tolerance = get_parameter("orientation_tolerance").as_double();
    config_.status_publish_rate = get_parameter("status_publish_rate").as_double();
    config_.default_frame_id = get_parameter("default_frame_id").as_string();
    config_.map_frame = get_parameter("map_frame").as_string();
    config_.base_frame = get_parameter("base_frame").as_string();
    config_.pose_tf_timeout_sec = get_parameter("pose_tf_timeout_sec").as_double();

    config_.obstacle_block_timeout = get_parameter("obstacle_block_timeout").as_double();
    config_.velocity_threshold = get_parameter("velocity_threshold").as_double();
    config_.blockage_pose_delta_deadzone = get_parameter("blockage_pose_delta_deadzone").as_double();
    config_.blockage_recovery_velocity_threshold =
      get_parameter("blockage_recovery_velocity_threshold").as_double();
    config_.blockage_recovery_confirm_sec = get_parameter("blockage_recovery_confirm_sec").as_double();
    config_.obstacle_wait_enable = get_parameter("obstacle_wait_enable").as_bool();
    config_.obstacle_wait_push_interval_sec =
      get_parameter("obstacle_wait_push_interval_sec").as_double();
    config_.obstacle_clear_required_frames = get_parameter("obstacle_clear_required_frames").as_int();
    config_.obstacle_clear_check_rate_hz = get_parameter("obstacle_clear_check_rate_hz").as_double();
    config_.obstacle_clear_cost_threshold = get_parameter("obstacle_clear_cost_threshold").as_int();
    config_.obstacle_clear_front_min_x_m = get_parameter("obstacle_clear_front_min_x_m").as_double();
    config_.obstacle_clear_front_max_x_m = get_parameter("obstacle_clear_front_max_x_m").as_double();
    config_.obstacle_clear_half_width_m = get_parameter("obstacle_clear_half_width_m").as_double();
    config_.obstacle_min_wait_before_resume_sec =
      get_parameter("obstacle_min_wait_before_resume_sec").as_double();
    config_.obstacle_clear_required_duration_sec =
      get_parameter("obstacle_clear_required_duration_sec").as_double();
    config_.obstacle_clear_required_duration_after_false_resume_sec =
      get_parameter("obstacle_clear_required_duration_after_false_resume_sec").as_double();
    config_.obstacle_false_resume_window_sec =
      get_parameter("obstacle_false_resume_window_sec").as_double();
    config_.obstacle_resume_use_roi = get_parameter("obstacle_resume_use_roi").as_bool();
    config_.obstacle_roi_timeout_sec = get_parameter("obstacle_roi_timeout_sec").as_double();
    config_.obstacle_roi_required_clear_frames =
      get_parameter("obstacle_roi_required_clear_frames").as_int();
    config_.local_costmap_topic = get_parameter("local_costmap_topic").as_string();
    config_.obstacle_roi_has_obstacle_topic =
      get_parameter("obstacle_roi_has_obstacle_topic").as_string();
    config_.obstacle_costmap_analyze_only_when_waiting =
      get_parameter("obstacle_costmap_analyze_only_when_waiting").as_bool();
    config_.obstacle_costmap_window_bounded_scan =
      get_parameter("obstacle_costmap_window_bounded_scan").as_bool();

    config_.require_walk_mode_for_navigation = get_parameter("require_walk_mode_for_navigation").as_bool();
    config_.robot_status_timeout = get_parameter("robot_status_timeout").as_double();
    config_.pending_navigation_timeout = get_parameter("pending_navigation_timeout").as_double();
    config_.localization_resume_stable_frames =
      get_parameter("localization_resume_stable_frames").as_int();
    config_.localization_health_status_topic =
      get_parameter("localization_health_status_topic").as_string();
    config_.map_status_topic = get_parameter("map_status_topic").as_string();
    config_.localization_health_timeout_sec = get_parameter("localization_health_timeout_sec").as_double();
    config_.localization_auto_pause_on_recovery_required =
      get_parameter("localization_auto_pause_on_recovery_required").as_bool();
    config_.localization_allow_start_with_last_good_tf =
      get_parameter("localization_allow_start_with_last_good_tf").as_bool();
    config_.localization_last_good_tf_max_age_sec =
      get_parameter("localization_last_good_tf_max_age_sec").as_double();
    config_.route_task_first_task_reached_tolerance_m =
      get_parameter("route_task.first_task_reached_tolerance_m").as_double();
    config_.route_task_transit_passed_tolerance_m =
      get_parameter("route_task.transit_passed_tolerance_m").as_double();
    config_.route_task_transit_projection_passed_enabled =
      get_parameter("route_task.transit_projection_passed_enabled").as_bool();
    config_.route_task_nav2_feedback_timeout_sec =
      get_parameter("route_task.nav2_feedback_timeout_sec").as_double();
    config_.route_task_goal_cancel_timeout_sec =
      get_parameter("route_task.goal_cancel_timeout_sec").as_double();
    config_.route_task_goal_reject_retry_timeout_sec =
      get_parameter("route_task.goal_reject_retry_timeout_sec").as_double();
    config_.route_task_default_interrupt_broadcast =
      get_parameter("route_task.default_interrupt_broadcast").as_bool();
    config_.reverse_navigation_bt_xml = get_parameter("reverse_navigation_bt_xml").as_string();

    config_.navigation_requests_topic = get_parameter("navigation_requests_topic").as_string();
    config_.navigation_waypoints_data_topic =
      get_parameter("navigation_waypoints_data_topic").as_string();
    config_.route_task_nav2_execution_enable =
      get_parameter("route_task.nav2_execution_enable").as_bool();
    config_.navigation_status_topic = get_parameter("navigation_status_topic").as_string();
    config_.navigation_acknowledgments_topic =
      get_parameter("navigation_acknowledgments_topic").as_string();
    config_.navigation_goal_topic = get_parameter("navigation_goal_topic").as_string();
    config_.cmd_vel_topic = get_parameter("cmd_vel_topic").as_string();
    config_.odom_topic = get_parameter("odom_topic").as_string();
    config_.robot_status_topic = get_parameter("robot_status_topic").as_string();
    config_.behavior_tree_log_topic = get_parameter("behavior_tree_log_topic").as_string();
    config_.navigate_to_pose_action = get_parameter("navigate_to_pose_action").as_string();
    config_.navigate_through_poses_action =
      get_parameter("navigate_through_poses_action").as_string();
  }

  void setup_interfaces()
  {
    navigation_status_pub_ =
      create_publisher<std_msgs::msg::String>(config_.navigation_status_topic, rclcpp::QoS(10));
    navigation_ack_pub_ =
      create_publisher<std_msgs::msg::String>(config_.navigation_acknowledgments_topic, rclcpp::QoS(10));
    navigation_goal_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(config_.navigation_goal_topic, rclcpp::QoS(10));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(config_.cmd_vel_topic, rclcpp::QoS(10));

    navigation_request_sub_ = create_subscription<std_msgs::msg::String>(
      config_.navigation_requests_topic, rclcpp::QoS(10),
      [this](std_msgs::msg::String::ConstSharedPtr msg) { on_navigation_request(msg); });
    waypoints_data_sub_ = create_subscription<std_msgs::msg::String>(
      config_.navigation_waypoints_data_topic, rclcpp::QoS(10),
      [this](std_msgs::msg::String::ConstSharedPtr msg) { on_waypoints_data(msg); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      config_.odom_topic, rclcpp::QoS(10),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { on_odom(msg); });
    local_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      config_.local_costmap_topic, rclcpp::QoS(10),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { on_local_costmap(msg); });
    roi_obstacle_sub_ = create_subscription<std_msgs::msg::Bool>(
      config_.obstacle_roi_has_obstacle_topic, rclcpp::QoS(10),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) { on_roi_obstacle(msg); });
    robot_status_sub_ = create_subscription<std_msgs::msg::String>(
      config_.robot_status_topic, rclcpp::QoS(10),
      [this](std_msgs::msg::String::ConstSharedPtr msg) { on_robot_status(msg); });
    behavior_tree_log_sub_ = create_subscription<nav2_msgs::msg::BehaviorTreeLog>(
      config_.behavior_tree_log_topic, rclcpp::QoS(10),
      [this](nav2_msgs::msg::BehaviorTreeLog::ConstSharedPtr msg) { on_behavior_tree_log(msg); });
    localization_status_sub_ = create_subscription<std_msgs::msg::String>(
      config_.localization_health_status_topic, rclcpp::QoS(10),
      [this](std_msgs::msg::String::ConstSharedPtr msg) { on_localization_status(msg); });
    map_status_sub_ = create_subscription<std_msgs::msg::String>(
      config_.map_status_topic, rclcpp::QoS(10),
      [this](std_msgs::msg::String::ConstSharedPtr msg) { on_map_status(msg); });

    nav2_controller_.create_clients(this);

    status_timer_ = create_wall_timer(
      to_duration(1.0 / safe_rate(config_.status_publish_rate), 1s),
      [this]() { publish_navigation_status(); });
    navigation_check_timer_ = create_wall_timer(
      500ms,
      [this]() {
        check_localization_timeout();
        process_localization_recovery_state();
        check_route_task_feedback_timeout();
      });
    pending_navigation_timer_ = create_wall_timer(
      200ms,
      [this]() { check_pending_navigation_timeout(); });
    obstacle_wait_timer_ = create_wall_timer(
      to_duration(1.0 / safe_rate(config_.obstacle_clear_check_rate_hz), 200ms),
      [this]() { process_obstacle_wait_state(); });
  }

  // ===========================================================================
  // 3. 输入链路
  // ===========================================================================

  void on_navigation_request(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    rapidjson::Document request;
    request.Parse(msg->data.c_str());
    if (request.HasParseError() || !request.IsObject()) {
      auto extra = make_extra_data("invalid_request_payload", "");
      send_acknowledgment("navigation_request", "error", "导航请求 payload 必须是 JSON 对象", &extra);
      return;
    }

    const std::string request_type = read_string_member(request, "request_type", "");
    if (request_type != "navigation_command") {
      auto extra = make_extra_data("unknown_request_type", request_type);
      send_acknowledgment("unknown_request_type", "error", "未知请求类型: " + request_type, &extra);
      return;
    }

    const rapidjson::Value * command_data = nullptr;
    if (request.HasMember("command_data") && request["command_data"].IsObject()) {
      command_data = &request["command_data"];
    }
    if (command_data == nullptr) {
      auto extra = make_extra_data("missing_command_data", request_type);
      send_acknowledgment("navigation_command", "error", "navigation_command 缺少 command_data", &extra);
      return;
    }

    const std::string command_type = read_string_member(*command_data, "command_type", "");
    if (command_type.empty()) {
      send_route_task_ack(
        "unknown", "error", "navigation command missing command_type", *command_data,
        "missing_command_type");
      return;
    }

    if (is_route_task_command(command_type)) {
      if (command_type == "stop_route_task" && cancel_pending_navigation(command_type, *command_data)) {
        return;
      }
      const std::string block_reason = build_start_block_reason();
      if (command_type == "start_route_task" && !block_reason.empty() &&
        defer_navigation_start_if_not_ready(request, *command_data, block_reason))
      {
        return;
      }
      if (command_type == "start_route_task") {
        merge_request_waypoints_data(*command_data);
      }
      std::string resume_localization_block_reason;
      if (command_type == "resume_route_task") {
        const auto reason =
          gatekeeper_.localization_start_block_reason(config_, localization_, now_seconds());
        if (reason.has_value()) {
          resume_localization_block_reason = reason.value();
        }
      }
      auto result = route_task_state_machine_.handle_command(
        command_type, *command_data, block_reason, resume_localization_block_reason,
        config_, map_, waypoints_, route_);

      rapidjson::Document extra;
      extra.SetObject();
      auto & allocator = extra.GetAllocator();
      extra.AddMember(
        "implementation_stage", rapidjson::Value(result.implementation_stage.c_str(), allocator).Move(),
        allocator);
      if (command_type == "start_route_task" && !block_reason.empty()) {
        extra.AddMember("start_block_reason", rapidjson::Value(block_reason.c_str(), allocator).Move(), allocator);
      }
      for (const auto & field : result.event_string_fields) {
        extra.AddMember(
          rapidjson::Value(field.first.c_str(), allocator).Move(),
          rapidjson::Value(field.second.c_str(), allocator).Move(),
          allocator);
      }
      for (const auto & field : result.event_bool_fields) {
        extra.AddMember(rapidjson::Value(field.first.c_str(), allocator).Move(), field.second, allocator);
      }
      for (const auto & field : result.event_double_fields) {
        extra.AddMember(rapidjson::Value(field.first.c_str(), allocator).Move(), field.second, allocator);
      }
      for (const auto & field : result.event_int_fields) {
        extra.AddMember(rapidjson::Value(field.first.c_str(), allocator).Move(), field.second, allocator);
      }
      for (const auto & field : result.event_string_array_fields) {
        rapidjson::Value values(rapidjson::kArrayType);
        for (const auto & item : field.second) {
          values.PushBack(rapidjson::Value(item.c_str(), allocator).Move(), allocator);
        }
        extra.AddMember(rapidjson::Value(field.first.c_str(), allocator).Move(), values, allocator);
      }
      for (const auto & field : result.event_string_object_fields) {
        rapidjson::Value object(rapidjson::kObjectType);
        for (const auto & item : field.second) {
          object.AddMember(
            rapidjson::Value(item.first.c_str(), allocator).Move(),
            rapidjson::Value(item.second.c_str(), allocator).Move(),
            allocator);
        }
        extra.AddMember(rapidjson::Value(field.first.c_str(), allocator).Move(), object, allocator);
      }

      if (result.status == "success" && config_.route_task_nav2_execution_enable) {
        bool nav_start_failed = false;
        last_route_task_failure_message_.clear();
        last_route_task_failure_code_.clear();

        if (command_type == "start_route_task") {
          const bool first_task_already_reached =
            route_.current_target_task_index == 0 &&
            route_.active_segment.has_value() &&
            route_.active_segment->transit_waypoint_ids.empty() &&
            should_complete_active_segment_without_navigation();
          nav_start_failed = !start_active_segment_navigation(command_type);
          if (!nav_start_failed && first_task_already_reached) {
            result.message = "first route task waypoint already reached";
            result.result_reason = "first_task_already_reached";
          }
        } else if (command_type == "jump_to_waypoint" && result.result_reason != "already_current_target") {
          // 跳点重规划时必须先让旧 Nav2 goal 失效，再启动新段。
          // 这样旧 goal 的取消回调或结果回调即使晚到，也不会推进新的路线任务。
          cancel_active_route_goal();
          nav_start_failed = !start_active_segment_navigation(command_type);
        } else if (command_type == "pause_route_task" && result.result_reason != "route_task_already_paused") {
          clear_obstacle_wait_runtime();
          reset_block_detection();
          cancel_active_route_goal();
          publish_zero_cmd_vel();
        } else if (command_type == "resume_route_task" && !route_.awaiting_broadcast &&
          route_.active && route_.current_state == NavigationState::Executing && route_.active_segment.has_value())
        {
          clear_obstacle_wait_runtime();
          reset_block_detection();
          nav_start_failed = !start_active_segment_navigation("resume_route_task");
        } else if (command_type == "stop_route_task") {
          clear_obstacle_wait_runtime();
          reset_block_detection();
          cancel_active_route_goal();
        }

        if (nav_start_failed) {
          const std::string error_code = last_route_task_failure_code_.empty() ?
            "route_task_start_failed" : last_route_task_failure_code_;
          const std::string message = last_route_task_failure_message_.empty() ?
            "route task navigation start failed" : last_route_task_failure_message_;
          rapidjson::Document error_extra;
          error_extra.SetObject();
          auto & error_allocator = error_extra.GetAllocator();
          error_extra.AddMember(
            "implementation_stage",
            rapidjson::Value(result.implementation_stage.c_str(), error_allocator).Move(),
            error_allocator);
          send_route_task_ack(
            command_type, "error", message, *command_data, error_code, "", &error_extra);
          return;
        }
      }

      send_route_task_ack(
        command_type, result.status, result.message, *command_data, result.error_code,
        result.result_reason, &extra);
      if (result.status != "success") {
        return;
      }
      if (command_type == "jump_to_waypoint" && result.result_reason == "already_current_target") {
        return;
      }
      if (command_type == "broadcast_finished" && result.result_reason == "duplicate_broadcast_finished") {
        return;
      }
      if (command_type == "pause_route_task" && result.result_reason == "route_task_already_paused") {
        return;
      }
      if (!result.event_type.empty()) {
        add_route_task_tracking_fields(extra, result.event_type);
        publish_status_update(result.event_type, extra);
      }
      for (const auto & event : result.followup_events) {
        rapidjson::Document followup;
        followup.SetObject();
        auto & followup_allocator = followup.GetAllocator();
        for (const auto & field : event.event_string_fields) {
          followup.AddMember(
            rapidjson::Value(field.first.c_str(), followup_allocator).Move(),
            rapidjson::Value(field.second.c_str(), followup_allocator).Move(),
            followup_allocator);
        }
        for (const auto & field : event.event_bool_fields) {
          followup.AddMember(
            rapidjson::Value(field.first.c_str(), followup_allocator).Move(), field.second,
            followup_allocator);
        }
        for (const auto & field : event.event_double_fields) {
          followup.AddMember(
            rapidjson::Value(field.first.c_str(), followup_allocator).Move(), field.second,
            followup_allocator);
        }
        for (const auto & field : event.event_int_fields) {
          followup.AddMember(
            rapidjson::Value(field.first.c_str(), followup_allocator).Move(), field.second,
            followup_allocator);
        }
        for (const auto & field : event.event_string_array_fields) {
          rapidjson::Value values(rapidjson::kArrayType);
          for (const auto & item : field.second) {
            values.PushBack(rapidjson::Value(item.c_str(), followup_allocator).Move(), followup_allocator);
          }
          followup.AddMember(
            rapidjson::Value(field.first.c_str(), followup_allocator).Move(), values,
            followup_allocator);
        }
        for (const auto & field : event.event_string_object_fields) {
          rapidjson::Value object(rapidjson::kObjectType);
          for (const auto & item : field.second) {
            object.AddMember(
              rapidjson::Value(item.first.c_str(), followup_allocator).Move(),
              rapidjson::Value(item.second.c_str(), followup_allocator).Move(),
              followup_allocator);
          }
          followup.AddMember(
            rapidjson::Value(field.first.c_str(), followup_allocator).Move(), object,
          followup_allocator);
        }
        add_route_task_tracking_fields(followup, event.event_type);
        publish_status_update(event.event_type, followup);
      }
      if (result.status == "success" && config_.route_task_nav2_execution_enable) {
        if (command_type == "broadcast_finished" && result.event_type == "task_waypoint_completed" &&
          route_.active && route_.current_state == NavigationState::Executing && route_.active_segment.has_value() &&
          !route_.awaiting_broadcast)
        {
          start_active_segment_navigation("next_segment");
        }
      }
      if (result.reset_route_after_publish) {
        route_task_state_machine_.reset_route_task_state(route_);
      }
      return;
    }

    rapidjson::Document extra;
    extra.SetObject();
    auto & allocator = extra.GetAllocator();
    extra.AddMember("error_code", "unknown_navigation_command", allocator);
    extra.AddMember("command_type", rapidjson::Value(command_type.c_str(), allocator).Move(), allocator);
    send_acknowledgment(
      "unknown_navigation_command",
      "error",
      "未知命令: " + command_type,
      &extra);
  }

  void on_waypoints_data(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    rapidjson::Document message;
    message.Parse(msg->data.c_str());
    if (message.HasParseError() || !message.IsObject()) {
      RCLCPP_WARN(get_logger(), "点位库消息 JSON 解析失败");
      return;
    }

    const auto result = update_waypoint_cache_from_message(message, waypoints_, config_);
    if (!result.accepted) {
      if (!result.error.empty()) {
        RCLCPP_WARN(get_logger(), "点位库消息未被接收: %s", result.error.c_str());
      }
      return;
    }
    RCLCPP_INFO(
      get_logger(), "收到点位库同步: map_id=%s revision=%s count=%zu",
      waypoints_.map_id.c_str(), waypoints_.revision.c_str(), waypoints_.count);
  }

  void merge_request_waypoints_data(const rapidjson::Value & command_data)
  {
    const rapidjson::Value * request_waypoints = nullptr;
    if (command_data.HasMember("waypoints_data") && command_data["waypoints_data"].IsObject()) {
      request_waypoints = &command_data["waypoints_data"];
    }
    if (request_waypoints == nullptr || request_waypoints->ObjectEmpty()) {
      return;
    }

    const std::string map_id = read_route_task_id_member(command_data, "map_id");
    std::map<std::string, StoredWaypoint> merged;
    collect_waypoints_from_bucket(*request_waypoints, map_id, config_, merged);
    if (merged.empty()) {
      return;
    }

    for (const auto & item : merged) {
      waypoints_.waypoints[item.first] = item.second;
      if (!map_id.empty()) {
        waypoints_.waypoints_by_map[map_id][item.first] = item.second;
      }
    }
    if (!map_id.empty() && waypoints_.map_id.empty()) {
      waypoints_.map_id = map_id;
    }
    const std::string requested_revision = read_route_task_id_member(command_data, "waypoints_revision");
    if (!map_id.empty() && !requested_revision.empty()) {
      waypoints_.revisions_by_map[map_id] = requested_revision;
      if (waypoints_.revision.empty()) {
        waypoints_.revision = requested_revision;
      }
    }
    waypoints_.count = count_cached_waypoints(waypoints_.waypoints_by_map);
    if (waypoints_.count == 0) {
      waypoints_.count = waypoints_.waypoints.size();
    }
    waypoints_.last_update = now_seconds();
  }

  void on_localization_status(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    const double now = now_seconds();
    const std::string text = trim_copy(msg->data);

    if (!text.empty() && text.front() == '{') {
      rapidjson::Document payload;
      payload.Parse(text.c_str());
      if (!payload.HasParseError() && payload.IsObject()) {
        const bool pose_initialized = read_bool_member(payload, "pose_initialized", false);
        const bool pose_trusted = read_bool_member(payload, "pose_trusted", false);
        const bool can_start_navigation = read_bool_member(payload, "can_start_navigation", false);
        const bool recovery_required = read_bool_member(payload, "localization_recovery_required", false) ||
          read_bool_member(payload, "recovery_requires_global_relocalization", false);
        localization_.state = read_string_member(payload, "state", "unknown");
        localization_.text = read_string_member(payload, "reason", "");
        localization_.recovery_required = recovery_required;
        localization_.last_status_time = now;
        localization_.has_last_good_tf = pose_initialized || localization_.has_last_good_tf;
        if (pose_trusted && can_start_navigation) {
          localization_.healthy = true;
          localization_.last_good_tf_time = now;
          ++localization_.resume_stable_count;
        } else {
          localization_.healthy = false;
          localization_.resume_stable_count = 0;
        }
        process_localization_recovery_state();
        return;
      }
    }

    const std::string state = text.substr(0, text.find(' '));
    localization_.state = state.empty() ? "UNKNOWN" : state;
    localization_.text = text;
    localization_.recovery_required = false;
    localization_.last_status_time = now;

    if (localization_.state == "ACCEPTED") {
      localization_.has_last_good_tf = true;
      localization_.last_good_tf_time = now;
      localization_.healthy = true;
      ++localization_.resume_stable_count;
      return;
    }

    if (gatekeeper_.localization_can_use_last_good_tf(config_, localization_, now)) {
      localization_.healthy = true;
      return;
    }

    localization_.healthy = false;
    localization_.resume_stable_count = 0;
    process_localization_recovery_state();
  }

  void on_map_status(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    rapidjson::Document payload;
    payload.Parse(msg->data.c_str());
    if (payload.HasParseError() || !payload.IsObject()) {
      return;
    }
    const rapidjson::Value * data = &payload;
    if (payload.HasMember("data") && payload["data"].IsObject()) {
      data = &payload["data"];
    }
    map_.active_map_id = read_string_member(*data, "current_map_id", map_.active_map_id);
    map_.map_state = read_string_member(*data, "map_state", map_.map_state);
    map_.localization_state = read_string_member(*data, "localization_state", map_.localization_state);
    map_.last_update = now_seconds();
  }

  void on_robot_status(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    rapidjson::Document payload;
    payload.Parse(msg->data.c_str());
    if (payload.HasParseError() || !payload.IsObject()) {
      return;
    }
    const rapidjson::Value * values = nullptr;
    if (payload.HasMember("values") && payload["values"].IsObject()) {
      values = &payload["values"];
    }
    if (values == nullptr) {
      return;
    }
    robot_.control_state =
      read_string_member(*values, "robot_status", read_string_member(*values, "robot_state", robot_.control_state));
    robot_.motion_busy = read_bool_member(*values, "motion_busy", false);
    robot_.current_motion = read_string_member(*values, "current_motion", "");
    if (values->HasMember("control_ready_for_navigation")) {
      robot_.ready_for_navigation = read_bool_member(*values, "control_ready_for_navigation", false);
    } else {
      robot_.ready_for_navigation = robot_.control_state == "Walk" && !robot_.motion_busy;
    }
    robot_.last_update = now_seconds();
  }

  void on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    const double now = now_seconds();
    environment_.last_pose_update = now;
    environment_.has_current_pose = true;
    environment_.current_position = {
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z};
    environment_.current_orientation = {
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w};
    environment_.velocity_linear_x = -deadzone(msg->twist.twist.linear.z);
    environment_.velocity_linear_y = deadzone(msg->twist.twist.linear.x);
    environment_.velocity_angular_z = -deadzone(msg->twist.twist.angular.y);
    update_pose_derived_speed(*msg, now);
    check_obstacle_blockage();
  }

  void on_local_costmap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
  {
    environment_.latest_costmap_stamp = now_seconds();
    environment_.latest_costmap_frame_id = msg->header.frame_id;
    environment_.latest_costmap_width = msg->info.width;
    environment_.latest_costmap_height = msg->info.height;
    environment_.latest_costmap_resolution = msg->info.resolution;
    environment_.latest_costmap_origin_x = msg->info.origin.position.x;
    environment_.latest_costmap_origin_y = msg->info.origin.position.y;
    environment_.latest_costmap_data = msg->data;
  }

  void on_roi_obstacle(const std_msgs::msg::Bool::ConstSharedPtr msg)
  {
    environment_.latest_roi_has_obstacle = msg->data;
    environment_.latest_roi_stamp = now_seconds();
    if (msg->data) {
      environment_.roi_clear_confirm_count = 0;
    } else {
      environment_.roi_clear_confirm_count += 1;
    }
  }

  void on_behavior_tree_log(const nav2_msgs::msg::BehaviorTreeLog::ConstSharedPtr msg)
  {
    environment_.latest_bt_event_count = msg->event_log.size();
    if (route_.current_state != NavigationState::Executing && route_.current_state != NavigationState::Planning) {
      nav2_blockage_suppression_nodes_.clear();
      if (route_.detailed_state == "RECOVERING" || route_.detailed_state == "TURNING") {
        route_.detailed_state = navigation_state_to_string(route_.current_state);
      }
      return;
    }

    for (const auto & event : msg->event_log) {
      if (is_nav2_blockage_suppression_node(event.node_name)) {
        if (event.current_status == "RUNNING") {
          nav2_blockage_suppression_nodes_.insert(event.node_name);
          route_.detailed_state = event.node_name == "SpinToPose" || event.node_name == "Spin" ?
            "TURNING" : "RECOVERING";
        } else {
          nav2_blockage_suppression_nodes_.erase(event.node_name);
          if (nav2_blockage_suppression_nodes_.empty() &&
            (route_.detailed_state == "RECOVERING" || route_.detailed_state == "TURNING"))
          {
            route_.detailed_state = route_.current_state == NavigationState::Executing ? "EXECUTING" :
              navigation_state_to_string(route_.current_state);
          }
        }
      }
      if (event.node_name.find("ComputePathToPose") != std::string::npos &&
        event.current_status == "FAILURE")
      {
        route_.detailed_state = "PLANNING_FAILED";
      }
    }
  }

  // ===========================================================================
  // 4. 输出链路
  // ===========================================================================

  void send_acknowledgment(
    const std::string & ack_type,
    const std::string & status,
    const std::string & message,
    const rapidjson::Value * extra_data = nullptr)
  {
    rapidjson::Document payload;
    payload.SetObject();
    auto & allocator = payload.GetAllocator();
    payload.AddMember("ack_type", rapidjson::Value(ack_type.c_str(), allocator).Move(), allocator);
    payload.AddMember("status", rapidjson::Value(status.c_str(), allocator).Move(), allocator);
    payload.AddMember("message", rapidjson::Value(message.c_str(), allocator).Move(), allocator);
    payload.AddMember("timestamp", now_seconds(), allocator);
    if (extra_data != nullptr && extra_data->IsObject()) {
      for (auto it = extra_data->MemberBegin(); it != extra_data->MemberEnd(); ++it) {
        rapidjson::Value key;
        key.CopyFrom(it->name, allocator);
        rapidjson::Value value;
        value.CopyFrom(it->value, allocator);
        payload.AddMember(key, value, allocator);
      }
    }
    std_msgs::msg::String out;
    out.data = json_to_string(payload);
    navigation_ack_pub_->publish(out);
  }

  void send_route_task_ack(
    const std::string & command_type,
    const std::string & status,
    const std::string & message,
    const rapidjson::Value & command_data,
    const std::string & error_code = "",
    const std::string & result_reason = "",
    const rapidjson::Value * extra_data = nullptr)
  {
    rapidjson::Document event_data;
    event_data.SetObject();
    auto & allocator = event_data.GetAllocator();
    const std::string request_message_id = read_string_member(command_data, "request_message_id", "");
    const std::string task_session_id = read_string_member(command_data, "task_session_id", "");
    const std::string route_id = read_string_member(command_data, "route_id", "");
    std::string map_id = read_string_member(command_data, "map_id", "");
    if (map_id.empty() && route_.active) {
      map_id = route_.map_id;
    }
    event_data.AddMember(
      "request_message_id", rapidjson::Value(request_message_id.c_str(), allocator).Move(), allocator);
    event_data.AddMember("ack_type", "navigation_command_result", allocator);
    event_data.AddMember("command_type", rapidjson::Value(command_type.c_str(), allocator).Move(), allocator);
    event_data.AddMember(
      "task_session_id", rapidjson::Value(task_session_id.c_str(), allocator).Move(), allocator);
    event_data.AddMember("route_id", rapidjson::Value(route_id.c_str(), allocator).Move(), allocator);
    event_data.AddMember("map_id", rapidjson::Value(map_id.c_str(), allocator).Move(), allocator);
    event_data.AddMember("status", rapidjson::Value(status.c_str(), allocator).Move(), allocator);
    event_data.AddMember(
      "result_reason", rapidjson::Value(status == "success" ? result_reason.c_str() : "", allocator).Move(),
      allocator);
    event_data.AddMember(
      "error_code", rapidjson::Value(status == "error" ? error_code.c_str() : "", allocator).Move(), allocator);
    event_data.AddMember("message", rapidjson::Value(message.c_str(), allocator).Move(), allocator);
    const std::string event_id = build_route_task_event_id(
      "navigation_command_result",
      read_string_member(event_data, "task_session_id", ""));
    event_data.AddMember("event_id", rapidjson::Value(event_id.c_str(), allocator).Move(), allocator);
    event_data.AddMember("timestamp", now_seconds(), allocator);
    if (extra_data != nullptr && extra_data->IsObject()) {
      for (auto it = extra_data->MemberBegin(); it != extra_data->MemberEnd(); ++it) {
        rapidjson::Value key;
        key.CopyFrom(it->name, allocator);
        rapidjson::Value value;
        value.CopyFrom(it->value, allocator);
        event_data.AddMember(key, value, allocator);
      }
    }
    publish_status_update("navigation_command_result", event_data);
  }

  void add_front_obstacle_stats(
    rapidjson::Value & parent,
    const char * member_name,
    rapidjson::Document::AllocatorType & allocator,
    const ObstacleWaitDecision & decision,
    const bool blocked) const
  {
    rapidjson::Value stats(rapidjson::kObjectType);
    const bool available = environment_.latest_costmap_stamp > 0.0 &&
      environment_.latest_costmap_width > 0 &&
      environment_.latest_costmap_height > 0 &&
      environment_.latest_costmap_resolution > 0.0 &&
      !environment_.latest_costmap_data.empty();
    stats.AddMember("available", available, allocator);
    stats.AddMember(
      "frame_id",
      rapidjson::Value(environment_.latest_costmap_frame_id.c_str(), allocator).Move(),
      allocator);
    stats.AddMember("window_front_min_x_m", config_.obstacle_clear_front_min_x_m, allocator);
    stats.AddMember("window_front_max_x_m", config_.obstacle_clear_front_max_x_m, allocator);
    stats.AddMember("window_half_width_m", config_.obstacle_clear_half_width_m, allocator);
    stats.AddMember("sample_cells", decision.sample_cells, allocator);
    stats.AddMember("candidate_cells", decision.candidate_cells, allocator);
    stats.AddMember("occupied_cells", decision.occupied_cells, allocator);
    stats.AddMember("max_cost", decision.max_cost, allocator);
    stats.AddMember("blocked", blocked, allocator);
    stats.AddMember("bounded_scan", config_.obstacle_costmap_window_bounded_scan, allocator);
    parent.AddMember(rapidjson::Value(member_name, allocator).Move(), stats, allocator);
  }

  void add_roi_obstacle_stats(
    rapidjson::Value & parent,
    const char * member_name,
    rapidjson::Document::AllocatorType & allocator,
    const double now) const
  {
    rapidjson::Value stats(rapidjson::kObjectType);
    const bool has_stamp = environment_.latest_roi_stamp > 0.0;
    const double age = has_stamp ? now - environment_.latest_roi_stamp : -1.0;
    const bool fresh = has_stamp && age <= config_.obstacle_roi_timeout_sec;
    std::string decision = "roi_clear";
    if (!config_.obstacle_resume_use_roi) {
      decision = "disabled";
    } else if (!fresh) {
      decision = "stale_fallback_to_costmap";
    } else if (environment_.latest_roi_has_obstacle) {
      decision = "roi_blocked";
    } else if (environment_.roi_clear_confirm_count < config_.obstacle_roi_required_clear_frames) {
      decision = "roi_clear_frames_not_enough";
    }

    stats.AddMember("enabled", config_.obstacle_resume_use_roi, allocator);
    stats.AddMember(
      "topic",
      rapidjson::Value(config_.obstacle_roi_has_obstacle_topic.c_str(), allocator).Move(),
      allocator);
    stats.AddMember("has_obstacle", environment_.latest_roi_has_obstacle, allocator);
    rapidjson::Value age_value;
    if (has_stamp) {
      age_value.SetDouble(age);
    } else {
      age_value.SetNull();
    }
    stats.AddMember("age_sec", age_value, allocator);
    stats.AddMember("fresh", fresh, allocator);
    stats.AddMember("clear_confirmed_frames", environment_.roi_clear_confirm_count, allocator);
    stats.AddMember("clear_required_frames", config_.obstacle_roi_required_clear_frames, allocator);
    stats.AddMember("decision", rapidjson::Value(decision.c_str(), allocator).Move(), allocator);
    parent.AddMember(rapidjson::Value(member_name, allocator).Move(), stats, allocator);
  }

  void publish_navigation_status()
  {
    rapidjson::Document status;
    status.SetObject();
    auto & allocator = status.GetAllocator();
    const double now = now_seconds();
    const std::string state = navigation_state_to_string(route_.current_state);
    const std::string suppression_reason = get_obstacle_blockage_suppression_reason();
    const int task_count = static_cast<int>(route_.master_route_task_ids.size());
    const int finished_task_count =
      static_cast<int>(route_.completed_task_ids.size() + route_.skipped_task_ids.size());
    double progress_percentage = 0.0;
    if (task_count > 0) {
      progress_percentage =
        std::min(100.0, std::max(0.0, 100.0 * static_cast<double>(finished_task_count) /
        static_cast<double>(task_count)));
    }

    status.AddMember("timestamp", now, allocator);
    status.AddMember("current_state", rapidjson::Value(state.c_str(), allocator).Move(), allocator);
    status.AddMember("navigation_mode", rapidjson::Value(route_.navigation_mode.c_str(), allocator).Move(), allocator);
    status.AddMember("current_waypoint_index", route_.current_waypoint_index, allocator);
    status.AddMember("total_waypoints", route_.total_waypoints, allocator);
    status.AddMember("progress_percentage", progress_percentage, allocator);
    status.AddMember("is_active", is_navigation_active(), allocator);
    status.AddMember(
      "navigation_duration",
      route_.navigation_start_time > 0 ? now - route_.navigation_start_time : 0.0,
      allocator);
    status.AddMember("detailed_state", rapidjson::Value(route_.detailed_state.c_str(), allocator).Move(), allocator);
    status.AddMember("recovery_active", route_.detailed_state == "RECOVERING", allocator);
    status.AddMember("obstacle_blocked", is_blocked_by_obstacle_ || obstacle_wait_manager_.active(), allocator);
    status.AddMember(
      "block_duration",
      obstacle_wait_manager_.active() ? now - obstacle_wait_manager_.started_at() :
      (is_blocked_by_obstacle_ && block_start_time_ > 0.0 ? now - block_start_time_ : 0.0),
      allocator);
    status.AddMember("block_reported", block_reported_, allocator);
    status.AddMember("obstacle_block_suppressed", !suppression_reason.empty(), allocator);
    status.AddMember(
      "obstacle_block_suppression_reason",
      rapidjson::Value(suppression_reason.c_str(), allocator).Move(),
      allocator);
    status.AddMember("pending_navigation", route_.pending_navigation_active, allocator);
    status.AddMember("pending_navigation_age", route_.pending_navigation_active ?
      now - route_.pending_navigation_created_at : 0.0, allocator);
    status.AddMember(
      "pending_navigation_reason",
      rapidjson::Value(route_.pending_navigation_reason.c_str(), allocator).Move(),
      allocator);
    status.AddMember("pause_source", rapidjson::Value(route_.pause_source.c_str(), allocator).Move(), allocator);
    status.AddMember("pause_reason", rapidjson::Value(route_.pause_reason.c_str(), allocator).Move(), allocator);
    status.AddMember("resume_mode", rapidjson::Value(route_.resume_mode.c_str(), allocator).Move(), allocator);
    status.AddMember("waiting_for_obstacle_clear", obstacle_wait_manager_.active(), allocator);
    status.AddMember("obstacle_wait_active", obstacle_wait_manager_.active(), allocator);
    status.AddMember(
      "obstacle_wait_duration",
      obstacle_wait_manager_.active() ? now - obstacle_wait_manager_.started_at() : 0.0,
      allocator);
    status.AddMember(
      "obstacle_clear_confirm_count",
      obstacle_wait_manager_.active() ? last_obstacle_wait_decision_.clear_confirmed_frames : 0,
      allocator);
    status.AddMember("obstacle_clear_required_frames", config_.obstacle_clear_required_frames, allocator);
    status.AddMember("obstacle_clear_required_duration_sec",
      obstacle_wait_manager_.current_required_clear_duration(config_), allocator);
    status.AddMember(
      "front_obstacle_blocked",
      obstacle_wait_manager_.active() && last_obstacle_wait_decision_.front_blocked,
      allocator);
    add_front_obstacle_stats(
      status, "front_obstacle_stats", allocator, last_obstacle_wait_decision_,
      obstacle_wait_manager_.active() && last_obstacle_wait_decision_.front_blocked);
    status.AddMember("localization_auto_paused", route_.localization_auto_paused, allocator);
    status.AddMember("localization_recovery_started_at", route_.localization_recovery_started_at, allocator);
    status.AddMember("roi_obstacle_has_obstacle", environment_.latest_roi_has_obstacle, allocator);
    status.AddMember("roi_obstacle_age_sec",
      environment_.latest_roi_stamp > 0.0 ? now - environment_.latest_roi_stamp : -1.0, allocator);
    status.AddMember("roi_obstacle_clear_confirm_count", environment_.roi_clear_confirm_count, allocator);
    add_roi_obstacle_stats(status, "roi_obstacle_stats", allocator, now);
    status.AddMember("costmap_age_sec",
      environment_.latest_costmap_stamp > 0.0 ? now - environment_.latest_costmap_stamp : -1.0, allocator);
    status.AddMember("costmap_width", static_cast<int>(environment_.latest_costmap_width), allocator);
    status.AddMember("costmap_height", static_cast<int>(environment_.latest_costmap_height), allocator);
    status.AddMember("localization_healthy", localization_.healthy, allocator);
    status.AddMember("localization_has_last_good_tf", localization_.has_last_good_tf, allocator);
    rapidjson::Value localization_health(rapidjson::kObjectType);
    localization_health.AddMember(
      "topic", rapidjson::Value(config_.localization_health_status_topic.c_str(), allocator).Move(), allocator);
    localization_health.AddMember(
      "state", rapidjson::Value(localization_.state.c_str(), allocator).Move(), allocator);
    localization_health.AddMember(
      "text", rapidjson::Value(localization_.text.c_str(), allocator).Move(), allocator);
    localization_health.AddMember("recovery_required", localization_.recovery_required, allocator);
    localization_health.AddMember("stamp_sec", localization_.last_status_time, allocator);
    localization_health.AddMember(
      "age_sec", localization_.last_status_time > 0.0 ? now - localization_.last_status_time : -1.0, allocator);
    localization_health.AddMember("resume_stable_count", localization_.resume_stable_count, allocator);
    status.AddMember("localization_health_status", localization_health, allocator);
    status.AddMember("active_map_id", rapidjson::Value(map_.active_map_id.c_str(), allocator).Move(), allocator);
    status.AddMember("map_state", rapidjson::Value(map_.map_state.c_str(), allocator).Move(), allocator);
    status.AddMember(
      "map_localization_state", rapidjson::Value(map_.localization_state.c_str(), allocator).Move(), allocator);
    status.AddMember("map_status_age",
      map_.last_update > 0.0 ? now - map_.last_update : 0.0, allocator);
    status.AddMember("cached_waypoints_count", static_cast<int>(waypoints_.count), allocator);
    status.AddMember("waypoints_revision", rapidjson::Value(waypoints_.revision.c_str(), allocator).Move(), allocator);
    status.AddMember("runtime_stage", "nav2_route_runtime", allocator);
    if (environment_.has_current_pose) {
      rapidjson::Value current_pose(rapidjson::kObjectType);
      current_pose.AddMember("frame_id", rapidjson::Value(config_.map_frame.c_str(), allocator).Move(), allocator);
      rapidjson::Value position(rapidjson::kObjectType);
      position.AddMember("x", environment_.current_position[0], allocator);
      position.AddMember("y", environment_.current_position[1], allocator);
      position.AddMember("z", environment_.current_position[2], allocator);
      current_pose.AddMember("position", position, allocator);
      rapidjson::Value orientation(rapidjson::kObjectType);
      orientation.AddMember("x", environment_.current_orientation[0], allocator);
      orientation.AddMember("y", environment_.current_orientation[1], allocator);
      orientation.AddMember("z", environment_.current_orientation[2], allocator);
      orientation.AddMember("w", environment_.current_orientation[3], allocator);
      current_pose.AddMember("orientation", orientation, allocator);
      status.AddMember("current_pose", current_pose, allocator);
    }
    rapidjson::Value current_velocity(rapidjson::kObjectType);
    rapidjson::Value linear(rapidjson::kObjectType);
    linear.AddMember("x", environment_.velocity_linear_x, allocator);
    linear.AddMember("y", environment_.velocity_linear_y, allocator);
    linear.AddMember("z", 0.0, allocator);
    current_velocity.AddMember("linear", linear, allocator);
    rapidjson::Value angular(rapidjson::kObjectType);
    angular.AddMember("x", 0.0, allocator);
    angular.AddMember("y", 0.0, allocator);
    angular.AddMember("z", environment_.velocity_angular_z, allocator);
    current_velocity.AddMember("angular", angular, allocator);
    status.AddMember("current_velocity", current_velocity, allocator);
    if (route_.active) {
      rapidjson::Value route_task(rapidjson::kObjectType);
      route_task.AddMember(
        "task_session_id", rapidjson::Value(route_.task_session_id.c_str(), allocator).Move(), allocator);
      route_task.AddMember("route_id", rapidjson::Value(route_.route_id.c_str(), allocator).Move(), allocator);
      route_task.AddMember("map_id", rapidjson::Value(route_.map_id.c_str(), allocator).Move(), allocator);
      route_task.AddMember(
        "route_waypoint_source", rapidjson::Value(route_.route_waypoint_source.c_str(), allocator).Move(),
        allocator);
      route_task.AddMember(
        "waypoints_revision", rapidjson::Value(route_.waypoints_revision.c_str(), allocator).Move(), allocator);
      route_task.AddMember(
        "current_anchor_task_id", rapidjson::Value(route_.current_anchor_task_id.c_str(), allocator).Move(),
        allocator);
      route_task.AddMember("current_anchor_task_index", route_.current_anchor_task_index, allocator);
      route_task.AddMember(
        "current_target_task_id", rapidjson::Value(route_.current_target_task_id.c_str(), allocator).Move(),
        allocator);
      route_task.AddMember("current_target_task_index", route_.current_target_task_index, allocator);
      route_task.AddMember("awaiting_broadcast", route_.awaiting_broadcast, allocator);
      route_task.AddMember(
        "waiting_broadcast_waypoint_id",
        rapidjson::Value(route_.waiting_broadcast_waypoint_id.c_str(), allocator).Move(),
        allocator);
      route_task.AddMember(
        "waiting_broadcast_id", rapidjson::Value(route_.waiting_broadcast_id.c_str(), allocator).Move(),
        allocator);
      route_task.AddMember("route_task_version", route_.route_task_version, allocator);
      route_task.AddMember("active_goal_generation", route_.current_goal_generation, allocator);

      rapidjson::Value master_ids(rapidjson::kArrayType);
      for (const auto & id : route_.master_route_task_ids) {
        master_ids.PushBack(rapidjson::Value(id.c_str(), allocator).Move(), allocator);
      }
      route_task.AddMember("master_route_task_ids", master_ids, allocator);

      rapidjson::Value completed_ids(rapidjson::kArrayType);
      for (const auto & id : route_.completed_task_ids) {
        completed_ids.PushBack(rapidjson::Value(id.c_str(), allocator).Move(), allocator);
      }
      route_task.AddMember("completed_task_ids", completed_ids, allocator);

      rapidjson::Value skipped_ids(rapidjson::kArrayType);
      for (const auto & id : route_.skipped_task_ids) {
        skipped_ids.PushBack(rapidjson::Value(id.c_str(), allocator).Move(), allocator);
      }
      route_task.AddMember("skipped_task_ids", skipped_ids, allocator);
      route_task.AddMember(
        "last_feedback_age_sec",
        route_task_last_feedback_time_ > 0.0 ? now - route_task_last_feedback_time_ : 0.0,
        allocator);

      if (route_.active_segment.has_value()) {
        rapidjson::Value segment(rapidjson::kObjectType);
        const auto & active_segment = route_.active_segment.value();
        segment.AddMember(
          "segment_id", rapidjson::Value(active_segment.segment_id.c_str(), allocator).Move(), allocator);
        segment.AddMember(
          "segment_direction",
          rapidjson::Value(active_segment.segment_direction.c_str(), allocator).Move(),
          allocator);
        segment.AddMember(
          "segment_start_task_id",
          rapidjson::Value(active_segment.segment_start_task_id.c_str(), allocator).Move(),
          allocator);
        segment.AddMember(
          "segment_target_task_id",
          rapidjson::Value(active_segment.segment_target_task_id.c_str(), allocator).Move(),
          allocator);
        segment.AddMember("segment_start_source_index", active_segment.segment_start_source_index, allocator);
        segment.AddMember("segment_target_source_index", active_segment.segment_target_source_index, allocator);
        segment.AddMember("current_segment_progress_index", active_segment.current_segment_progress_index, allocator);
        segment.AddMember("segment_goal_generation", active_segment.segment_goal_generation, allocator);

        rapidjson::Value transit_ids(rapidjson::kArrayType);
        for (const auto & id : active_segment.transit_waypoint_ids) {
          transit_ids.PushBack(rapidjson::Value(id.c_str(), allocator).Move(), allocator);
        }
        segment.AddMember("transit_waypoint_ids", transit_ids, allocator);

        rapidjson::Value execution_ids(rapidjson::kArrayType);
        for (const auto & id : active_segment.execution_waypoint_ids) {
          execution_ids.PushBack(rapidjson::Value(id.c_str(), allocator).Move(), allocator);
        }
        segment.AddMember("execution_waypoint_ids", execution_ids, allocator);

        rapidjson::Value passed_transit_ids(rapidjson::kArrayType);
        for (const auto & id : active_segment.passed_transit_waypoint_ids) {
          passed_transit_ids.PushBack(rapidjson::Value(id.c_str(), allocator).Move(), allocator);
        }
        segment.AddMember("passed_transit_waypoint_ids", passed_transit_ids, allocator);
        route_task.AddMember("active_segment", segment, allocator);
      }
      status.AddMember("route_task", route_task, allocator);
    }

    std_msgs::msg::String out;
    out.data = json_to_string(status);
    navigation_status_pub_->publish(out);
  }

  static bool is_nav2_blockage_suppression_node(const std::string & node_name)
  {
    return node_name == "SpinToPose" || node_name == "Spin" || node_name == "BackUp";
  }

  double distance_to_current_target_task() const
  {
    if (!environment_.has_current_pose || route_.current_target_task_id.empty()) {
      return std::numeric_limits<double>::infinity();
    }
    for (const auto & waypoint : route_.route_waypoints) {
      if (waypoint.waypoint_id != route_.current_target_task_id) {
        continue;
      }
      const double dx = environment_.current_position[0] - waypoint.position[0];
      const double dy = environment_.current_position[1] - waypoint.position[1];
      const double dz = environment_.current_position[2] - waypoint.position[2];
      return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    return std::numeric_limits<double>::infinity();
  }

  std::string get_obstacle_blockage_suppression_reason() const
  {
    if (route_.active && route_.awaiting_broadcast) {
      return "route task 等待 APP 播报完成";
    }
    if (robot_.motion_busy) {
      return robot_.current_motion.empty() ? "机器人动作执行阶段" :
        "机器人动作执行阶段(" + robot_.current_motion + ")";
    }
    if (robot_.control_state == "Menu") {
      return "机器人动作库模式";
    }
    if (!nav2_blockage_suppression_nodes_.empty()) {
      std::string active_nodes;
      for (const auto & node : nav2_blockage_suppression_nodes_) {
        if (!active_nodes.empty()) {
          active_nodes += ", ";
        }
        active_nodes += node;
      }
      return "Nav2主动转向/后退阶段(" + active_nodes + ")";
    }
    if (config_.obstacle_block_near_goal_distance <= 0.0) {
      return "";
    }
    if (std::isfinite(distance_remaining_) && distance_remaining_ <= config_.obstacle_block_near_goal_distance) {
      return "接近目标点阶段(剩余路径 " + std::to_string(distance_remaining_) + "m)";
    }
    const double distance = distance_to_current_target_task();
    if (std::isfinite(distance) && distance <= config_.obstacle_block_near_goal_distance) {
      return "接近目标点阶段(直线距离 " + std::to_string(distance) + "m)";
    }
    return "";
  }

  void update_pose_derived_speed(const nav_msgs::msg::Odometry & msg, const double fallback_now)
  {
    double stamp = static_cast<double>(msg.header.stamp.sec) +
      static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
    if (stamp <= 0.0) {
      stamp = fallback_now;
    }

    const std::array<double, 3> current_position = {
      msg.pose.pose.position.x,
      msg.pose.pose.position.y,
      msg.pose.pose.position.z};
    if (environment_.last_motion_pose_time <= 0.0) {
      environment_.last_motion_position = current_position;
      environment_.last_motion_pose_time = stamp;
      environment_.has_pose_derived_speed = false;
      return;
    }

    const double dt = stamp - environment_.last_motion_pose_time;
    if (dt <= 0.0 || dt > 2.0) {
      environment_.last_motion_position = current_position;
      environment_.last_motion_pose_time = stamp;
      environment_.has_pose_derived_speed = false;
      return;
    }

    const double dx = current_position[0] - environment_.last_motion_position[0];
    const double dz = current_position[2] - environment_.last_motion_position[2];
    environment_.pose_derived_speed = std::sqrt(dx * dx + dz * dz) / dt;
    environment_.has_pose_derived_speed = true;
    environment_.last_motion_position = current_position;
    environment_.last_motion_pose_time = stamp;
  }

  std::pair<double, std::string> get_blockage_motion_speed() const
  {
    const double linear_velocity = std::sqrt(
      environment_.velocity_linear_x * environment_.velocity_linear_x +
      environment_.velocity_linear_y * environment_.velocity_linear_y);
    const double angular_velocity = std::abs(environment_.velocity_angular_z);
    const double twist_speed = std::sqrt(linear_velocity * linear_velocity + angular_velocity * angular_velocity);

    if (!environment_.has_pose_derived_speed) {
      return {twist_speed, "odom_twist"};
    }
    double pose_delta_speed = environment_.pose_derived_speed;
    if (pose_delta_speed < config_.blockage_pose_delta_deadzone) {
      pose_delta_speed = 0.0;
    }
    if (pose_delta_speed >= twist_speed) {
      return {pose_delta_speed, "pose_delta"};
    }
    return {twist_speed, "odom_twist"};
  }

  bool is_stopped_for_blockage(const double total_velocity, const std::string & velocity_source) const
  {
    if (velocity_source == "pose_delta") {
      return total_velocity < config_.blockage_recovery_velocity_threshold;
    }
    return total_velocity < config_.velocity_threshold;
  }

  void clear_block_recovery_candidate()
  {
    block_recovery_candidate_start_time_ = 0.0;
    block_recovery_candidate_source_.clear();
  }

  bool has_confirmed_blockage_recovery(const double total_velocity, const std::string & velocity_source)
  {
    if (total_velocity < config_.blockage_recovery_velocity_threshold) {
      clear_block_recovery_candidate();
      return false;
    }
    const double now = now_seconds();
    if (block_recovery_candidate_start_time_ <= 0.0) {
      block_recovery_candidate_start_time_ = now;
      block_recovery_candidate_source_ = velocity_source;
      return config_.blockage_recovery_confirm_sec <= 0.0;
    }
    return now - block_recovery_candidate_start_time_ >= config_.blockage_recovery_confirm_sec;
  }

  void reset_block_detection()
  {
    is_blocked_by_obstacle_ = false;
    block_start_time_ = 0.0;
    block_reported_ = false;
    clear_block_recovery_candidate();
    if (route_.detailed_state == "BLOCKED_BY_OBSTACLE") {
      route_.detailed_state = "EXECUTING";
    }
  }

  void check_obstacle_blockage()
  {
    if (route_.current_state != NavigationState::Executing) {
      if (is_blocked_by_obstacle_ && !obstacle_wait_manager_.active()) {
        reset_block_detection();
      }
      return;
    }

    const auto [total_velocity, velocity_source] = get_blockage_motion_speed();
    if (block_reported_) {
      if (has_confirmed_blockage_recovery(total_velocity, velocity_source)) {
        reset_block_detection();
      }
      return;
    }

    const std::string suppression_reason = get_obstacle_blockage_suppression_reason();
    if (!suppression_reason.empty()) {
      if (is_blocked_by_obstacle_) {
        reset_block_detection();
      }
      return;
    }

    const double now = now_seconds();
    if (is_stopped_for_blockage(total_velocity, velocity_source)) {
      clear_block_recovery_candidate();
      if (!is_blocked_by_obstacle_) {
        is_blocked_by_obstacle_ = true;
        block_start_time_ = now;
        route_.detailed_state = "BLOCKED_BY_OBSTACLE";
        return;
      }
      const double block_duration = block_start_time_ > 0.0 ? now - block_start_time_ : 0.0;
      if (block_duration > config_.obstacle_block_timeout) {
        handle_obstacle_block_timeout(block_duration);
      }
      return;
    }

    if (is_blocked_by_obstacle_) {
      if (has_confirmed_blockage_recovery(total_velocity, velocity_source)) {
        reset_block_detection();
      } else {
        const double block_duration = block_start_time_ > 0.0 ? now - block_start_time_ : 0.0;
        if (block_duration > config_.obstacle_block_timeout) {
          handle_obstacle_block_timeout(block_duration);
        }
      }
    }
  }

  void publish_status_update(const std::string & event_type, const rapidjson::Value & event_data)
  {
    rapidjson::Document update;
    update.SetObject();
    auto & allocator = update.GetAllocator();
    const std::string state = navigation_state_to_string(route_.current_state);
    update.AddMember("event_type", rapidjson::Value(event_type.c_str(), allocator).Move(), allocator);
    rapidjson::Value data_copy;
    data_copy.CopyFrom(event_data, allocator);
    update.AddMember("event_data", data_copy, allocator);
    update.AddMember("timestamp", now_seconds(), allocator);
    update.AddMember("current_state", rapidjson::Value(state.c_str(), allocator).Move(), allocator);
    update.AddMember("navigation_mode", rapidjson::Value(route_.navigation_mode.c_str(), allocator).Move(), allocator);

    std_msgs::msg::String out;
    out.data = json_to_string(update);
    navigation_status_pub_->publish(out);
  }

  void append_route_task_event_fields(
    rapidjson::Document & document,
    const RouteTaskEventData & event)
  {
    auto & allocator = document.GetAllocator();
    for (const auto & field : event.event_string_fields) {
      document.AddMember(
        rapidjson::Value(field.first.c_str(), allocator).Move(),
        rapidjson::Value(field.second.c_str(), allocator).Move(),
        allocator);
    }
    for (const auto & field : event.event_bool_fields) {
      document.AddMember(rapidjson::Value(field.first.c_str(), allocator).Move(), field.second, allocator);
    }
    for (const auto & field : event.event_double_fields) {
      document.AddMember(rapidjson::Value(field.first.c_str(), allocator).Move(), field.second, allocator);
    }
    for (const auto & field : event.event_int_fields) {
      document.AddMember(rapidjson::Value(field.first.c_str(), allocator).Move(), field.second, allocator);
    }
    for (const auto & field : event.event_string_array_fields) {
      rapidjson::Value values(rapidjson::kArrayType);
      for (const auto & item : field.second) {
        values.PushBack(rapidjson::Value(item.c_str(), allocator).Move(), allocator);
      }
      document.AddMember(rapidjson::Value(field.first.c_str(), allocator).Move(), values, allocator);
    }
    for (const auto & field : event.event_string_object_fields) {
      rapidjson::Value object(rapidjson::kObjectType);
      for (const auto & item : field.second) {
        object.AddMember(
          rapidjson::Value(item.first.c_str(), allocator).Move(),
          rapidjson::Value(item.second.c_str(), allocator).Move(),
          allocator);
      }
      document.AddMember(rapidjson::Value(field.first.c_str(), allocator).Move(), object, allocator);
    }
  }

  void add_string_member_if_missing(
    rapidjson::Document & document,
    const char * name,
    const std::string & value)
  {
    if (document.HasMember(name)) {
      return;
    }
    auto & allocator = document.GetAllocator();
    document.AddMember(
      rapidjson::Value(name, allocator).Move(),
      rapidjson::Value(value.c_str(), allocator).Move(),
      allocator);
  }

  void add_bool_member_if_missing(rapidjson::Document & document, const char * name, const bool value)
  {
    if (!document.HasMember(name)) {
      auto & allocator = document.GetAllocator();
      document.AddMember(rapidjson::Value(name, allocator).Move(), value, allocator);
    }
  }

  void add_int_member_if_missing(rapidjson::Document & document, const char * name, const int value)
  {
    if (!document.HasMember(name)) {
      auto & allocator = document.GetAllocator();
      document.AddMember(rapidjson::Value(name, allocator).Move(), value, allocator);
    }
  }

  void add_position_member_if_missing(
    rapidjson::Document & document,
    const char * name,
    const std::array<double, 3> & position)
  {
    if (document.HasMember(name)) {
      return;
    }
    auto & allocator = document.GetAllocator();
    rapidjson::Value values(rapidjson::kArrayType);
    values.PushBack(position[0], allocator);
    values.PushBack(position[1], allocator);
    values.PushBack(position[2], allocator);
    document.AddMember(rapidjson::Value(name, allocator).Move(), values, allocator);
  }

  void add_current_target_protocol_fields(rapidjson::Document & document)
  {
    const auto * waypoint = find_route_waypoint_by_id(route_.current_target_task_id);
    add_string_member_if_missing(document, "task_session_id", route_.task_session_id);
    add_string_member_if_missing(document, "route_id", route_.route_id);
    add_string_member_if_missing(document, "map_id", route_.map_id);
    add_bool_member_if_missing(document, "route_task", true);
    add_string_member_if_missing(document, "current_target_task_id", route_.current_target_task_id);
    add_string_member_if_missing(document, "current_waypoint_id", route_.current_target_task_id);
    add_string_member_if_missing(
      document, "current_waypoint_name",
      waypoint != nullptr && !waypoint->waypoint_name.empty() ?
      waypoint->waypoint_name : route_.current_target_task_id);
    add_int_member_if_missing(document, "waypoint_index", route_.current_target_task_index);
    add_int_member_if_missing(document, "total_waypoints", static_cast<int>(route_.master_route_task_ids.size()));
    if (route_.active_segment.has_value()) {
      add_string_member_if_missing(document, "segment_id", route_.active_segment->segment_id);
    }
  }

  void add_blocked_target_protocol_fields(rapidjson::Document & document)
  {
    const auto * waypoint = find_route_waypoint_by_id(route_.current_target_task_id);
    add_current_target_protocol_fields(document);
    add_string_member_if_missing(document, "blocked_waypoint_id", route_.current_target_task_id);
    add_string_member_if_missing(
      document, "blocked_waypoint_name",
      waypoint != nullptr && !waypoint->waypoint_name.empty() ?
      waypoint->waypoint_name : route_.current_target_task_id);
    add_int_member_if_missing(document, "blocked_waypoint_index", route_.current_target_task_index);
    add_int_member_if_missing(document, "total_waypoints", static_cast<int>(route_.master_route_task_ids.size()));
    if (waypoint != nullptr) {
      add_position_member_if_missing(document, "position", waypoint->position);
    }
  }

  std::string route_task_session_for_event(const rapidjson::Document & payload) const
  {
    if (payload.HasMember("task_session_id")) {
      return route_task_id(payload["task_session_id"]);
    }
    if (route_.active) {
      return route_.task_session_id;
    }
    return "";
  }

  void add_route_task_tracking_fields(
    rapidjson::Document & payload,
    const std::string & event_type)
  {
    auto & allocator = payload.GetAllocator();
    if (!payload.HasMember("event_id")) {
      const std::string event_id = build_route_task_event_id(event_type, route_task_session_for_event(payload));
      payload.AddMember("event_id", rapidjson::Value(event_id.c_str(), allocator).Move(), allocator);
    }
    if (!payload.HasMember("timestamp")) {
      payload.AddMember("timestamp", now_seconds(), allocator);
    }
    if (route_.active) {
      add_string_member_if_missing(payload, "task_session_id", route_.task_session_id);
      add_string_member_if_missing(payload, "route_id", route_.route_id);
      add_string_member_if_missing(payload, "map_id", route_.map_id);
    }
    if (event_type == "route_task_completed" && !payload.HasMember("summary")) {
      rapidjson::Value summary(rapidjson::kObjectType);
      summary.AddMember(
        "task_count",
        payload.HasMember("task_count") && payload["task_count"].IsInt() ? payload["task_count"].GetInt() : 0,
        allocator);
      summary.AddMember(
        "completed_count",
        payload.HasMember("completed_count") && payload["completed_count"].IsInt() ?
        payload["completed_count"].GetInt() : 0,
        allocator);
      summary.AddMember(
        "skipped_count",
        payload.HasMember("skipped_count") && payload["skipped_count"].IsInt() ?
        payload["skipped_count"].GetInt() : 0,
        allocator);
      payload.AddMember("summary", summary, allocator);
    }
  }

  void publish_route_task_event_data(const RouteTaskEventData & event)
  {
    rapidjson::Document payload;
    payload.SetObject();
    append_route_task_event_fields(payload, event);
    add_route_task_tracking_fields(payload, event.event_type);
    publish_status_update(event.event_type, payload);
  }

  void publish_route_task_result_events(const RouteTaskCommandResult & result)
  {
    publish_route_task_event_data(result);
    for (const auto & event : result.followup_events) {
      publish_route_task_event_data(event);
    }
    if (result.reset_route_after_publish) {
      route_task_state_machine_.reset_route_task_state(route_);
    }
  }

  void publish_zero_cmd_vel()
  {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
  }

  // ===========================================================================
  // 5. Nav2 执行链路：路线段下发、最终 task 对齐和到达闭环
  // ===========================================================================

  const RouteWaypoint * find_route_waypoint_by_id(const std::string & waypoint_id) const
  {
    for (const auto & waypoint : route_.route_waypoints) {
      if (waypoint.waypoint_id == waypoint_id) {
        return &waypoint;
      }
    }
    return nullptr;
  }

  std::optional<geometry_msgs::msg::PoseStamped> route_waypoint_to_pose_stamped(
    const std::string & waypoint_id) const
  {
    const RouteWaypoint * waypoint = find_route_waypoint_by_id(waypoint_id);
    if (waypoint == nullptr) {
      return std::nullopt;
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    pose.header.frame_id = waypoint->frame_id.empty() ? config_.default_frame_id : waypoint->frame_id;
    pose.pose.position.x = waypoint->position[0];
    pose.pose.position.y = waypoint->position[1];
    pose.pose.position.z = waypoint->position[2];
    pose.pose.orientation.x = waypoint->orientation[0];
    pose.pose.orientation.y = waypoint->orientation[1];
    pose.pose.orientation.z = waypoint->orientation[2];
    pose.pose.orientation.w = waypoint->orientation[3];
    return pose;
  }

  std::string route_waypoint_walk_direction(const RouteWaypoint & waypoint) const
  {
    std::string normalized = trim_copy(waypoint.walk_direction);
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    if (normalized == "backward" || normalized == "reverse" || normalized == "back" ||
      normalized == "倒走" || normalized == "倒车" || normalized == "后退")
    {
      return "backward";
    }
    return "forward";
  }

  bool is_current_pose_near_route_waypoint(
    const std::string & waypoint_id,
    const double tolerance_m) const
  {
    if (!environment_.has_current_pose) {
      return false;
    }
    const RouteWaypoint * waypoint = find_route_waypoint_by_id(waypoint_id);
    if (waypoint == nullptr) {
      return false;
    }

    const double dx = environment_.current_position[0] - waypoint->position[0];
    const double dy = environment_.current_position[1] - waypoint->position[1];
    return std::hypot(dx, dy) <= std::max(0.0, tolerance_m);
  }

  bool should_complete_active_segment_without_navigation() const
  {
    if (!route_.active_segment.has_value() || !environment_.has_current_pose) {
      return false;
    }
    if (route_.active_segment->execution_waypoint_ids.empty()) {
      return false;
    }

    for (const auto & waypoint_id : route_.active_segment->execution_waypoint_ids) {
      const bool is_target_task = waypoint_id == route_.current_target_task_id;
      const double tolerance = is_target_task ?
        config_.route_task_first_task_reached_tolerance_m :
        config_.route_task_transit_passed_tolerance_m;
      if (!is_current_pose_near_route_waypoint(waypoint_id, tolerance)) {
        return false;
      }
    }
    return true;
  }

  bool complete_active_segment_without_navigation()
  {
    if (!route_.active_segment.has_value()) {
      handle_route_task_navigation_failed("active segment is empty", "invalid_route_waypoints");
      return false;
    }

    route_.current_state = NavigationState::Executing;
    route_.detailed_state = "ROUTE_TASK_SEGMENT_ALREADY_REACHED";
    route_.navigation_mode = "route_task";
    route_.navigation_start_time = now_seconds();

    const auto transit_waypoint_ids = route_.active_segment->transit_waypoint_ids;
    for (const auto & waypoint_id : transit_waypoint_ids) {
      mark_transit_passed(waypoint_id);
    }

    RCLCPP_INFO(
      get_logger(),
      "route task segment already reached by current pose, skip NavigateThroughPoses: segment_id=%s, target=%s",
      route_.active_segment->segment_id.c_str(), route_.current_target_task_id.c_str());
    return start_active_segment_final_pose_navigation("ROUTE_TASK_FINAL_ALIGNING");
  }

  std::vector<std::string> remaining_active_segment_execution_waypoint_ids() const
  {
    std::vector<std::string> remaining;
    if (!route_.active_segment.has_value()) {
      return remaining;
    }
    const auto & segment = route_.active_segment.value();
    for (const auto & waypoint_id : segment.execution_waypoint_ids) {
      const bool already_passed = std::find(
        segment.passed_transit_waypoint_ids.begin(),
        segment.passed_transit_waypoint_ids.end(),
        waypoint_id) != segment.passed_transit_waypoint_ids.end();
      if (!already_passed) {
        remaining.push_back(waypoint_id);
      }
    }
    return remaining;
  }

  bool is_active_segment_transit(const std::string & waypoint_id) const
  {
    if (!route_.active_segment.has_value()) {
      return false;
    }
    const auto & transit_ids = route_.active_segment->transit_waypoint_ids;
    return std::find(transit_ids.begin(), transit_ids.end(), waypoint_id) != transit_ids.end();
  }

  bool start_active_segment_navigation(const std::string & reason)
  {
    if (!config_.route_task_nav2_execution_enable) {
      return true;
    }
    if (!route_.active_segment.has_value()) {
      handle_route_task_navigation_failed("active segment is empty", "invalid_route_waypoints");
      return false;
    }
    if (should_complete_active_segment_without_navigation()) {
      route_.active_goal_generation += 1;
      route_.current_goal_generation = route_.active_goal_generation;
      route_.active_segment->segment_goal_generation = route_.current_goal_generation;
      return complete_active_segment_without_navigation();
    }
    if (reason != "start_route_task" && reason != "next_segment") {
      // 恢复或重试前使用最新可信位姿补记已经越过的 transit，进度只前进不回退。
      resolve_transit_progress_from_pose();
    }
    const auto execution_waypoint_ids = remaining_active_segment_execution_waypoint_ids();
    const bool has_remaining_transit = std::any_of(
      execution_waypoint_ids.begin(), execution_waypoint_ids.end(),
      [this](const std::string & waypoint_id) {return is_active_segment_transit(waypoint_id);});
    if (!has_remaining_transit) {
      return start_active_segment_final_pose_navigation("ROUTE_TASK_FINAL_POSE_NAVIGATING");
    }

    if (!nav2_controller_.wait_for_navigate_through_poses_server(5s)) {
      handle_route_task_navigation_failed(
        "NavigateThroughPoses action server unavailable", "navigation_busy");
      return false;
    }

    NavigateThroughPoses::Goal goal;
    for (const auto & waypoint_id : execution_waypoint_ids) {
      auto pose = route_waypoint_to_pose_stamped(waypoint_id);
      if (!pose.has_value()) {
        handle_route_task_navigation_failed(
          "waypoint " + waypoint_id + " has no valid pose", "missing_waypoint_pose");
        return false;
      }
      goal.poses.push_back(pose.value());
    }
    if (goal.poses.empty()) {
      handle_route_task_navigation_failed(
        "active segment has no execution waypoint", "invalid_route_waypoints");
      return false;
    }

    route_.active_goal_generation += 1;
    route_.current_goal_generation = route_.active_goal_generation;
    route_.active_segment->segment_goal_generation = route_.current_goal_generation;
    route_.current_state = NavigationState::Executing;
    route_.detailed_state = "ROUTE_TASK_SEGMENT_NAVIGATING";
    route_.navigation_mode = "route_task";
    route_.navigation_start_time = now_seconds();
    route_task_last_feedback_time_ = now_seconds();
    route_goal_execution_waypoint_ids_ = execution_waypoint_ids;
    if (route_goal_reject_retry_segment_id_ != route_.active_segment->segment_id) {
      route_goal_reject_retry_segment_id_ = route_.active_segment->segment_id;
      route_goal_reject_retry_deadline_ =
        now_seconds() + std::max(0.0, config_.route_task_goal_reject_retry_timeout_sec);
      route_goal_reject_retry_count_ = 0;
    }

    const int64_t generation = route_.current_goal_generation;
    const int64_t version = route_.route_task_version;
    auto options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    options.goal_response_callback =
      [this, generation, version](const GoalHandleNavigateThroughPoses::SharedPtr & goal_handle) {
        route_task_through_goal_response_callback(goal_handle, generation, version);
      };
    options.feedback_callback =
      [this, generation, version](
        GoalHandleNavigateThroughPoses::SharedPtr,
        const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback) {
        route_task_through_feedback_callback(feedback, generation, version);
      };
    options.result_callback =
      [this, generation, version](const GoalHandleNavigateThroughPoses::WrappedResult & result) {
        route_task_through_result_callback(result, generation, version);
      };
    try {
      nav2_controller_.async_send_navigate_through_poses_goal(goal, options);
    } catch (const std::exception & exc) {
      route_goal_execution_waypoint_ids_.clear();
      handle_route_task_navigation_failed(
        std::string("NavigateThroughPoses send goal failed: ") + exc.what(), "send_goal_failed");
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "route task through segment started: reason=%s, segment_id=%s, target=%s, generation=%ld, remaining=%zu",
      reason.c_str(), route_.active_segment->segment_id.c_str(), route_.current_target_task_id.c_str(),
      static_cast<long>(generation), execution_waypoint_ids.size());
    return true;
  }

  void cancel_active_route_goal()
  {
    cancel_route_goal_retry_timer();
    route_task_last_feedback_time_ = 0.0;
    route_.current_goal_generation = -1;
    route_goal_execution_waypoint_ids_.clear();
    nav2_controller_.cancel_active_goals();
  }

  void cancel_route_goal_retry_timer()
  {
    if (route_goal_retry_timer_) {
      route_goal_retry_timer_->cancel();
      route_goal_retry_timer_.reset();
    }
  }

  bool start_active_segment_final_pose_navigation(const std::string & detailed_state)
  {
    route_goal_execution_waypoint_ids_.clear();
    const RouteWaypoint * target_task = find_route_waypoint_by_id(route_.current_target_task_id);
    if (target_task == nullptr) {
      handle_route_task_navigation_failed("target task missing", "target_task_missing");
      return false;
    }
    auto pose = route_waypoint_to_pose_stamped(route_.current_target_task_id);
    if (!pose.has_value()) {
      handle_route_task_navigation_failed(
        "waypoint " + route_.current_target_task_id + " has no valid pose", "missing_waypoint_pose");
      return false;
    }
    if (!nav2_controller_.wait_for_navigate_to_pose_server(5s)) {
      handle_route_task_navigation_failed("NavigateToPose action server unavailable", "navigation_busy");
      return false;
    }

    route_.active_goal_generation += 1;
    route_.current_goal_generation = route_.active_goal_generation;
    if (route_.active_segment.has_value()) {
      route_.active_segment->segment_goal_generation = route_.current_goal_generation;
    }
    route_.current_state = NavigationState::Executing;
    route_.detailed_state = detailed_state;
    route_.navigation_mode = "route_task";
    route_.navigation_start_time = now_seconds();

    NavigateToPose::Goal goal;
    goal.pose = pose.value();
    const std::string walk_direction = route_waypoint_walk_direction(*target_task);
    if (walk_direction == "backward") {
      goal.behavior_tree = config_.reverse_navigation_bt_xml;
    }

    const int64_t generation = route_.current_goal_generation;
    const int64_t version = route_.route_task_version;
    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.goal_response_callback =
      [this, generation, version](const GoalHandleNavigateToPose::SharedPtr & goal_handle) {
        route_task_final_pose_goal_response_callback(goal_handle, generation, version);
      };
    options.result_callback =
      [this, generation, version](const GoalHandleNavigateToPose::WrappedResult & result) {
        route_task_final_pose_result_callback(result, generation, version);
      };
    try {
      nav2_controller_.async_send_navigate_to_pose_goal(goal, options);
    } catch (const std::exception & exc) {
      handle_route_task_navigation_failed(
        std::string("NavigateToPose send goal failed: ") + exc.what(), "send_goal_failed");
      return false;
    }

    rapidjson::Document event;
    event.SetObject();
    auto & allocator = event.GetAllocator();
    const std::string segment_id = route_.active_segment.has_value() ? route_.active_segment->segment_id : "";
    event.AddMember("segment_id", rapidjson::Value(segment_id.c_str(), allocator).Move(), allocator);
    event.AddMember(
      "waypoint_id", rapidjson::Value(route_.current_target_task_id.c_str(), allocator).Move(), allocator);
    event.AddMember("walk_direction", rapidjson::Value(walk_direction.c_str(), allocator).Move(), allocator);
    event.AddMember("behavior_tree", rapidjson::Value(goal.behavior_tree.c_str(), allocator).Move(), allocator);
    event.AddMember("align_reason", rapidjson::Value(detailed_state.c_str(), allocator).Move(), allocator);
    add_route_task_tracking_fields(event, "final_align_started");
    publish_status_update("final_align_started", event);
    return true;
  }

  void route_task_through_goal_response_callback(
    const GoalHandleNavigateThroughPoses::SharedPtr & goal_handle,
    const int64_t generation,
    const int64_t version)
  {
    if (version != route_.route_task_version || generation != route_.current_goal_generation) {
      return;
    }
    if (!goal_handle) {
      if (retry_route_task_goal_rejected(generation, version)) {
        return;
      }
      handle_route_task_navigation_failed("NavigateThroughPoses goal rejected", "goal_rejected");
      return;
    }
    route_goal_reject_retry_count_ = 0;
    route_task_last_feedback_time_ = now_seconds();
    nav2_controller_.set_through_goal_handle(goal_handle);
  }

  bool retry_route_task_goal_rejected(const int64_t generation, const int64_t version)
  {
    if (version != route_.route_task_version || generation != route_.current_goal_generation ||
      !route_.active || !route_.active_segment.has_value())
    {
      return false;
    }
    if (now_seconds() > route_goal_reject_retry_deadline_) {
      return false;
    }

    ++route_goal_reject_retry_count_;
    const int retry_count = route_goal_reject_retry_count_;
    const std::string segment_id = route_.active_segment->segment_id;
    RCLCPP_WARN(
      get_logger(),
      "NavigateThroughPoses goal rejected，可能是 Nav2 lifecycle 尚未 active，0.5s 后重试同一路线段: "
      "segment_id=%s, retry=%d",
      segment_id.c_str(), retry_count);

    if (route_goal_retry_timer_) {
      route_goal_retry_timer_->cancel();
    }
    route_goal_retry_timer_ = create_wall_timer(
      500ms,
      [this, generation, version, segment_id]() {
        if (route_goal_retry_timer_) {
          route_goal_retry_timer_->cancel();
          route_goal_retry_timer_.reset();
        }
        if (version != route_.route_task_version || generation != route_.current_goal_generation ||
          !route_.active || !route_.active_segment.has_value() ||
          route_.active_segment->segment_id != segment_id)
        {
          return;
        }
        if (!start_active_segment_navigation("route_task_goal_retry")) {
          handle_route_task_navigation_failed(
            "NavigateThroughPoses retry start failed", "goal_rejected_retry_start_failed");
        }
      });
    return true;
  }

  void route_task_through_feedback_callback(
    const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback,
    const int64_t generation,
    const int64_t version)
  {
    if (version != route_.route_task_version || generation != route_.current_goal_generation || !feedback) {
      return;
    }
    route_task_last_feedback_time_ = now_seconds();
    distance_remaining_ = feedback->distance_remaining;
    resolve_transit_progress_from_feedback(feedback->number_of_poses_remaining);
    resolve_transit_progress_from_pose();
  }

  void check_route_task_feedback_timeout()
  {
    if (!route_.active || route_.detailed_state != "ROUTE_TASK_SEGMENT_NAVIGATING") {
      return;
    }
    if (!nav2_controller_.has_through_goal()) {
      return;
    }
    const double timeout_sec = std::max(0.0, config_.route_task_nav2_feedback_timeout_sec);
    if (timeout_sec <= 0.0 || route_task_last_feedback_time_ <= 0.0) {
      return;
    }
    const double feedback_age = now_seconds() - route_task_last_feedback_time_;
    if (feedback_age <= timeout_sec) {
      return;
    }
    const std::string segment_id = route_.active_segment.has_value() ? route_.active_segment->segment_id : "";
    RCLCPP_ERROR(
      get_logger(),
      "route task through feedback 超时: age=%.2fs, timeout=%.2fs, segment_id=%s",
      feedback_age, timeout_sec, segment_id.c_str());
    handle_route_task_navigation_failed("NavigateThroughPoses feedback timeout", "feedback_timeout");
  }

  void resolve_transit_progress_from_feedback(const int poses_remaining)
  {
    if (!route_.active_segment.has_value()) {
      return;
    }
    auto & segment = route_.active_segment.value();
    if (route_goal_execution_waypoint_ids_.empty() || segment.transit_waypoint_ids.empty()) {
      return;
    }

    const int execution_count = static_cast<int>(route_goal_execution_waypoint_ids_.size());
    const int normalized_remaining = std::max(0, poses_remaining);
    const int passed_pose_count =
      std::max(0, std::min(execution_count, execution_count - normalized_remaining));
    for (int index = 0; index < passed_pose_count; ++index) {
      const auto & waypoint_id = route_goal_execution_waypoint_ids_[static_cast<std::size_t>(index)];
      if (is_active_segment_transit(waypoint_id)) {
        mark_transit_passed(waypoint_id);
      }
    }
  }

  std::optional<std::array<double, 3>> route_waypoint_position(const std::string & waypoint_id) const
  {
    const RouteWaypoint * waypoint = find_route_waypoint_by_id(waypoint_id);
    if (waypoint == nullptr) {
      return std::nullopt;
    }
    return waypoint->position;
  }

  std::optional<std::array<double, 3>> resolve_previous_execution_position_for_transit(
    const std::string & waypoint_id) const
  {
    if (!route_.active_segment.has_value()) {
      return std::nullopt;
    }
    const auto & execution_ids = route_.active_segment->execution_waypoint_ids;
    const auto it = std::find(execution_ids.begin(), execution_ids.end(), waypoint_id);
    if (it != execution_ids.end()) {
      const auto index = static_cast<std::size_t>(std::distance(execution_ids.begin(), it));
      if (index > 0) {
        return route_waypoint_position(execution_ids[index - 1]);
      }
    }

    const int start_source_index = route_.active_segment->segment_start_source_index;
    if (start_source_index >= 0 &&
      start_source_index < static_cast<int>(route_.route_waypoints.size()))
    {
      return route_.route_waypoints[static_cast<std::size_t>(start_source_index)].position;
    }
    return std::nullopt;
  }

  bool is_transit_passed_by_current_pose(const std::string & waypoint_id) const
  {
    if (!environment_.has_current_pose) {
      return false;
    }
    const auto waypoint_position = route_waypoint_position(waypoint_id);
    if (!waypoint_position.has_value()) {
      return false;
    }

    const double robot_x = environment_.current_position[0];
    const double robot_y = environment_.current_position[1];
    const double dx = robot_x - waypoint_position->at(0);
    const double dy = robot_y - waypoint_position->at(1);
    const double threshold = std::max(0.0, config_.route_task_transit_passed_tolerance_m);
    if (std::hypot(dx, dy) <= threshold) {
      return true;
    }

    if (!config_.route_task_transit_projection_passed_enabled) {
      return false;
    }
    const auto previous_position = resolve_previous_execution_position_for_transit(waypoint_id);
    if (!previous_position.has_value()) {
      return false;
    }

    const double seg_dx = waypoint_position->at(0) - previous_position->at(0);
    const double seg_dy = waypoint_position->at(1) - previous_position->at(1);
    const double seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy;
    if (seg_len_sq <= 1e-6) {
      return false;
    }

    const double projection =
      ((robot_x - previous_position->at(0)) * seg_dx +
      (robot_y - previous_position->at(1)) * seg_dy) / seg_len_sq;
    if (projection <= 1.0) {
      return false;
    }
    const double cross_track_distance = std::abs(
      (robot_x - previous_position->at(0)) * seg_dy -
      (robot_y - previous_position->at(1)) * seg_dx) / std::sqrt(seg_len_sq);
    return cross_track_distance <= std::max(threshold * 2.0, threshold + 0.2);
  }

  void resolve_transit_progress_from_pose()
  {
    if (!route_.active_segment.has_value() || !environment_.has_current_pose) {
      return;
    }
    const auto transit_ids = route_.active_segment->transit_waypoint_ids;
    for (const auto & waypoint_id : transit_ids) {
      const auto & passed = route_.active_segment->passed_transit_waypoint_ids;
      if (std::find(passed.begin(), passed.end(), waypoint_id) != passed.end()) {
        continue;
      }
      if (is_transit_passed_by_current_pose(waypoint_id)) {
        mark_transit_passed(waypoint_id);
        continue;
      }
      break;
    }
  }

  void mark_transit_passed(const std::string & waypoint_id)
  {
    if (!route_.active_segment.has_value()) {
      return;
    }
    auto & segment = route_.active_segment.value();
    if (std::find(
        segment.passed_transit_waypoint_ids.begin(), segment.passed_transit_waypoint_ids.end(),
        waypoint_id) != segment.passed_transit_waypoint_ids.end())
    {
      return;
    }

    segment.passed_transit_waypoint_ids.push_back(waypoint_id);
    segment.current_segment_progress_index =
      static_cast<int>(segment.passed_transit_waypoint_ids.size());

    rapidjson::Document event;
    event.SetObject();
    auto & allocator = event.GetAllocator();
    event.AddMember(
      "segment_id", rapidjson::Value(segment.segment_id.c_str(), allocator).Move(), allocator);
    event.AddMember("waypoint_id", rapidjson::Value(waypoint_id.c_str(), allocator).Move(), allocator);
    event.AddMember("waypoint_role", "transit", allocator);
    rapidjson::Value passed_ids(rapidjson::kArrayType);
    for (const auto & passed_id : segment.passed_transit_waypoint_ids) {
      passed_ids.PushBack(rapidjson::Value(passed_id.c_str(), allocator).Move(), allocator);
    }
    event.AddMember("passed_transit_waypoint_ids", passed_ids, allocator);
    event.AddMember(
      "current_target_task_id",
      rapidjson::Value(route_.current_target_task_id.c_str(), allocator).Move(),
      allocator);
    publish_status_update("waypoint_passed", event);
  }

  void route_task_through_result_callback(
    const GoalHandleNavigateThroughPoses::WrappedResult & result,
    const int64_t generation,
    const int64_t version)
  {
    if (version != route_.route_task_version || generation != route_.current_goal_generation) {
      return;
    }
    nav2_controller_.clear_through_goal_handle();
    route_task_last_feedback_time_ = 0.0;
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      const auto completed_goal_ids = route_goal_execution_waypoint_ids_;
      for (const auto & waypoint_id : completed_goal_ids) {
        if (is_active_segment_transit(waypoint_id)) {
          mark_transit_passed(waypoint_id);
        }
      }
      route_goal_execution_waypoint_ids_.clear();
      start_active_segment_final_pose_navigation("ROUTE_TASK_FINAL_ALIGNING");
      return;
    }
    route_goal_execution_waypoint_ids_.clear();
    if (result.code == rclcpp_action::ResultCode::CANCELED) {
      if (obstacle_wait_manager_.active()) {
        return;
      }
      handle_route_task_navigation_failed("NavigateThroughPoses goal canceled", "goal_canceled");
      return;
    }
    handle_route_task_navigation_failed("NavigateThroughPoses goal failed", "goal_failed");
  }

  void route_task_final_pose_goal_response_callback(
    const GoalHandleNavigateToPose::SharedPtr & goal_handle,
    const int64_t generation,
    const int64_t version)
  {
    if (version != route_.route_task_version || generation != route_.current_goal_generation) {
      return;
    }
    if (!goal_handle) {
      handle_route_task_navigation_failed("NavigateToPose goal rejected", "final_pose_goal_rejected");
      return;
    }
    nav2_controller_.set_final_pose_goal_handle(goal_handle);
  }

  void route_task_final_pose_result_callback(
    const GoalHandleNavigateToPose::WrappedResult & result,
    const int64_t generation,
    const int64_t version)
  {
    if (version != route_.route_task_version || generation != route_.current_goal_generation) {
      return;
    }
    nav2_controller_.clear_final_pose_goal_handle();
    if (result.code == rclcpp_action::ResultCode::CANCELED) {
      if (obstacle_wait_manager_.active()) {
        return;
      }
      handle_route_task_navigation_failed("NavigateToPose goal canceled", "final_pose_goal_canceled");
      return;
    }
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      handle_route_task_navigation_failed("NavigateToPose goal failed", "final_pose_goal_failed");
      return;
    }

    rapidjson::Document event;
    event.SetObject();
    auto & allocator = event.GetAllocator();
    const std::string segment_id = route_.active_segment.has_value() ? route_.active_segment->segment_id : "";
    event.AddMember("segment_id", rapidjson::Value(segment_id.c_str(), allocator).Move(), allocator);
    event.AddMember(
      "waypoint_id", rapidjson::Value(route_.current_target_task_id.c_str(), allocator).Move(), allocator);
    add_route_task_tracking_fields(event, "final_align_completed");
    publish_status_update("final_align_completed", event);

    auto arrival_result = route_task_state_machine_.handle_target_task_arrived(route_);
    publish_route_task_result_events(arrival_result);
    if (route_.active && route_.current_state == NavigationState::Executing && route_.active_segment.has_value() &&
      !route_.awaiting_broadcast && arrival_result.event_type == "task_waypoint_completed")
    {
      start_active_segment_navigation("next_segment");
    }
  }

  bool try_enter_obstacle_wait_from_nav_failure(const std::string & reason, const std::string & failure_code)
  {
    if (failure_code != "goal_failed" && failure_code != "final_pose_goal_failed") {
      return false;
    }
    if (!config_.obstacle_wait_enable || obstacle_wait_manager_.active()) {
      return false;
    }
    if (!route_.active || !route_.active_segment.has_value()) {
      return false;
    }
    if (route_.current_state != NavigationState::Executing && route_.current_state != NavigationState::Planning) {
      return false;
    }
    const std::string suppression_reason = get_obstacle_blockage_suppression_reason();
    if (!suppression_reason.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "Nav2 执行失败但当前处于%s，不进入障碍等待: %s",
        suppression_reason.c_str(), reason.c_str());
      return false;
    }

    RCLCPP_WARN(
      get_logger(),
      "Nav2 执行失败，进入障碍等待恢复流程: %s",
      reason.c_str());
    enter_obstacle_wait_state(reason);
    return true;
  }

  bool localization_recovery_required() const
  {
    if (localization_.recovery_required) {
      return true;
    }
    if (localization_.state == "recovery_requires_global_relocalization") {
      return true;
    }
    if (localization_.state == "bridge_holding_last_good_tf" &&
      localization_.text.find("large_jump") != std::string::npos)
    {
      return true;
    }
    return false;
  }

  bool should_enter_localization_recovery_pause() const
  {
    if (!config_.localization_auto_pause_on_recovery_required || route_.localization_auto_paused) {
      return false;
    }
    if (!route_.active || !route_.active_segment.has_value()) {
      return false;
    }
    if (route_.current_state != NavigationState::Executing) {
      return false;
    }
    if (obstacle_wait_manager_.active() || route_.pause_source == "obstacle_wait") {
      return false;
    }
    return localization_recovery_required();
  }

  void enter_localization_recovery_wait_state()
  {
    const double now = now_seconds();
    const std::string reason =
      "定位发生大跳变，已暂停导航并等待全局重定位确认: " + localization_.state + " " + localization_.text;
    cancel_active_route_goal();
    route_.current_state = NavigationState::Paused;
    route_.detailed_state = "LOCALIZATION_RECOVERY_WAITING";
    route_.pause_source = "localization_recovery";
    route_.pause_reason = reason;
    route_.resume_mode = "auto";
    route_.localization_auto_paused = true;
    route_.localization_recovery_started_at = now;
    reset_block_detection();
    nav2_blockage_suppression_nodes_.clear();
    publish_zero_cmd_vel();

    rapidjson::Document event;
    event.SetObject();
    auto & allocator = event.GetAllocator();
    event.AddMember("pause_source", "localization_recovery", allocator);
    event.AddMember("reason", rapidjson::Value(reason.c_str(), allocator).Move(), allocator);
    event.AddMember("resume_mode", "auto", allocator);
    event.AddMember("localization_state", rapidjson::Value(localization_.state.c_str(), allocator).Move(), allocator);
    event.AddMember("localization_text", rapidjson::Value(localization_.text.c_str(), allocator).Move(), allocator);
    event.AddMember("resume_stable_count", localization_.resume_stable_count, allocator);
    event.AddMember("resume_required_frames", config_.localization_resume_stable_frames, allocator);
    event.AddMember("pause_time", now, allocator);
    event.AddMember(
      "task_session_id", rapidjson::Value(route_.task_session_id.c_str(), allocator).Move(), allocator);
    event.AddMember("route_id", rapidjson::Value(route_.route_id.c_str(), allocator).Move(), allocator);
    event.AddMember(
      "current_target_task_id",
      rapidjson::Value(route_.current_target_task_id.c_str(), allocator).Move(),
      allocator);
    if (route_.active_segment.has_value()) {
      event.AddMember(
        "segment_id", rapidjson::Value(route_.active_segment->segment_id.c_str(), allocator).Move(),
        allocator);
    }
    publish_status_update("navigation_paused", event);
    send_acknowledgment("navigation_paused", "success", reason, &event);
  }

  void process_localization_recovery_state()
  {
    if (should_enter_localization_recovery_pause()) {
      enter_localization_recovery_wait_state();
      return;
    }
    if (!route_.localization_auto_paused) {
      return;
    }
    if (!route_.active || !route_.active_segment.has_value() || route_.pause_source != "localization_recovery") {
      route_.localization_auto_paused = false;
      route_.localization_recovery_started_at = 0.0;
      return;
    }
    if (localization_recovery_required()) {
      return;
    }
    const int required_frames = std::max(1, config_.localization_resume_stable_frames);
    if (!localization_.healthy || localization_.resume_stable_count < required_frames) {
      return;
    }
    resume_from_localization_recovery_wait();
  }

  void resume_from_localization_recovery_wait()
  {
    if (!route_.active || !route_.active_segment.has_value()) {
      route_.localization_auto_paused = false;
      route_.localization_recovery_started_at = 0.0;
      return;
    }
    const double now = now_seconds();
    const double paused_duration =
      route_.localization_recovery_started_at > 0.0 ? now - route_.localization_recovery_started_at : 0.0;

    route_.current_state = NavigationState::Executing;
    route_.detailed_state = "EXECUTING";
    route_.pause_source.clear();
    route_.pause_reason.clear();
    route_.resume_mode.clear();
    route_.localization_auto_paused = false;
    route_.localization_recovery_started_at = 0.0;

    rapidjson::Document event;
    event.SetObject();
    auto & allocator = event.GetAllocator();
    event.AddMember("resume_source", "localization_recovery", allocator);
    event.AddMember("resume_reason", "localization_trusted_auto_resume", allocator);
    event.AddMember("localization_state", rapidjson::Value(localization_.state.c_str(), allocator).Move(), allocator);
    event.AddMember("localization_text", rapidjson::Value(localization_.text.c_str(), allocator).Move(), allocator);
    event.AddMember("resume_stable_count", localization_.resume_stable_count, allocator);
    event.AddMember("resume_required_frames", std::max(1, config_.localization_resume_stable_frames), allocator);
    event.AddMember("pause_duration_actual", paused_duration, allocator);
    event.AddMember(
      "task_session_id", rapidjson::Value(route_.task_session_id.c_str(), allocator).Move(), allocator);
    event.AddMember("route_id", rapidjson::Value(route_.route_id.c_str(), allocator).Move(), allocator);
    event.AddMember(
      "current_target_task_id",
      rapidjson::Value(route_.current_target_task_id.c_str(), allocator).Move(),
      allocator);
    if (route_.active_segment.has_value()) {
      event.AddMember(
        "segment_id", rapidjson::Value(route_.active_segment->segment_id.c_str(), allocator).Move(),
        allocator);
    }
    publish_status_update("navigation_resumed", event);
    send_acknowledgment("navigation_resumed", "success", "定位已恢复可信，导航自动恢复", &event);

    if (!start_active_segment_navigation("localization_trusted_auto_resume")) {
      handle_route_task_navigation_failed(
        "localization trusted auto resume failed",
        "localization_auto_resume_failed");
    }
  }

  void handle_obstacle_block_timeout(const double block_duration)
  {
    if (block_reported_ || !config_.obstacle_wait_enable) {
      return;
    }
    block_reported_ = true;
    enter_obstacle_wait_state("检测到障碍物，前方路径被挡住", block_duration);
  }

  void enter_obstacle_wait_state(const std::string & reason, const double block_duration = 0.0)
  {
    (void)reason;
    const std::string obstacle_reason = "检测到障碍物，前方路径被挡住";
    const double rounded_block_duration = std::round(block_duration * 10.0) / 10.0;
    const double now = now_seconds();
    obstacle_wait_manager_.enter(now, config_);
    last_obstacle_wait_decision_ = ObstacleWaitDecision{};
    last_obstacle_wait_decision_.active = true;
    last_obstacle_wait_decision_.wait_duration_sec = block_duration;
    cancel_active_route_goal();
    route_.current_state = NavigationState::Paused;
    route_.detailed_state = "OBSTACLE_WAITING";
    route_.pause_source = "obstacle_wait";
    route_.pause_reason = "检测到障碍物，前方路径被挡住";
    route_.resume_mode = "auto";
    publish_zero_cmd_vel();

    rapidjson::Document event;
    event.SetObject();
    auto & allocator = event.GetAllocator();
    event.AddMember("pause_source", "obstacle_wait", allocator);
    event.AddMember("reason", rapidjson::Value(obstacle_reason.c_str(), allocator).Move(), allocator);
    event.AddMember("resume_mode", "auto", allocator);
    event.AddMember("waiting_for_obstacle_clear", true, allocator);
    event.AddMember("block_duration", rounded_block_duration, allocator);
    event.AddMember("pause_time", now, allocator);
    event.AddMember("clear_confirmed_frames", 0, allocator);
    event.AddMember("clear_required_frames", config_.obstacle_clear_required_frames, allocator);
    event.AddMember(
      "clear_required_duration_sec",
      obstacle_wait_manager_.current_required_clear_duration(config_),
      allocator);
    event.AddMember("false_resume_count", obstacle_wait_manager_.recent_false_resume_count(), allocator);
    event.AddMember(
      "task_session_id", rapidjson::Value(route_.task_session_id.c_str(), allocator).Move(), allocator);
    event.AddMember("route_id", rapidjson::Value(route_.route_id.c_str(), allocator).Move(), allocator);
    event.AddMember("map_id", rapidjson::Value(route_.map_id.c_str(), allocator).Move(), allocator);
    event.AddMember(
      "current_target_task_id",
      rapidjson::Value(route_.current_target_task_id.c_str(), allocator).Move(),
      allocator);
    if (route_.active_segment.has_value()) {
      event.AddMember(
        "segment_id", rapidjson::Value(route_.active_segment->segment_id.c_str(), allocator).Move(),
        allocator);
    }
    add_current_target_protocol_fields(event);
    publish_status_update("navigation_paused", event);
    send_acknowledgment("navigation_paused", "success", "导航已因障碍物暂停", &event);

    rapidjson::Document blocked;
    blocked.SetObject();
    auto & blocked_allocator = blocked.GetAllocator();
    blocked.AddMember(
      "reason", rapidjson::Value(obstacle_reason.c_str(), blocked_allocator).Move(), blocked_allocator);
    blocked.AddMember("block_duration", rounded_block_duration, blocked_allocator);
    blocked.AddMember("pause_source", "obstacle_wait", blocked_allocator);
    blocked.AddMember("waiting_for_obstacle_clear", true, blocked_allocator);
    blocked.AddMember("clear_confirmed_frames", 0, blocked_allocator);
    blocked.AddMember("clear_required_frames", config_.obstacle_clear_required_frames, blocked_allocator);
    blocked.AddMember("clear_duration_sec", 0.0, blocked_allocator);
    blocked.AddMember(
      "clear_required_duration_sec",
      obstacle_wait_manager_.current_required_clear_duration(config_),
      blocked_allocator);
    blocked.AddMember(
      "min_wait_before_resume_sec",
      config_.obstacle_min_wait_before_resume_sec,
      blocked_allocator);
    blocked.AddMember(
      "false_resume_count",
      obstacle_wait_manager_.recent_false_resume_count(),
      blocked_allocator);
    blocked.AddMember(
      "current_target_task_id",
      rapidjson::Value(route_.current_target_task_id.c_str(), blocked_allocator).Move(),
      blocked_allocator);
    add_blocked_target_protocol_fields(blocked);
    publish_status_update("navigation_obstacle_blocked", blocked);
    send_acknowledgment("navigation_obstacle_blocked", "error", obstacle_reason);
  }

  void process_obstacle_wait_state()
  {
    const auto decision = obstacle_wait_manager_.update(
      now_seconds(), config_, environment_, route_.current_state);
    last_obstacle_wait_decision_ = decision;
    if (!decision.active) {
      return;
    }
    if (!decision.ready_to_resume) {
      return;
    }
    resume_from_obstacle_wait(decision);
  }

  void resume_from_obstacle_wait(const ObstacleWaitDecision & decision)
  {
    if (!route_.active || !route_.active_segment.has_value()) {
      clear_obstacle_wait_runtime();
      return;
    }

    clear_obstacle_wait_runtime();
    route_.current_state = NavigationState::Executing;
    route_.detailed_state = "EXECUTING";
    route_.pause_source.clear();
    route_.pause_reason.clear();
    route_.resume_mode.clear();

    const std::string resumed_waypoint_id = route_.active_segment->segment_target_task_id;
    std::string resumed_waypoint_name = resumed_waypoint_id;
    if (const auto * waypoint = find_route_waypoint_by_id(resumed_waypoint_id); waypoint != nullptr) {
      if (!waypoint->waypoint_name.empty()) {
        resumed_waypoint_name = waypoint->waypoint_name;
      }
    }

    rapidjson::Document event;
    event.SetObject();
    auto & allocator = event.GetAllocator();
    event.AddMember("resume_source", "obstacle_wait", allocator);
    event.AddMember("resume_reason", "obstacle_cleared_auto_resume", allocator);
    event.AddMember(
      "resumed_waypoint_id", rapidjson::Value(resumed_waypoint_id.c_str(), allocator).Move(), allocator);
    event.AddMember(
      "resumed_waypoint_name", rapidjson::Value(resumed_waypoint_name.c_str(), allocator).Move(), allocator);
    event.AddMember("waypoint_index", route_.current_target_task_index, allocator);
    event.AddMember("total_waypoints", static_cast<int>(route_.master_route_task_ids.size()), allocator);
    const double rounded_pause_duration = std::round(decision.wait_duration_sec * 10.0) / 10.0;
    event.AddMember("pause_duration_actual", rounded_pause_duration, allocator);
    event.AddMember("route_task", true, allocator);
    event.AddMember("awaiting_broadcast", route_.awaiting_broadcast, allocator);
    event.AddMember(
      "waiting_broadcast_waypoint_id",
      rapidjson::Value(route_.waiting_broadcast_waypoint_id.c_str(), allocator).Move(),
      allocator);
    event.AddMember(
      "waiting_broadcast_id",
      rapidjson::Value(route_.waiting_broadcast_id.c_str(), allocator).Move(),
      allocator);
    event.AddMember("clear_duration_sec", decision.clear_duration_sec, allocator);
    event.AddMember("clear_confirmed_frames", decision.clear_confirmed_frames, allocator);
    event.AddMember("roi_clear_confirmed_frames", decision.roi_clear_confirmed_frames, allocator);
    event.AddMember("front_occupied_cells", decision.occupied_cells, allocator);
    event.AddMember("front_sample_cells", decision.sample_cells, allocator);
    event.AddMember("front_max_cost", decision.max_cost, allocator);
    add_front_obstacle_stats(event, "front_obstacle_stats", allocator, decision, decision.front_blocked);
    add_roi_obstacle_stats(event, "roi_obstacle_stats", allocator, now_seconds());
    event.AddMember(
      "task_session_id", rapidjson::Value(route_.task_session_id.c_str(), allocator).Move(), allocator);
    event.AddMember("route_id", rapidjson::Value(route_.route_id.c_str(), allocator).Move(), allocator);
    event.AddMember(
      "current_target_task_id",
      rapidjson::Value(route_.current_target_task_id.c_str(), allocator).Move(),
      allocator);
    if (route_.active_segment.has_value()) {
      event.AddMember(
        "segment_id", rapidjson::Value(route_.active_segment->segment_id.c_str(), allocator).Move(),
        allocator);
    }
    send_acknowledgment("navigation_resumed", "success", "障碍物已消失，导航自动恢复", &event);
    publish_status_update("navigation_resumed", event);

    if (!start_active_segment_navigation("obstacle_cleared_auto_resume")) {
      handle_route_task_navigation_failed(
        "obstacle cleared auto resume failed",
        "obstacle_auto_resume_failed");
    }
  }

  void handle_route_task_navigation_failed(const std::string & message, const std::string & failure_code)
  {
    last_route_task_failure_message_ = message;
    last_route_task_failure_code_ = failure_code;
    if (try_enter_obstacle_wait_from_nav_failure(message, failure_code)) {
      return;
    }

    rapidjson::Document event;
    event.SetObject();
    auto & allocator = event.GetAllocator();
    event.AddMember("reason", rapidjson::Value(message.c_str(), allocator).Move(), allocator);
    event.AddMember("failure_code", rapidjson::Value(failure_code.c_str(), allocator).Move(), allocator);
    event.AddMember("route_task", true, allocator);
    event.AddMember(
      "task_session_id", rapidjson::Value(route_.task_session_id.c_str(), allocator).Move(), allocator);
    event.AddMember("route_id", rapidjson::Value(route_.route_id.c_str(), allocator).Move(), allocator);
    event.AddMember(
      "current_target_task_id",
      rapidjson::Value(route_.current_target_task_id.c_str(), allocator).Move(),
      allocator);
    if (route_.active_segment.has_value()) {
      const auto & segment = route_.active_segment.value();
      event.AddMember(
        "segment_id", rapidjson::Value(segment.segment_id.c_str(), allocator).Move(),
        allocator);
      event.AddMember(
        "segment_direction", rapidjson::Value(segment.segment_direction.c_str(), allocator).Move(),
        allocator);
      rapidjson::Value execution_ids(rapidjson::kArrayType);
      for (const auto & id : segment.execution_waypoint_ids) {
        execution_ids.PushBack(rapidjson::Value(id.c_str(), allocator).Move(), allocator);
      }
      event.AddMember("execution_waypoint_ids", execution_ids, allocator);
      rapidjson::Value passed_transit_ids(rapidjson::kArrayType);
      for (const auto & id : segment.passed_transit_waypoint_ids) {
        passed_transit_ids.PushBack(rapidjson::Value(id.c_str(), allocator).Move(), allocator);
      }
      event.AddMember("passed_transit_waypoint_ids", passed_transit_ids, allocator);
    } else {
      event.AddMember("segment_id", "", allocator);
      event.AddMember("segment_direction", "", allocator);
      rapidjson::Value execution_ids(rapidjson::kArrayType);
      event.AddMember("execution_waypoint_ids", execution_ids, allocator);
      rapidjson::Value passed_transit_ids(rapidjson::kArrayType);
      event.AddMember("passed_transit_waypoint_ids", passed_transit_ids, allocator);
    }
    rapidjson::Value completed_task_ids(rapidjson::kArrayType);
    for (const auto & id : route_.completed_task_ids) {
      completed_task_ids.PushBack(rapidjson::Value(id.c_str(), allocator).Move(), allocator);
    }
    event.AddMember("completed_task_ids", completed_task_ids, allocator);
    rapidjson::Value skipped_task_ids(rapidjson::kArrayType);
    for (const auto & id : route_.skipped_task_ids) {
      skipped_task_ids.PushBack(rapidjson::Value(id.c_str(), allocator).Move(), allocator);
    }
    event.AddMember("skipped_task_ids", skipped_task_ids, allocator);
    event.AddMember("failed_at", now_seconds(), allocator);
    publish_status_update("navigation_failed", event);
    cancel_active_route_goal();
    route_goal_reject_retry_deadline_ = 0.0;
    route_goal_reject_retry_count_ = 0;
    route_goal_reject_retry_segment_id_.clear();
    route_task_last_feedback_time_ = 0.0;
    reset_block_detection();
    nav2_blockage_suppression_nodes_.clear();
    distance_remaining_ = std::numeric_limits<double>::infinity();
    route_task_state_machine_.reset_route_task_state(route_);
    publish_zero_cmd_vel();
  }

  // ===========================================================================
  // 6. 运行态工具
  // ===========================================================================

  std::string build_event_id(const std::string & event_type)
  {
    ++route_.event_counter;
    return event_type + "_" + std::to_string(static_cast<int64_t>(now_seconds() * 1000.0)) + "_" +
           std::to_string(route_.event_counter);
  }

  std::string build_route_task_event_id(const std::string & event_type, const std::string & session_id)
  {
    ++route_.event_counter;
    const std::string normalized_session = session_id.empty() ? "no_session" : session_id;
    return "route_task_" + normalized_session + "_" + event_type + "_" +
           std::to_string(route_.event_counter) + "_" +
           std::to_string(static_cast<int64_t>(now_seconds() * 1000.0));
  }

  std::string build_start_block_reason() const
  {
    return gatekeeper_.start_block_reason(config_, robot_, localization_, map_, now_seconds());
  }

  void clear_obstacle_wait_runtime()
  {
    obstacle_wait_manager_.clear();
    last_obstacle_wait_decision_ = ObstacleWaitDecision{};
  }

  bool can_defer_for_robot_state(const double now) const
  {
    if (!config_.require_walk_mode_for_navigation) {
      return false;
    }
    const bool status_fresh = robot_.last_update > 0.0 && now - robot_.last_update <= config_.robot_status_timeout;
    const bool status_unavailable = !status_fresh;
    const bool can_defer = status_fresh && (robot_.motion_busy || robot_.control_state == "Menu");
    return status_unavailable || can_defer;
  }

  bool defer_navigation_start_if_not_ready(
    const rapidjson::Value & request,
    const rapidjson::Value & command_data,
    const std::string & block_reason)
  {
    const double now = now_seconds();
    const auto robot_reason = gatekeeper_.robot_start_block_reason(config_, robot_, now);
    const auto localization_reason =
      gatekeeper_.localization_start_block_reason(config_, localization_, now);
    const bool waiting_for_robot = robot_reason.has_value() && can_defer_for_robot_state(now);
    const bool waiting_for_localization = localization_reason.has_value();
    if (!waiting_for_robot && !waiting_for_localization) {
      return false;
    }

    route_.pending_navigation_active = true;
    route_.pending_navigation_created_at = now;
    route_.pending_navigation_request_json = json_to_string(request);
    if (waiting_for_localization) {
      route_.pending_navigation_reason =
        block_reason + "。已缓存导航请求，等待定位恢复可信后自动开始导航。";
    } else if (robot_.last_update <= 0.0 || now - robot_.last_update > config_.robot_status_timeout) {
      route_.pending_navigation_reason =
        block_reason + "。已缓存导航请求，等待底层状态恢复并确认 Walk 后再开始导航。";
    } else if (robot_.motion_busy) {
      route_.pending_navigation_reason =
        block_reason + "。已缓存导航请求，等待动作执行完成并回到 Walk 后再开始导航。";
    } else {
      route_.pending_navigation_reason =
        block_reason + "。已缓存导航请求，等待机器人回到 Walk 后再开始导航。";
    }

    rapidjson::Document event;
    event.SetObject();
    auto & allocator = event.GetAllocator();
    event.AddMember(
      "reason", rapidjson::Value(route_.pending_navigation_reason.c_str(), allocator).Move(), allocator);
    event.AddMember("block_reason", rapidjson::Value(block_reason.c_str(), allocator).Move(), allocator);
    event.AddMember(
      "command_type",
      rapidjson::Value(read_string_member(command_data, "command_type", "").c_str(), allocator).Move(),
      allocator);
    event.AddMember("timeout_sec", config_.pending_navigation_timeout, allocator);
    publish_status_update("navigation_pending", event);
    send_acknowledgment("navigation_pending", "pending", route_.pending_navigation_reason);
    return true;
  }

  bool cancel_pending_navigation(
    const std::string & command_type,
    const rapidjson::Value & command_data)
  {
    if (!route_.pending_navigation_active || route_.current_state != NavigationState::Idle) {
      return false;
    }

    rapidjson::Document pending_request;
    pending_request.Parse(route_.pending_navigation_request_json.c_str());
    if (pending_request.HasParseError() || !pending_request.IsObject() ||
      !pending_request.HasMember("command_data") || !pending_request["command_data"].IsObject())
    {
      return false;
    }
    const auto & pending_data = pending_request["command_data"];
    const std::string pending_session = read_route_task_id_member(pending_data, "task_session_id");
    const std::string pending_route = read_route_task_id_member(pending_data, "route_id");
    const std::string requested_session = read_route_task_id_member(command_data, "task_session_id");
    const std::string requested_route = read_route_task_id_member(command_data, "route_id");
    if ((!requested_session.empty() && requested_session != pending_session) ||
      (!requested_route.empty() && requested_route != pending_route))
    {
      return false;
    }

    route_.pending_navigation_active = false;
    route_.pending_navigation_reason.clear();
    route_.pending_navigation_request_json.clear();

    rapidjson::Document event;
    event.SetObject();
    auto & allocator = event.GetAllocator();
    event.AddMember("reason", "上层已终止等待定位恢复的导航请求", allocator);
    event.AddMember("command_type", rapidjson::Value(command_type.c_str(), allocator).Move(), allocator);
    event.AddMember(
      "task_session_id", rapidjson::Value(pending_session.c_str(), allocator).Move(), allocator);
    event.AddMember("route_id", rapidjson::Value(pending_route.c_str(), allocator).Move(), allocator);
    publish_status_update("navigation_pending_cancelled", event);
    send_route_task_ack(
      command_type, "success", "待执行路线任务已终止", command_data,
      "pending_route_task_stopped");
    return true;
  }

  void check_localization_timeout()
  {
    gatekeeper_.update_localization_timeout(config_, localization_, now_seconds());
  }

  void check_pending_navigation_timeout()
  {
    if (!route_.pending_navigation_active) {
      return;
    }
    try_execute_pending_navigation();
    if (!route_.pending_navigation_active) {
      return;
    }
    if (config_.pending_navigation_timeout <= 0.0) {
      return;
    }
    const double now = now_seconds();
    if (now - route_.pending_navigation_created_at <= config_.pending_navigation_timeout) {
      return;
    }
    std::string command_type;
    rapidjson::Document pending_request;
    pending_request.Parse(route_.pending_navigation_request_json.c_str());
    if (!pending_request.HasParseError() && pending_request.IsObject() &&
      pending_request.HasMember("command_data") && pending_request["command_data"].IsObject())
    {
      command_type = read_string_member(pending_request["command_data"], "command_type", "");
    }
    const std::string reason = build_start_block_reason();
    route_.pending_navigation_active = false;
    route_.pending_navigation_reason.clear();
    route_.pending_navigation_request_json.clear();

    rapidjson::Document event;
    event.SetObject();
    auto & allocator = event.GetAllocator();
    const std::string message = reason.empty() ? "待执行导航超时" : "待执行导航超时: " + reason;
    const std::string event_reason = reason.empty() ? message : reason;
    event.AddMember("reason", rapidjson::Value(event_reason.c_str(), allocator).Move(), allocator);
    event.AddMember("command_type", rapidjson::Value(command_type.c_str(), allocator).Move(), allocator);
    publish_status_update("navigation_pending_timeout", event);
    send_acknowledgment("navigation_started", "error", message);
  }

  void try_execute_pending_navigation()
  {
    if (!route_.pending_navigation_active) {
      return;
    }
    if (route_.current_state != NavigationState::Idle) {
      rapidjson::Document pending_request;
      pending_request.Parse(route_.pending_navigation_request_json.c_str());
      std::string command_type;
      if (!pending_request.HasParseError() && pending_request.IsObject() &&
        pending_request.HasMember("command_data") && pending_request["command_data"].IsObject())
      {
        command_type = read_string_member(pending_request["command_data"], "command_type", "");
      }
      route_.pending_navigation_active = false;
      route_.pending_navigation_reason.clear();
      route_.pending_navigation_request_json.clear();
      rapidjson::Document event;
      event.SetObject();
      auto & allocator = event.GetAllocator();
      event.AddMember("reason", "已有其他导航任务启动，取消待执行导航", allocator);
      event.AddMember("command_type", rapidjson::Value(command_type.c_str(), allocator).Move(), allocator);
      publish_status_update("navigation_pending_cancelled", event);
      send_acknowledgment("navigation_pending_cancelled", "error", "已有其他导航任务启动，取消待执行导航");
      return;
    }

    const std::string reason = build_start_block_reason();
    if (!reason.empty()) {
      return;
    }

    const std::string request_json = route_.pending_navigation_request_json;
    route_.pending_navigation_active = false;
    route_.pending_navigation_reason.clear();
    route_.pending_navigation_request_json.clear();

    send_acknowledgment("navigation_pending", "success", "启动条件已就绪，开始执行待启动导航");
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = request_json;
    on_navigation_request(msg);
  }

  bool is_navigation_active() const
  {
    return route_.current_state == NavigationState::Planning ||
           route_.current_state == NavigationState::Executing ||
           route_.current_state == NavigationState::Paused;
  }

  RouteRuntimeConfig config_;
  RouteTaskRuntimeState route_;
  WaypointCacheState waypoints_;
  MapRuntimeState map_;
  LocalizationRuntimeState localization_;
  RobotRuntimeState robot_;
  EnvironmentRuntimeState environment_;

  NavigationGatekeeper gatekeeper_;
  RouteTaskStateMachine route_task_state_machine_;
  ObstacleWaitManager obstacle_wait_manager_;
  ObstacleWaitDecision last_obstacle_wait_decision_;
  Nav2ActionController nav2_controller_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_ack_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr navigation_goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_request_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr waypoints_data_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr roi_obstacle_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_status_sub_;
  rclcpp::Subscription<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr behavior_tree_log_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_status_sub_;

  std::set<std::string> nav2_blockage_suppression_nodes_;
  bool is_blocked_by_obstacle_{false};
  bool block_reported_{false};
  double block_start_time_{0.0};
  double block_recovery_candidate_start_time_{0.0};
  std::string block_recovery_candidate_source_;
  double distance_remaining_{std::numeric_limits<double>::infinity()};
  double route_goal_reject_retry_deadline_{0.0};
  int route_goal_reject_retry_count_{0};
  std::string route_goal_reject_retry_segment_id_;
  double route_task_last_feedback_time_{0.0};
  std::vector<std::string> route_goal_execution_waypoint_ids_;
  std::string last_route_task_failure_message_;
  std::string last_route_task_failure_code_;
  rclcpp::TimerBase::SharedPtr route_goal_retry_timer_;

  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr navigation_check_timer_;
  rclcpp::TimerBase::SharedPtr pending_navigation_timer_;
  rclcpp::TimerBase::SharedPtr obstacle_wait_timer_;
};

}  // namespace humanoid_route_runtime

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<humanoid_route_runtime::NavigationStateManagerNode>());
  rclcpp::shutdown();
  return 0;
}
