/*
 * app_gateway_types.hpp
 *
 * 文件用途：
 * 1. 集中定义 APP 网关运行层的配置结构和轻量数据类型。
 * 2. 让 APP WebSocket 网关、数据整合节点、协议模块和 adapter 模块共用同一套字段含义。
 * 3. 所有 YAML 可调参数优先落到这里，避免散落在各个 cpp 文件里。
 * 4. 上游：app_gateway.yaml、data_integration.yaml 和 ROS 参数服务器。
 * 5. 下游：app_gateway_node、data_integration_node 以及所有 APP 网关内部模块。
 *
 * 代码块顺序：
 * 1. APP 网关配置：WebSocket、客户端管理和 APP 命令 topic。
 * 2. 数据整合配置：缓存、过期时间、输入输出 topic。
 * 3. 通用数据结构：协议消息、缓存条目和运行态摘要。
 */

#pragma once

#include <string>

namespace humanoid_app_gateway_runtime
{

struct AppGatewayConfig
{
  bool websocket_server_enable{false};
  std::string websocket_host{"0.0.0.0"};
  int websocket_port{8765};
  double client_health_check_interval_sec{30.0};
  double status_report_interval_sec{60.0};
  bool send_initial_snapshot_on_connect{true};

  std::string data_requests_topic{"/websocket/data_requests"};
  std::string data_subscriptions_topic{"/websocket/data_subscriptions"};
  std::string integration_data_responses_topic{"/integration/data_responses"};
  std::string integration_push_messages_topic{"/integration/push_messages"};
  std::string integration_subscription_responses_topic{"/integration/subscription_responses"};

  std::string app_waypoint_command_topic{"/app/waypoint_command"};
  std::string app_navigation_command_topic{"/app/navigation_command"};
  std::string app_map_command_topic{"/app/map_command"};
  std::string app_robot_control_topic{"/app/robot_control"};
  std::string app_system_command_topic{"/app/system_command"};
  std::string facial_raw_command_topic{"/robot/facial_raw_cmd"};
  std::string initial_pose_topic{"/initialpose"};
};

struct DataIntegrationConfig
{
  bool data_integration_enable{false};
  double push_update_rate_hz{10.0};
  double cleanup_interval_sec{1.0};
  double monitor_interval_sec{5.0};
  bool enable_imu_input{false};

  double robot_pose_ttl_sec{5.0};
  double robot_speed_ttl_sec{1.0};
  double odom_raw_ttl_sec{5.0};
  double navigation_status_ttl_sec{10.0};
  double navigation_path_ttl_sec{30.0};
  double system_status_ttl_sec{60.0};
  double action_result_ttl_sec{30.0};
  double map_status_ttl_sec{15.0};
  double map_response_ttl_sec{60.0};
  double waypoints_data_ttl_sec{600.0};
  double system_exception_ttl_sec{120.0};
  double sensor_data_ttl_sec{2.0};

  std::string robot_status_raw_topic{"/robot_status_raw"};
  std::string robot_realpose_topic{"/robot_realpose"};
  std::string odom_topic{"/odom"};
  std::string imu_topic{"/imu"};
  std::string navigation_path_topic{"/plan"};
  std::string navigation_status_topic{"/navigation/status"};
  std::string navigation_acknowledgments_topic{"/navigation/acknowledgments"};
  std::string map_response_topic{"/map/response"};
  std::string map_status_topic{"/map/status"};
  std::string waypoints_data_topic{"/navigation/waypoints_data"};
  std::string localization_recovery_status_topic{"/localization/recovery_status"};
  std::string robot_action_result_topic{"/robot/action_result"};
  std::string gesture_list_updated_topic{"/system/gesture_list_updated"};
  std::string gesture_list_yaml_path{""};
  std::string facial_gesture_list_yaml_path{""};
  std::string data_requests_topic{"/websocket/data_requests"};
  std::string data_subscriptions_topic{"/websocket/data_subscriptions"};
  std::string integration_data_responses_topic{"/integration/data_responses"};
  std::string integration_push_messages_topic{"/integration/push_messages"};
  std::string integration_subscription_responses_topic{"/integration/subscription_responses"};
};

struct RuntimeSummary
{
  std::size_t cached_data_count{0};
  std::size_t active_subscription_count{0};
};

}  // namespace humanoid_app_gateway_runtime
