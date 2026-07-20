/*
 * robot_gateway_config.cpp
 *
 * 文件用途：
 * 1. 实现机器人本体网关参数声明与读取。
 * 2. 让真实 WebSocket、速度发送和动作执行开关保持集中可审查。
 * 3. 上游：robot_gateway.yaml 和 ROS 参数服务器。
 * 4. 下游：robot_gateway_node、robot_ws_client、walk_velocity_controller、motion_controller。
 */

#include "humanoid_robot_gateway_runtime/robot_gateway_config.hpp"

namespace humanoid_robot_gateway_runtime
{

void declare_robot_gateway_parameters(rclcpp::Node & node, const RobotGatewayConfig & defaults)
{
  node.declare_parameter<bool>("robot_ws_enable", defaults.robot_ws_enable);
  node.declare_parameter<std::string>("robot_ws_server", defaults.robot_ws_server);
  node.declare_parameter<double>("reconnect_interval_sec", defaults.reconnect_interval_sec);
  node.declare_parameter<double>("command_timeout_sec", defaults.command_timeout_sec);
  node.declare_parameter<std::string>("fallback_accid", defaults.fallback_accid);
  node.declare_parameter<std::string>("fallback_sn", defaults.fallback_sn);
  node.declare_parameter<bool>("walk_velocity_send_enable", defaults.walk_velocity_send_enable);
  node.declare_parameter<double>("walk_velocity_send_rate_hz", defaults.walk_velocity_send_rate_hz);
  node.declare_parameter<double>("cmd_vel_timeout_sec", defaults.cmd_vel_timeout_sec);
  node.declare_parameter<bool>("motion_execution_enable", defaults.motion_execution_enable);
  node.declare_parameter<bool>("motion_allow_enter_menu", defaults.motion_allow_enter_menu);
  node.declare_parameter<bool>("motion_allow_return_walk", defaults.motion_allow_return_walk);
  node.declare_parameter<double>("motion_default_timeout_sec", defaults.motion_default_timeout_sec);
  node.declare_parameter<double>("motion_timeout_buffer_sec", defaults.motion_timeout_buffer_sec);
  node.declare_parameter<double>("motion_max_timeout_sec", defaults.motion_max_timeout_sec);
  node.declare_parameter<bool>("gesture_sync_enable", defaults.gesture_sync_enable);
  node.declare_parameter<std::string>("gestures_yaml_path", defaults.gestures_yaml_path);
  node.declare_parameter<double>("gesture_sync_delay_sec", defaults.gesture_sync_delay_sec);
  node.declare_parameter<std::string>("robot_status_raw_topic", defaults.robot_status_raw_topic);
  node.declare_parameter<std::string>("joy_raw_topic", defaults.joy_raw_topic);
  node.declare_parameter<std::string>("robot_action_result_topic", defaults.robot_action_result_topic);
  node.declare_parameter<std::string>("gesture_list_updated_topic", defaults.gesture_list_updated_topic);
  node.declare_parameter<std::string>("cmd_vel_topic", defaults.cmd_vel_topic);
  node.declare_parameter<std::string>("app_robot_control_topic", defaults.app_robot_control_topic);
  node.declare_parameter<std::string>(
    "navigation_action_interlock_service", defaults.navigation_action_interlock_service);
}

RobotGatewayConfig load_robot_gateway_config(rclcpp::Node & node, const RobotGatewayConfig & defaults)
{
  RobotGatewayConfig config = defaults;
  node.get_parameter("robot_ws_enable", config.robot_ws_enable);
  node.get_parameter("robot_ws_server", config.robot_ws_server);
  node.get_parameter("reconnect_interval_sec", config.reconnect_interval_sec);
  node.get_parameter("command_timeout_sec", config.command_timeout_sec);
  node.get_parameter("fallback_accid", config.fallback_accid);
  node.get_parameter("fallback_sn", config.fallback_sn);
  node.get_parameter("walk_velocity_send_enable", config.walk_velocity_send_enable);
  node.get_parameter("walk_velocity_send_rate_hz", config.walk_velocity_send_rate_hz);
  node.get_parameter("cmd_vel_timeout_sec", config.cmd_vel_timeout_sec);
  node.get_parameter("motion_execution_enable", config.motion_execution_enable);
  node.get_parameter("motion_allow_enter_menu", config.motion_allow_enter_menu);
  node.get_parameter("motion_allow_return_walk", config.motion_allow_return_walk);
  node.get_parameter("motion_default_timeout_sec", config.motion_default_timeout_sec);
  node.get_parameter("motion_timeout_buffer_sec", config.motion_timeout_buffer_sec);
  node.get_parameter("motion_max_timeout_sec", config.motion_max_timeout_sec);
  node.get_parameter("gesture_sync_enable", config.gesture_sync_enable);
  node.get_parameter("gestures_yaml_path", config.gestures_yaml_path);
  node.get_parameter("gesture_sync_delay_sec", config.gesture_sync_delay_sec);
  node.get_parameter("robot_status_raw_topic", config.robot_status_raw_topic);
  node.get_parameter("joy_raw_topic", config.joy_raw_topic);
  node.get_parameter("robot_action_result_topic", config.robot_action_result_topic);
  node.get_parameter("gesture_list_updated_topic", config.gesture_list_updated_topic);
  node.get_parameter("cmd_vel_topic", config.cmd_vel_topic);
  node.get_parameter("app_robot_control_topic", config.app_robot_control_topic);
  node.get_parameter(
    "navigation_action_interlock_service", config.navigation_action_interlock_service);
  return config;
}

}  // namespace humanoid_robot_gateway_runtime
