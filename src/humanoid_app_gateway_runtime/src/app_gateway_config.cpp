/*
 * app_gateway_config.cpp
 *
 * 文件用途：
 * 1. 实现 APP 网关和数据整合节点参数声明与读取。
 * 2. 让 YAML、默认值和 C++ 运行态配置保持集中一致。
 * 3. 上游：app_gateway.yaml、data_integration.yaml 和 ROS 参数服务器。
 * 4. 下游：app_gateway_node、data_integration_node 以及各运行模块。
 */

#include "humanoid_app_gateway_runtime/app_gateway_config.hpp"

namespace humanoid_app_gateway_runtime
{

void declare_app_gateway_parameters(rclcpp::Node & node, const AppGatewayConfig & defaults)
{
  node.declare_parameter<bool>("websocket_server_enable", defaults.websocket_server_enable);
  node.declare_parameter<std::string>("websocket_host", defaults.websocket_host);
  node.declare_parameter<int>("websocket_port", defaults.websocket_port);
  node.declare_parameter<double>("client_health_check_interval_sec", defaults.client_health_check_interval_sec);
  node.declare_parameter<double>("status_report_interval_sec", defaults.status_report_interval_sec);
  node.declare_parameter<bool>("send_initial_snapshot_on_connect", defaults.send_initial_snapshot_on_connect);
  node.declare_parameter<std::string>("data_requests_topic", defaults.data_requests_topic);
  node.declare_parameter<std::string>("data_subscriptions_topic", defaults.data_subscriptions_topic);
  node.declare_parameter<std::string>(
    "integration_data_responses_topic", defaults.integration_data_responses_topic);
  node.declare_parameter<std::string>("integration_push_messages_topic", defaults.integration_push_messages_topic);
  node.declare_parameter<std::string>(
    "integration_subscription_responses_topic", defaults.integration_subscription_responses_topic);
  node.declare_parameter<std::string>("app_waypoint_command_topic", defaults.app_waypoint_command_topic);
  node.declare_parameter<std::string>("app_navigation_command_topic", defaults.app_navigation_command_topic);
  node.declare_parameter<std::string>("app_map_command_topic", defaults.app_map_command_topic);
  node.declare_parameter<std::string>("app_robot_control_topic", defaults.app_robot_control_topic);
  node.declare_parameter<std::string>("app_system_command_topic", defaults.app_system_command_topic);
  node.declare_parameter<std::string>("facial_raw_command_topic", defaults.facial_raw_command_topic);
  node.declare_parameter<std::string>("initial_pose_topic", defaults.initial_pose_topic);
}

AppGatewayConfig load_app_gateway_config(rclcpp::Node & node, const AppGatewayConfig & defaults)
{
  AppGatewayConfig config = defaults;
  node.get_parameter("websocket_server_enable", config.websocket_server_enable);
  node.get_parameter("websocket_host", config.websocket_host);
  node.get_parameter("websocket_port", config.websocket_port);
  node.get_parameter("client_health_check_interval_sec", config.client_health_check_interval_sec);
  node.get_parameter("status_report_interval_sec", config.status_report_interval_sec);
  node.get_parameter("send_initial_snapshot_on_connect", config.send_initial_snapshot_on_connect);
  node.get_parameter("data_requests_topic", config.data_requests_topic);
  node.get_parameter("data_subscriptions_topic", config.data_subscriptions_topic);
  node.get_parameter("integration_data_responses_topic", config.integration_data_responses_topic);
  node.get_parameter("integration_push_messages_topic", config.integration_push_messages_topic);
  node.get_parameter("integration_subscription_responses_topic", config.integration_subscription_responses_topic);
  node.get_parameter("app_waypoint_command_topic", config.app_waypoint_command_topic);
  node.get_parameter("app_navigation_command_topic", config.app_navigation_command_topic);
  node.get_parameter("app_map_command_topic", config.app_map_command_topic);
  node.get_parameter("app_robot_control_topic", config.app_robot_control_topic);
  node.get_parameter("app_system_command_topic", config.app_system_command_topic);
  node.get_parameter("facial_raw_command_topic", config.facial_raw_command_topic);
  node.get_parameter("initial_pose_topic", config.initial_pose_topic);
  return config;
}

void declare_data_integration_parameters(rclcpp::Node & node, const DataIntegrationConfig & defaults)
{
  node.declare_parameter<bool>("data_integration_enable", defaults.data_integration_enable);
  node.declare_parameter<double>("push_update_rate_hz", defaults.push_update_rate_hz);
  node.declare_parameter<double>("cleanup_interval_sec", defaults.cleanup_interval_sec);
  node.declare_parameter<double>("monitor_interval_sec", defaults.monitor_interval_sec);
  node.declare_parameter<bool>("enable_imu_input", defaults.enable_imu_input);
  node.declare_parameter<double>("robot_pose_ttl_sec", defaults.robot_pose_ttl_sec);
  node.declare_parameter<double>("robot_speed_ttl_sec", defaults.robot_speed_ttl_sec);
  node.declare_parameter<double>("odom_raw_ttl_sec", defaults.odom_raw_ttl_sec);
  node.declare_parameter<double>("navigation_status_ttl_sec", defaults.navigation_status_ttl_sec);
  node.declare_parameter<double>("navigation_path_ttl_sec", defaults.navigation_path_ttl_sec);
  node.declare_parameter<double>("system_status_ttl_sec", defaults.system_status_ttl_sec);
  node.declare_parameter<double>("action_result_ttl_sec", defaults.action_result_ttl_sec);
  node.declare_parameter<double>("map_status_ttl_sec", defaults.map_status_ttl_sec);
  node.declare_parameter<double>("map_response_ttl_sec", defaults.map_response_ttl_sec);
  node.declare_parameter<double>("waypoints_data_ttl_sec", defaults.waypoints_data_ttl_sec);
  node.declare_parameter<double>("system_exception_ttl_sec", defaults.system_exception_ttl_sec);
  node.declare_parameter<double>("sensor_data_ttl_sec", defaults.sensor_data_ttl_sec);
  node.declare_parameter<std::string>("robot_status_raw_topic", defaults.robot_status_raw_topic);
  node.declare_parameter<std::string>("robot_realpose_topic", defaults.robot_realpose_topic);
  node.declare_parameter<std::string>("odom_topic", defaults.odom_topic);
  node.declare_parameter<std::string>("imu_topic", defaults.imu_topic);
  node.declare_parameter<std::string>("navigation_path_topic", defaults.navigation_path_topic);
  node.declare_parameter<std::string>("navigation_status_topic", defaults.navigation_status_topic);
  node.declare_parameter<std::string>(
    "navigation_acknowledgments_topic", defaults.navigation_acknowledgments_topic);
  node.declare_parameter<std::string>("map_response_topic", defaults.map_response_topic);
  node.declare_parameter<std::string>("map_status_topic", defaults.map_status_topic);
  node.declare_parameter<std::string>("waypoints_data_topic", defaults.waypoints_data_topic);
  node.declare_parameter<std::string>(
    "localization_recovery_status_topic", defaults.localization_recovery_status_topic);
  node.declare_parameter<std::string>("robot_action_result_topic", defaults.robot_action_result_topic);
  node.declare_parameter<std::string>("gesture_list_updated_topic", defaults.gesture_list_updated_topic);
  node.declare_parameter<std::string>("gesture_list_yaml_path", defaults.gesture_list_yaml_path);
  node.declare_parameter<std::string>(
    "facial_gesture_list_yaml_path", defaults.facial_gesture_list_yaml_path);
  node.declare_parameter<std::string>("data_requests_topic", defaults.data_requests_topic);
  node.declare_parameter<std::string>("data_subscriptions_topic", defaults.data_subscriptions_topic);
  node.declare_parameter<std::string>(
    "integration_data_responses_topic", defaults.integration_data_responses_topic);
  node.declare_parameter<std::string>("integration_push_messages_topic", defaults.integration_push_messages_topic);
  node.declare_parameter<std::string>(
    "integration_subscription_responses_topic", defaults.integration_subscription_responses_topic);
}

DataIntegrationConfig load_data_integration_config(rclcpp::Node & node, const DataIntegrationConfig & defaults)
{
  DataIntegrationConfig config = defaults;
  node.get_parameter("data_integration_enable", config.data_integration_enable);
  node.get_parameter("push_update_rate_hz", config.push_update_rate_hz);
  node.get_parameter("cleanup_interval_sec", config.cleanup_interval_sec);
  node.get_parameter("monitor_interval_sec", config.monitor_interval_sec);
  node.get_parameter("enable_imu_input", config.enable_imu_input);
  node.get_parameter("robot_pose_ttl_sec", config.robot_pose_ttl_sec);
  node.get_parameter("robot_speed_ttl_sec", config.robot_speed_ttl_sec);
  node.get_parameter("odom_raw_ttl_sec", config.odom_raw_ttl_sec);
  node.get_parameter("navigation_status_ttl_sec", config.navigation_status_ttl_sec);
  node.get_parameter("navigation_path_ttl_sec", config.navigation_path_ttl_sec);
  node.get_parameter("system_status_ttl_sec", config.system_status_ttl_sec);
  node.get_parameter("action_result_ttl_sec", config.action_result_ttl_sec);
  node.get_parameter("map_status_ttl_sec", config.map_status_ttl_sec);
  node.get_parameter("map_response_ttl_sec", config.map_response_ttl_sec);
  node.get_parameter("waypoints_data_ttl_sec", config.waypoints_data_ttl_sec);
  node.get_parameter("system_exception_ttl_sec", config.system_exception_ttl_sec);
  node.get_parameter("sensor_data_ttl_sec", config.sensor_data_ttl_sec);
  node.get_parameter("robot_status_raw_topic", config.robot_status_raw_topic);
  node.get_parameter("robot_realpose_topic", config.robot_realpose_topic);
  node.get_parameter("odom_topic", config.odom_topic);
  node.get_parameter("imu_topic", config.imu_topic);
  node.get_parameter("navigation_path_topic", config.navigation_path_topic);
  node.get_parameter("navigation_status_topic", config.navigation_status_topic);
  node.get_parameter("navigation_acknowledgments_topic", config.navigation_acknowledgments_topic);
  node.get_parameter("map_response_topic", config.map_response_topic);
  node.get_parameter("map_status_topic", config.map_status_topic);
  node.get_parameter("waypoints_data_topic", config.waypoints_data_topic);
  node.get_parameter("localization_recovery_status_topic", config.localization_recovery_status_topic);
  node.get_parameter("robot_action_result_topic", config.robot_action_result_topic);
  node.get_parameter("gesture_list_updated_topic", config.gesture_list_updated_topic);
  node.get_parameter("gesture_list_yaml_path", config.gesture_list_yaml_path);
  node.get_parameter("facial_gesture_list_yaml_path", config.facial_gesture_list_yaml_path);
  node.get_parameter("data_requests_topic", config.data_requests_topic);
  node.get_parameter("data_subscriptions_topic", config.data_subscriptions_topic);
  node.get_parameter("integration_data_responses_topic", config.integration_data_responses_topic);
  node.get_parameter("integration_push_messages_topic", config.integration_push_messages_topic);
  node.get_parameter("integration_subscription_responses_topic", config.integration_subscription_responses_topic);
  return config;
}

}  // namespace humanoid_app_gateway_runtime
