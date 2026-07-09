/*
 * robot_gateway_types.hpp
 *
 * 文件用途：
 * 1. 集中定义机器人本体网关的配置结构和通用运行态字段。
 * 2. 让 WebSocket 客户端、速度控制、动作控制、状态解析等模块共用同一套字段含义。
 * 3. 所有涉及真实运动的开关都集中在这里，后续接入时方便审查安全边界。
 * 4. 上游：robot_gateway.yaml 和 ROS 参数服务器。
 * 5. 下游：robot_gateway_node、robot_ws_client、walk_velocity_controller、motion_controller 等内部模块。
 */

#pragma once

#include <string>

namespace humanoid_robot_gateway_runtime
{

struct RobotGatewayConfig
{
  bool robot_ws_enable{false};
  std::string robot_ws_server{"ws://10.192.1.2:5000"};
  double reconnect_interval_sec{5.0};
  double command_timeout_sec{5.0};
  std::string fallback_accid{""};
  std::string fallback_sn{""};

  bool walk_velocity_send_enable{true};
  double walk_velocity_send_rate_hz{50.0};
  double cmd_vel_timeout_sec{0.5};

  bool motion_execution_enable{true};
  bool motion_allow_enter_menu{true};
  bool motion_allow_return_walk{true};
  double motion_default_timeout_sec{25.0};
  double motion_timeout_buffer_sec{8.0};
  double motion_max_timeout_sec{90.0};

  bool gesture_sync_enable{true};
  std::string gestures_yaml_path{""};
  double gesture_sync_delay_sec{2.0};

  std::string robot_status_raw_topic{"/robot_status_raw"};
  std::string joy_raw_topic{"/joy_raw"};
  std::string robot_action_result_topic{"/robot/action_result"};
  std::string gesture_list_updated_topic{"/system/gesture_list_updated"};
  std::string cmd_vel_topic{"/cmd_vel"};
  std::string app_robot_control_topic{"/app/robot_control"};
};

}  // namespace humanoid_robot_gateway_runtime
