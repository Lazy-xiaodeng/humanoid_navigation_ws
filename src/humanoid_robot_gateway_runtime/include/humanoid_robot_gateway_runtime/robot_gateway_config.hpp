/*
 * robot_gateway_config.hpp
 *
 * 文件用途：
 * 1. 负责声明和读取机器人本体网关 ROS 参数。
 * 2. 保证 YAML 默认值与 C++ 默认值集中管理。
 * 3. 后续接入真实机器人 WebSocket 前，所有危险开关都必须在这里可见。
 * 4. 上游：robot_gateway.yaml 和 launch 参数。
 * 5. 下游：robot_gateway_node 的运行态配置。
 */

#pragma once

#include "humanoid_robot_gateway_runtime/robot_gateway_types.hpp"
#include "rclcpp/rclcpp.hpp"

namespace humanoid_robot_gateway_runtime
{

void declare_robot_gateway_parameters(rclcpp::Node & node, const RobotGatewayConfig & defaults);
RobotGatewayConfig load_robot_gateway_config(rclcpp::Node & node, const RobotGatewayConfig & defaults);

}  // namespace humanoid_robot_gateway_runtime
