/*
 * app_gateway_config.hpp
 *
 * 文件用途：
 * 1. 负责声明和读取 APP 网关、数据整合节点的 ROS 参数。
 * 2. 保证 C++ 默认值与 YAML 默认值有统一入口。
 * 3. 避免在 Node 外壳中堆积大量 declare/get_parameter 代码。
 * 4. 上游：launch 加载的 YAML 参数。
 * 5. 下游：app_gateway_node 和 data_integration_node 的运行态配置。
 */

#pragma once

#include "humanoid_app_gateway_runtime/app_gateway_types.hpp"
#include "rclcpp/rclcpp.hpp"

namespace humanoid_app_gateway_runtime
{

void declare_app_gateway_parameters(rclcpp::Node & node, const AppGatewayConfig & defaults);
AppGatewayConfig load_app_gateway_config(rclcpp::Node & node, const AppGatewayConfig & defaults);

void declare_data_integration_parameters(rclcpp::Node & node, const DataIntegrationConfig & defaults);
DataIntegrationConfig load_data_integration_config(rclcpp::Node & node, const DataIntegrationConfig & defaults);

}  // namespace humanoid_app_gateway_runtime
