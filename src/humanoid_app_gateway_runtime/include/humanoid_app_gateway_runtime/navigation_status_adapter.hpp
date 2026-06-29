/*
 * navigation_status_adapter.hpp
 *
 * 文件用途：
 * 1. 负责导航状态、导航 ack、路线事件和异常事件到 APP 协议的转换。
 * 2. 对齐现有导航状态、导航 ack 和导航异常事件转换语义。
 * 3. 本模块必须保持字段兼容，避免导航页按钮、任务详情和事件日志错乱。
 * 4. 上游：humanoid_route_runtime / navigation_state_manager 发布的 /navigation/status 和 /navigation/acknowledgments。
 * 5. 下游：data_store、/integration/push_messages 和 APP 导航页任务详情。
 */

#pragma once

#include <string>

namespace humanoid_app_gateway_runtime
{

struct NavigationStatusBuildResult
{
  bool ok{false};
  std::string error;
  std::string json;
};

class NavigationStatusAdapter
{
public:
  std::string name() const;

  // 保持线上协议 should_push_navigation_status_immediately()。
  bool should_push_navigation_status_immediately(const std::string & event_type) const;

  // 保持线上协议 navigation_ack_callback() 中 push_msg["data"] 的业务数据构造。
  NavigationStatusBuildResult build_navigation_command_result_data(
    const std::string & ack_json,
    double timestamp_sec) const;
};

}  // namespace humanoid_app_gateway_runtime
