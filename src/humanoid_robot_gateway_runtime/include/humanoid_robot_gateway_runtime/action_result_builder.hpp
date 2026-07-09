/*
 * action_result_builder.hpp
 *
 * 文件用途：
 * 1. 统一构建 /robot/action_result 输出 JSON。
 * 2. 保持动作成功、失败、超时、拒绝等字段与线上协议兼容。
 * 3. 本模块不执行动作，只负责动作结果协议结构。
 * 4. 上游：motion_controller 的动作执行状态。
 * 5. 下游：/robot/action_result、data_integration_node 和 APP 事件日志。
 */

#pragma once

#include <string>

namespace humanoid_robot_gateway_runtime
{

struct ActionResultBuildResult
{
  bool ok{false};
  std::string error;
  std::string json;
};

class ActionResultBuilder
{
public:
  std::string name() const;

  // 保持线上协议 publish_action_result() 的 /robot/action_result JSON 字段。
  ActionResultBuildResult build_action_result(
    const std::string & motion_name,
    const std::string & status,
    const std::string & result_code,
    const std::string & message,
    double started_at,
    double completed_at,
    const std::string & client_id,
    const std::string & command_timestamp_json,
    const std::string & notify_data_json,
    const std::string & robot_state,
    bool motion_busy,
    bool walk_ready) const;
};

}  // namespace humanoid_robot_gateway_runtime
