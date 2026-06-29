/*
 * robot_status_adapter.hpp
 *
 * 文件用途：
 * 1. 负责将 /robot_status_raw 原始 JSON 转换为 APP 使用的 system_status。
 * 2. 这里承接机器人状态协议归一化逻辑，避免额外桥接节点重复转换。
 * 3. 本模块是适合单元测试覆盖的纯数据转换模块。
 * 4. 上游：robot_gateway_node 发布的 /robot_status_raw。
 * 5. 下游：data_store 中的 system_status，以及 navigation_state_manager 的机器人状态参考链路。
 */

#pragma once

#include <string>
#include <vector>

namespace humanoid_app_gateway_runtime
{

struct RobotStatusConversionResult
{
  // 转换是否成功；JSON 格式错误或字段类型严重异常时为 false。
  bool ok{false};

  // 失败时的人类可读错误原因；成功时为空。
  std::string error;

  // 保持线上协议 convert_raw_robot_status() 的中间结构，便于调试和单元验证。
  std::string processed_status_json;

  // 保持线上协议 update_system_status() 后写入 data_storage['system_status'] 的最终结构。
  std::string system_status_json;
};

class RobotStatusAdapter
{
public:
  std::string name() const;

  // 将 /robot_status_raw 原始 JSON 转换成 APP system_status。
  // timestamp_sec 由调用方传入，便于测试时固定时间戳，运行时使用节点当前时间。
  RobotStatusConversionResult convert_raw_to_system_status(
    const std::string & raw_status_json,
    double timestamp_sec) const;
};

}  // namespace humanoid_app_gateway_runtime
