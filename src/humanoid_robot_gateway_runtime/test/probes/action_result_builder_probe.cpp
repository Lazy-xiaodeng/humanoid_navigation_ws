/*
 * action_result_builder_probe.cpp
 *
 * 文件用途：
 * 1. 作为开发期动作结果组包验证工具，构造固定动作执行结果输入。
 * 2. 验证 /robot/action_result 的字段名称、字段值、时间戳透传和耗时计算规则。
 * 3. 上游：协议一致性验证脚本。
 * 4. 下游：stdout，供脚本比较动作结果 JSON 是否与 既有机器人网关协议一致。
 */

#include <iostream>

#include "humanoid_robot_gateway_runtime/action_result_builder.hpp"

int main()
{
  humanoid_robot_gateway_runtime::ActionResultBuilder builder;
  const auto result = builder.build_action_result(
    "wave_hand",
    "completed",
    "success",
    "动作执行完成",
    100.1234,
    102.9876,
    "app-client-01",
    R"("2026-06-28T13:18:00+08:00")",
    R"({"notify_title":"notify_motion","step":2})",
    "WALK",
    false,
    true);

  if (!result.ok) {
    std::cerr << result.error << std::endl;
    return 2;
  }

  std::cout << result.json << std::endl;
  return 0;
}
