/*
 * navigation_gatekeeper.hpp
 *
 * 文件用途：
 * 1. 负责“能不能开始导航”的启动门控判断。
 * 2. 门控只读取机器人底层状态、定位健康状态和地图状态，不发送 goal、不修改路线任务。
 * 3. 这样后续调试“按钮为什么置灰/开始为什么被拒绝”时，可以优先看这个模块。
 *
 * 代码块顺序：
 * 1. 定位 last-good TF 兜底判断。
 * 2. 机器人 Walk/忙碌/超时判断。
 * 3. 定位健康判断。
 * 4. 地图 ready 判断和综合启动阻塞原因。
 * 5. 周期性定位超时刷新。
 */

#pragma once

#include <optional>
#include <string>

#include "humanoid_route_runtime/route_runtime_types.hpp"

namespace humanoid_route_runtime
{

class NavigationGatekeeper
{
public:
  bool localization_can_use_last_good_tf(
    const RouteRuntimeConfig & config,
    const LocalizationRuntimeState & localization,
    double now = -1.0) const;

  std::optional<std::string> robot_start_block_reason(
    const RouteRuntimeConfig & config,
    const RobotRuntimeState & robot,
    double now) const;

  std::optional<std::string> localization_start_block_reason(
    const RouteRuntimeConfig & config,
    const LocalizationRuntimeState & localization,
    double now) const;

  std::string start_block_reason(
    const RouteRuntimeConfig & config,
    const RobotRuntimeState & robot,
    const LocalizationRuntimeState & localization,
    const MapRuntimeState & map,
    double now) const;

  bool update_localization_timeout(
    const RouteRuntimeConfig & config,
    LocalizationRuntimeState & localization,
    double now) const;
};

}  // namespace humanoid_route_runtime
