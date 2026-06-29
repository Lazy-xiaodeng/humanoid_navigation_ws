/*
 * navigation_gatekeeper.cpp
 *
 * 文件用途：
 * 1. 实现路线任务启动前的安全门控逻辑。
 * 2. 当前门控与外部硬件解耦，只根据已经缓存的状态字段生成可读的中文阻塞原因。
 * 3. 这里不发布 ROS 消息，便于后续直接用单元测试覆盖各种状态组合。
 */

#include "humanoid_route_runtime/navigation_gatekeeper.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>

namespace humanoid_route_runtime
{

bool NavigationGatekeeper::localization_can_use_last_good_tf(
  const RouteRuntimeConfig & config,
  const LocalizationRuntimeState & localization,
  const double now) const
{
  if (!config.localization_allow_start_with_last_good_tf || !localization.has_last_good_tf) {
    return false;
  }
  const double max_age = std::max(0.0, config.localization_last_good_tf_max_age_sec);
  if (max_age <= 0.0) {
    return true;
  }
  const double current_time = now > 0.0 ? now : now_seconds();
  return current_time - localization.last_good_tf_time <= max_age;
}

std::optional<std::string> NavigationGatekeeper::robot_start_block_reason(
  const RouteRuntimeConfig & config,
  const RobotRuntimeState & robot,
  const double now) const
{
  if (!config.require_walk_mode_for_navigation) {
    return std::nullopt;
  }
  if (robot.last_update <= 0.0) {
    return "尚未收到机器人底层状态，暂不启动导航";
  }
  const double age = now - robot.last_update;
  if (age > config.robot_status_timeout) {
    std::ostringstream stream;
    stream << "机器人底层状态超时 " << std::fixed << std::setprecision(1) << age << "s，暂不启动导航";
    return stream.str();
  }
  if (robot.ready_for_navigation) {
    return std::nullopt;
  }
  if (robot.motion_busy) {
    const std::string motion_text = robot.current_motion.empty() ? "" : " (" + robot.current_motion + ")";
    return "机器人正在执行动作" + motion_text + "，尚未回到 Walk，暂不启动导航";
  }
  return "机器人当前状态为 " + robot.control_state + "，尚未回到 Walk，暂不启动导航";
}

std::optional<std::string> NavigationGatekeeper::localization_start_block_reason(
  const RouteRuntimeConfig & config,
  const LocalizationRuntimeState & localization,
  const double now) const
{
  if (localization.healthy) {
    return std::nullopt;
  }
  if (localization.last_status_time <= 0.0) {
    return "尚未收到定位状态 " + config.localization_health_status_topic + "，暂不启动导航";
  }
  const double age = now - localization.last_status_time;
  if (age > config.localization_health_timeout_sec) {
    std::ostringstream stream;
    stream << "定位状态超时 " << std::fixed << std::setprecision(1) << age << "s > " <<
      config.localization_health_timeout_sec << "s，暂不启动导航";
    return stream.str();
  }
  if (localization_can_use_last_good_tf(config, localization, now)) {
    return std::nullopt;
  }
  if (!localization.has_last_good_tf) {
    return "prior-map bridge 尚未接受过 map->odom，暂不启动导航";
  }
  if (!config.localization_allow_start_with_last_good_tf) {
    return "prior-map定位未接受最新结果: " + localization.state + " " + localization.text;
  }

  std::ostringstream stream;
  stream << "last good map->odom 已超限 " << std::fixed << std::setprecision(1) <<
    (now - localization.last_good_tf_time) << "s > " <<
    config.localization_last_good_tf_max_age_sec << "s，暂不启动导航";
  return stream.str();
}

std::string NavigationGatekeeper::start_block_reason(
  const RouteRuntimeConfig & config,
  const RobotRuntimeState & robot,
  const LocalizationRuntimeState & localization,
  const MapRuntimeState & map,
  const double now) const
{
  const auto robot_reason = robot_start_block_reason(config, robot, now);
  if (robot_reason.has_value()) {
    return robot_reason.value();
  }
  const auto localization_reason = localization_start_block_reason(config, localization, now);
  if (localization_reason.has_value()) {
    return localization_reason.value();
  }
  if (map.active_map_id.empty()) {
    return "尚未收到地图状态，暂不启动导航";
  }
  if (map.map_state != "ready") {
    return "地图尚未 ready，暂不启动导航";
  }
  return "";
}

bool NavigationGatekeeper::update_localization_timeout(
  const RouteRuntimeConfig & config,
  LocalizationRuntimeState & localization,
  const double now) const
{
  if (localization.last_status_time <= 0.0 || !localization.healthy) {
    return false;
  }
  if (now - localization.last_status_time <= config.localization_health_timeout_sec) {
    return false;
  }
  localization.healthy = false;
  localization.resume_stable_count = 0;
  return true;
}

}  // namespace humanoid_route_runtime
