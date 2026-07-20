/*
 * obstacle_wait_manager.hpp
 *
 * 文件用途：
 * 1. 管理“Nav2 执行失败后暂停等待障碍清除”的轻量状态机。
 * 2. 根据 local costmap 前方窗口和 ROI 障碍检测结果，判断是否满足自动恢复条件。
 * 3. 只输出判断结果，不直接发布 ROS 消息、不直接操作 Nav2 goal，确保节点外壳统一处理副作用。
 *
 * 代码块顺序：
 * 1. 结果结构：描述当前等待状态、阻塞原因和恢复条件。
 * 2. 管理器接口：进入等待、清理等待、周期判断。
 * 3. 内部工具：前方窗口扫描、ROI fresh/clear 判断、恢复时间计算。
 */

#pragma once

#include <string>

#include "humanoid_route_runtime/route_runtime_types.hpp"

namespace humanoid_route_runtime
{

struct ObstacleWaitDecision
{
  bool active{false};
  bool ready_to_resume{false};
  bool front_blocked{false};
  bool roi_clear{true};
  std::string reason;
  double wait_duration_sec{0.0};
  double clear_duration_sec{0.0};
  int clear_confirmed_frames{0};
  int roi_clear_confirmed_frames{0};
  int occupied_cells{0};
  int sample_cells{0};
  int candidate_cells{0};
  int max_cost{0};
};

struct ObstacleConfirmationDecision
{
  bool confirmed{false};
  bool roi_available{false};
  bool roi_blocked{false};
  bool costmap_available{false};
  bool costmap_blocked{false};
  int costmap_occupied_cells{0};
  int costmap_sample_cells{0};
  int costmap_max_cost{0};
  std::string reason{"low_speed_unconfirmed"};
};

class ObstacleWaitManager
{
public:
  bool active() const;

  void enter(double now, const RouteRuntimeConfig & config);

  void clear();

  ObstacleWaitDecision update(
    double now,
    const RouteRuntimeConfig & config,
    const EnvironmentRuntimeState & environment,
    NavigationState current_state);

  double started_at() const;

  double current_required_clear_duration(const RouteRuntimeConfig & config) const;

  int recent_false_resume_count() const;

  ObstacleConfirmationDecision confirm_obstacle(
    double now,
    const RouteRuntimeConfig & config,
    const EnvironmentRuntimeState & environment) const;

private:
  bool analyze_front_window(
    const RouteRuntimeConfig & config,
    const EnvironmentRuntimeState & environment,
    ObstacleWaitDecision & decision) const;

  bool is_roi_clear(
    double now,
    const RouteRuntimeConfig & config,
    const EnvironmentRuntimeState & environment,
    ObstacleWaitDecision & decision) const;

  bool active_{false};
  double started_at_{0.0};
  double clear_started_at_{0.0};
  int clear_confirm_count_{0};
  int recent_false_resume_count_{0};
  double last_resume_at_{0.0};
};

}  // namespace humanoid_route_runtime
