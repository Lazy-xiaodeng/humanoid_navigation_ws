/*
 * obstacle_wait_manager.cpp
 *
 * 文件用途：
 * 1. 实现障碍等待恢复判断：local costmap 前方窗口 clear + ROI 连续 clear。
 * 2. 将高频 costmap 处理限制在等待恢复阶段，降低正常导航期间的 CPU 消耗。
 * 3. 只维护恢复判断所需的计数和时间，不直接修改路线任务状态。
 *
 * 代码块顺序：
 * 1. 局部数学工具：四元数转 yaw、数值夹紧。
 * 2. 生命周期接口：active / enter / clear。
 * 3. 周期判断：先检查等待态，再检查 costmap、ROI、帧数和持续时间。
 * 4. 前方窗口扫描：把 costmap cell 转到机器人前向坐标系，只统计窗口内高代价格子。
 */

#include "humanoid_route_runtime/obstacle_wait_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace humanoid_route_runtime
{
namespace
{

double yaw_from_xyzw(const std::array<double, 4> & q)
{
  const double x = q[0];
  const double y = q[1];
  const double z = q[2];
  const double w = q[3];
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

int clamp_cell(const int value, const int low, const int high)
{
  return std::max(low, std::min(value, high));
}

}  // namespace

bool ObstacleWaitManager::active() const
{
  return active_;
}

void ObstacleWaitManager::enter(const double now, const RouteRuntimeConfig & config)
{
  active_ = true;
  started_at_ = now;
  clear_started_at_ = 0.0;
  clear_confirm_count_ = 0;
  if (last_resume_at_ > 0.0 && now - last_resume_at_ <= config.obstacle_false_resume_window_sec) {
    recent_false_resume_count_ += 1;
  } else {
    recent_false_resume_count_ = 0;
  }
}

void ObstacleWaitManager::clear()
{
  if (active_) {
    last_resume_at_ = now_seconds();
  }
  active_ = false;
  started_at_ = 0.0;
  clear_started_at_ = 0.0;
  clear_confirm_count_ = 0;
}

double ObstacleWaitManager::started_at() const
{
  return started_at_;
}

double ObstacleWaitManager::current_required_clear_duration(const RouteRuntimeConfig & config) const
{
  if (recent_false_resume_count_ > 0) {
    return std::max(
      config.obstacle_clear_required_duration_sec,
      config.obstacle_clear_required_duration_after_false_resume_sec);
  }
  return config.obstacle_clear_required_duration_sec;
}

int ObstacleWaitManager::recent_false_resume_count() const
{
  return recent_false_resume_count_;
}

ObstacleConfirmationDecision ObstacleWaitManager::confirm_obstacle(
  const double now,
  const RouteRuntimeConfig & config,
  const EnvironmentRuntimeState & environment) const
{
  ObstacleConfirmationDecision confirmation;
  if (!config.obstacle_block_require_sensor_confirmation) {
    confirmation.confirmed = true;
    confirmation.reason = "legacy_speed_only";
    return confirmation;
  }

  if (config.obstacle_resume_use_roi && environment.latest_roi_stamp > 0.0 &&
    now - environment.latest_roi_stamp <= config.obstacle_block_sensor_timeout_sec)
  {
    confirmation.roi_available = true;
    confirmation.roi_blocked = environment.latest_roi_has_obstacle;
  }

  const bool costmap_fresh = environment.latest_costmap_stamp > 0.0 &&
    now - environment.latest_costmap_stamp <= config.obstacle_block_sensor_timeout_sec;
  const bool costmap_valid = environment.has_current_pose &&
    environment.latest_costmap_width > 0U && environment.latest_costmap_height > 0U &&
    environment.latest_costmap_resolution > 0.0 && !environment.latest_costmap_data.empty();
  if (costmap_fresh && costmap_valid) {
    confirmation.costmap_available = true;
    ObstacleWaitDecision costmap_decision;
    confirmation.costmap_blocked = analyze_front_window(config, environment, costmap_decision);
    confirmation.costmap_occupied_cells = costmap_decision.occupied_cells;
    confirmation.costmap_sample_cells = costmap_decision.sample_cells;
    confirmation.costmap_max_cost = costmap_decision.max_cost;
  }

  confirmation.confirmed = confirmation.roi_blocked || confirmation.costmap_blocked;
  if (confirmation.roi_blocked && confirmation.costmap_blocked) {
    confirmation.reason = "confirmed_by_roi_and_costmap";
  } else if (confirmation.roi_blocked) {
    confirmation.reason = "confirmed_by_roi";
  } else if (confirmation.costmap_blocked) {
    confirmation.reason = "confirmed_by_costmap";
  } else if (!confirmation.roi_available && !confirmation.costmap_available) {
    confirmation.reason = "sensor_unavailable_or_stale";
  }
  return confirmation;
}

ObstacleWaitDecision ObstacleWaitManager::update(
  const double now,
  const RouteRuntimeConfig & config,
  const EnvironmentRuntimeState & environment,
  const NavigationState current_state)
{
  ObstacleWaitDecision decision;
  decision.active = active_;
  decision.wait_duration_sec = active_ && started_at_ > 0.0 ? now - started_at_ : 0.0;

  if (!active_) {
    decision.reason = "not_active";
    return decision;
  }
  if (current_state != NavigationState::Paused) {
    clear_confirm_count_ = 0;
    clear_started_at_ = 0.0;
    decision.reason = "not_paused";
    return decision;
  }

  const double max_costmap_age = std::max(1.0, 2.0 / std::max(0.1, config.obstacle_clear_check_rate_hz));
  if (environment.latest_costmap_stamp <= 0.0 || now - environment.latest_costmap_stamp > max_costmap_age) {
    clear_confirm_count_ = 0;
    clear_started_at_ = 0.0;
    decision.reason = "costmap_stale";
    return decision;
  }

  const bool front_blocked = analyze_front_window(config, environment, decision);
  decision.front_blocked = front_blocked;
  if (front_blocked) {
    clear_confirm_count_ = 0;
    clear_started_at_ = 0.0;
    decision.reason = "front_blocked";
    return decision;
  }

  if (!is_roi_clear(now, config, environment, decision)) {
    clear_confirm_count_ = 0;
    clear_started_at_ = 0.0;
    decision.reason = "roi_not_clear";
    return decision;
  }

  clear_confirm_count_ += 1;
  decision.clear_confirmed_frames = clear_confirm_count_;
  if (clear_confirm_count_ < config.obstacle_clear_required_frames) {
    decision.reason = "clear_frames_not_enough";
    return decision;
  }

  if (decision.wait_duration_sec < config.obstacle_min_wait_before_resume_sec) {
    decision.reason = "min_wait_not_reached";
    return decision;
  }

  if (clear_started_at_ <= 0.0) {
    clear_started_at_ = now;
    decision.reason = "clear_duration_started";
    return decision;
  }

  decision.clear_duration_sec = now - clear_started_at_;
  if (decision.clear_duration_sec < current_required_clear_duration(config)) {
    decision.reason = "clear_duration_not_enough";
    return decision;
  }

  decision.ready_to_resume = true;
  decision.reason = "ready_to_resume";
  return decision;
}

bool ObstacleWaitManager::analyze_front_window(
  const RouteRuntimeConfig & config,
  const EnvironmentRuntimeState & environment,
  ObstacleWaitDecision & decision) const
{
  const int width = static_cast<int>(environment.latest_costmap_width);
  const int height = static_cast<int>(environment.latest_costmap_height);
  const double resolution = environment.latest_costmap_resolution;
  if (width <= 0 || height <= 0 || resolution <= 0.0 || environment.latest_costmap_data.empty()) {
    decision.reason = "costmap_unavailable";
    return true;
  }

  const double robot_x = environment.current_position[0];
  const double robot_y = environment.current_position[1];
  const double robot_yaw = yaw_from_xyzw(environment.current_orientation);
  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);
  const double origin_x = environment.latest_costmap_origin_x;
  const double origin_y = environment.latest_costmap_origin_y;

  int cell_x_begin = 0;
  int cell_x_end = width;
  int cell_y_begin = 0;
  int cell_y_end = height;

  if (config.obstacle_costmap_window_bounded_scan) {
    const double corners[4][2] = {
      {config.obstacle_clear_front_min_x_m, -config.obstacle_clear_half_width_m},
      {config.obstacle_clear_front_min_x_m, config.obstacle_clear_half_width_m},
      {config.obstacle_clear_front_max_x_m, -config.obstacle_clear_half_width_m},
      {config.obstacle_clear_front_max_x_m, config.obstacle_clear_half_width_m},
    };
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (const auto & corner : corners) {
      const double world_x = robot_x + cos_yaw * corner[0] - sin_yaw * corner[1];
      const double world_y = robot_y + sin_yaw * corner[0] + cos_yaw * corner[1];
      min_x = std::min(min_x, world_x);
      max_x = std::max(max_x, world_x);
      min_y = std::min(min_y, world_y);
      max_y = std::max(max_y, world_y);
    }
    cell_x_begin = clamp_cell(static_cast<int>(std::floor((min_x - origin_x) / resolution)) - 1, 0, width);
    cell_x_end = clamp_cell(static_cast<int>(std::ceil((max_x - origin_x) / resolution)) + 2, 0, width);
    cell_y_begin = clamp_cell(static_cast<int>(std::floor((min_y - origin_y) / resolution)) - 1, 0, height);
    cell_y_end = clamp_cell(static_cast<int>(std::ceil((max_y - origin_y) / resolution)) + 2, 0, height);
  }

  decision.candidate_cells =
    std::max(0, cell_x_end - cell_x_begin) * std::max(0, cell_y_end - cell_y_begin);

  int occupied_cells = 0;
  int sample_cells = 0;
  int max_cost = 0;
  for (int cell_y = cell_y_begin; cell_y < cell_y_end; ++cell_y) {
    const double world_y = origin_y + (static_cast<double>(cell_y) + 0.5) * resolution;
    for (int cell_x = cell_x_begin; cell_x < cell_x_end; ++cell_x) {
      const double world_x = origin_x + (static_cast<double>(cell_x) + 0.5) * resolution;
      const double dx = world_x - robot_x;
      const double dy = world_y - robot_y;
      const double forward_x = cos_yaw * dx + sin_yaw * dy;
      const double lateral_y = -sin_yaw * dx + cos_yaw * dy;
      if (forward_x < config.obstacle_clear_front_min_x_m ||
        forward_x > config.obstacle_clear_front_max_x_m ||
        std::abs(lateral_y) > config.obstacle_clear_half_width_m)
      {
        continue;
      }
      sample_cells += 1;
      const int index = cell_y * width + cell_x;
      if (index < 0 || index >= static_cast<int>(environment.latest_costmap_data.size())) {
        continue;
      }
      const int cost = static_cast<int>(environment.latest_costmap_data[static_cast<std::size_t>(index)]);
      max_cost = std::max(max_cost, cost);
      if (cost >= config.obstacle_clear_cost_threshold) {
        occupied_cells += 1;
      }
    }
  }

  decision.occupied_cells = occupied_cells;
  decision.sample_cells = sample_cells;
  decision.max_cost = max_cost;
  return occupied_cells > 0;
}

bool ObstacleWaitManager::is_roi_clear(
  const double now,
  const RouteRuntimeConfig & config,
  const EnvironmentRuntimeState & environment,
  ObstacleWaitDecision & decision) const
{
  decision.roi_clear_confirmed_frames = environment.roi_clear_confirm_count;
  if (!config.obstacle_resume_use_roi) {
    decision.roi_clear = true;
    return true;
  }

  const bool fresh =
    environment.latest_roi_stamp > 0.0 && now - environment.latest_roi_stamp <= config.obstacle_roi_timeout_sec;
  if (!fresh) {
    // ROI 超时时沿用原逻辑降级到 costmap，避免 ROI 节点掉线导致永远无法恢复。
    decision.roi_clear = true;
    return true;
  }
  if (environment.latest_roi_has_obstacle) {
    decision.roi_clear = false;
    return false;
  }
  decision.roi_clear = environment.roi_clear_confirm_count >= config.obstacle_roi_required_clear_frames;
  return decision.roi_clear;
}

}  // namespace humanoid_route_runtime
