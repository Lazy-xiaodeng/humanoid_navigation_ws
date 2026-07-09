/*
 * walk_velocity_controller.cpp
 *
 * 文件用途：
 * 1. 实现 /cmd_vel 缓存、定频发送决策、速度限幅和零速度退出逻辑。
 * 2. 上游：Nav2/controller_server 发布的 /cmd_vel。
 * 3. 下游：robot_protocol 和 robot_ws_client 发送机器人本体行走速度命令。
 * 4. 本模块只做纯逻辑决策，不直接发送 WebSocket，真实发送仍受 walk_velocity_send_enable 控制。
 */

#include "humanoid_robot_gateway_runtime/walk_velocity_controller.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <utility>

#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace humanoid_robot_gateway_runtime
{

namespace
{

constexpr double kSpeedEpsilon = 1e-3;

double clamp_unit(const double value)
{
  return std::max(-1.0, std::min(value, 1.0));
}

double round3(const double value)
{
  return std::round(value * 1000.0) / 1000.0;
}

std::string velocity_to_json(const WalkVelocity & velocity)
{
  rapidjson::Document document;
  document.SetObject();
  auto & allocator = document.GetAllocator();
  document.AddMember("x", velocity.x, allocator);
  document.AddMember("y", velocity.y, allocator);
  document.AddMember("yaw", velocity.yaw, allocator);

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  return buffer.GetString();
}

std::string uppercase(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::toupper(ch));
  });
  return value;
}

bool is_walk_state(const std::string & robot_state)
{
  return uppercase(robot_state) == "WALK";
}

}  // namespace

WalkVelocityController::WalkVelocityController(RobotGatewayConfig config)
: config_(std::move(config))
{
  if (config_.walk_velocity_send_rate_hz <= 0.0) {
    config_.walk_velocity_send_rate_hz = 50.0;
  }
}

std::string WalkVelocityController::name() const { return "walk_velocity_controller"; }

bool WalkVelocityController::on_cmd_vel(
  const double linear_x,
  const double linear_y,
  const double angular_z)
{
  target_velocity_ = WalkVelocity{linear_x, linear_y, angular_z};
  const bool should_start = has_speed(target_velocity_) && !timer_running_;
  if (should_start) {
    timer_running_ = true;
  }
  return should_start;
}

WalkVelocityTickDecision WalkVelocityController::evaluate_tick(
  const std::string & robot_state,
  const bool is_executing_motion)
{
  WalkVelocityTickDecision decision;
  decision.timer_running = timer_running_;

  if (!timer_running_) {
    decision.reason = "timer_not_running";
    return decision;
  }

  if (is_executing_motion) {
    decision.blocked_by_motion = true;
    decision.reason = "motion_executing";
    return decision;
  }

  if (has_speed(target_velocity_)) {
    if (!is_walk_state(robot_state)) {
      decision.blocked_by_robot_state = true;
      decision.reason = "robot_not_walk";
      return decision;
    }

    const auto safe_velocity = clamp_and_round(target_velocity_);
    decision.should_send = config_.walk_velocity_send_enable;
    decision.title = "request_set_walk_vel_sync";
    decision.data_json = velocity_to_json(safe_velocity);
    decision.reason = config_.walk_velocity_send_enable ? "send_velocity" : "send_disabled";
    return decision;
  }

  if (is_walk_state(robot_state)) {
    decision.should_send = config_.walk_velocity_send_enable;
    decision.title = "request_set_walk_vel_sync";
    decision.data_json = velocity_to_json(WalkVelocity{});
    decision.reason = config_.walk_velocity_send_enable ? "send_zero_and_stop" : "send_disabled_zero_and_stop";
  } else {
    decision.reason = "zero_velocity_stop_without_send";
  }

  timer_running_ = false;
  decision.timer_running = false;
  decision.should_stop_timer = true;
  return decision;
}

bool WalkVelocityController::timer_running() const
{
  return timer_running_;
}

WalkVelocity WalkVelocityController::target_velocity() const
{
  return target_velocity_;
}

double WalkVelocityController::send_interval_sec() const
{
  return 1.0 / config_.walk_velocity_send_rate_hz;
}

bool WalkVelocityController::has_speed(const WalkVelocity & velocity)
{
  return std::abs(velocity.x) > kSpeedEpsilon ||
         std::abs(velocity.y) > kSpeedEpsilon ||
         std::abs(velocity.yaw) > kSpeedEpsilon;
}

WalkVelocity WalkVelocityController::clamp_and_round(const WalkVelocity & velocity)
{
  return WalkVelocity{
    round3(clamp_unit(velocity.x)),
    round3(clamp_unit(velocity.y)),
    round3(clamp_unit(velocity.yaw))};
}

}  // namespace humanoid_robot_gateway_runtime
