/*
 * walk_velocity_controller.hpp
 *
 * 文件用途：
 * 1. 负责 /cmd_vel 缓存、定频发送、超时停和零速度保护。
 * 2. 对齐现有机器人本体网关的速度缓存、定频发送和零速度停车语义。
 * 3. 本模块涉及真实底盘运动；正式导航默认允许发送，离线验证可通过参数关闭。
 * 4. 上游：Nav2/controller_server 发布的 /cmd_vel。
 * 5. 下游：robot_protocol 和 robot_ws_client 发送机器人本体行走速度命令。
 */

#pragma once

#include <string>

#include "humanoid_robot_gateway_runtime/robot_gateway_types.hpp"

namespace humanoid_robot_gateway_runtime
{

struct WalkVelocity
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct WalkVelocityTickDecision
{
  bool timer_running{false};
  bool should_send{false};
  bool should_stop_timer{false};
  bool blocked_by_motion{false};
  bool blocked_by_robot_state{false};
  std::string title;
  std::string data_json;
  std::string reason;
};

class WalkVelocityController
{
public:
  explicit WalkVelocityController(RobotGatewayConfig config = RobotGatewayConfig{});

  std::string name() const;

  // 接收 /cmd_vel 后只更新目标速度缓存；是否真实发送由周期 tick 决策。
  bool on_cmd_vel(double linear_x, double linear_y, double angular_z);

  // 保持线上协议 walk_command_loop() 的单周期判断逻辑。
  WalkVelocityTickDecision evaluate_tick(
    const std::string & robot_state,
    bool is_executing_motion);

  bool timer_running() const;
  WalkVelocity target_velocity() const;
  double send_interval_sec() const;

  static bool has_speed(const WalkVelocity & velocity);
  static WalkVelocity clamp_and_round(const WalkVelocity & velocity);

private:
  RobotGatewayConfig config_;
  WalkVelocity target_velocity_;
  bool timer_running_{false};
};

}  // namespace humanoid_robot_gateway_runtime
