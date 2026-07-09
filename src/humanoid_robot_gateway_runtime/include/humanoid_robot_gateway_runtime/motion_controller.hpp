/*
 * motion_controller.hpp
 *
 * 文件用途：
 * 1. 负责 APP 上半身动作执行、Menu/Walk 切换、动作完成等待和超时处理。
 * 2. 对齐现有上半身动作入口、忙碌拒绝、动作超时和任务超时语义。
 * 3. 本模块涉及机器人模式切换，必须保留严格开关和动作结果回报。
 * 4. 上游：/app/robot_control 中的动作执行命令。
 * 5. 下游：robot_protocol、robot_ws_client 和 /robot/action_result。
 */

#pragma once

#include <map>
#include <string>

#include "humanoid_robot_gateway_runtime/robot_gateway_types.hpp"

namespace humanoid_robot_gateway_runtime
{

struct MotionCommandParseResult
{
  bool ok{false};
  bool should_start_motion{false};
  std::string reason;
  std::string motion_name;
  std::string client_id;
  std::string command_timestamp_json;
};

struct MotionStartDecision
{
  bool accepted{false};
  bool should_publish_result{false};
  std::string reason;
  std::string motion_name;
  std::string client_id;
  std::string command_timestamp_json;
  std::string status;
  std::string result_code;
  std::string message;
  bool walk_ready{false};
  double motion_timeout_sec{0.0};
  double task_timeout_sec{0.0};
};

class MotionController
{
public:
  explicit MotionController(RobotGatewayConfig config = RobotGatewayConfig{});

  std::string name() const;

  // 保持线上协议 robot_control_callback()：只处理 execute_gesture，并从 parameters.gesture_id 取动作名。
  MotionCommandParseResult parse_robot_control_command(
    const std::string & command_json,
    double fallback_timestamp_sec) const;

  // 保持线上协议 _run_motion_task() 的入口门控：忙碌时拒绝，否则计算动作/任务超时。
  MotionStartDecision evaluate_start_request(
    const MotionCommandParseResult & command,
    bool is_executing_motion,
    const std::string & robot_state) const;

  void set_motion_expected_duration(const std::string & motion_name, double duration_sec);
  void clear_motion_expected_durations();
  double get_motion_completion_timeout(const std::string & motion_name) const;
  double get_motion_task_timeout(const std::string & motion_name) const;

private:
  RobotGatewayConfig config_;
  std::map<std::string, double> motion_expected_durations_;
};

}  // namespace humanoid_robot_gateway_runtime
