/*
 * robot_status_parser.hpp
 *
 * 文件用途：
 * 1. 解析机器人本体 notify_robot_info、状态、身份 accid/sn 和 joy 数据。
 * 2. 对齐现有机器人本体状态发布、notify_robot_info 和 notify_joy_data 解析语义。
 * 3. 输出必须保持 /robot_status_raw 字段兼容，否则数据整合和导航门控会受影响。
 * 4. 上游：robot_ws_client 接收到的机器人 notify/status/joy 消息。
 * 5. 下游：/robot_status_raw、/joy_raw、data_integration_node 和 navigation_state_manager。
 */

#pragma once

#include <string>
#include <vector>

namespace humanoid_robot_gateway_runtime
{

struct RobotStatusParseResult
{
  bool ok{false};
  std::string error;
  std::string robot_status_raw_json;
};

struct JoyParseResult
{
  bool ok{false};
  std::string error;
  std::vector<double> axes;
  std::vector<int> buttons;
  std::string frame_id{"joy_controller"};
};

class RobotStatusParser
{
public:
  std::string name() const;

  // 从机器人 notify_robot_info 的 data 字段生成 /robot_status_raw JSON。
  RobotStatusParseResult parse_notify_robot_info(
    const std::string & info_raw_json,
    const std::string & accid,
    const std::string & sn,
    const std::string & identity_source,
    const std::string & robot_state,
    bool is_executing_motion,
    const std::string & current_motion_name,
    double current_time_sec,
    double previous_msg_time_sec,
    const std::vector<double> & previous_intervals_ms) const;

  // 从 notify_joy_data 的 data 字段解析 Joy 所需 axes/buttons。
  JoyParseResult parse_notify_joy_data(const std::string & joy_data_json) const;
};

}  // namespace humanoid_robot_gateway_runtime
