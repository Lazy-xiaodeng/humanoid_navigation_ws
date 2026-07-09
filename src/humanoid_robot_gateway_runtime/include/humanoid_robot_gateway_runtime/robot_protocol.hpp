/*
 * robot_protocol.hpp
 *
 * 文件用途：
 * 1. 负责机器人本体协议 title/guid/data 的组包和解析边界。
 * 2. 对齐现有机器人本体协议的 guid、title、timestamp、data payload 构造逻辑。
 * 3. 本模块只处理协议字段，不处理线程等待和 ROS 发布。
 * 4. 上游：motion_controller、walk_velocity_controller、gesture_sync 发起的机器人命令。
 * 5. 下游：robot_ws_client 发送到机器人本体 WebSocket。
 */

#pragma once

#include <string>

namespace humanoid_robot_gateway_runtime
{

struct RobotProtocolBuildResult
{
  bool ok{false};
  std::string error;
  std::string json;
};

struct RobotMessageParseResult
{
  bool ok{false};
  std::string error;
  std::string accid;
  std::string sn;
  std::string title;
  std::string guid;
  std::string data_json;
};

class RobotProtocol
{
public:
  std::string name() const;

  // 保持线上协议 send_command()/send_command_no_response() 的底层消息结构。
  RobotProtocolBuildResult build_command_message(
    const std::string & accid,
    const std::string & title,
    const std::string & guid,
    long long timestamp_ms,
    const std::string & data_json) const;

  // 解析机器人本体返回消息，提取 title/guid/data/身份字段。
  RobotMessageParseResult parse_robot_message(const std::string & message_json) const;

  // 保持线上协议 _normalize_identity_value()。
  std::string normalize_identity_value(const std::string & value) const;
};

}  // namespace humanoid_robot_gateway_runtime
