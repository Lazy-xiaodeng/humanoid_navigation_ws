/*
 * robot_ws_client.hpp
 *
 * 文件用途：
 * 1. 封装机器人本体 WebSocket 连接、重连、发送命令和接收消息。
 * 2. 对齐现有机器人 WebSocket 连接、命令发送、响应等待和接收循环语义。
 * 3. 本模块不直接发布 ROS topic，所有消息通过上层 Node 分发。
 * 4. 上游：机器人本体 WebSocket 服务。
 * 5. 下游：robot_status_parser、response_waiter、gesture_sync、walk_velocity_controller、motion_controller。
 */

#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "humanoid_robot_gateway_runtime/response_waiter.hpp"
#include "humanoid_robot_gateway_runtime/robot_gateway_types.hpp"
#include "humanoid_robot_gateway_runtime/robot_protocol.hpp"

namespace humanoid_robot_gateway_runtime
{

struct RobotWsUrlParts
{
  bool ok{false};
  std::string error;
  std::string host;
  std::string port;
  std::string target{"/"};
};

struct RobotWsSendResult
{
  bool ok{false};
  std::string error;
  std::string guid;
  std::string request_json;
  std::string response_json;
};

struct RobotWsIncomingEvent
{
  bool ok{false};
  std::string error;
  std::string title;
  std::string guid;
  std::string accid;
  std::string sn;
  std::string data_json;
  bool response_matched{false};
  std::string route;
  std::string walk_velocity_result;
  std::string walk_velocity_message;
};

class RobotWsClient
{
public:
  using MessageCallback = std::function<void(const RobotWsIncomingEvent &)>;
  using RawMessageCallback = std::function<void(const std::string &)>;

  explicit RobotWsClient(RobotGatewayConfig config = RobotGatewayConfig{});
  ~RobotWsClient();

  std::string name() const;

  // 解析 ws://host:port/path，供真实连接和离线验证共用。
  static RobotWsUrlParts parse_ws_url(const std::string & url);

  // 真实连接线程；只有 robot_ws_enable=true 时才会尝试连接机器人。
  bool start();
  void stop();
  bool connected() const;

  void set_message_callback(MessageCallback callback);
  void set_raw_message_callback(RawMessageCallback callback);

  // 保持线上协议 send_command()：组包、注册 guid、发送，并在超时时间内等待响应。
  RobotWsSendResult send_command(
    const std::string & title,
    const std::string & data_json,
    double timeout_sec);

  // 保持线上协议 send_command_no_response()：组包并发送，不注册响应等待。
  RobotWsSendResult send_command_no_response(
    const std::string & title,
    const std::string & data_json);

  // 离线可测的消息入口：解析机器人原始 JSON，命中 guid 响应，并输出分发结果。
  RobotWsIncomingEvent handle_incoming_message(const std::string & message_json);

  void update_robot_identity(
    const std::string & accid,
    const std::string & sn,
    const std::string & source);
  std::string accid() const;
  std::string sn() const;
  std::string identity_source() const;

private:
  std::string generate_guid() const;
  long long now_ms() const;
  double now_sec() const;
  RobotWsSendResult build_request(
    const std::string & title,
    const std::string & data_json) const;
  bool send_text(const std::string & text, std::string * error);
  void connection_loop();
  RobotWsIncomingEvent classify_message(
    const RobotMessageParseResult & parsed,
    bool response_matched) const;
  void refresh_identity_from_parsed_message(const RobotMessageParseResult & parsed);

  RobotGatewayConfig config_;
  RobotProtocol protocol_;
  ResponseWaiter response_waiter_;

  mutable std::mutex state_mutex_;
  std::string accid_;
  std::string sn_;
  std::string identity_source_{"unknown"};

  std::atomic_bool running_{false};
  std::atomic_bool connected_{false};
  std::thread thread_;

  mutable std::mutex callback_mutex_;
  MessageCallback message_callback_;
  RawMessageCallback raw_message_callback_;

  struct WsImpl;
  std::unique_ptr<WsImpl> impl_;
};

}  // namespace humanoid_robot_gateway_runtime
