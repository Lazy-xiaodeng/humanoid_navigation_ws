/*
 * robot_ws_client.cpp
 *
 * 文件用途：
 * 1. 实现机器人本体 WebSocket 连接、重连、发送命令和接收消息分发。
 * 2. 对齐现有机器人 WebSocket 连接、接收循环、命令发送和消息分发核心语义。
 * 3. 本模块不直接发布 ROS topic，只把解析后的事件交给上层 robot_gateway_node。
 * 4. 默认受 robot_ws_enable 控制，不会在未开启时连接真实机器人。
 *
 * 代码块顺序：
 * 1. URL/身份字段/机器人返回码辅助函数。
 * 2. 生命周期：start/stop/connection_loop，负责连接、重连和退出清理。
 * 3. 发送链路：build_request、send_text、send_command、send_command_no_response。
 * 4. 接收链路：handle_incoming_message、身份刷新、guid 响应匹配和 route 分类。
 * 5. 状态查询：accid/sn/identity_source/connected 等线程安全访问。
 */

#include "humanoid_robot_gateway_runtime/robot_ws_client.hpp"

#include <chrono>
#include <algorithm>
#include <cctype>
#include <memory>
#include <sstream>
#include <thread>
#include <utility>

#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>
#include "rapidjson/document.h"

namespace humanoid_robot_gateway_runtime
{

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;

struct RobotWsClient::WsImpl
{
  net::io_context ioc;
  std::unique_ptr<websocket::stream<tcp::socket>> ws;
  std::mutex write_mutex;
};

namespace
{

std::string value_to_string(const rapidjson::Value & value)
{
  if (value.IsString()) {
    return value.GetString();
  }
  if (value.IsInt64()) {
    return std::to_string(value.GetInt64());
  }
  if (value.IsUint64()) {
    return std::to_string(value.GetUint64());
  }
  if (value.IsDouble()) {
    std::ostringstream stream;
    stream << value.GetDouble();
    return stream.str();
  }
  return "";
}

void extract_identity_from_payload(
  const std::string & data_json,
  const RobotProtocol & protocol,
  std::string & accid,
  std::string & sn)
{
  rapidjson::Document data;
  data.Parse(data_json.c_str());
  if (data.HasParseError() || !data.IsObject()) {
    return;
  }

  if (data.HasMember("accid")) {
    accid = protocol.normalize_identity_value(value_to_string(data["accid"]));
  }
  if (accid.empty() && data.HasMember("robot_accid")) {
    accid = protocol.normalize_identity_value(value_to_string(data["robot_accid"]));
  }
  if (data.HasMember("sn")) {
    sn = protocol.normalize_identity_value(value_to_string(data["sn"]));
  }
  if (sn.empty() && data.HasMember("robot_sn")) {
    sn = protocol.normalize_identity_value(value_to_string(data["robot_sn"]));
  }
  if (sn.empty() && data.HasMember("serial_number")) {
    sn = protocol.normalize_identity_value(value_to_string(data["serial_number"]));
  }

  if (!data.HasMember("result") || !data["result"].IsArray()) {
    return;
  }

  const auto & results = data["result"];
  for (rapidjson::SizeType i = 0; i < results.Size(); ++i) {
    const auto & component = results[i];
    if (!component.IsObject() || !component.HasMember("values") || !component["values"].IsArray()) {
      continue;
    }
    const auto & values = component["values"];
    for (rapidjson::SizeType j = 0; j < values.Size(); ++j) {
      const auto & item = values[j];
      if (!item.IsObject() || !item.HasMember("key") || !item.HasMember("value")) {
        continue;
      }
      std::string key = value_to_string(item["key"]);
      std::transform(key.begin(), key.end(), key.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
      });
      const std::string value = protocol.normalize_identity_value(value_to_string(item["value"]));
      if (value.empty()) {
        continue;
      }
      if (accid.empty() && (key == "accid" || key == "robot_accid")) {
        accid = value;
      }
      if (sn.empty() && (key == "sn" || key == "robot_sn" || key == "serial_number" || key == "serial_no")) {
        sn = value;
      }
      if (!accid.empty() && !sn.empty()) {
        return;
      }
    }
  }
}

std::string walk_velocity_result_to_message(const std::string & result)
{
  if (result.empty() || result == "success") {
    return "";
  }
  if (result == "fail_motor") {
    return "行走速度指令失败: 电机错误";
  }
  if (result == "fail_imu") {
    return "行走速度指令失败: IMU 错误";
  }
  if (result == "fail_invalid_cmd") {
    return "行走速度指令失败: 参数错误";
  }
  if (result == "fail_invalid_mode") {
    return "行走速度指令失败: 当前状态不允许执行";
  }
  if (result == "fail_timeout") {
    return "行走速度指令失败: 切换状态超时";
  }
  return "行走速度指令失败: 未知错误 " + result;
}

std::string get_json_string_field(const std::string & data_json, const char * key)
{
  rapidjson::Document data;
  data.Parse(data_json.c_str());
  if (data.HasParseError() || !data.IsObject() || !data.HasMember(key)) {
    return "";
  }
  return value_to_string(data[key]);
}

}  // namespace

RobotWsClient::RobotWsClient(RobotGatewayConfig config)
: config_(std::move(config)), impl_(std::make_unique<WsImpl>())
{
  accid_ = config_.fallback_accid;
  sn_ = config_.fallback_sn;
  identity_source_ = (!accid_.empty() || !sn_.empty()) ? "fallback_config" : "unknown";
}

RobotWsClient::~RobotWsClient()
{
  stop();
}

std::string RobotWsClient::name() const { return "robot_ws_client"; }

RobotWsUrlParts RobotWsClient::parse_ws_url(const std::string & url)
{
  // 只支持 ws://，机器人本体当前协议没有启用 TLS。
  // target 保留 path，便于后续如果机器人服务带路径也能直接连接。
  RobotWsUrlParts result;
  const std::string prefix = "ws://";
  if (url.rfind(prefix, 0) != 0) {
    result.error = "only_ws_scheme_supported";
    return result;
  }

  const std::string without_scheme = url.substr(prefix.size());
  const auto slash_pos = without_scheme.find('/');
  const std::string host_port = slash_pos == std::string::npos
    ? without_scheme
    : without_scheme.substr(0, slash_pos);
  result.target = slash_pos == std::string::npos ? "/" : without_scheme.substr(slash_pos);
  if (host_port.empty()) {
    result.error = "missing_host";
    return result;
  }

  const auto colon_pos = host_port.rfind(':');
  if (colon_pos == std::string::npos) {
    result.host = host_port;
    result.port = "80";
  } else {
    result.host = host_port.substr(0, colon_pos);
    result.port = host_port.substr(colon_pos + 1);
  }

  if (result.host.empty() || result.port.empty()) {
    result.error = "invalid_host_or_port";
    return result;
  }

  result.ok = true;
  return result;
}

bool RobotWsClient::start()
{
  // start 只负责启动连接线程；真正的 TCP/WebSocket 建连在 connection_loop() 中循环执行。
  // 这样断线后可以自动重连，不影响 ROS 节点主线程。
  if (!config_.robot_ws_enable) {
    return false;
  }
  if (running_.exchange(true)) {
    return true;
  }
  thread_ = std::thread([this]() { connection_loop(); });
  return true;
}

void RobotWsClient::stop()
{
  // stop 是 best effort：先标记退出，再关闭 socket 唤醒阻塞读，最后 join 连接线程。
  running_ = false;
  connected_ = false;
  if (impl_ && impl_->ws) {
    try {
      std::lock_guard<std::mutex> lock(impl_->write_mutex);
      impl_->ws->close(websocket::close_code::normal);
    } catch (...) {
      // 关闭阶段只做 best effort，避免析构或节点退出被异常打断。
    }
  }
  if (thread_.joinable()) {
    thread_.join();
  }
}

bool RobotWsClient::connected() const
{
  return connected_;
}

void RobotWsClient::set_message_callback(MessageCallback callback)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  message_callback_ = std::move(callback);
}

void RobotWsClient::set_raw_message_callback(RawMessageCallback callback)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  raw_message_callback_ = std::move(callback);
}

RobotWsSendResult RobotWsClient::send_command(
  const std::string & title,
  const std::string & data_json,
  const double timeout_sec)
{
  // 同步命令：发送前注册 guid，收到相同 guid 的响应后返回。
  // motion/gesture 等需要确认结果的链路使用该函数。
  auto result = build_request(title, data_json);
  if (!result.ok) {
    return result;
  }

  response_waiter_.register_guid(result.guid, now_sec());
  if (!send_text(result.request_json, &result.error)) {
    response_waiter_.remove_guid(result.guid);
    result.ok = false;
    return result;
  }

  const auto timeout_ms = std::chrono::milliseconds(
    static_cast<int64_t>(std::max(0.0, timeout_sec) * 1000.0));
  const auto response = response_waiter_.wait_for_response(result.guid, timeout_ms);
  if (response.has_value() && running_) {
    result.response_json = response.value();
    response_waiter_.remove_guid(result.guid);
    result.ok = true;
    return result;
  }

  response_waiter_.remove_guid(result.guid);
  result.ok = false;
  result.error = "response_timeout";
  return result;
}

RobotWsSendResult RobotWsClient::send_command_no_response(
  const std::string & title,
  const std::string & data_json)
{
  // 无响应命令：只负责组包和写 socket，不等待 guid 返回。
  // 高频行走速度命令使用该路径，避免每帧阻塞等待。
  auto result = build_request(title, data_json);
  if (!result.ok) {
    return result;
  }
  if (!send_text(result.request_json, &result.error)) {
    result.ok = false;
    return result;
  }
  result.ok = true;
  return result;
}

RobotWsIncomingEvent RobotWsClient::handle_incoming_message(const std::string & message_json)
{
  // 接收消息统一入口：
  // 1. RobotProtocol 提取 title/guid/data；
  // 2. 刷新 accid/sn；
  // 3. 尝试命中同步等待中的 guid；
  // 4. 分类成上层 robot_gateway_node 可处理的 route。
  const auto parsed = protocol_.parse_robot_message(message_json);
  if (!parsed.ok) {
    RobotWsIncomingEvent event;
    event.error = parsed.error;
    return event;
  }

  refresh_identity_from_parsed_message(parsed);
  const bool response_matched = !parsed.guid.empty() &&
    response_waiter_.complete_response(parsed.guid, message_json);
  auto event = classify_message(parsed, response_matched);

  MessageCallback message_callback;
  RawMessageCallback raw_callback;
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    message_callback = message_callback_;
    raw_callback = raw_message_callback_;
  }
  if (raw_callback) {
    raw_callback(message_json);
  }
  if (message_callback) {
    message_callback(event);
  }
  return event;
}

void RobotWsClient::update_robot_identity(
  const std::string & accid,
  const std::string & sn,
  const std::string & source)
{
  // 外部可主动写入身份字段；机器人 notify/response 也会在接收链路中持续刷新。
  // identity_source 用于调试当前 accid/sn 来自 fallback、消息字段还是 result/values。
  const std::string clean_accid = protocol_.normalize_identity_value(accid);
  const std::string clean_sn = protocol_.normalize_identity_value(sn);
  std::lock_guard<std::mutex> lock(state_mutex_);
  bool changed = false;
  if (!clean_accid.empty() && clean_accid != accid_) {
    accid_ = clean_accid;
    changed = true;
  }
  if (!clean_sn.empty() && clean_sn != sn_) {
    sn_ = clean_sn;
    changed = true;
  }
  if (changed) {
    identity_source_ = source;
  }
}

std::string RobotWsClient::accid() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return accid_;
}

std::string RobotWsClient::sn() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return sn_;
}

std::string RobotWsClient::identity_source() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return identity_source_;
}

std::string RobotWsClient::generate_guid() const
{
  return boost::uuids::to_string(boost::uuids::random_generator()());
}

long long RobotWsClient::now_ms() const
{
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
}

double RobotWsClient::now_sec() const
{
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

RobotWsSendResult RobotWsClient::build_request(
  const std::string & title,
  const std::string & data_json) const
{
  RobotWsSendResult result;
  result.guid = generate_guid();
  const auto built = protocol_.build_command_message(accid(), title, result.guid, now_ms(), data_json);
  if (!built.ok) {
    result.error = built.error;
    return result;
  }
  result.ok = true;
  result.request_json = built.json;
  return result;
}

bool RobotWsClient::send_text(const std::string & text, std::string * error)
{
  if (!connected_ || !impl_ || !impl_->ws) {
    if (error != nullptr) {
      *error = "websocket_not_connected";
    }
    return false;
  }

  try {
    std::lock_guard<std::mutex> lock(impl_->write_mutex);
    impl_->ws->write(net::buffer(text));
    return true;
  } catch (const std::exception & ex) {
    connected_ = false;
    if (error != nullptr) {
      *error = ex.what();
    }
    return false;
  }
}

void RobotWsClient::connection_loop()
{
  while (running_) {
    const auto url = parse_ws_url(config_.robot_ws_server);
    if (!url.ok) {
      connected_ = false;
      std::this_thread::sleep_for(std::chrono::duration<double>(config_.reconnect_interval_sec));
      continue;
    }

    try {
      tcp::resolver resolver(impl_->ioc);
      impl_->ws = std::make_unique<websocket::stream<tcp::socket>>(impl_->ioc);
      const auto results = resolver.resolve(url.host, url.port);
      net::connect(impl_->ws->next_layer(), results.begin(), results.end());
      impl_->ws->handshake(url.host, url.target);
      connected_ = true;

      while (running_ && connected_) {
        beast::flat_buffer buffer;
        impl_->ws->read(buffer);
        handle_incoming_message(beast::buffers_to_string(buffer.data()));
      }
    } catch (const std::exception &) {
      connected_ = false;
      impl_->ws.reset();
      if (running_) {
        std::this_thread::sleep_for(std::chrono::duration<double>(config_.reconnect_interval_sec));
      }
    }
  }
}

RobotWsIncomingEvent RobotWsClient::classify_message(
  const RobotMessageParseResult & parsed,
  const bool response_matched) const
{
  RobotWsIncomingEvent event;
  event.ok = true;
  event.title = parsed.title;
  event.guid = parsed.guid;
  event.accid = parsed.accid;
  event.sn = parsed.sn;
  event.data_json = parsed.data_json;
  event.response_matched = response_matched;

  if (parsed.title == "response_prepare") {
    event.route = "response_prepare";
  } else if (parsed.title == "notify_robot_info") {
    event.route = "notify_robot_info";
  } else if (parsed.title == "notify_joy_data") {
    event.route = "notify_joy_data";
  } else if (parsed.title == "notify_execute_atomic_motion") {
    event.route = "notify_execute_atomic_motion";
  } else if (parsed.title == "response_set_motion_engine") {
    event.route = "response_set_motion_engine";
  } else if (parsed.title == "response_execute_atomic_motion") {
    event.route = "response_execute_atomic_motion";
  } else if (parsed.title == "response_get_atomic_motion_list") {
    event.route = "response_get_atomic_motion_list";
  } else if (parsed.title == "response_set_walk_vel_sync") {
    event.route = "response_set_walk_vel_sync";
    event.walk_velocity_result = get_json_string_field(parsed.data_json, "result");
    event.walk_velocity_message = walk_velocity_result_to_message(event.walk_velocity_result);
  } else {
    event.route = "unknown";
  }

  return event;
}

void RobotWsClient::refresh_identity_from_parsed_message(const RobotMessageParseResult & parsed)
{
  std::string runtime_accid = parsed.accid;
  std::string runtime_sn = parsed.sn;
  extract_identity_from_payload(parsed.data_json, protocol_, runtime_accid, runtime_sn);
  update_robot_identity(runtime_accid, runtime_sn, parsed.title.empty() ? "runtime_message" : parsed.title);
}

}  // namespace humanoid_robot_gateway_runtime
