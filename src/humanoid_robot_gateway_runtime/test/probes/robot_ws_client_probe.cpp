/*
 * robot_ws_client_probe.cpp
 *
 * 文件用途：
 * 1. 作为开发期机器人 WebSocket 客户端验证工具，不连接真实机器人。
 * 2. 验证 ws URL 解析、动态身份学习、机器人消息 title 分发、身份字段刷新和行走速度错误分类。
 * 3. 上游：协议一致性验证脚本。
 * 4. 下游：stdout，供脚本验证 WebSocket 客户端核心分发语义是否符合机器人网关协议。
 */

#include <iostream>

#include "humanoid_robot_gateway_runtime/robot_ws_client.hpp"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace
{

void add_string(
  rapidjson::Value & object,
  rapidjson::Document::AllocatorType & allocator,
  const char * key,
  const std::string & value)
{
  object.AddMember(
    rapidjson::Value(key, allocator).Move(),
    rapidjson::Value(value.c_str(), allocator).Move(),
    allocator);
}

void add_url(
  rapidjson::Value & object,
  rapidjson::Document::AllocatorType & allocator,
  const char * key,
  const humanoid_robot_gateway_runtime::RobotWsUrlParts & url)
{
  rapidjson::Value value(rapidjson::kObjectType);
  value.AddMember("ok", url.ok, allocator);
  add_string(value, allocator, "error", url.error);
  add_string(value, allocator, "host", url.host);
  add_string(value, allocator, "port", url.port);
  add_string(value, allocator, "target", url.target);
  object.AddMember(rapidjson::Value(key, allocator).Move(), value, allocator);
}

void add_event(
  rapidjson::Value & array,
  rapidjson::Document::AllocatorType & allocator,
  const char * label,
  const humanoid_robot_gateway_runtime::RobotWsIncomingEvent & event)
{
  rapidjson::Value object(rapidjson::kObjectType);
  add_string(object, allocator, "label", label);
  object.AddMember("ok", event.ok, allocator);
  add_string(object, allocator, "error", event.error);
  add_string(object, allocator, "title", event.title);
  add_string(object, allocator, "guid", event.guid);
  add_string(object, allocator, "accid", event.accid);
  add_string(object, allocator, "sn", event.sn);
  object.AddMember("response_matched", event.response_matched, allocator);
  add_string(object, allocator, "route", event.route);
  add_string(object, allocator, "walk_velocity_result", event.walk_velocity_result);
  add_string(object, allocator, "walk_velocity_message", event.walk_velocity_message);
  array.PushBack(object, allocator);
}

}  // namespace

int main()
{
  humanoid_robot_gateway_runtime::RobotGatewayConfig config;
  config.robot_ws_enable = false;
  config.robot_ws_server = "ws://10.192.1.2:5000/robot";
  config.fallback_accid = "";
  config.fallback_sn = "";

  humanoid_robot_gateway_runtime::RobotWsClient client(config);

  rapidjson::Document document;
  document.SetObject();
  auto & allocator = document.GetAllocator();

  add_url(document, allocator, "url_with_path", humanoid_robot_gateway_runtime::RobotWsClient::parse_ws_url(config.robot_ws_server));
  add_url(document, allocator, "url_default_port", humanoid_robot_gateway_runtime::RobotWsClient::parse_ws_url("ws://robot.local"));
  add_url(document, allocator, "url_invalid", humanoid_robot_gateway_runtime::RobotWsClient::parse_ws_url("wss://robot.local"));

  add_string(document, allocator, "initial_accid", client.accid());
  add_string(document, allocator, "initial_sn", client.sn());
  add_string(document, allocator, "initial_identity_source", client.identity_source());

  const auto before_identity_speed = client.send_command_no_response(
    "request_set_walk_vel_sync",
    R"({"vel_des":[0.1,0.0,0.0],"step_height":[0.05,0.05]})");
  add_string(document, allocator, "speed_before_identity_error", before_identity_speed.error);

  rapidjson::Value events(rapidjson::kArrayType);
  add_event(
    events,
    allocator,
    "notify_robot_info",
    client.handle_incoming_message(
      R"({"title":"notify_robot_info","guid":"g-info","data":{"result":[{"name":"system","message":"ok","values":[{"key":"accid","value":" HU_RUNTIME "},{"key":"serial_no","value":" SN_RUNTIME "}]}]}})"));
  add_event(
    events,
    allocator,
    "notify_joy_data",
    client.handle_incoming_message(
      R"({"title":"notify_joy_data","guid":"g-joy","data":{"axes":[0,1],"buttons":[true,0]}})"));
  add_event(
    events,
    allocator,
    "walk_vel_fail",
    client.handle_incoming_message(
      R"({"title":"response_set_walk_vel_sync","guid":"g-walk","data":{"result":"fail_invalid_mode"}})"));
  add_event(
    events,
    allocator,
    "unknown",
    client.handle_incoming_message(
      R"({"title":"notify_unknown","guid":"g-unknown","data":{"foo":1}})"));
  add_event(
    events,
    allocator,
    "invalid_json",
    client.handle_incoming_message("{bad-json"));
  document.AddMember("events", events, allocator);

  const auto after_identity_speed = client.send_command_no_response(
    "request_set_walk_vel_sync",
    R"({"vel_des":[0.1,0.0,0.0],"step_height":[0.05,0.05]})");
  rapidjson::Document speed_request;
  speed_request.Parse(after_identity_speed.request_json.c_str());
  add_string(
    document,
    allocator,
    "speed_after_identity_accid",
    speed_request.IsObject() && speed_request.HasMember("accid") && speed_request["accid"].IsString()
    ? speed_request["accid"].GetString()
    : "");
  add_string(document, allocator, "speed_after_identity_error", after_identity_speed.error);

  add_string(document, allocator, "final_accid", client.accid());
  add_string(document, allocator, "final_sn", client.sn());
  add_string(document, allocator, "final_identity_source", client.identity_source());

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  std::cout << buffer.GetString() << std::endl;
  return 0;
}
