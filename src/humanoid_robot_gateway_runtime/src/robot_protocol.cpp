/*
 * robot_protocol.cpp
 *
 * 文件用途：
 * 1. 实现机器人本体 WebSocket 协议的 title/guid/data 组包和基础解析。
 * 2. 对齐现有 send_command、send_command_no_response、身份字段清洗和消息解析基础协议语义。
 * 3. 本模块只处理 JSON 字段，不处理 WebSocket 连接、等待响应和 ROS 发布。
 * 4. 上游：motion_controller、walk_velocity_controller、gesture_sync 发起的机器人命令。
 * 5. 下游：robot_ws_client 发送到机器人本体 WebSocket。
 */

#include "humanoid_robot_gateway_runtime/robot_protocol.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <string>

#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace humanoid_robot_gateway_runtime
{

std::string RobotProtocol::name() const { return "robot_protocol"; }

namespace
{

const rapidjson::Value * find_member(const rapidjson::Value & object, const char * key)
{
  if (!object.IsObject()) {
    return nullptr;
  }
  const auto it = object.FindMember(key);
  return it == object.MemberEnd() ? nullptr : &it->value;
}

std::string value_to_string(const rapidjson::Value * value, const std::string & fallback = "")
{
  if (value == nullptr || value->IsNull()) {
    return fallback;
  }
  if (value->IsString()) {
    return std::string(value->GetString(), value->GetStringLength());
  }
  if (value->IsBool()) {
    return value->GetBool() ? "True" : "False";
  }
  if (value->IsInt64()) {
    return std::to_string(value->GetInt64());
  }
  if (value->IsUint64()) {
    return std::to_string(value->GetUint64());
  }
  if (value->IsDouble()) {
    return std::to_string(value->GetDouble());
  }
  return fallback;
}

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

std::string document_to_json(const rapidjson::Document & document)
{
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  return buffer.GetString();
}

std::string trim(std::string value)
{
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), [](unsigned char ch) {
    return !std::isspace(ch);
  }));
  value.erase(std::find_if(value.rbegin(), value.rend(), [](unsigned char ch) {
    return !std::isspace(ch);
  }).base(), value.end());
  return value;
}

}  // namespace

RobotProtocolBuildResult RobotProtocol::build_command_message(
  const std::string & accid,
  const std::string & title,
  const std::string & guid,
  const long long timestamp_ms,
  const std::string & data_json) const
{
  RobotProtocolBuildResult result;
  if (normalize_identity_value(accid).empty()) {
    result.error = "missing_accid";
    return result;
  }

  rapidjson::Document data;
  data.Parse(data_json.c_str());
  if (data.HasParseError()) {
    result.error = "invalid_data_json";
    return result;
  }

  rapidjson::Document document;
  document.SetObject();
  auto & allocator = document.GetAllocator();
  add_string(document, allocator, "accid", accid);
  add_string(document, allocator, "title", title);
  document.AddMember("timestamp", static_cast<std::int64_t>(timestamp_ms), allocator);
  add_string(document, allocator, "guid", guid);
  document.AddMember("data", rapidjson::Value(data, allocator), allocator);

  result.ok = true;
  result.json = document_to_json(document);
  return result;
}

RobotMessageParseResult RobotProtocol::parse_robot_message(
  const std::string & message_json) const
{
  RobotMessageParseResult result;
  rapidjson::Document message;
  message.Parse(message_json.c_str());
  if (message.HasParseError() || !message.IsObject()) {
    result.error = "invalid_robot_message_json";
    return result;
  }

  result.accid = normalize_identity_value(value_to_string(find_member(message, "accid")));
  result.sn = normalize_identity_value(value_to_string(find_member(message, "sn")));
  result.title = value_to_string(find_member(message, "title"));
  result.guid = value_to_string(find_member(message, "guid"));

  const auto * data = find_member(message, "data");
  if (data != nullptr) {
    rapidjson::Document data_doc;
    data_doc.CopyFrom(*data, data_doc.GetAllocator());
    result.data_json = document_to_json(data_doc);
  } else {
    result.data_json = "{}";
  }

  result.ok = true;
  return result;
}

std::string RobotProtocol::normalize_identity_value(const std::string & value) const
{
  std::string text = trim(value);
  std::string lower = text;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return (!text.empty() && lower != "none") ? text : "";
}

}  // namespace humanoid_robot_gateway_runtime
