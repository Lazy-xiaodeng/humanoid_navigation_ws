/*
 * app_protocol.cpp
 *
 * 文件用途：
 * 1. 实现 APP 请求消息和订阅消息的基础协议校验。
 * 2. 对齐现有数据请求和订阅请求的字段、消息类型和协议版本兼容语义。
 * 3. 上游：APP / 导航页 WebSocket JSON 消息。
 * 4. 下游：business_command_router、integration_forwarder 和 data request 转发链路。
 */

#include "humanoid_app_gateway_runtime/app_protocol.hpp"

#include <array>
#include <string>

#include "rapidjson/document.h"

namespace humanoid_app_gateway_runtime
{

std::string AppProtocol::name() const { return "app_protocol"; }

namespace
{

constexpr const char * kProtocolVersion = "2.0";

const rapidjson::Value * find_member(const rapidjson::Value & object, const char * key)
{
  if (!object.IsObject()) {
    return nullptr;
  }
  const auto it = object.FindMember(key);
  return it == object.MemberEnd() ? nullptr : &it->value;
}

std::string value_to_string(const rapidjson::Value * value)
{
  if (value == nullptr || value->IsNull()) {
    return "";
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
  return "";
}

ProtocolValidationResult parse_message(
  const std::string & message_json,
  rapidjson::Document & document)
{
  ProtocolValidationResult result;
  document.Parse(message_json.c_str());
  if (document.HasParseError() || !document.IsObject()) {
    result.error = "invalid_json";
    return result;
  }
  result.valid = true;
  return result;
}

}  // namespace

ProtocolValidationResult AppProtocol::validate_request_message(
  const std::string & message_json) const
{
  rapidjson::Document document;
  auto result = parse_message(message_json, document);
  if (!result.valid) {
    return result;
  }

  constexpr std::array<const char *, 5> required_fields{
    "protocol_version", "message_id", "message_type", "data_type", "source"};
  for (const auto * field : required_fields) {
    if (find_member(document, field) == nullptr) {
      result.valid = false;
      result.error = std::string("missing_required_field:") + field;
      return result;
    }
  }

  const std::string message_type = value_to_string(find_member(document, "message_type"));
  if (message_type != "request" && message_type != "subscription") {
    result.valid = false;
    result.error = "unsupported_message_type:" + message_type;
    return result;
  }

  const std::string protocol_version = value_to_string(find_member(document, "protocol_version"));
  if (protocol_version != kProtocolVersion) {
    result.warning = "protocol_version_mismatch:" + protocol_version;
  }

  return result;
}

ProtocolValidationResult AppProtocol::validate_subscription_message(
  const std::string & message_json) const
{
  rapidjson::Document document;
  auto result = parse_message(message_json, document);
  if (!result.valid) {
    return result;
  }

  result = validate_request_message(message_json);
  if (!result.valid) {
    return result;
  }

  const auto * data = find_member(document, "data");
  if (data == nullptr) {
    result.valid = false;
    result.error = "missing_data";
    return result;
  }
  if (!data->IsObject()) {
    result.valid = false;
    result.error = "invalid_data";
    return result;
  }

  if (find_member(*data, "action") == nullptr || find_member(*data, "data_types") == nullptr) {
    result.valid = false;
    result.error = "missing_action_or_data_types";
    return result;
  }

  const std::string action = value_to_string(find_member(*data, "action"));
  if (action != "subscribe" && action != "unsubscribe") {
    result.valid = false;
    result.error = "invalid_subscription_action:" + action;
    return result;
  }

  return result;
}

}  // namespace humanoid_app_gateway_runtime
