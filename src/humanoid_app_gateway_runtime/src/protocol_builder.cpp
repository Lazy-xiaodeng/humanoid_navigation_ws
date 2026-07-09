/*
 * protocol_builder.cpp
 *
 * 文件用途：
 * 1. 实现 APP 数据响应、订阅响应、错误响应和订阅推送消息的统一组包逻辑。
 * 2. 组包字段对齐现有基础消息、数据响应、订阅响应、错误响应和主动推送业务语义。
 * 3. 上游：data_store、adapter 模块、APP 请求消息和订阅请求消息。
 * 4. 下游：/integration/data_responses、/integration/push_messages、
 *    /integration/subscription_responses，以及后续 APP WebSocket 回推链路。
 */

#include "humanoid_app_gateway_runtime/protocol_builder.hpp"

#include <chrono>
#include <iomanip>
#include <random>
#include <sstream>
#include <string>

#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace humanoid_app_gateway_runtime
{

std::string ProtocolBuilder::protocol_version() const { return "2.0"; }

namespace
{

std::string generate_message_id(const std::string & prefix, const double timestamp_sec)
{
  const auto timestamp_ms = static_cast<long long>(timestamp_sec * 1000.0);
  static thread_local std::mt19937 rng{std::random_device{}()};
  std::uniform_int_distribution<int> dist(0, 0xFFFFFF);

  std::ostringstream suffix;
  suffix << std::hex << std::nouppercase << std::setw(6) << std::setfill('0') << dist(rng);
  return prefix + "_" + std::to_string(timestamp_ms) + "_" + suffix.str().substr(0, 6);
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

const rapidjson::Value * find_member(const rapidjson::Value & object, const char * key)
{
  if (!object.IsObject()) {
    return nullptr;
  }
  const auto it = object.FindMember(key);
  return it == object.MemberEnd() ? nullptr : &it->value;
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

void add_default_metadata(
  rapidjson::Value & message,
  rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value metadata(rapidjson::kObjectType);
  add_string(metadata, allocator, "status", "success");
  add_string(metadata, allocator, "error_code", "");
  add_string(metadata, allocator, "error_message", "");
  add_string(metadata, allocator, "request_id", "");
  metadata.AddMember("data_freshness", 0.0, allocator);
  add_string(metadata, allocator, "qos_level", "standard");
  message.AddMember("metadata", metadata, allocator);
}

std::string document_to_json(const rapidjson::Document & document)
{
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  return buffer.GetString();
}

bool parse_json_object(
  const std::string & json,
  rapidjson::Document & document,
  ProtocolBuildResult & result,
  const char * error_code)
{
  document.Parse(json.c_str());
  if (document.HasParseError() || !document.IsObject()) {
    result.error = error_code;
    return false;
  }
  return true;
}

rapidjson::Value copy_or_empty_object(
  const rapidjson::Value & value,
  rapidjson::Document::AllocatorType & allocator)
{
  if (value.IsObject() || value.IsArray()) {
    return rapidjson::Value(value, allocator);
  }
  rapidjson::Value empty(rapidjson::kObjectType);
  return empty;
}

rapidjson::Value copy_data_types_or_empty_array(
  const rapidjson::Value * value,
  rapidjson::Document::AllocatorType & allocator)
{
  if (value != nullptr && value->IsArray()) {
    return rapidjson::Value(*value, allocator);
  }
  rapidjson::Value empty(rapidjson::kArrayType);
  return empty;
}

}  // namespace

ProtocolBuildResult ProtocolBuilder::create_base_message(
  const std::string & message_type,
  const std::string & data_type,
  const std::string & source,
  const std::string & destination,
  const double timestamp_sec,
  const std::string & message_id) const
{
  ProtocolBuildResult result;

  rapidjson::Document document;
  document.SetObject();
  auto & allocator = document.GetAllocator();

  add_string(document, allocator, "protocol_version", protocol_version());
  add_string(
    document,
    allocator,
    "message_id",
    message_id.empty() ? generate_message_id(message_type, timestamp_sec) : message_id);
  document.AddMember("timestamp", timestamp_sec, allocator);
  add_string(document, allocator, "message_type", message_type);
  add_string(document, allocator, "data_type", data_type);
  add_string(document, allocator, "source", source);
  add_string(document, allocator, "destination", destination);

  rapidjson::Value data(rapidjson::kObjectType);
  document.AddMember("data", data, allocator);
  add_default_metadata(document, allocator);

  result.ok = true;
  result.json = document_to_json(document);
  return result;
}

ProtocolBuildResult ProtocolBuilder::create_data_response(
  const std::string & client_id,
  const std::string & request_id,
  const std::string & data_type,
  const std::string & response_data_json,
  const double timestamp_sec,
  const double last_update_time_sec,
  const std::string & message_id) const
{
  ProtocolBuildResult result;
  rapidjson::Document response_data;
  if (!parse_json_object(response_data_json, response_data, result, "invalid_response_data_json")) {
    return result;
  }

  rapidjson::Document document;
  document.Parse(create_base_message("response", data_type, "data_integration", client_id, timestamp_sec, message_id).json.c_str());
  auto & allocator = document.GetAllocator();

  document["data"] = rapidjson::Value(response_data, allocator);
  document["metadata"]["request_id"].SetString(request_id.c_str(), allocator);
  document["metadata"]["data_freshness"].SetDouble(timestamp_sec - last_update_time_sec);

  result.ok = true;
  result.json = document_to_json(document);
  return result;
}

ProtocolBuildResult ProtocolBuilder::create_subscription_response(
  const std::string & original_message_json,
  const bool success,
  const std::string & message,
  const double timestamp_sec,
  const std::string & message_id) const
{
  ProtocolBuildResult result;
  rapidjson::Document original;
  if (!parse_json_object(original_message_json, original, result, "invalid_original_message_json")) {
    return result;
  }

  const std::string destination = value_to_string(find_member(original, "source"), "unknown");
  rapidjson::Document document;
  document.Parse(
    create_base_message(
      "response", "subscription_manage", "data_integration", destination, timestamp_sec, message_id)
      .json.c_str());
  auto & allocator = document.GetAllocator();

  document["metadata"]["request_id"].SetString(value_to_string(find_member(original, "message_id")).c_str(), allocator);
  document["metadata"]["status"].SetString(success ? "success" : "error", allocator);
  if (!success) {
    document["metadata"]["error_code"].SetString("SUBSCRIPTION_FAILED", allocator);
    document["metadata"]["error_message"].SetString(message.c_str(), allocator);
  }

  const auto * data = find_member(original, "data");
  const std::string action = data != nullptr ? value_to_string(find_member(*data, "action"), "unknown") : "unknown";
  const auto * data_types = data != nullptr ? find_member(*data, "data_types") : nullptr;

  rapidjson::Value response_data(rapidjson::kObjectType);
  add_string(response_data, allocator, "action", action);
  response_data.AddMember("data_types", copy_data_types_or_empty_array(data_types, allocator), allocator);
  add_string(response_data, allocator, "result", success ? "success" : "failed");
  add_string(response_data, allocator, "message", message);
  document["data"] = response_data;

  result.ok = true;
  result.json = document_to_json(document);
  return result;
}

ProtocolBuildResult ProtocolBuilder::create_error_response(
  const std::string & original_message_json,
  const std::string & error_message,
  const std::string & error_code,
  const double timestamp_sec,
  const std::string & message_id) const
{
  ProtocolBuildResult result;
  rapidjson::Document original;
  if (!parse_json_object(original_message_json, original, result, "invalid_original_message_json")) {
    return result;
  }

  const std::string data_type = value_to_string(find_member(original, "data_type"), "unknown");
  const std::string destination = value_to_string(find_member(original, "source"), "unknown");

  rapidjson::Document document;
  document.Parse(
    create_base_message("response", data_type, "data_integration", destination, timestamp_sec, message_id)
      .json.c_str());
  auto & allocator = document.GetAllocator();

  document["metadata"]["status"].SetString("error", allocator);
  document["metadata"]["error_code"].SetString(error_code.c_str(), allocator);
  document["metadata"]["error_message"].SetString(error_message.c_str(), allocator);
  document["metadata"]["request_id"].SetString(value_to_string(find_member(original, "message_id")).c_str(), allocator);

  result.ok = true;
  result.json = document_to_json(document);
  return result;
}

ProtocolBuildResult ProtocolBuilder::create_specific_error(
  const std::string & client_id,
  const std::string & request_id,
  const std::string & data_type,
  const std::string & error_code,
  const std::string & error_message,
  const double timestamp_sec,
  const std::string & message_id) const
{
  ProtocolBuildResult result;
  rapidjson::Document document;
  document.Parse(
    create_base_message("response", data_type, "data_integration", client_id, timestamp_sec, message_id)
      .json.c_str());
  auto & allocator = document.GetAllocator();

  document["metadata"]["status"].SetString("error", allocator);
  document["metadata"]["error_code"].SetString(error_code.c_str(), allocator);
  document["metadata"]["error_message"].SetString(error_message.c_str(), allocator);
  document["metadata"]["request_id"].SetString(request_id.c_str(), allocator);

  result.ok = true;
  result.json = document_to_json(document);
  return result;
}

ProtocolBuildResult ProtocolBuilder::create_push_message(
  const std::string & data_type,
  const std::string & data_json,
  const std::string & client_id,
  const double timestamp_sec,
  const double last_update_time_sec,
  const int subscription_count,
  const std::string & message_id) const
{
  ProtocolBuildResult result;
  rapidjson::Document data;
  if (!parse_json_object(data_json, data, result, "invalid_push_data_json")) {
    return result;
  }

  rapidjson::Document document;
  document.Parse(
    create_base_message("push", data_type, "data_integration", client_id, timestamp_sec, message_id)
      .json.c_str());
  auto & allocator = document.GetAllocator();

  document["timestamp"].SetDouble(timestamp_sec);
  document["data"] = copy_or_empty_object(data, allocator);
  if (const auto * request_id = find_member(document["data"], "request_message_id")) {
    document["metadata"]["request_id"].SetString(value_to_string(request_id).c_str(), allocator);
  } else if (const auto * event_data = find_member(document["data"], "event_data");
    event_data != nullptr && event_data->IsObject())
  {
    // 导航命令结果会以 navigation_status/event_data 形式推送。
    // 将关键回执字段同步到 metadata，APP 可直接按 request_id/status/error_code 做通用匹配。
    if (const auto * nested_request_id = find_member(*event_data, "request_message_id")) {
      document["metadata"]["request_id"].SetString(value_to_string(nested_request_id).c_str(), allocator);
    }
  }
  if (const auto * status = find_member(document["data"], "status")) {
    document["metadata"]["status"].SetString(value_to_string(status, "success").c_str(), allocator);
  } else if (const auto * response_type = find_member(document["data"], "response_type")) {
    document["metadata"]["status"].SetString(value_to_string(response_type, "success").c_str(), allocator);
  } else if (const auto * event_data = find_member(document["data"], "event_data");
    event_data != nullptr && event_data->IsObject())
  {
    if (const auto * nested_status = find_member(*event_data, "status")) {
      document["metadata"]["status"].SetString(value_to_string(nested_status, "success").c_str(), allocator);
    }
  }
  if (const auto * error_code = find_member(document["data"], "error_code")) {
    document["metadata"]["error_code"].SetString(value_to_string(error_code).c_str(), allocator);
  } else if (const auto * event_data = find_member(document["data"], "event_data");
    event_data != nullptr && event_data->IsObject())
  {
    if (const auto * nested_error_code = find_member(*event_data, "error_code")) {
      document["metadata"]["error_code"].SetString(value_to_string(nested_error_code).c_str(), allocator);
    }
  }
  if (const auto * event_data = find_member(document["data"], "event_data");
    event_data != nullptr && event_data->IsObject())
  {
    if (const auto * nested_message = find_member(*event_data, "message")) {
      const std::string status = value_to_string(find_member(document["metadata"], "status"), "success");
      if (status == "error") {
        document["metadata"]["error_message"].SetString(value_to_string(nested_message).c_str(), allocator);
      }
    }
  }
  if (const auto * error_message = find_member(document["data"], "message")) {
    const std::string status = value_to_string(find_member(document["metadata"], "status"), "success");
    if (status == "error") {
      document["metadata"]["error_message"].SetString(value_to_string(error_message).c_str(), allocator);
    }
  }
  add_string(document["metadata"], allocator, "push_reason", "periodic_update");
  document["metadata"]["data_freshness"].SetDouble(timestamp_sec - last_update_time_sec);
  document["metadata"].AddMember("subscription_count", subscription_count, allocator);

  result.ok = true;
  result.json = document_to_json(document);
  return result;
}

}  // namespace humanoid_app_gateway_runtime
