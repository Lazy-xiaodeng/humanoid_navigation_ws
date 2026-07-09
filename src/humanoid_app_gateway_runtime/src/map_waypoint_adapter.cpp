/*
 * map_waypoint_adapter.cpp
 *
 * 文件用途：
 * 1. 实现地图响应、地图状态和动态点位全量数据的 APP 推送业务体转换。
 * 2. 对齐现有地图消息和点位全量消息对 data_type、message_type、data、metadata 的处理语义。
 * 3. 上游：/map/status、/map/response、/navigation/waypoints_data。
 * 4. 下游：data_store、APP 初始快照、地图页和导航页点位展示。
 */

#include "humanoid_app_gateway_runtime/map_waypoint_adapter.hpp"

#include <string>

#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace humanoid_app_gateway_runtime
{

std::string MapWaypointAdapter::name() const { return "map_waypoint_adapter"; }

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

std::string error_message_for_metadata(
  const rapidjson::Value * metadata,
  const rapidjson::Value & data,
  const std::string & status)
{
  if (metadata != nullptr) {
    const std::string existing = value_to_string(find_member(*metadata, "error_message"));
    if (!existing.empty()) {
      return existing;
    }
  }
  if (status == "error" || status == "failed" || status == "rejected") {
    return value_to_string(find_member(data, "message"));
  }
  return "";
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

rapidjson::Value normalized_data(
  const rapidjson::Value * data,
  rapidjson::Document::AllocatorType & allocator)
{
  if (data != nullptr && data->IsObject()) {
    return rapidjson::Value(*data, allocator);
  }

  rapidjson::Value wrapped(rapidjson::kObjectType);
  if (data == nullptr) {
    rapidjson::Value empty(rapidjson::kObjectType);
    wrapped.AddMember("raw_data", empty, allocator);
  } else {
    wrapped.AddMember("raw_data", rapidjson::Value(*data, allocator), allocator);
  }
  return wrapped;
}

const rapidjson::Value * object_or_null(const rapidjson::Value * value)
{
  return value != nullptr && value->IsObject() ? value : nullptr;
}

void add_copied_or_default(
  rapidjson::Value & object,
  rapidjson::Document::AllocatorType & allocator,
  const char * key,
  const rapidjson::Value * value,
  const std::string & string_default)
{
  if (value != nullptr) {
    object.AddMember(
      rapidjson::Value(key, allocator).Move(),
      rapidjson::Value(*value, allocator),
      allocator);
  } else {
    add_string(object, allocator, key, string_default);
  }
}

void add_copied_or_int_default(
  rapidjson::Value & object,
  rapidjson::Document::AllocatorType & allocator,
  const char * key,
  const rapidjson::Value * value,
  const int int_default)
{
  if (value != nullptr) {
    object.AddMember(
      rapidjson::Value(key, allocator).Move(),
      rapidjson::Value(*value, allocator),
      allocator);
  } else {
    object.AddMember(rapidjson::Value(key, allocator).Move(), int_default, allocator);
  }
}

}  // namespace

MapWaypointBuildResult MapWaypointAdapter::build_map_message_payload(
  const std::string & payload_json) const
{
  MapWaypointBuildResult result;
  rapidjson::Document payload;
  payload.Parse(payload_json.c_str());
  if (payload.HasParseError() || !payload.IsObject()) {
    result.error = "invalid_map_payload_json";
    return result;
  }

  rapidjson::Document document;
  document.SetObject();
  auto & allocator = document.GetAllocator();

  const std::string data_type = value_to_string(find_member(payload, "data_type"), "map_response");
  const std::string message_type = value_to_string(find_member(payload, "message_type"), "push");
  rapidjson::Value data = normalized_data(find_member(payload, "data"), allocator);
  const auto * metadata = object_or_null(find_member(payload, "metadata"));

  add_string(document, allocator, "message_type", message_type);
  add_string(document, allocator, "data_type", data_type);
  document.AddMember("data", rapidjson::Value(data, allocator), allocator);

  rapidjson::Value out_metadata(rapidjson::kObjectType);
  add_string(out_metadata, allocator, "push_reason", data_type);
  add_copied_or_default(
    out_metadata,
    allocator,
    "status",
    metadata != nullptr ? find_member(*metadata, "status") : find_member(data, "status"),
    value_to_string(find_member(data, "status"), "success"));
  const std::string metadata_status = value_to_string(find_member(out_metadata, "status"), "success");
  add_copied_or_default(
    out_metadata,
    allocator,
    "error_code",
    metadata != nullptr ? find_member(*metadata, "error_code") : find_member(data, "error_code"),
    value_to_string(find_member(data, "error_code"), ""));
  add_copied_or_default(
    out_metadata,
    allocator,
    "error_message",
    metadata != nullptr ? find_member(*metadata, "error_message") : nullptr,
    error_message_for_metadata(metadata, data, metadata_status));
  add_copied_or_default(
    out_metadata,
    allocator,
    "request_id",
    metadata != nullptr ? find_member(*metadata, "request_id") : find_member(data, "request_message_id"),
    value_to_string(find_member(data, "request_message_id"), ""));
  out_metadata.AddMember("data_freshness", 0.0, allocator);
  document.AddMember("metadata", out_metadata, allocator);

  result.ok = true;
  result.json = document_to_json(document);
  return result;
}

MapWaypointBuildResult MapWaypointAdapter::build_waypoints_data_payload(
  const std::string & payload_json) const
{
  MapWaypointBuildResult result;
  rapidjson::Document payload;
  payload.Parse(payload_json.c_str());
  if (payload.HasParseError() || !payload.IsObject()) {
    result.error = "invalid_waypoints_payload_json";
    return result;
  }

  rapidjson::Document document;
  document.SetObject();
  auto & allocator = document.GetAllocator();

  const std::string data_type = value_to_string(find_member(payload, "data_type"), "waypoints_data");
  const std::string message_type = value_to_string(find_member(payload, "message_type"), "push");
  rapidjson::Value data = normalized_data(find_member(payload, "data"), allocator);
  const auto * metadata = object_or_null(find_member(payload, "metadata"));
  const auto * inner_metadata = object_or_null(find_member(data, "metadata"));

  add_string(document, allocator, "message_type", message_type);
  add_string(document, allocator, "data_type", data_type);
  document.AddMember("data", rapidjson::Value(data, allocator), allocator);

  rapidjson::Value out_metadata(rapidjson::kObjectType);
  add_string(out_metadata, allocator, "push_reason", data_type);
  add_copied_or_default(
    out_metadata,
    allocator,
    "status",
    metadata != nullptr ? find_member(*metadata, "status") : find_member(data, "status"),
    value_to_string(find_member(data, "response_type"), value_to_string(find_member(data, "status"), "success")));
  const std::string metadata_status = value_to_string(find_member(out_metadata, "status"), "success");
  add_copied_or_default(
    out_metadata,
    allocator,
    "error_code",
    metadata != nullptr ? find_member(*metadata, "error_code") : find_member(data, "error_code"),
    value_to_string(find_member(data, "error_code"), ""));
  add_copied_or_default(
    out_metadata,
    allocator,
    "error_message",
    metadata != nullptr ? find_member(*metadata, "error_message") : nullptr,
    error_message_for_metadata(metadata, data, metadata_status));
  add_copied_or_default(
    out_metadata,
    allocator,
    "request_id",
    metadata != nullptr ? find_member(*metadata, "request_id") : find_member(data, "request_message_id"),
    value_to_string(find_member(data, "request_message_id"), ""));

  const rapidjson::Value * map_id = metadata != nullptr ? find_member(*metadata, "map_id") : nullptr;
  if (map_id == nullptr && inner_metadata != nullptr) {
    map_id = find_member(*inner_metadata, "map_id");
  }
  if (map_id == nullptr) {
    map_id = find_member(data, "map_id");
  }
  add_copied_or_default(out_metadata, allocator, "map_id", map_id, "");

  const rapidjson::Value * revision =
    metadata != nullptr ? find_member(*metadata, "waypoints_revision") : nullptr;
  if (revision == nullptr && inner_metadata != nullptr) {
    revision = find_member(*inner_metadata, "waypoints_revision");
  }
  if (revision == nullptr) {
    revision = find_member(data, "waypoints_revision");
  }
  add_copied_or_int_default(out_metadata, allocator, "waypoints_revision", revision, 0);
  out_metadata.AddMember("data_freshness", 0.0, allocator);
  document.AddMember("metadata", out_metadata, allocator);

  result.ok = true;
  result.json = document_to_json(document);
  return result;
}

}  // namespace humanoid_app_gateway_runtime
