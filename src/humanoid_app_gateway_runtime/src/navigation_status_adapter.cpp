/*
 * navigation_status_adapter.cpp
 *
 * 文件用途：
 * 1. 实现导航事件立即推送判断和导航 ack 到 navigation_command_result 业务数据转换。
 * 2. 对齐现有导航事件立即推送判断和导航 ack 转换基础语义。
 * 3. 上游：/navigation/status、/navigation/acknowledgments。
 * 4. 下游：data_store、/integration/push_messages 和 APP 导航页任务详情。
 */

#include "humanoid_app_gateway_runtime/navigation_status_adapter.hpp"

#include <set>
#include <string>

#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace humanoid_app_gateway_runtime
{

std::string NavigationStatusAdapter::name() const { return "navigation_status_adapter"; }

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

}  // namespace

bool NavigationStatusAdapter::should_push_navigation_status_immediately(
  const std::string & event_type) const
{
  static const std::set<std::string> immediate_events{
    "waypoint_started",
    "waypoint_reached",
    "navigation_completed",
    "navigation_stopped",
    "navigation_failed",
    "navigation_cancelled",
    "navigation_paused",
    "navigation_resumed",
    "navigation_obstacle_blocked",
    "navigation_pending_cancelled",
    "navigation_retry_failed_waypoint",
    "navigation_skip_failed_waypoint",
    "navigation_aborted",
    "navigation_localization_recovery_requested",
    "navigation_localization_recovery_started",
    "navigation_localization_recovery_progress",
    "navigation_localization_recovery_failed",
    "navigation_localization_manual_override",
    "navigation_localization_recovered",
    "navigation_localization_resume_waiting",
    "navigation_localization_resume_failed",
    "navigation_command_result",
    "broadcast_requested",
    "waypoint_passed",
    "jump_updated",
    "final_align_started",
    "final_align_completed",
    "task_waypoint_completed",
    "route_task_completed"};
  return immediate_events.find(event_type) != immediate_events.end();
}

NavigationStatusBuildResult NavigationStatusAdapter::build_navigation_command_result_data(
  const std::string & ack_json,
  const double timestamp_sec) const
{
  NavigationStatusBuildResult result;
  rapidjson::Document ack;
  ack.Parse(ack_json.c_str());
  if (ack.HasParseError() || !ack.IsObject()) {
    result.error = "invalid_ack_json";
    return result;
  }

  rapidjson::Document document;
  document.SetObject();
  auto & allocator = document.GetAllocator();

  rapidjson::Value event_data(ack, allocator);
  const std::string ack_type = value_to_string(find_member(ack, "ack_type"));
  const std::string status = value_to_string(find_member(ack, "status"));
  const std::string message = value_to_string(find_member(ack, "message"));

  if (find_member(event_data, "event_type") == nullptr) {
    add_string(event_data, allocator, "event_type", "navigation_command_result");
  }
  if (find_member(event_data, "ack_type") == nullptr) {
    add_string(event_data, allocator, "ack_type", ack_type);
  }
  if (find_member(event_data, "command_type") == nullptr) {
    add_string(event_data, allocator, "command_type", ack_type);
  }
  if (find_member(event_data, "status") == nullptr) {
    add_string(event_data, allocator, "status", status);
  }
  if (find_member(event_data, "message") == nullptr) {
    add_string(event_data, allocator, "message", message);
  }
  if (find_member(event_data, "timestamp") == nullptr) {
    event_data.AddMember("timestamp", timestamp_sec, allocator);
  }

  add_string(document, allocator, "event_type", "navigation_command_result");
  document.AddMember("event_data", rapidjson::Value(event_data, allocator), allocator);
  for (auto it = event_data.MemberBegin(); it != event_data.MemberEnd(); ++it) {
    if (document.FindMember(it->name) == document.MemberEnd()) {
      document.AddMember(
        rapidjson::Value(it->name, allocator).Move(),
        rapidjson::Value(it->value, allocator).Move(),
        allocator);
    }
  }

  result.ok = true;
  result.json = document_to_json(document);
  return result;
}

}  // namespace humanoid_app_gateway_runtime
