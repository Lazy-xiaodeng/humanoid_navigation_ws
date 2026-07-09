/*
 * action_result_builder.cpp
 *
 * 文件用途：
 * 1. 实现 /robot/action_result JSON 组包逻辑。
 * 2. 对齐现有动作结果 publish_action_result 的字段语义。
 * 3. 本模块不执行动作，只负责动作结果协议结构。
 * 4. 上游：motion_controller 的动作执行状态。
 * 5. 下游：/robot/action_result、data_integration_node 和 APP 事件日志。
 */

#include "humanoid_robot_gateway_runtime/action_result_builder.hpp"

#include <algorithm>
#include <cmath>

#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace humanoid_robot_gateway_runtime
{

std::string ActionResultBuilder::name() const { return "action_result_builder"; }

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

bool parse_json_value(
  const std::string & json,
  rapidjson::Document & document,
  const bool null_when_empty = true)
{
  if (json.empty()) {
    if (null_when_empty) {
      document.SetNull();
    } else {
      document.SetObject();
    }
    return true;
  }
  document.Parse(json.c_str());
  return !document.HasParseError();
}

double round3(const double value)
{
  return std::round(value * 1000.0) / 1000.0;
}

std::string document_to_json(const rapidjson::Document & document)
{
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  return buffer.GetString();
}

}  // namespace

ActionResultBuildResult ActionResultBuilder::build_action_result(
  const std::string & motion_name,
  const std::string & status,
  const std::string & result_code,
  const std::string & message,
  const double started_at,
  const double completed_at,
  const std::string & client_id,
  const std::string & command_timestamp_json,
  const std::string & notify_data_json,
  const std::string & robot_state,
  const bool motion_busy,
  const bool walk_ready) const
{
  ActionResultBuildResult result;

  rapidjson::Document command_timestamp;
  if (!parse_json_value(command_timestamp_json, command_timestamp, true)) {
    result.error = "invalid_command_timestamp_json";
    return result;
  }

  rapidjson::Document notify_data;
  if (!parse_json_value(notify_data_json.empty() ? "{}" : notify_data_json, notify_data, false)) {
    result.error = "invalid_notify_data_json";
    return result;
  }
  if (!notify_data.IsObject()) {
    notify_data.SetObject();
  }

  rapidjson::Document document;
  document.SetObject();
  auto & allocator = document.GetAllocator();
  add_string(document, allocator, "event_type", "action_completed");
  add_string(document, allocator, "action_type", "execute_gesture");
  add_string(document, allocator, "gesture_id", motion_name);
  add_string(document, allocator, "action_name", motion_name);
  add_string(document, allocator, "status", status);
  add_string(document, allocator, "result", result_code);
  add_string(document, allocator, "message", message);
  add_string(document, allocator, "client_id", client_id);
  document.AddMember("command_timestamp", rapidjson::Value(command_timestamp, allocator), allocator);
  document.AddMember("started_at", started_at, allocator);
  document.AddMember("completed_at", completed_at, allocator);
  document.AddMember("duration", round3(std::max(0.0, completed_at - started_at)), allocator);
  add_string(document, allocator, "robot_state", robot_state);
  document.AddMember("motion_busy", motion_busy, allocator);
  document.AddMember("control_ready_for_navigation", walk_ready, allocator);
  document.AddMember("notify_data", rapidjson::Value(notify_data, allocator), allocator);
  document.AddMember("timestamp", completed_at, allocator);

  result.ok = true;
  result.json = document_to_json(document);
  return result;
}

}  // namespace humanoid_robot_gateway_runtime
