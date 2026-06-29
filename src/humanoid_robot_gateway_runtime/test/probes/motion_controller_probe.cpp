/*
 * motion_controller_probe.cpp
 *
 * 文件用途：
 * 1. 作为开发期动作控制入口验证工具，构造固定 APP 控制命令和动作时长配置。
 * 2. 验证 execute_gesture 解析、缺少 gesture_id 忽略、忙碌拒绝、动作完成超时和任务超时计算。
 * 3. 上游：协议一致性验证脚本。
 * 4. 下游：stdout，供脚本比较动作控制入口逻辑是否与 既有机器人网关协议一致。
 */

#include <iostream>

#include "humanoid_robot_gateway_runtime/motion_controller.hpp"
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

void add_parse(
  rapidjson::Value & array,
  rapidjson::Document::AllocatorType & allocator,
  const char * label,
  const humanoid_robot_gateway_runtime::MotionCommandParseResult & parse)
{
  rapidjson::Value object(rapidjson::kObjectType);
  add_string(object, allocator, "label", label);
  object.AddMember("ok", parse.ok, allocator);
  object.AddMember("should_start_motion", parse.should_start_motion, allocator);
  add_string(object, allocator, "reason", parse.reason);
  add_string(object, allocator, "motion_name", parse.motion_name);
  add_string(object, allocator, "client_id", parse.client_id);
  rapidjson::Document timestamp_doc;
  if (!parse.command_timestamp_json.empty()) {
    timestamp_doc.Parse(parse.command_timestamp_json.c_str());
  } else {
    timestamp_doc.SetNull();
  }
  object.AddMember("command_timestamp", rapidjson::Value(timestamp_doc, allocator), allocator);
  array.PushBack(object, allocator);
}

void add_decision(
  rapidjson::Value & array,
  rapidjson::Document::AllocatorType & allocator,
  const char * label,
  const humanoid_robot_gateway_runtime::MotionStartDecision & decision)
{
  rapidjson::Value object(rapidjson::kObjectType);
  add_string(object, allocator, "label", label);
  object.AddMember("accepted", decision.accepted, allocator);
  object.AddMember("should_publish_result", decision.should_publish_result, allocator);
  add_string(object, allocator, "reason", decision.reason);
  add_string(object, allocator, "motion_name", decision.motion_name);
  add_string(object, allocator, "client_id", decision.client_id);
  add_string(object, allocator, "status", decision.status);
  add_string(object, allocator, "result_code", decision.result_code);
  add_string(object, allocator, "message", decision.message);
  object.AddMember("walk_ready", decision.walk_ready, allocator);
  object.AddMember("motion_timeout_sec", decision.motion_timeout_sec, allocator);
  object.AddMember("task_timeout_sec", decision.task_timeout_sec, allocator);
  array.PushBack(object, allocator);
}

}  // namespace

int main()
{
  humanoid_robot_gateway_runtime::RobotGatewayConfig config;
  config.motion_execution_enable = false;
  config.motion_default_timeout_sec = 25.0;
  config.motion_timeout_buffer_sec = 8.0;
  config.motion_max_timeout_sec = 90.0;

  humanoid_robot_gateway_runtime::MotionController controller(config);
  controller.set_motion_expected_duration("long_motion", 40.0);
  controller.set_motion_expected_duration("very_long_motion", 100.0);

  const auto valid = controller.parse_robot_control_command(
    R"({"command_type":"execute_gesture","client_id":"client-a","timestamp":123.5,"parameters":{"gesture_id":"long_motion","loop":true}})",
    999.0);
  const auto missing_timestamp = controller.parse_robot_control_command(
    R"({"command_type":"execute_gesture","client_id":"client-b","parameters":{"gesture_id":"short_motion"}})",
    999.0);
  const auto missing_gesture = controller.parse_robot_control_command(
    R"({"command_type":"execute_gesture","parameters":{}})",
    999.0);
  const auto unsupported = controller.parse_robot_control_command(
    R"({"command_type":"other","parameters":{"gesture_id":"ignored"}})",
    999.0);

  rapidjson::Document document;
  document.SetObject();
  auto & allocator = document.GetAllocator();

  rapidjson::Value parses(rapidjson::kArrayType);
  add_parse(parses, allocator, "valid", valid);
  add_parse(parses, allocator, "missing_timestamp", missing_timestamp);
  add_parse(parses, allocator, "missing_gesture", missing_gesture);
  add_parse(parses, allocator, "unsupported", unsupported);
  document.AddMember("parses", parses, allocator);

  rapidjson::Value decisions(rapidjson::kArrayType);
  add_decision(decisions, allocator, "accepted_disabled", controller.evaluate_start_request(valid, false, "WALK"));
  add_decision(decisions, allocator, "busy_rejected", controller.evaluate_start_request(valid, true, "WALK"));
  add_decision(decisions, allocator, "ignored_missing_gesture", controller.evaluate_start_request(missing_gesture, false, "WALK"));
  document.AddMember("decisions", decisions, allocator);
  document.AddMember("short_timeout", controller.get_motion_completion_timeout("short_motion"), allocator);
  document.AddMember("long_timeout", controller.get_motion_completion_timeout("long_motion"), allocator);
  document.AddMember("very_long_timeout", controller.get_motion_completion_timeout("very_long_motion"), allocator);

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  std::cout << buffer.GetString() << std::endl;
  return 0;
}
