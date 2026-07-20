/*
 * motion_controller.cpp
 *
 * 文件用途：
 * 1. 实现 APP 动作命令解析、动作入口门控和动作超时计算。
 * 2. 上游：/app/robot_control 中的动作执行命令。
 * 3. 下游：robot_protocol、robot_ws_client 和 /robot/action_result。
 * 4. 本模块当前不直接切换 Menu/Walk、不直接下发动作，只提供可离线验证的安全决策。
 */

#include "humanoid_robot_gateway_runtime/motion_controller.hpp"

#include <algorithm>
#include <cctype>
#include <memory>
#include <utility>

#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace humanoid_robot_gateway_runtime
{

namespace
{

std::string value_to_json(const rapidjson::Value & value)
{
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  value.Accept(writer);
  return buffer.GetString();
}

std::string double_to_json(const double value)
{
  rapidjson::Document document;
  document.SetDouble(value);
  return value_to_json(document);
}

std::string get_string_field(
  const rapidjson::Value & object,
  const char * key)
{
  if (!object.IsObject() || !object.HasMember(key) || !object[key].IsString()) {
    return "";
  }
  return object[key].GetString();
}

std::string uppercase(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::toupper(ch));
  });
  return value;
}

bool is_walk_state(const std::string & robot_state)
{
  return uppercase(robot_state) == "WALK";
}

std::string trim_copy(const std::string & value)
{
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1);
}

std::string lowercase(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return value;
}

}  // namespace

MotionController::MotionController(RobotGatewayConfig config)
: config_(std::move(config)),
  available_motion_names_(std::make_shared<const std::set<std::string>>())
{
  if (config_.motion_default_timeout_sec <= 0.0) {
    config_.motion_default_timeout_sec = 25.0;
  }
  if (config_.motion_timeout_buffer_sec < 0.0) {
    config_.motion_timeout_buffer_sec = 8.0;
  }
  if (config_.motion_max_timeout_sec <= 0.0) {
    config_.motion_max_timeout_sec = 90.0;
  }
}

std::string MotionController::name() const { return "motion_controller"; }

MotionCommandParseResult MotionController::parse_robot_control_command(
  const std::string & command_json,
  const double fallback_timestamp_sec) const
{
  MotionCommandParseResult result;
  rapidjson::Document document;
  document.Parse(command_json.c_str());
  if (document.HasParseError() || !document.IsObject()) {
    result.reason = "invalid_json";
    return result;
  }

  const std::string command_type = get_string_field(document, "command_type");
  if (command_type != "execute_gesture") {
    result.ok = true;
    result.reason = "unsupported_command_type";
    return result;
  }

  result.client_id = get_string_field(document, "client_id");
  if (document.HasMember("timestamp")) {
    result.command_timestamp_json = value_to_json(document["timestamp"]);
  } else {
    result.command_timestamp_json = double_to_json(fallback_timestamp_sec);
  }

  if (!document.HasMember("parameters") || !document["parameters"].IsObject()) {
    result.ok = true;
    result.reason = "missing_parameters";
    return result;
  }

  const auto & parameters = document["parameters"];
  const std::string motion_name = trim_copy(get_string_field(parameters, "gesture_id"));
  const std::string placeholder = lowercase(motion_name);
  if (motion_name.empty() || placeholder == "null" || placeholder == "none" ||
    placeholder == "undefined")
  {
    result.ok = true;
    result.reason = "missing_gesture_id";
    return result;
  }

  result.ok = true;
  result.should_start_motion = true;
  result.reason = "execute_gesture";
  result.motion_name = motion_name;
  return result;
}

MotionStartDecision MotionController::evaluate_start_request(
  const MotionCommandParseResult & command,
  const bool is_executing_motion,
  const std::string & robot_state) const
{
  MotionStartDecision decision;
  decision.motion_name = command.motion_name;
  decision.client_id = command.client_id;
  decision.command_timestamp_json = command.command_timestamp_json;

  if (!command.ok) {
    decision.reason = command.reason;
    return decision;
  }

  if (!command.should_start_motion) {
    decision.reason = command.reason;
    if (command.reason == "missing_parameters" || command.reason == "missing_gesture_id") {
      decision.should_publish_result = true;
      decision.status = "rejected";
      decision.result_code = "missing_gesture_id";
      decision.message = "动作指令缺少有效的 gesture_id 字符串";
      decision.walk_ready = is_walk_state(robot_state) && !is_executing_motion;
    }
    return decision;
  }

  const auto available = std::atomic_load(&available_motion_names_);
  if (!available || available->empty()) {
    decision.should_publish_result = true;
    decision.reason = "motion_catalog_unavailable";
    decision.status = "rejected";
    decision.result_code = "motion_catalog_unavailable";
    decision.message = "机器人 OTA 动作库尚未加载，已拒绝执行";
    decision.walk_ready = is_walk_state(robot_state) && !is_executing_motion;
    return decision;
  }

  if (available->find(command.motion_name) == available->end()) {
    decision.should_publish_result = true;
    decision.reason = "invalid_gesture_id";
    decision.status = "rejected";
    decision.result_code = "invalid_gesture_id";
    decision.message = "动作 '" + command.motion_name + "' 不存在于当前机器人 OTA 动作库";
    decision.walk_ready = is_walk_state(robot_state) && !is_executing_motion;
    return decision;
  }

  if (is_executing_motion) {
    decision.should_publish_result = true;
    decision.reason = "motion_busy";
    decision.status = "rejected";
    decision.result_code = "motion_busy";
    decision.message = "当前已有动作在执行，新的动作请求已忽略";
    decision.walk_ready = is_walk_state(robot_state);
    return decision;
  }

  decision.accepted = true;
  decision.reason = config_.motion_execution_enable ? "accepted" : "accepted_but_execution_disabled";
  decision.motion_timeout_sec = get_motion_completion_timeout(command.motion_name);
  decision.task_timeout_sec = get_motion_task_timeout(command.motion_name);
  return decision;
}

void MotionController::set_motion_expected_duration(
  const std::string & motion_name,
  const double duration_sec)
{
  if (!motion_name.empty() && duration_sec > 0.0) {
    motion_expected_durations_[motion_name] = duration_sec;
  }
}

void MotionController::clear_motion_expected_durations()
{
  motion_expected_durations_.clear();
}

void MotionController::replace_available_motion_names(
  const std::set<std::string> & motion_names)
{
  std::atomic_store(
    &available_motion_names_,
    std::make_shared<const std::set<std::string>>(motion_names));
}

std::size_t MotionController::available_motion_count() const
{
  const auto available = std::atomic_load(&available_motion_names_);
  return available ? available->size() : 0U;
}

double MotionController::get_motion_completion_timeout(const std::string & motion_name) const
{
  double timeout = config_.motion_default_timeout_sec;
  const auto iter = motion_expected_durations_.find(motion_name);
  if (iter != motion_expected_durations_.end()) {
    timeout = std::max(timeout, iter->second + config_.motion_timeout_buffer_sec);
  }
  return std::min(timeout, config_.motion_max_timeout_sec);
}

double MotionController::get_motion_task_timeout(const std::string & motion_name) const
{
  return get_motion_completion_timeout(motion_name) + 20.0;
}

}  // namespace humanoid_robot_gateway_runtime
