/*
 * robot_status_parser.cpp
 *
 * 文件用途：
 * 1. 实现机器人 notify_robot_info 到 /robot_status_raw JSON 的解析和单位转换。
 * 2. 实现 notify_joy_data 到 Joy axes/buttons 所需数组的解析。
 * 3. 对齐现有 publish_notify_robot_info 和 publish_notify_joy_data 的核心语义。
 * 4. 上游：robot_ws_client 接收到的机器人 notify/status/joy 消息。
 * 5. 下游：/robot_status_raw、/joy_raw、data_integration_node 和 navigation_state_manager。
 */

#include "humanoid_robot_gateway_runtime/robot_status_parser.hpp"

#include <cmath>
#include <numeric>
#include <string>

#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace humanoid_robot_gateway_runtime
{

std::string RobotStatusParser::name() const { return "robot_status_parser"; }

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

double to_double(const std::string & value, const double fallback = 0.0)
{
  try {
    return std::stod(value);
  } catch (...) {
    return fallback;
  }
}

bool is_integer_string(const std::string & value)
{
  if (value.empty()) {
    return false;
  }
  std::size_t start = value[0] == '-' ? 1 : 0;
  if (start >= value.size()) {
    return false;
  }
  for (std::size_t i = start; i < value.size(); ++i) {
    if (!std::isdigit(static_cast<unsigned char>(value[i]))) {
      return false;
    }
  }
  return true;
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

void add_parsed_value(
  rapidjson::Value & values,
  rapidjson::Document::AllocatorType & allocator,
  const std::string & key,
  const std::string & raw_value)
{
  if (key.size() >= 4 && key.compare(key.size() - 4, 4, "_vol") == 0) {
    values.AddMember(
      rapidjson::Value(key.c_str(), allocator).Move(),
      to_double(raw_value) / 1000.0,
      allocator);
  } else if (key == "bat_vol") {
    values.AddMember(rapidjson::Value(key.c_str(), allocator).Move(), to_double(raw_value) / 1000.0, allocator);
  } else if (key.size() >= 4 && key.compare(key.size() - 4, 4, "_cur") == 0) {
    values.AddMember(
      rapidjson::Value(key.c_str(), allocator).Move(),
      to_double(raw_value) / 1000.0,
      allocator);
  } else if (key == "bat_cur") {
    values.AddMember(rapidjson::Value(key.c_str(), allocator).Move(), to_double(raw_value) / 1000.0, allocator);
  } else if (key.rfind("bat_temp", 0) == 0) {
    values.AddMember(rapidjson::Value(key.c_str(), allocator).Move(), to_double(raw_value) / 10.0, allocator);
  } else if (is_integer_string(raw_value)) {
    values.AddMember(rapidjson::Value(key.c_str(), allocator).Move(), std::stoi(raw_value), allocator);
  } else {
    add_string(values, allocator, key.c_str(), raw_value);
  }
}

double round1(const double value)
{
  return std::round(value * 10.0) / 10.0;
}

std::string document_to_json(const rapidjson::Document & document)
{
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  return buffer.GetString();
}

}  // namespace

RobotStatusParseResult RobotStatusParser::parse_notify_robot_info(
  const std::string & info_raw_json,
  const std::string & accid,
  const std::string & sn,
  const std::string & identity_source,
  const std::string & robot_state,
  const bool is_executing_motion,
  const std::string & current_motion_name,
  const double current_time_sec,
  const double previous_msg_time_sec,
  const std::vector<double> & previous_intervals_ms) const
{
  RobotStatusParseResult result;
  rapidjson::Document info;
  info.Parse(info_raw_json.c_str());
  if (info.HasParseError() || !info.IsObject()) {
    result.error = "invalid_notify_robot_info_json";
    return result;
  }

  const double interval_ms = (current_time_sec - previous_msg_time_sec) * 1000.0;
  std::vector<double> intervals = previous_intervals_ms;
  intervals.push_back(interval_ms);
  if (intervals.size() > 20) {
    intervals.erase(intervals.begin());
  }

  double avg_interval = interval_ms;
  double jitter = 0.0;
  if (intervals.size() >= 3) {
    avg_interval = std::accumulate(intervals.begin(), intervals.end(), 0.0) /
      static_cast<double>(intervals.size());
    double variance = 0.0;
    for (const auto item : intervals) {
      const double diff = item - avg_interval;
      variance += diff * diff;
    }
    variance /= static_cast<double>(intervals.size());
    jitter = std::sqrt(variance);
  }

  const double expected_interval = 500.0;
  const double latency = std::min(std::max(0.0, avg_interval - expected_interval) + jitter, 5000.0);

  rapidjson::Document output;
  output.SetObject();
  auto & allocator = output.GetAllocator();
  rapidjson::Value values(rapidjson::kObjectType);
  rapidjson::Value health(rapidjson::kObjectType);

  const auto * result_items = find_member(info, "result");
  if (result_items != nullptr && result_items->IsArray()) {
    for (const auto & component : result_items->GetArray()) {
      if (!component.IsObject()) {
        continue;
      }
      const std::string component_name = value_to_string(find_member(component, "name"), "unknown");
      add_string(health, allocator, component_name.c_str(), value_to_string(find_member(component, "message")));

      const auto * component_values = find_member(component, "values");
      if (component_values == nullptr || !component_values->IsArray()) {
        continue;
      }
      for (const auto & item : component_values->GetArray()) {
        if (!item.IsObject()) {
          continue;
        }
        const std::string key = value_to_string(find_member(item, "key"));
        const std::string raw_value = value_to_string(find_member(item, "value"));
        add_parsed_value(values, allocator, key, raw_value);
      }
    }
  }

  values.AddMember("motion_busy", is_executing_motion, allocator);
  add_string(values, allocator, "current_motion", current_motion_name);
  values.AddMember("control_ready_for_navigation", robot_state == "Walk" && !is_executing_motion, allocator);
  add_string(values, allocator, "robot_accid", accid);
  add_string(values, allocator, "robot_sn", sn);

  add_string(output, allocator, "accid", accid);
  add_string(output, allocator, "sn", sn);
  add_string(output, allocator, "identity_source", identity_source);
  output.AddMember("values", values, allocator);
  output.AddMember("health", health, allocator);
  output.AddMember("latency", round1(latency), allocator);
  output.AddMember("jitter", round1(jitter), allocator);
  output.AddMember("msg_interval", round1(avg_interval), allocator);
  output.AddMember("timestamp", current_time_sec, allocator);

  result.ok = true;
  result.robot_status_raw_json = document_to_json(output);
  return result;
}

JoyParseResult RobotStatusParser::parse_notify_joy_data(
  const std::string & joy_data_json) const
{
  JoyParseResult result;
  rapidjson::Document joy;
  joy.Parse(joy_data_json.c_str());
  if (joy.HasParseError() || !joy.IsObject()) {
    result.error = "invalid_notify_joy_data_json";
    return result;
  }

  const auto * axes = find_member(joy, "axes");
  if (axes != nullptr && axes->IsArray()) {
    for (const auto & item : axes->GetArray()) {
      result.axes.push_back(to_double(value_to_string(&item), 0.0));
    }
  }
  const auto * buttons = find_member(joy, "buttons");
  if (buttons != nullptr && buttons->IsArray()) {
    for (const auto & item : buttons->GetArray()) {
      if (item.IsBool()) {
        result.buttons.push_back(item.GetBool() ? 1 : 0);
      } else {
        result.buttons.push_back(static_cast<int>(to_double(value_to_string(&item), 0.0)));
      }
    }
  }

  result.ok = true;
  return result;
}

}  // namespace humanoid_robot_gateway_runtime
