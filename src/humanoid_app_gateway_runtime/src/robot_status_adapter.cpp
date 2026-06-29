/*
 * robot_status_adapter.cpp
 *
 * 文件用途：
 * 1. 实现机器人原始状态到 APP system_status 的转换逻辑。
 * 2. 上游：robot_gateway_node 发布的 /robot_status_raw。
 * 3. 下游：data_store 中的 system_status，以及 APP 顶部状态栏/系统健康面板。
 * 4. 当前状态：已保持线上协议 convert_raw_robot_status() 和 update_system_status() 的主要语义。
 */

#include "humanoid_app_gateway_runtime/robot_status_adapter.hpp"

#include <cerrno>
#include <cstdlib>
#include <string>

#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace humanoid_app_gateway_runtime
{

std::string RobotStatusAdapter::name() const { return "robot_status_adapter"; }

namespace
{

bool is_truthy_json_like(const rapidjson::Value & value)
{
  if (value.IsBool()) {
    return value.GetBool();
  }
  if (value.IsNull()) {
    return false;
  }
  if (value.IsString()) {
    return std::string(value.GetString(), value.GetStringLength()).empty() == false;
  }
  if (value.IsNumber()) {
    return value.IsDouble() ? value.GetDouble() != 0.0 : value.GetInt64() != 0;
  }
  if (value.IsArray()) {
    return !value.Empty();
  }
  if (value.IsObject()) {
    return !value.ObjectEmpty();
  }
  return false;
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

std::string truthy_string_or_empty(const rapidjson::Value * value)
{
  const auto text = value_to_string(value);
  return text.empty() ? "" : text;
}

const rapidjson::Value * find_member(const rapidjson::Value & object, const char * key)
{
  if (!object.IsObject()) {
    return nullptr;
  }
  const auto it = object.FindMember(key);
  return it == object.MemberEnd() ? nullptr : &it->value;
}

double to_float_json_like(const rapidjson::Value * value, const double fallback)
{
  if (value == nullptr || value->IsNull()) {
    return fallback;
  }
  if (value->IsNumber()) {
    return value->GetDouble();
  }
  if (value->IsString()) {
    char * end = nullptr;
    errno = 0;
    const double parsed = std::strtod(value->GetString(), &end);
    if (errno == 0 && end != value->GetString() && *end == '\0') {
      return parsed;
    }
  }
  return fallback;
}

int to_int_json_like(const rapidjson::Value * value, const int fallback)
{
  if (value == nullptr || value->IsNull()) {
    return fallback;
  }
  if (value->IsInt()) {
    return value->GetInt();
  }
  if (value->IsUint()) {
    return static_cast<int>(value->GetUint());
  }
  if (value->IsDouble()) {
    return static_cast<int>(value->GetDouble());
  }
  if (value->IsString()) {
    char * end = nullptr;
    errno = 0;
    const long parsed = std::strtol(value->GetString(), &end, 10);
    if (errno == 0 && end != value->GetString() && *end == '\0') {
      return static_cast<int>(parsed);
    }
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

}  // namespace

RobotStatusConversionResult RobotStatusAdapter::convert_raw_to_system_status(
  const std::string & raw_status_json,
  const double timestamp_sec) const
{
  RobotStatusConversionResult result;

  rapidjson::Document raw;
  raw.Parse(raw_status_json.c_str());
  if (raw.HasParseError() || !raw.IsObject()) {
    result.error = "invalid_robot_status_raw_json";
    return result;
  }

  const auto * values_ptr = find_member(raw, "values");
  const rapidjson::Value empty_object(rapidjson::kObjectType);
  const rapidjson::Value & values =
    (values_ptr != nullptr && values_ptr->IsObject()) ? *values_ptr : empty_object;

  const auto * health_ptr = find_member(raw, "health");

  double latency = to_float_json_like(find_member(raw, "latency"), 0.0);
  if (latency < 0.0) {
    latency = 5.0;
  }

  int signal_pct = 100;
  std::string signal_desc = "Excellent";
  if (latency < 50.0) {
    signal_pct = 100;
    signal_desc = "Excellent";
  } else if (latency < 150.0) {
    signal_pct = 85;
    signal_desc = "Good";
  } else if (latency < 500.0) {
    signal_pct = 50;
    signal_desc = "Fair";
  } else if (latency < 2000.0) {
    signal_pct = 20;
    signal_desc = "Poor";
  } else {
    signal_pct = 5;
    signal_desc = "Unstable";
  }

  std::string robot_accid = truthy_string_or_empty(find_member(raw, "accid"));
  if (robot_accid.empty()) {
    robot_accid = truthy_string_or_empty(find_member(values, "robot_accid"));
  }

  std::string robot_sn = truthy_string_or_empty(find_member(raw, "sn"));
  if (robot_sn.empty()) {
    robot_sn = truthy_string_or_empty(find_member(values, "robot_sn"));
  }

  const int battery_pct = to_int_json_like(find_member(values, "battery"), 0);
  std::string robot_state = value_to_string(find_member(values, "robot_status"));
  if (robot_state.empty()) {
    robot_state = "Unknown";
  }

  const auto * motion_busy_value = find_member(values, "motion_busy");
  const bool motion_busy = motion_busy_value != nullptr && is_truthy_json_like(*motion_busy_value);

  std::string current_motion = value_to_string(find_member(values, "current_motion"));

  const auto * control_ready_value = find_member(values, "control_ready_for_navigation");
  const bool control_ready_for_navigation =
    control_ready_value != nullptr ? is_truthy_json_like(*control_ready_value) :
    (robot_state == "Walk" && !motion_busy);

  const bool is_estop = value_to_string(find_member(values, "estop")) == "ON";

  rapidjson::Document processed;
  processed.SetObject();
  auto & allocator = processed.GetAllocator();
  processed.AddMember("battery_level", battery_pct, allocator);
  processed.AddMember("signal_quality", signal_pct, allocator);
  add_string(processed, allocator, "signal_status", signal_desc);
  add_string(processed, allocator, "network_latency", std::to_string(static_cast<int>(latency)) + "ms");
  add_string(processed, allocator, "robot_accid", robot_accid);
  add_string(processed, allocator, "robot_sn", robot_sn);

  rapidjson::Value identity(rapidjson::kObjectType);
  add_string(identity, allocator, "accid", robot_accid);
  add_string(identity, allocator, "sn", robot_sn);
  processed.AddMember("robot_identity", identity, allocator);

  add_string(processed, allocator, "robot_state", robot_state);

  rapidjson::Value power_info(rapidjson::kObjectType);
  const auto * total_voltage = find_member(values, "bat_vol");
  const auto * total_current = find_member(values, "bat_cur");
  const auto * bat_temperature = find_member(values, "bat_temp0");
  power_info.AddMember(
    "total_voltage",
    total_voltage != nullptr ? rapidjson::Value(*total_voltage, allocator) : rapidjson::Value(0.0),
    allocator);
  power_info.AddMember(
    "total_current",
    total_current != nullptr ? rapidjson::Value(*total_current, allocator) : rapidjson::Value(0.0),
    allocator);
  power_info.AddMember(
    "bat_temperature",
    bat_temperature != nullptr ? rapidjson::Value(*bat_temperature, allocator) : rapidjson::Value(0.0),
    allocator);
  processed.AddMember("power_info", power_info, allocator);

  std::string system_mode = value_to_string(find_member(values, "mode"));
  if (system_mode.empty()) {
    system_mode = "Unknown";
  }
  add_string(processed, allocator, "system_mode", system_mode);
  processed.AddMember("motion_busy", motion_busy, allocator);
  add_string(processed, allocator, "current_motion", current_motion);
  processed.AddMember("control_ready_for_navigation", control_ready_for_navigation, allocator);
  processed.AddMember("estop_active", is_estop, allocator);
  processed.AddMember(
    "health_check",
    health_ptr != nullptr ? rapidjson::Value(*health_ptr, allocator) : rapidjson::Value(rapidjson::kObjectType),
    allocator);
  processed.AddMember("timestamp", timestamp_sec, allocator);

  const int error_count = to_int_json_like(find_member(processed, "error_count"), 0);
  std::string system_health = "normal";
  if (error_count > 0 || battery_pct < 10) {
    system_health = "error";
  } else if (battery_pct < 30) {
    system_health = "warning";
  }

  const auto * emergency_stop_value = find_member(processed, "emergency_stop");
  const auto * paused_value = find_member(processed, "paused");
  const auto * navigating_value = find_member(processed, "navigating");
  std::string operational_status = "idle";
  if (emergency_stop_value != nullptr && is_truthy_json_like(*emergency_stop_value)) {
    operational_status = "emergency_stop";
  } else if (paused_value != nullptr && is_truthy_json_like(*paused_value)) {
    operational_status = "paused";
  } else if (navigating_value != nullptr && is_truthy_json_like(*navigating_value)) {
    operational_status = "navigating";
  }

  rapidjson::Document system;
  system.SetObject();
  auto & system_allocator = system.GetAllocator();
  system.AddMember("battery_level", battery_pct, system_allocator);
  system.AddMember("signal_quality", signal_pct, system_allocator);
  add_string(system, system_allocator, "signal_status", signal_desc);
  add_string(system, system_allocator, "network_latency", std::to_string(static_cast<int>(latency)) + "ms");
  add_string(system, system_allocator, "robot_accid", robot_accid);
  add_string(system, system_allocator, "robot_sn", robot_sn);

  rapidjson::Value system_identity(rapidjson::kObjectType);
  add_string(system_identity, system_allocator, "accid", robot_accid);
  add_string(system_identity, system_allocator, "sn", robot_sn);
  system.AddMember("robot_identity", system_identity, system_allocator);

  add_string(system, system_allocator, "robot_status", robot_state);
  add_string(system, system_allocator, "system_health", system_health);
  add_string(system, system_allocator, "operational_status", operational_status);
  system.AddMember("details", rapidjson::Value(processed, system_allocator), system_allocator);
  system.AddMember("timestamp", timestamp_sec, system_allocator);

  result.ok = true;
  result.processed_status_json = document_to_json(processed);
  result.system_status_json = document_to_json(system);
  return result;
}

}  // namespace humanoid_app_gateway_runtime
