/*
 * business_command_router.cpp
 *
 * 文件用途：
 * 1. 实现 APP business_command 到导航、地图、机器人控制、表情、系统命令和 initialpose 的路由决策。
 * 2. 对齐现有 APP business_command、点位/导航/地图/机器人控制/表情/初始位姿/系统命令字段整理语义。
 * 3. 本模块只做命令分类和 payload/ack 组包，不直接发布 ROS topic、不调用真实服务。
 * 4. 上游：app_protocol 校验后的 APP business_command。
 * 5. 下游：/app/navigation_command、/app/map_command、/app/robot_control、/robot/facial_raw_cmd、
 *    /initialpose 和播报音量服务调用外壳。
 */

#include "humanoid_app_gateway_runtime/business_command_router.hpp"

#include <cmath>
#include <string>

#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace humanoid_app_gateway_runtime
{

std::string BusinessCommandRouter::name() const { return "business_command_router"; }

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

std::string non_empty_string_or_fallback(const rapidjson::Value * value, const std::string & fallback)
{
  const std::string text = value_to_string(value);
  return text.empty() ? fallback : text;
}

double value_to_double(const rapidjson::Value * value, const double fallback = 0.0)
{
  if (value == nullptr || value->IsNull()) {
    return fallback;
  }
  if (value->IsNumber()) {
    return value->GetDouble();
  }
  if (value->IsString()) {
    try {
      return std::stod(std::string(value->GetString(), value->GetStringLength()));
    } catch (...) {
      return fallback;
    }
  }
  return fallback;
}

bool value_to_bool(const rapidjson::Value * value, const bool fallback)
{
  if (value == nullptr || value->IsNull()) {
    return fallback;
  }
  if (value->IsBool()) {
    return value->GetBool();
  }
  if (value->IsNumber()) {
    return value->GetDouble() != 0.0;
  }
  if (value->IsString()) {
    const std::string text(value->GetString(), value->GetStringLength());
    return text == "true" || text == "1" || text == "yes" || text == "on";
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

void add_copy_or_empty_object(
  rapidjson::Value & object,
  rapidjson::Document::AllocatorType & allocator,
  const char * key,
  const rapidjson::Value * value)
{
  if (value != nullptr && value->IsObject()) {
    object.AddMember(rapidjson::Value(key, allocator).Move(), rapidjson::Value(*value, allocator), allocator);
  } else {
    rapidjson::Value empty(rapidjson::kObjectType);
    object.AddMember(rapidjson::Value(key, allocator).Move(), empty, allocator);
  }
}

void add_copy_or_empty_array(
  rapidjson::Value & object,
  rapidjson::Document::AllocatorType & allocator,
  const char * key,
  const rapidjson::Value * value)
{
  if (value != nullptr && value->IsArray()) {
    object.AddMember(rapidjson::Value(key, allocator).Move(), rapidjson::Value(*value, allocator), allocator);
  } else {
    rapidjson::Value empty(rapidjson::kArrayType);
    object.AddMember(rapidjson::Value(key, allocator).Move(), empty, allocator);
  }
}

void add_copy_or_string_default(
  rapidjson::Value & object,
  rapidjson::Document::AllocatorType & allocator,
  const char * key,
  const rapidjson::Value * value,
  const std::string & fallback)
{
  if (value != nullptr) {
    object.AddMember(rapidjson::Value(key, allocator).Move(), rapidjson::Value(*value, allocator), allocator);
  } else {
    add_string(object, allocator, key, fallback);
  }
}

std::string document_to_json(const rapidjson::Document & document)
{
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  return buffer.GetString();
}

BusinessCommandRouteResult error_result(const std::string & message)
{
  BusinessCommandRouteResult result;
  result.error = message;
  return result;
}

std::string ack_json(
  const std::string & command_type,
  const std::string & action,
  const std::string & status,
  const std::string & message)
{
  rapidjson::Document document;
  document.SetObject();
  auto & allocator = document.GetAllocator();
  add_string(document, allocator, "command_type", command_type);
  if (!action.empty()) {
    add_string(document, allocator, "action", action);
  }
  add_string(document, allocator, "status", status);
  add_string(document, allocator, "message", message);
  return document_to_json(document);
}

std::string initial_pose_ack_json(const double x, const double y, const double yaw, const std::string & frame_id)
{
  rapidjson::Document document;
  document.SetObject();
  auto & allocator = document.GetAllocator();
  add_string(document, allocator, "command_type", "initial_pose");
  add_string(document, allocator, "status", "executed");
  rapidjson::Value pose(rapidjson::kObjectType);
  pose.AddMember("x", x, allocator);
  pose.AddMember("y", y, allocator);
  pose.AddMember("yaw", yaw, allocator);
  add_string(pose, allocator, "frame_id", frame_id);
  document.AddMember("pose", pose, allocator);
  add_string(document, allocator, "message", "初始位姿已设置");
  return document_to_json(document);
}

}  // namespace

BusinessCommandRouteResult BusinessCommandRouter::route_business_command(
  const std::string & message_json,
  const std::string & client_id,
  const double timestamp_sec) const
{
  rapidjson::Document message;
  message.Parse(message_json.c_str());
  if (message.HasParseError() || !message.IsObject()) {
    return error_result("无效的命令 JSON");
  }

  const std::string command_type = value_to_string(find_member(message, "data_type"));
  if (command_type.empty()) {
    return error_result("命令类型不能为空");
  }

  const auto * command_data_ptr = find_member(message, "data");
  if (command_data_ptr == nullptr || !command_data_ptr->IsObject()) {
    return error_result("命令数据不能为空");
  }
  const auto & command_data = *command_data_ptr;
  const std::string request_message_id = value_to_string(find_member(message, "message_id"));

  rapidjson::Document payload;
  payload.SetObject();
  auto & allocator = payload.GetAllocator();

  BusinessCommandRouteResult result;

  if (command_type == "waypoint_management") {
    const std::string inner_command = value_to_string(find_member(command_data, "command_type"));
    if (inner_command.empty()) {
      return error_result("路点管理命令缺少 command_type");
    }
    add_string(payload, allocator, "command_type", inner_command);
    add_string(payload, allocator, "map_id", value_to_string(find_member(command_data, "map_id")));
    add_string(payload, allocator, "clear_scope", value_to_string(find_member(command_data, "clear_scope")));
    add_copy_or_empty_object(payload, allocator, "waypoint_data", find_member(command_data, "waypoint_data"));
    add_string(payload, allocator, "waypoint_id", value_to_string(find_member(command_data, "waypoint_id")));
    add_string(payload, allocator, "waypoint_type", value_to_string(find_member(command_data, "waypoint_type")));
    add_string(payload, allocator, "request_message_id", request_message_id);
    payload.AddMember("include_details", value_to_bool(find_member(command_data, "include_details"), true), allocator);
    add_string(payload, allocator, "client_id", client_id);
    payload.AddMember("timestamp", timestamp_sec, allocator);
    result.target = BusinessCommandTarget::WaypointCommand;
    result.ack_request_id = request_message_id;
  } else if (command_type == "navigation_control") {
    const std::string inner_command = value_to_string(find_member(command_data, "command_type"));
    if (inner_command.empty()) {
      return error_result("导航控制命令缺少 command_type");
    }
    if (inner_command == "set_broadcast_volume") {
      int volume = static_cast<int>(value_to_double(
        find_member(command_data, "broadcast_volume") != nullptr ?
        find_member(command_data, "broadcast_volume") :
        find_member(command_data, "volume_percent"),
        72.0));
      if (volume < 0) {
        volume = 0;
      }
      if (volume > 100) {
        volume = 100;
      }
      add_string(payload, allocator, "command_type", "set_broadcast_volume");
      payload.AddMember("volume_percent", volume, allocator);
      add_string(
        payload,
        allocator,
        "request_message_id",
        non_empty_string_or_fallback(find_member(command_data, "request_message_id"), request_message_id));
      payload.AddMember("timestamp", timestamp_sec, allocator);
      result.target = BusinessCommandTarget::BroadcastVolumeService;
      result.ack_request_id = non_empty_string_or_fallback(
        find_member(command_data, "request_message_id"), request_message_id);
    } else {
      add_string(payload, allocator, "command_type", inner_command);
      add_string(payload, allocator, "task_session_id", value_to_string(find_member(command_data, "task_session_id")));
      add_string(payload, allocator, "route_id", value_to_string(find_member(command_data, "route_id")));
      add_string(payload, allocator, "map_id", value_to_string(find_member(command_data, "map_id")));
      add_copy_or_empty_array(payload, allocator, "route_waypoints", find_member(command_data, "route_waypoints"));
      add_copy_or_empty_array(payload, allocator, "route_waypoint_ids", find_member(command_data, "route_waypoint_ids"));
      add_copy_or_string_default(payload, allocator, "waypoints_revision", find_member(command_data, "waypoints_revision"), "");
      add_string(payload, allocator, "target_waypoint_id", value_to_string(find_member(command_data, "target_waypoint_id")));
      payload.AddMember("interrupt_broadcast", value_to_bool(find_member(command_data, "interrupt_broadcast"), true), allocator);
      add_string(payload, allocator, "broadcast_id", value_to_string(find_member(command_data, "broadcast_id")));
      add_copy_or_empty_object(payload, allocator, "pause_parameters", find_member(command_data, "pause_parameters"));
      add_copy_or_empty_object(payload, allocator, "stop_parameters", find_member(command_data, "stop_parameters"));
      add_copy_or_string_default(payload, allocator, "broadcast_result", find_member(command_data, "broadcast_result"), "completed");
      payload.AddMember("broadcast_duration_sec", value_to_double(find_member(command_data, "broadcast_duration_sec"), 0.0), allocator);
      add_string(payload, allocator, "reason", value_to_string(find_member(command_data, "reason")));
      add_string(
        payload,
        allocator,
        "request_message_id",
        non_empty_string_or_fallback(find_member(command_data, "request_message_id"), request_message_id));
      add_string(payload, allocator, "client_id", client_id);
      payload.AddMember("timestamp", timestamp_sec, allocator);
      result.target = BusinessCommandTarget::NavigationCommand;
      result.ack_request_id = non_empty_string_or_fallback(
        find_member(command_data, "request_message_id"), request_message_id);
    }
  } else if (command_type == "map_management") {
    const std::string inner_command = value_to_string(find_member(command_data, "command_type"));
    if (inner_command.empty()) {
      return error_result("地图管理命令缺少 command_type");
    }
    add_string(payload, allocator, "command_type", inner_command);
    add_string(payload, allocator, "target_map_id", value_to_string(find_member(command_data, "target_map_id")));
    add_string(payload, allocator, "reason", value_to_string(find_member(command_data, "reason")));
    add_string(
      payload,
      allocator,
      "request_message_id",
      non_empty_string_or_fallback(find_member(command_data, "request_message_id"), request_message_id));
    add_string(payload, allocator, "client_id", client_id);
    payload.AddMember("timestamp", timestamp_sec, allocator);
    result.target = BusinessCommandTarget::MapCommand;
    result.ack_request_id = non_empty_string_or_fallback(
      find_member(command_data, "request_message_id"), request_message_id);
  } else if (command_type == "robot_control") {
    const std::string action = value_to_string(find_member(command_data, "action"));
    add_string(payload, allocator, "command_type", action);
    add_copy_or_empty_object(payload, allocator, "parameters", find_member(command_data, "parameters"));
    add_string(payload, allocator, "client_id", client_id);
    payload.AddMember("timestamp", timestamp_sec, allocator);
    result.target = BusinessCommandTarget::RobotControl;
    result.ack_json = ack_json("robot_control", action, "received", "机器人控制命令'" + action + "'已接收");
    result.ack_request_id = request_message_id;
  } else if (command_type == "facial_control") {
    const std::string action = value_to_string(find_member(command_data, "action"));
    if (action.empty()) {
      return error_result("面部控制命令缺少 'action' 字段");
    }
    add_string(payload, allocator, "action", action);
    add_string(payload, allocator, "request_id", request_message_id);
    result.target = BusinessCommandTarget::FacialRawCommand;
    result.ack_json = ack_json("facial_control", action, "executed", "面部动作 '" + action + "' 已执行");
    result.ack_request_id = request_message_id;
  } else if (command_type == "initial_pose") {
    const double x = value_to_double(find_member(command_data, "x"));
    const double y = value_to_double(find_member(command_data, "y"));
    const double yaw = value_to_double(find_member(command_data, "yaw"));
    const std::string frame_id = value_to_string(find_member(command_data, "frame_id"), "map");
    add_string(payload, allocator, "frame_id", frame_id);
    payload.AddMember("x", x, allocator);
    payload.AddMember("y", y, allocator);
    payload.AddMember("yaw", yaw, allocator);
    payload.AddMember("qw", std::cos(yaw * 0.5), allocator);
    payload.AddMember("qz", std::sin(yaw * 0.5), allocator);
    result.target = BusinessCommandTarget::InitialPose;
    result.ack_json = initial_pose_ack_json(x, y, yaw, frame_id);
    result.ack_request_id = request_message_id;
  } else {
    return error_result("不支持的命令类型: " + command_type);
  }

  result.ok = true;
  result.payload_json = document_to_json(payload);
  return result;
}

}  // namespace humanoid_app_gateway_runtime
