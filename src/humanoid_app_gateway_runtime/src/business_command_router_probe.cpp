/*
 * business_command_router_probe.cpp
 *
 * 文件用途：
 * 1. 作为开发期 APP 业务命令路由验证工具，构造固定 business_command 场景。
 * 2. 验证路点、导航、播报音量、地图、机器人控制、表情、初始位姿、系统命令和错误路径。
 * 3. 上游：协议一致性验证脚本。
 * 4. 下游：stdout，供脚本比较业务命令路由结果是否与 既有 APP 网关协议一致。
 */

#include <iostream>
#include <string>
#include <vector>

#include "humanoid_app_gateway_runtime/business_command_router.hpp"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace
{

std::string target_to_string(const humanoid_app_gateway_runtime::BusinessCommandTarget target)
{
  switch (target) {
    case humanoid_app_gateway_runtime::BusinessCommandTarget::WaypointCommand:
      return "waypoint_command";
    case humanoid_app_gateway_runtime::BusinessCommandTarget::NavigationCommand:
      return "navigation_command";
    case humanoid_app_gateway_runtime::BusinessCommandTarget::MapCommand:
      return "map_command";
    case humanoid_app_gateway_runtime::BusinessCommandTarget::RobotControl:
      return "robot_control";
    case humanoid_app_gateway_runtime::BusinessCommandTarget::FacialRawCommand:
      return "facial_raw_command";
    case humanoid_app_gateway_runtime::BusinessCommandTarget::InitialPose:
      return "initial_pose";
    case humanoid_app_gateway_runtime::BusinessCommandTarget::SystemCommand:
      return "system_command";
    case humanoid_app_gateway_runtime::BusinessCommandTarget::BroadcastVolumeService:
      return "broadcast_volume_service";
    case humanoid_app_gateway_runtime::BusinessCommandTarget::Error:
    default:
      return "error";
  }
}

rapidjson::Value result_to_json(
  const humanoid_app_gateway_runtime::BusinessCommandRouteResult & result,
  rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value value(rapidjson::kObjectType);
  value.AddMember("ok", result.ok, allocator);
  value.AddMember("target", rapidjson::Value(target_to_string(result.target).c_str(), allocator).Move(), allocator);
  value.AddMember("error", rapidjson::Value(result.error.c_str(), allocator).Move(), allocator);

  rapidjson::Document payload;
  if (!result.payload_json.empty()) {
    payload.Parse(result.payload_json.c_str());
    value.AddMember("payload", rapidjson::Value(payload, allocator), allocator);
  } else {
    rapidjson::Value empty(rapidjson::kObjectType);
    value.AddMember("payload", empty, allocator);
  }

  rapidjson::Document ack;
  if (!result.ack_json.empty()) {
    ack.Parse(result.ack_json.c_str());
    value.AddMember("ack", rapidjson::Value(ack, allocator), allocator);
  } else {
    rapidjson::Value empty(rapidjson::kObjectType);
    value.AddMember("ack", empty, allocator);
  }
  return value;
}

}  // namespace

int main()
{
  humanoid_app_gateway_runtime::BusinessCommandRouter router;
  const std::string client_id = "client_a";
  constexpr double timestamp = 123.0;

  const std::vector<std::pair<std::string, std::string>> cases{
    {"waypoint", R"({"message_id":"msg_wp","data_type":"waypoint_management","data":{"command_type":"add_waypoint","map_id":"hall","waypoint_id":"wp1","waypoint_type":"task","include_details":false,"waypoint_data":{"name":"点1"}}})"},
    {"navigation", R"({"message_id":"msg_nav","data_type":"navigation_control","data":{"command_type":"start_route_task","task_session_id":"ts1","route_id":"r1","map_id":"hall","route_waypoint_ids":["wp1","wp2"],"waypoints_revision":5}})"},
    {"volume", R"({"message_id":"msg_vol","data_type":"navigation_control","data":{"command_type":"set_broadcast_volume","broadcast_volume":120}})"},
    {"map", R"({"message_id":"msg_map","data_type":"map_management","data":{"command_type":"list_maps","target_map_id":"hall","reason":"test"}})"},
    {"robot", R"({"message_id":"msg_robot","data_type":"robot_control","data":{"action":"wave","parameters":{"speed":1}}})"},
    {"facial", R"({"message_id":"msg_face","data_type":"facial_control","data":{"action":"blink"}})"},
    {"initial_pose", R"({"message_id":"msg_pose","data_type":"initial_pose","data":{"x":1.0,"y":2.0,"yaw":1.57079632679,"frame_id":"map"}})"},
    {"system", R"({"message_id":"msg_sys","data_type":"system_command","data":{"action":"restart","parameters":{"target":"nav"}}})"},
    {"error", R"({"message_id":"msg_err","data_type":"unknown_command","data":{}})"}
  };

  rapidjson::Document document;
  document.SetObject();
  auto & allocator = document.GetAllocator();
  for (const auto & item : cases) {
    document.AddMember(
      rapidjson::Value(item.first.c_str(), allocator).Move(),
      result_to_json(router.route_business_command(item.second, client_id, timestamp), allocator),
      allocator);
  }

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  std::cout << buffer.GetString() << std::endl;
  return 0;
}
