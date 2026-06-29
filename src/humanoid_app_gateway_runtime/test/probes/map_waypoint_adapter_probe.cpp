/*
 * map_waypoint_adapter_probe.cpp
 *
 * 文件用途：
 * 1. 作为开发期地图/点位转换验证工具，构造固定 map payload 和 waypoints payload。
 * 2. 验证 data_type、message_type、data 包装、metadata fallback、map_id 和 revision 优先级。
 * 3. 上游：协议一致性验证脚本。
 * 4. 下游：stdout，供脚本验证地图/点位转换结果是否符合线上协议语义。
 */

#include <iostream>

#include "humanoid_app_gateway_runtime/map_waypoint_adapter.hpp"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

int main()
{
  humanoid_app_gateway_runtime::MapWaypointAdapter adapter;
  const std::string map_payload =
    R"({"data_type":"map_status","data":{"status":"ready","message":"地图就绪","request_message_id":"req_map"},"metadata":{"status":"warning","error_code":"map_warn"}})";
  const std::string map_raw_payload = R"({"data_type":"map_response","message_type":"response","data":["raw","list"]})";
  const std::string waypoints_payload =
    R"({"data_type":"waypoints_data","data":{"map_id":"map_from_data","waypoints_revision":7,"metadata":{"map_id":"map_from_inner","waypoints_revision":8}},"metadata":{"status":"success","map_id":"map_from_meta","waypoints_revision":9}})";

  const auto map_result = adapter.build_map_message_payload(map_payload);
  const auto map_raw_result = adapter.build_map_message_payload(map_raw_payload);
  const auto waypoints_result = adapter.build_waypoints_data_payload(waypoints_payload);
  if (!map_result.ok || !map_raw_result.ok || !waypoints_result.ok) {
    std::cerr << map_result.error << map_raw_result.error << waypoints_result.error << std::endl;
    return 2;
  }

  rapidjson::Document map_doc;
  rapidjson::Document map_raw_doc;
  rapidjson::Document waypoints_doc;
  map_doc.Parse(map_result.json.c_str());
  map_raw_doc.Parse(map_raw_result.json.c_str());
  waypoints_doc.Parse(waypoints_result.json.c_str());

  rapidjson::Document document;
  document.SetObject();
  auto & allocator = document.GetAllocator();
  document.AddMember("map", rapidjson::Value(map_doc, allocator), allocator);
  document.AddMember("map_raw", rapidjson::Value(map_raw_doc, allocator), allocator);
  document.AddMember("waypoints", rapidjson::Value(waypoints_doc, allocator), allocator);

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  std::cout << buffer.GetString() << std::endl;
  return 0;
}
