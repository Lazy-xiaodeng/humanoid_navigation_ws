/*
 * navigation_status_adapter_probe.cpp
 *
 * 文件用途：
 * 1. 作为开发期导航状态转换验证工具，构造固定 ack 和 event_type 场景。
 * 2. 验证立即推送事件集合和 navigation_command_result 业务数据结构。
 * 3. 上游：协议一致性验证脚本。
 * 4. 下游：stdout，供脚本验证导航状态转换结果是否符合线上协议语义。
 */

#include <iostream>

#include "humanoid_app_gateway_runtime/navigation_status_adapter.hpp"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace
{

void add_bool(
  rapidjson::Value & object,
  rapidjson::Document::AllocatorType & allocator,
  const char * key,
  const bool value)
{
  object.AddMember(rapidjson::Value(key, allocator).Move(), value, allocator);
}

}  // namespace

int main()
{
  humanoid_app_gateway_runtime::NavigationStatusAdapter adapter;
  const std::string ack_json =
    R"({"ack_type":"pause","status":"error","message":"暂停失败","error_code":"pause_failed","extra":42})";
  const auto ack_result = adapter.build_navigation_command_result_data(ack_json, 123.0);
  if (!ack_result.ok) {
    std::cerr << ack_result.error << std::endl;
    return 2;
  }

  rapidjson::Document ack_data;
  ack_data.Parse(ack_result.json.c_str());

  rapidjson::Document document;
  document.SetObject();
  auto & allocator = document.GetAllocator();
  add_bool(document, allocator, "push_waypoint_reached", adapter.should_push_navigation_status_immediately("waypoint_reached"));
  add_bool(document, allocator, "push_progress_update", adapter.should_push_navigation_status_immediately("progress_update"));
  add_bool(document, allocator, "push_route_task_completed", adapter.should_push_navigation_status_immediately("route_task_completed"));
  document.AddMember("ack_data", rapidjson::Value(ack_data, allocator), allocator);

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  std::cout << buffer.GetString() << std::endl;
  return 0;
}
