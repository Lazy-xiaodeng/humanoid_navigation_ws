/*
 * client_registry_probe.cpp
 *
 * 文件用途：
 * 1. 作为开发期客户端连接/订阅缓存验证工具，构造固定客户端连接和订阅场景。
 * 2. 验证注册、活跃时间更新、订阅/退订、按 data_type 查询订阅者、断开清理返切换订类型。
 * 3. 上游：协议一致性验证脚本。
 * 4. 下游：stdout，供脚本验证客户端状态管理结果是否符合线上协议语义。
 */

#include <iostream>
#include <string>
#include <vector>

#include "humanoid_app_gateway_runtime/client_registry.hpp"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace
{

rapidjson::Value strings_to_array(
  const std::vector<std::string> & values,
  rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value array(rapidjson::kArrayType);
  for (const auto & value : values) {
    array.PushBack(rapidjson::Value(value.c_str(), allocator).Move(), allocator);
  }
  return array;
}

void add_size(
  rapidjson::Value & object,
  rapidjson::Document::AllocatorType & allocator,
  const char * key,
  const std::size_t value)
{
  object.AddMember(rapidjson::Value(key, allocator).Move(), static_cast<unsigned int>(value), allocator);
}

}  // namespace

int main()
{
  humanoid_app_gateway_runtime::ClientRegistry registry;
  registry.register_client("client_a", "192.168.1.10", 10.0);
  registry.register_client("client_b", "", 11.0);
  registry.update_activity("client_a", 12.5);
  registry.apply_subscription_request("client_a", "subscribe", {"system_status", "navigation_status"}, 2.0, 13.0);
  registry.apply_subscription_request("client_b", "subscribe", {"system_status"}, 1.0, 14.0);
  registry.apply_subscription_request("client_a", "unsubscribe", {"navigation_status"}, 1.0, 15.0);

  rapidjson::Document document;
  document.SetObject();
  auto & allocator = document.GetAllocator();

  add_size(document, allocator, "client_count_before_cleanup", registry.client_count());
  add_size(document, allocator, "subscription_count_before_cleanup", registry.subscription_count());
  document.AddMember(
    "system_status_clients",
    strings_to_array(registry.get_subscribed_clients("system_status"), allocator),
    allocator);
  document.AddMember(
    "client_a_subscriptions",
    strings_to_array(registry.get_client_subscriptions("client_a"), allocator),
    allocator);

  const auto session = registry.get_session("client_a");
  if (session.has_value()) {
    rapidjson::Value session_json(rapidjson::kObjectType);
    session_json.AddMember("connected_time", session->connected_time, allocator);
    session_json.AddMember("last_activity", session->last_activity, allocator);
    session_json.AddMember("authenticated", session->authenticated, allocator);
    session_json.AddMember("ip", rapidjson::Value(session->ip.c_str(), allocator).Move(), allocator);
    document.AddMember("client_a_session", session_json, allocator);
  }

  document.AddMember(
    "cleanup_client_a",
    strings_to_array(registry.cleanup_client("client_a"), allocator),
    allocator);
  add_size(document, allocator, "client_count_after_cleanup", registry.client_count());
  add_size(document, allocator, "subscription_count_after_cleanup", registry.subscription_count());
  document.AddMember("has_client_a", registry.has_client("client_a"), allocator);
  document.AddMember("has_client_b", registry.has_client("client_b"), allocator);

  registry.clear();
  add_size(document, allocator, "client_count_after_clear", registry.client_count());

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  std::cout << buffer.GetString() << std::endl;
  return 0;
}
