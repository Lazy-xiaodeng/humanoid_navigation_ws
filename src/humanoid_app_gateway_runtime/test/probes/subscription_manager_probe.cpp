/*
 * subscription_manager_probe.cpp
 *
 * 文件用途：
 * 1. 作为开发期订阅管理验证工具，构造固定订阅场景并输出检查结果。
 * 2. 验证首次推送、频率节流、frequency=0、部分取消订阅、移除客户端和统计信息。
 * 3. 上游：协议一致性验证脚本。
 * 4. 下游：stdout，供脚本比较 C++ 订阅管理行为是否与 订阅管理语义一致。
 */

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include "humanoid_app_gateway_runtime/subscription_manager.hpp"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace
{

rapidjson::Value strings_to_array(
  std::vector<std::string> values,
  rapidjson::Document::AllocatorType & allocator)
{
  std::sort(values.begin(), values.end());
  rapidjson::Value array(rapidjson::kArrayType);
  for (const auto & value : values) {
    array.PushBack(rapidjson::Value(value.c_str(), allocator).Move(), allocator);
  }
  return array;
}

std::vector<std::string> subscriber_ids(
  const std::vector<humanoid_app_gateway_runtime::SubscriptionInfo> & subscribers)
{
  std::vector<std::string> ids;
  for (const auto & subscriber : subscribers) {
    ids.push_back(subscriber.client_id);
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

void add_size(
  rapidjson::Value & object,
  rapidjson::Document::AllocatorType & allocator,
  const char * key,
  const std::size_t value)
{
  object.AddMember(
    rapidjson::Value(key, allocator).Move(),
    static_cast<unsigned int>(value),
    allocator);
}

}  // namespace

int main()
{
  humanoid_app_gateway_runtime::SubscriptionManager manager;
  manager.subscribe(
    "client_a",
    {"system_status", "navigation_status"},
    2.0,
    "{\"source\":\"client_a\"}",
    10.0);
  manager.subscribe("client_b", {"system_status"}, 0.0, "{\"source\":\"client_b\"}", 11.0);

  rapidjson::Document document;
  document.SetObject();
  auto & allocator = document.GetAllocator();

  document.AddMember(
    "ready_at_10_4",
    strings_to_array(subscriber_ids(manager.get_subscribers("system_status", 10.4)), allocator),
    allocator);

  manager.update_push_time("client_a", "system_status", 10.4);
  document.AddMember(
    "ready_at_10_6",
    strings_to_array(subscriber_ids(manager.get_subscribers("system_status", 10.6)), allocator),
    allocator);
  document.AddMember(
    "ready_at_11_0",
    strings_to_array(subscriber_ids(manager.get_subscribers("system_status", 11.0)), allocator),
    allocator);
  document.AddMember(
    "client_a_subscriptions_before_unsubscribe",
    strings_to_array(manager.get_client_subscriptions("client_a"), allocator),
    allocator);

  manager.unsubscribe("client_a", std::vector<std::string>{"navigation_status"});
  document.AddMember(
    "client_a_subscriptions_after_partial_unsubscribe",
    strings_to_array(manager.get_client_subscriptions("client_a"), allocator),
    allocator);

  const auto stats = manager.get_statistics();
  add_size(document, allocator, "stats_total_data_types", stats.total_data_types);
  add_size(document, allocator, "stats_total_subscriptions", stats.total_subscriptions);

  rapidjson::Value clients_by_type(rapidjson::kObjectType);
  for (const auto & [data_type, count] : stats.clients_by_type) {
    clients_by_type.AddMember(
      rapidjson::Value(data_type.c_str(), allocator).Move(),
      static_cast<unsigned int>(count),
      allocator);
  }
  document.AddMember("stats_clients_by_type", clients_by_type, allocator);

  manager.remove_client("client_b");
  add_size(document, allocator, "active_after_remove_client_b", manager.active_count());
  document.AddMember(
    "system_status_after_remove_client_b",
    strings_to_array(subscriber_ids(manager.get_subscribers("system_status", 11.0)), allocator),
    allocator);

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  std::cout << buffer.GetString() << std::endl;
  return 0;
}
