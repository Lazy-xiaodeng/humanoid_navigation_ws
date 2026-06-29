/*
 * integration_forwarder_probe.cpp
 *
 * 文件用途：
 * 1. 作为开发期 integration 路由验证工具，构造固定 response/push/subscription_response 场景。
 * 2. 验证广播、指定客户端、订阅客户端、断开客户端丢弃和业务状态同步标记。
 * 3. 上游：协议一致性验证脚本。
 * 4. 下游：stdout，供脚本比较路由决策是否与 既有 APP 网关协议一致。
 */

#include <iostream>
#include <string>

#include "humanoid_app_gateway_runtime/integration_forwarder.hpp"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace
{

std::string kind_to_string(const humanoid_app_gateway_runtime::ForwardRouteKind kind)
{
  switch (kind) {
    case humanoid_app_gateway_runtime::ForwardRouteKind::BroadcastAll:
      return "broadcast_all";
    case humanoid_app_gateway_runtime::ForwardRouteKind::DirectClient:
      return "direct_client";
    case humanoid_app_gateway_runtime::ForwardRouteKind::SubscribedClients:
      return "subscribed_clients";
    case humanoid_app_gateway_runtime::ForwardRouteKind::Drop:
    default:
      return "drop";
  }
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

rapidjson::Value decision_to_json(
  const humanoid_app_gateway_runtime::ForwardRouteDecision & decision,
  rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value value(rapidjson::kObjectType);
  add_string(value, allocator, "kind", kind_to_string(decision.kind));
  add_string(value, allocator, "data_type", decision.data_type);
  add_string(value, allocator, "destination", decision.destination);
  rapidjson::Value clients(rapidjson::kArrayType);
  for (const auto & client : decision.target_clients) {
    clients.PushBack(rapidjson::Value(client.c_str(), allocator).Move(), allocator);
  }
  value.AddMember("target_clients", clients, allocator);
  value.AddMember("update_business_state", decision.update_business_state, allocator);
  add_string(value, allocator, "business_event", decision.business_event);
  return value;
}

}  // namespace

int main()
{
  humanoid_app_gateway_runtime::IntegrationForwarder forwarder;
  const std::vector<std::string> connected{"client_a", "client_b"};
  const std::vector<std::string> subscribed{"client_a", "client_missing"};

  rapidjson::Document document;
  document.SetObject();
  auto & allocator = document.GetAllocator();

  document.AddMember(
    "response_all",
    decision_to_json(forwarder.route_data_response(R"({"data_type":"system_status","destination":"all"})", connected), allocator),
    allocator);
  document.AddMember(
    "response_direct",
    decision_to_json(forwarder.route_data_response(R"({"data_type":"system_status","destination":"client_a"})", connected), allocator),
    allocator);
  document.AddMember(
    "response_missing",
    decision_to_json(forwarder.route_data_response(R"({"data_type":"system_status","destination":"client_x"})", connected), allocator),
    allocator);
  document.AddMember(
    "push_subscribed",
    decision_to_json(
      forwarder.route_push_message(
        R"({"data_type":"navigation_status_update","destination":"subscribed","data":{"event_type":"waypoint_reached"}})",
        connected,
        subscribed),
      allocator),
    allocator);
  document.AddMember(
    "push_gesture",
    decision_to_json(
      forwarder.route_push_message(R"({"data_type":"gesture_list","destination":"client_a","data":{}})", connected, subscribed),
      allocator),
    allocator);
  document.AddMember(
    "subscription_direct",
    decision_to_json(
      forwarder.route_subscription_response(R"({"data_type":"subscription_manage","destination":"client_b"})", connected),
      allocator),
    allocator);

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  std::cout << buffer.GetString() << std::endl;
  return 0;
}
