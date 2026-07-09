/*
 * integration_forwarder.cpp
 *
 * 文件用途：
 * 1. 实现 integration 数据响应、主动推送、订阅响应到 APP 客户端的路由决策。
 * 2. 对齐现有数据响应、主动推送和订阅响应的客户端路由核心语义。
 * 3. 上游：/integration/data_responses、/integration/push_messages、/integration/subscription_responses。
 * 4. 下游：APP / 导航页 WebSocket 客户端。
 */

#include "humanoid_app_gateway_runtime/integration_forwarder.hpp"

#include <algorithm>
#include <string>

#include "rapidjson/document.h"

namespace humanoid_app_gateway_runtime
{

std::string IntegrationForwarder::name() const { return "integration_forwarder"; }

namespace
{

bool contains_client(const std::vector<std::string> & clients, const std::string & client_id)
{
  return std::find(clients.begin(), clients.end(), client_id) != clients.end();
}

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

std::string normalize_data_type(std::string data_type)
{
  const std::string suffix = "_update";
  if (data_type.size() >= suffix.size() &&
    data_type.compare(data_type.size() - suffix.size(), suffix.size(), suffix) == 0)
  {
    data_type.erase(data_type.size() - suffix.size());
  }
  return data_type;
}

bool parse_object(const std::string & json, rapidjson::Document & document)
{
  document.Parse(json.c_str());
  return !document.HasParseError() && document.IsObject();
}

ForwardRouteDecision direct_or_drop(
  const std::string & destination,
  const std::string & data_type,
  const std::vector<std::string> & connected_clients)
{
  ForwardRouteDecision decision;
  decision.destination = destination;
  decision.data_type = data_type;
  if (contains_client(connected_clients, destination)) {
    decision.kind = ForwardRouteKind::DirectClient;
    decision.target_clients = {destination};
  }
  return decision;
}

}  // namespace

ForwardRouteDecision IntegrationForwarder::route_data_response(
  const std::string & response_json,
  const std::vector<std::string> & connected_clients) const
{
  rapidjson::Document response;
  ForwardRouteDecision decision;
  if (!parse_object(response_json, response)) {
    return decision;
  }

  const std::string destination = value_to_string(find_member(response, "destination"), "all");
  const std::string data_type = value_to_string(find_member(response, "data_type"));
  decision.destination = destination;
  decision.data_type = data_type;

  if (destination == "all") {
    decision.kind = ForwardRouteKind::BroadcastAll;
    return decision;
  }
  return direct_or_drop(destination, data_type, connected_clients);
}

ForwardRouteDecision IntegrationForwarder::route_push_message(
  const std::string & push_json,
  const std::vector<std::string> & connected_clients,
  const std::vector<std::string> & subscribed_clients) const
{
  rapidjson::Document push;
  ForwardRouteDecision decision;
  if (!parse_object(push_json, push)) {
    return decision;
  }

  const auto * data = find_member(push, "data");
  const std::string data_type = normalize_data_type(value_to_string(find_member(push, "data_type")));
  const std::string destination = value_to_string(find_member(push, "destination"), "all");
  decision.destination = destination;
  decision.data_type = data_type;

  if (data_type == "navigation_status" && data != nullptr && data->IsObject()) {
    const std::string event_type = value_to_string(find_member(*data, "event_type"));
    if (event_type == "navigation_started" ||
      event_type == "waypoint_reached" ||
      event_type == "navigation_completed" ||
      event_type == "navigation_stopped")
    {
      decision.update_business_state = true;
      decision.business_event = event_type;
    }
  } else if (data_type == "gesture_list" || data_type == "facial_gesture_list") {
    decision.update_business_state = true;
    decision.business_event = data_type;
    decision.kind = ForwardRouteKind::BroadcastAll;
    return decision;
  }

  if (destination == "subscribed") {
    decision.kind = ForwardRouteKind::SubscribedClients;
    for (const auto & client_id : subscribed_clients) {
      if (contains_client(connected_clients, client_id)) {
        decision.target_clients.push_back(client_id);
      }
    }
    return decision;
  }
  if (destination == "all") {
    decision.kind = ForwardRouteKind::BroadcastAll;
    return decision;
  }
  return direct_or_drop(destination, data_type, connected_clients);
}

ForwardRouteDecision IntegrationForwarder::route_subscription_response(
  const std::string & response_json,
  const std::vector<std::string> & connected_clients) const
{
  rapidjson::Document response;
  ForwardRouteDecision decision;
  if (!parse_object(response_json, response)) {
    return decision;
  }

  const std::string destination = value_to_string(find_member(response, "destination"), "all");
  const std::string data_type = value_to_string(find_member(response, "data_type"));
  return direct_or_drop(destination, data_type, connected_clients);
}

}  // namespace humanoid_app_gateway_runtime
