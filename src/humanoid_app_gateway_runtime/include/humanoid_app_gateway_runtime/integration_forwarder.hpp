/*
 * integration_forwarder.hpp
 *
 * 文件用途：
 * 1. 负责把 integration 系列话题的响应、主动推送和订阅响应转发给 APP 客户端。
 * 2. 对齐现有 integration response、push 和 subscription_response 转发语义。
 * 3. 本模块不生成业务数据，只负责根据 client_id / subscription 信息选择发送目标。
 * 4. 上游：/integration/data_responses、/integration/push_messages、/integration/subscription_responses。
 * 5. 下游：APP / 导航页 WebSocket 客户端。
 */

#pragma once

#include <string>
#include <vector>

namespace humanoid_app_gateway_runtime
{

enum class ForwardRouteKind
{
  BroadcastAll,
  DirectClient,
  SubscribedClients,
  Drop
};

struct ForwardRouteDecision
{
  // 路由方式：广播、指定客户端、订阅客户端或丢弃。
  ForwardRouteKind kind{ForwardRouteKind::Drop};

  // 标准化后的 data_type，线上协议中会去掉 "_update" 后缀。
  std::string data_type;

  // 原始 destination 字段。
  std::string destination{"all"};

  // 需要发送的客户端列表；广播时为空，由调用方发送给所有连接。
  std::vector<std::string> target_clients;

  // 是否需要同步 websocket_server 内部 business_state。
  bool update_business_state{false};

  // business_state 更新类型，例如 navigation_started、waypoint_reached、gesture_list。
  std::string business_event;
};

class IntegrationForwarder
{
public:
  std::string name() const;

  // 保持线上协议 route_data_response() 的路由决策。
  ForwardRouteDecision route_data_response(
    const std::string & response_json,
    const std::vector<std::string> & connected_clients) const;

  // 保持线上协议 unified_push_message_callback()/route_push_message() 的路由决策。
  ForwardRouteDecision route_push_message(
    const std::string & push_json,
    const std::vector<std::string> & connected_clients,
    const std::vector<std::string> & subscribed_clients) const;

  // 保持线上协议 route_subscription_response() 的路由决策。
  ForwardRouteDecision route_subscription_response(
    const std::string & response_json,
    const std::vector<std::string> & connected_clients) const;
};

}  // namespace humanoid_app_gateway_runtime
