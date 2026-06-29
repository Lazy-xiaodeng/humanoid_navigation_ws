/*
 * client_registry.hpp
 *
 * 文件用途：
 * 1. 管理 APP WebSocket 客户端连接、客户端 ID、心跳和订阅关系。
 * 2. 对齐现有 APP 客户端会话、订阅关系和连接清理语义。
 * 3. 本模块只维护运行态，不解析业务命令，不直接访问 ROS publisher。
 * 4. 上游：APP WebSocket 连接、断开和订阅请求。
 * 5. 下游：integration_forwarder、订阅推送和连接健康检查。
 */

#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>
#include <mutex>

namespace humanoid_app_gateway_runtime
{

struct ClientSession
{
  std::string id;
  std::string ip{"unknown"};
  double connected_time{0.0};
  double last_activity{0.0};
  bool authenticated{false};
};

struct ClientSubscriptionInfo
{
  double frequency{1.0};
  double subscription_time{0.0};
};

class ClientRegistry
{
public:
  // 注册客户端连接，对齐 websocket_server.py handle_client_connection() 的会话字段。
  void register_client(const std::string & client_id, const std::string & ip, double timestamp_sec);

  // 更新客户端活跃时间。
  void update_activity(const std::string & client_id, double timestamp_sec);

  // 清理客户端连接，返回需要通知 data_integration 退订的数据类型列表。
  std::vector<std::string> cleanup_client(const std::string & client_id);

  // 按 线上协议 handle_subscription_request() 语义更新本地订阅缓存。
  void apply_subscription_request(
    const std::string & client_id,
    const std::string & action,
    const std::vector<std::string> & data_types,
    double push_frequency,
    double timestamp_sec);

  // 获取订阅某个 data_type 的客户端列表。
  std::vector<std::string> get_subscribed_clients(const std::string & data_type) const;

  // 获取某客户端的订阅类型列表。
  std::vector<std::string> get_client_subscriptions(const std::string & client_id) const;

  // 连接是否存在。
  bool has_client(const std::string & client_id) const;

  // 查询会话信息，便于状态报告和测试。
  std::optional<ClientSession> get_session(const std::string & client_id) const;

  // 找出超过 max_idle_sec 没有活动的客户端，供 WebSocket 外壳做连接健康清理。
  std::vector<std::string> find_inactive_clients(double now_sec, double max_idle_sec) const;

  std::size_t client_count() const;

  std::size_t subscription_count() const;

  void clear();

private:
  mutable std::mutex mutex_;
  std::unordered_map<std::string, ClientSession> client_sessions_;
  std::unordered_map<std::string, std::unordered_map<std::string, ClientSubscriptionInfo>> client_subscriptions_;
};

}  // namespace humanoid_app_gateway_runtime
