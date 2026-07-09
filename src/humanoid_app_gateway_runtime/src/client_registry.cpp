/*
 * client_registry.cpp
 *
 * 文件用途：
 * 1. 实现 APP 客户端连接表、客户端会话和本地订阅缓存管理。
 * 2. 对齐现有 connected_clients、client_sessions、client_subscriptions、cleanup_client、
 *    订阅请求处理和订阅客户端查询核心状态语义。
 * 3. 上游：APP WebSocket 连接、断开和订阅请求。
 * 4. 下游：integration_forwarder、订阅推送和连接健康检查。
 */

#include "humanoid_app_gateway_runtime/client_registry.hpp"

#include <algorithm>

namespace humanoid_app_gateway_runtime
{

void ClientRegistry::register_client(
  const std::string & client_id,
  const std::string & ip,
  const double timestamp_sec)
{
  std::lock_guard<std::mutex> lock(mutex_);
  client_sessions_[client_id] = ClientSession{
    client_id,
    ip.empty() ? "unknown" : ip,
    timestamp_sec,
    timestamp_sec,
    false};
}

void ClientRegistry::update_activity(
  const std::string & client_id,
  const double timestamp_sec)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = client_sessions_.find(client_id);
  if (it != client_sessions_.end()) {
    it->second.last_activity = timestamp_sec;
  }
}

std::vector<std::string> ClientRegistry::cleanup_client(const std::string & client_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> subscriptions_to_remove;

  client_sessions_.erase(client_id);
  const auto sub_it = client_subscriptions_.find(client_id);
  if (sub_it != client_subscriptions_.end()) {
    for (const auto & [data_type, info] : sub_it->second) {
      (void)info;
      subscriptions_to_remove.push_back(data_type);
    }
    client_subscriptions_.erase(sub_it);
  }
  std::sort(subscriptions_to_remove.begin(), subscriptions_to_remove.end());
  return subscriptions_to_remove;
}

void ClientRegistry::apply_subscription_request(
  const std::string & client_id,
  const std::string & action,
  const std::vector<std::string> & data_types,
  const double push_frequency,
  const double timestamp_sec)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (action == "subscribe") {
    auto & subscriptions = client_subscriptions_[client_id];
    for (const auto & data_type : data_types) {
      subscriptions[data_type] = ClientSubscriptionInfo{push_frequency, timestamp_sec};
    }
    return;
  }

  if (action == "unsubscribe") {
    if (data_types.empty()) {
      client_subscriptions_.erase(client_id);
      return;
    }

    const auto sub_it = client_subscriptions_.find(client_id);
    if (sub_it == client_subscriptions_.end()) {
      return;
    }
    for (const auto & data_type : data_types) {
      sub_it->second.erase(data_type);
    }
    if (sub_it->second.empty()) {
      client_subscriptions_.erase(sub_it);
    }
  }
}

std::vector<std::string> ClientRegistry::get_subscribed_clients(
  const std::string & data_type) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> subscribed_clients;
  for (const auto & [client_id, subscriptions] : client_subscriptions_) {
    if (subscriptions.find(data_type) != subscriptions.end()) {
      subscribed_clients.push_back(client_id);
    }
  }
  std::sort(subscribed_clients.begin(), subscribed_clients.end());
  return subscribed_clients;
}

std::vector<std::string> ClientRegistry::get_client_subscriptions(
  const std::string & client_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> subscriptions;
  const auto sub_it = client_subscriptions_.find(client_id);
  if (sub_it == client_subscriptions_.end()) {
    return subscriptions;
  }
  for (const auto & [data_type, info] : sub_it->second) {
    (void)info;
    subscriptions.push_back(data_type);
  }
  std::sort(subscriptions.begin(), subscriptions.end());
  return subscriptions;
}

bool ClientRegistry::has_client(const std::string & client_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return client_sessions_.find(client_id) != client_sessions_.end();
}

std::optional<ClientSession> ClientRegistry::get_session(const std::string & client_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = client_sessions_.find(client_id);
  if (it == client_sessions_.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::vector<std::string> ClientRegistry::find_inactive_clients(
  const double now_sec,
  const double max_idle_sec) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> inactive_clients;
  for (const auto & [client_id, session] : client_sessions_) {
    if (max_idle_sec > 0.0 && now_sec - session.last_activity > max_idle_sec) {
      inactive_clients.push_back(client_id);
    }
  }
  std::sort(inactive_clients.begin(), inactive_clients.end());
  return inactive_clients;
}

std::size_t ClientRegistry::client_count() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return client_sessions_.size();
}

std::size_t ClientRegistry::subscription_count() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::size_t count = 0;
  for (const auto & [client_id, subscriptions] : client_subscriptions_) {
    (void)client_id;
    count += subscriptions.size();
  }
  return count;
}

void ClientRegistry::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  client_sessions_.clear();
  client_subscriptions_.clear();
}

}  // namespace humanoid_app_gateway_runtime
