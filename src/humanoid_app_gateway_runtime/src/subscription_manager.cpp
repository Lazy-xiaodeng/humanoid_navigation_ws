/*
 * subscription_manager.cpp
 *
 * 文件用途：
 * 1. 实现 APP 数据订阅关系、取消订阅、推送频率节流和订阅统计。
 * 2. 对齐现有订阅管理器的 subscribe、unsubscribe、get_subscribers、update_push_time、
 *    get_client_subscriptions、remove_client 和 get_statistics 语义。
 * 3. 上游：/websocket/data_subscriptions。
 * 4. 下游：data_integration_node 的 push_data_updates 和 /integration/push_messages。
 */

#include "humanoid_app_gateway_runtime/subscription_manager.hpp"

#include <limits>

namespace humanoid_app_gateway_runtime
{

bool SubscriptionManager::subscribe(
  const std::string & client_id,
  const std::vector<std::string> & data_types,
  const double frequency,
  const std::string & subscription_info_json,
  const double subscription_time_sec)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & data_type : data_types) {
    subscriptions_[data_type][client_id] = SubscriptionInfo{
      client_id,
      frequency,
      0.0,
      subscription_info_json.empty() ? "{}" : subscription_info_json,
      true,
      subscription_time_sec};
  }
  return true;
}

bool SubscriptionManager::unsubscribe(
  const std::string & client_id,
  const std::optional<std::vector<std::string>> & data_types)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!data_types.has_value() || data_types->empty()) {
    for (auto & [data_type, clients] : subscriptions_) {
      (void)data_type;
      clients.erase(client_id);
    }
    return true;
  }

  for (const auto & data_type : *data_types) {
    const auto it = subscriptions_.find(data_type);
    if (it != subscriptions_.end()) {
      it->second.erase(client_id);
    }
  }
  return true;
}

std::vector<SubscriptionInfo> SubscriptionManager::get_subscribers(
  const std::string & data_type,
  const double current_time_sec) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<SubscriptionInfo> active_subscribers;

  const auto data_it = subscriptions_.find(data_type);
  if (data_it == subscriptions_.end()) {
    return active_subscribers;
  }

  for (const auto & [client_id, info] : data_it->second) {
    (void)client_id;
    if (!info.active) {
      continue;
    }

    const double time_since_last_push = current_time_sec - info.last_push_time;
    const double min_interval =
      info.frequency > 0.0 ? 1.0 / info.frequency : std::numeric_limits<double>::infinity();
    if (time_since_last_push >= min_interval) {
      active_subscribers.push_back(info);
    }
  }

  return active_subscribers;
}

void SubscriptionManager::update_push_time(
  const std::string & client_id,
  const std::string & data_type,
  const double current_time_sec)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto data_it = subscriptions_.find(data_type);
  if (data_it == subscriptions_.end()) {
    return;
  }

  const auto client_it = data_it->second.find(client_id);
  if (client_it != data_it->second.end()) {
    client_it->second.last_push_time = current_time_sec;
  }
}

std::vector<std::string> SubscriptionManager::get_client_subscriptions(
  const std::string & client_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> subscribed_types;

  for (const auto & [data_type, clients] : subscriptions_) {
    if (clients.find(client_id) != clients.end()) {
      subscribed_types.push_back(data_type);
    }
  }

  return subscribed_types;
}

void SubscriptionManager::remove_client(const std::string & client_id)
{
  unsubscribe(client_id);
}

SubscriptionStatistics SubscriptionManager::get_statistics() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  SubscriptionStatistics stats;
  stats.total_data_types = subscriptions_.size();

  for (const auto & [data_type, clients] : subscriptions_) {
    stats.total_subscriptions += clients.size();
    stats.clients_by_type[data_type] = clients.size();
  }

  return stats;
}

std::size_t SubscriptionManager::active_count() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::size_t count = 0;
  for (const auto & [data_type, clients] : subscriptions_) {
    (void)data_type;
    for (const auto & [client_id, info] : clients) {
      (void)client_id;
      if (info.active) {
        ++count;
      }
    }
  }
  return count;
}

}  // namespace humanoid_app_gateway_runtime
