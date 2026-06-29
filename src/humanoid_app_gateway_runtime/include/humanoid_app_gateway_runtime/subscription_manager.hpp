/*
 * subscription_manager.hpp
 *
 * 文件用途：
 * 1. 管理 APP 对 system_status、navigation_status、map_status 等数据类型的订阅关系。
 * 2. 负责订阅频率、last_push_time 和订阅取消。
 * 3. 对齐现有订阅关系、推送频率节流和取消订阅语义。
 * 4. 上游：/websocket/data_subscriptions。
 * 5. 下游：data_integration_node 的 push_data_updates 和 /integration/push_messages。
 */

#pragma once

#include <cstddef>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace humanoid_app_gateway_runtime
{

struct SubscriptionInfo
{
  std::string client_id;
  double frequency{1.0};
  double last_push_time{0.0};
  std::string subscription_info_json{"{}"};
  bool active{true};
  double subscription_time{0.0};
};

struct SubscriptionStatistics
{
  std::size_t total_data_types{0};
  std::size_t total_subscriptions{0};
  std::unordered_map<std::string, std::size_t> clients_by_type;
};

class SubscriptionManager
{
public:
  // 添加或覆盖订阅；保持线上协议 subscribe()，同一 data_type/client_id 后写覆盖前写。
  bool subscribe(
    const std::string & client_id,
    const std::vector<std::string> & data_types,
    double frequency,
    const std::string & subscription_info_json,
    double subscription_time_sec);

  // 取消订阅；data_types 为空时，保持线上协议 语义，取消该客户端所有订阅。
  bool unsubscribe(
    const std::string & client_id,
    const std::optional<std::vector<std::string>> & data_types = std::nullopt);

  // 获取当前达到推送频率条件的活跃订阅者。
  std::vector<SubscriptionInfo> get_subscribers(
    const std::string & data_type,
    double current_time_sec) const;

  // 更新某个客户端某类数据的最近推送时间。
  void update_push_time(
    const std::string & client_id,
    const std::string & data_type,
    double current_time_sec);

  // 获取某客户端当前订阅的全部数据类型。
  std::vector<std::string> get_client_subscriptions(const std::string & client_id) const;

  // 移除某客户端所有订阅。
  void remove_client(const std::string & client_id);

  // 获取统计信息，保持线上协议 get_statistics()。
  SubscriptionStatistics get_statistics() const;

  std::size_t active_count() const;

private:
  mutable std::mutex mutex_;
  std::unordered_map<std::string, std::unordered_map<std::string, SubscriptionInfo>> subscriptions_;
};

}  // namespace humanoid_app_gateway_runtime
