/*
 * data_store.hpp
 *
 * 文件用途：
 * 1. 统一管理 data_storage、last_update_times、数据可用性和过期清理。
 * 2. 对齐现有数据整合链路的数据缓存、更新时间和 TTL 判断语义。
 * 3. 本模块只保存数据和时间戳，不关心 ROS topic 来源，也不关心 APP 协议格式。
 * 4. 上游：robot_status_adapter、navigation_status_adapter、map_waypoint_adapter、pose_speed_adapter、path_metrics。
 * 5. 下游：protocol_builder、subscription_manager 和 data_integration_node 请求响应链路。
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

struct StoredData
{
  // 数据内容，保持为 JSON 字符串，避免缓存层理解业务字段。
  std::string json;

  // 该数据最近一次更新的时间戳，单位秒。
  double updated_at_sec{0.0};
};

class DataStore
{
public:
  DataStore();

  // 设置某类数据的过期时间；未配置过期时间的数据按 线上协议 语义视为永久新鲜。
  void set_expiry(const std::string & data_type, double expiry_sec);

  // 写入或覆盖某类数据，同时更新 last_update_times。
  void set_data(const std::string & data_type, const std::string & json, double timestamp_sec);

  // 获取某类数据；不存在时返回空 optional。
  std::optional<StoredData> get_data(const std::string & data_type) const;

  // 一次性获取仍然新鲜的数据，避免调用方反复加锁做 available/fresh/get 三段判断。
  std::optional<StoredData> get_fresh_data(const std::string & data_type, double now_sec) const;

  // 保持线上协议 is_data_available()：存在且数据不是 None 即可用。
  bool is_data_available(const std::string & data_type) const;

  // 保持线上协议 is_data_fresh()：没有更新时间为 false；没有 TTL 配置则永久新鲜。
  bool is_data_fresh(const std::string & data_type, double now_sec) const;

  // 获取数据新鲜度，等价于 线上协议中 time.time() - last_update_times.get(data_type, fallback)。
  double data_freshness(const std::string & data_type, double now_sec, double fallback_update_sec) const;

  // 清理过期数据，返回本轮被清理的数据类型列表。
  std::vector<std::string> cleanup_expired(double now_sec);

  // 清空全部数据和更新时间，用于节点退出或测试复位。
  void clear();

  std::size_t size() const;

private:
  mutable std::mutex mutex_;
  std::unordered_map<std::string, StoredData> data_storage_;
  std::unordered_map<std::string, double> data_expiry_config_;
};

}  // namespace humanoid_app_gateway_runtime
