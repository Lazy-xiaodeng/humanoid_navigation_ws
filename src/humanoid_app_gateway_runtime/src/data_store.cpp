/*
 * data_store.cpp
 *
 * 文件用途：
 * 1. 实现统一数据缓存、更新时间和 TTL 过期判断。
 * 2. 对齐现有 data_storage、last_update_times、data_expiry_config、可用性/新鲜度判断和过期清理核心语义。
 * 3. 上游：robot_status_adapter、navigation_status_adapter、map_waypoint_adapter、
 *    pose_speed_adapter、path_metrics 等数据转换模块。
 * 4. 下游：protocol_builder、subscription_manager 和 data_integration_node 请求响应链路。
 */

#include "humanoid_app_gateway_runtime/data_store.hpp"

#include <limits>

namespace humanoid_app_gateway_runtime
{

DataStore::DataStore()
{
  // 默认 TTL 与当前数据整合链路保持一致；后续可继续接入 YAML 覆盖。
  data_expiry_config_["robot_pose"] = 5.0;
  data_expiry_config_["robot_speed"] = 1.0;
  data_expiry_config_["odom_raw"] = 5.0;
  data_expiry_config_["navigation_status"] = 10.0;
  data_expiry_config_["navigation_path"] = 30.0;
  data_expiry_config_["system_status"] = 60.0;
  data_expiry_config_["action_result"] = 30.0;
  data_expiry_config_["map_status"] = 15.0;
  data_expiry_config_["map_response"] = 60.0;
  data_expiry_config_["waypoints_data"] = 600.0;
  data_expiry_config_["system_exception"] = 120.0;
  data_expiry_config_["sensor_data"] = 2.0;
}

void DataStore::set_expiry(const std::string & data_type, const double expiry_sec)
{
  std::lock_guard<std::mutex> lock(mutex_);
  data_expiry_config_[data_type] = expiry_sec;
}

void DataStore::set_data(
  const std::string & data_type,
  const std::string & json,
  const double timestamp_sec)
{
  std::lock_guard<std::mutex> lock(mutex_);
  data_storage_[data_type] = StoredData{json, timestamp_sec};
}

std::optional<StoredData> DataStore::get_data(const std::string & data_type) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = data_storage_.find(data_type);
  if (it == data_storage_.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::optional<StoredData> DataStore::get_fresh_data(
  const std::string & data_type,
  const double now_sec) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto data_it = data_storage_.find(data_type);
  if (data_it == data_storage_.end() || data_it->second.json == "null") {
    return std::nullopt;
  }

  const auto expiry_it = data_expiry_config_.find(data_type);
  if (expiry_it != data_expiry_config_.end() &&
    now_sec - data_it->second.updated_at_sec > expiry_it->second)
  {
    return std::nullopt;
  }
  return data_it->second;
}

bool DataStore::is_data_available(const std::string & data_type) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = data_storage_.find(data_type);
  if (it == data_storage_.end()) {
    return false;
  }

  // 缓存中显式 "null" 表示数据不可用，避免把空值误当作有效业务数据。
  return it->second.json != "null";
}

bool DataStore::is_data_fresh(const std::string & data_type, const double now_sec) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto data_it = data_storage_.find(data_type);
  if (data_it == data_storage_.end()) {
    return false;
  }

  const auto expiry_it = data_expiry_config_.find(data_type);
  if (expiry_it == data_expiry_config_.end()) {
    return true;
  }

  return now_sec - data_it->second.updated_at_sec <= expiry_it->second;
}

double DataStore::data_freshness(
  const std::string & data_type,
  const double now_sec,
  const double fallback_update_sec) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = data_storage_.find(data_type);
  const double updated_at = it == data_storage_.end() ? fallback_update_sec : it->second.updated_at_sec;
  return now_sec - updated_at;
}

std::vector<std::string> DataStore::cleanup_expired(const double now_sec)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> expired_types;

  for (const auto & [data_type, data] : data_storage_) {
    const auto expiry_it = data_expiry_config_.find(data_type);
    const double expiry_sec =
      expiry_it == data_expiry_config_.end() ? std::numeric_limits<double>::infinity() : expiry_it->second;
    if (now_sec - data.updated_at_sec > expiry_sec) {
      expired_types.push_back(data_type);
    }
  }

  for (const auto & data_type : expired_types) {
    data_storage_.erase(data_type);
  }
  return expired_types;
}

void DataStore::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  data_storage_.clear();
}

std::size_t DataStore::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return data_storage_.size();
}

}  // namespace humanoid_app_gateway_runtime
