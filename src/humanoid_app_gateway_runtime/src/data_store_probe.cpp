/*
 * data_store_probe.cpp
 *
 * 文件用途：
 * 1. 作为开发期数据缓存验证工具，构造固定缓存场景并输出检查结果。
 * 2. 验证 DataStore 的可用性、新鲜度、TTL 清理、未知类型永久新鲜等语义。
 * 3. 上游：协议一致性验证脚本。
 * 4. 下游：stdout，供脚本验证缓存行为是否符合线上协议 data_storage 逻辑。
 */

#include <iostream>
#include <string>

#include "humanoid_app_gateway_runtime/data_store.hpp"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace
{

void add_bool(
  rapidjson::Value & object,
  rapidjson::Document::AllocatorType & allocator,
  const char * key,
  const bool value)
{
  object.AddMember(rapidjson::Value(key, allocator).Move(), value, allocator);
}

void add_number(
  rapidjson::Value & object,
  rapidjson::Document::AllocatorType & allocator,
  const char * key,
  const double value)
{
  object.AddMember(rapidjson::Value(key, allocator).Move(), value, allocator);
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
  constexpr double now = 100.0;

  humanoid_app_gateway_runtime::DataStore store;
  store.set_data("robot_speed", "{\"linear_speed\":0.1}", 99.5);
  store.set_data("robot_pose", "{\"x\":1.0}", 90.0);
  store.set_data("custom_data", "{\"hello\":\"world\"}", 1.0);
  store.set_data("null_data", "null", 100.0);

  rapidjson::Document document;
  document.SetObject();
  auto & allocator = document.GetAllocator();

  add_bool(document, allocator, "speed_available", store.is_data_available("robot_speed"));
  add_bool(document, allocator, "speed_fresh", store.is_data_fresh("robot_speed", now));
  add_number(document, allocator, "speed_freshness", store.data_freshness("robot_speed", now, now));
  add_bool(document, allocator, "pose_available_before_cleanup", store.is_data_available("robot_pose"));
  add_bool(document, allocator, "pose_fresh_before_cleanup", store.is_data_fresh("robot_pose", now));
  add_bool(document, allocator, "custom_fresh", store.is_data_fresh("custom_data", now));
  add_bool(document, allocator, "missing_available", store.is_data_available("missing"));
  add_bool(document, allocator, "missing_fresh", store.is_data_fresh("missing", now));
  add_number(document, allocator, "missing_freshness", store.data_freshness("missing", now, now));
  add_bool(document, allocator, "null_available", store.is_data_available("null_data"));
  add_size(document, allocator, "size_before_cleanup", store.size());

  const auto expired = store.cleanup_expired(now);
  rapidjson::Value expired_array(rapidjson::kArrayType);
  for (const auto & data_type : expired) {
    expired_array.PushBack(rapidjson::Value(data_type.c_str(), allocator).Move(), allocator);
  }
  document.AddMember("expired", expired_array, allocator);
  add_size(document, allocator, "size_after_cleanup", store.size());
  add_bool(document, allocator, "pose_available_after_cleanup", store.is_data_available("robot_pose"));

  store.clear();
  add_size(document, allocator, "size_after_clear", store.size());

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  std::cout << buffer.GetString() << std::endl;
  return 0;
}
