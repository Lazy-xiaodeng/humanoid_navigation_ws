/*
 * response_waiter_probe.cpp
 *
 * 文件用途：
 * 1. 作为开发期响应等待缓存验证工具，构造固定 guid 注册、完成、清理场景。
 * 2. 验证 response_events 等价语义：注册等待、按 guid 命中、读取响应、删除和超时清理。
 * 3. 上游：协议一致性验证脚本。
 * 4. 下游：stdout，供脚本验证响应等待管理结果是否符合线上协议语义。
 */

#include <iostream>

#include "humanoid_robot_gateway_runtime/response_waiter.hpp"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace
{

void add_size(
  rapidjson::Value & object,
  rapidjson::Document::AllocatorType & allocator,
  const char * key,
  const std::size_t value)
{
  object.AddMember(rapidjson::Value(key, allocator).Move(), static_cast<unsigned int>(value), allocator);
}

}  // namespace

int main()
{
  humanoid_robot_gateway_runtime::ResponseWaiter waiter;
  waiter.register_guid("guid_a", 10.0);
  waiter.register_guid("guid_b", 11.0);

  const bool hit = waiter.complete_response("guid_a", R"({"guid":"guid_a","title":"response_prepare"})");
  const bool miss = waiter.complete_response("guid_missing", R"({"guid":"guid_missing"})");
  const auto response = waiter.get_response("guid_a");
  const bool removed = waiter.remove_guid("guid_a");
  const auto expired = waiter.cleanup_expired(20.0, 5.0);

  rapidjson::Document document;
  document.SetObject();
  auto & allocator = document.GetAllocator();
  document.AddMember("hit", hit, allocator);
  document.AddMember("miss", miss, allocator);
  document.AddMember("guid_a_completed_before_remove", response.has_value(), allocator);

  rapidjson::Document response_doc;
  response_doc.Parse(response.value_or("{}").c_str());
  document.AddMember("guid_a_response", rapidjson::Value(response_doc, allocator), allocator);

  document.AddMember("removed_guid_a", removed, allocator);
  add_size(document, allocator, "expired_count", expired);
  add_size(document, allocator, "pending_count", waiter.pending_count());

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  std::cout << buffer.GetString() << std::endl;
  return 0;
}
