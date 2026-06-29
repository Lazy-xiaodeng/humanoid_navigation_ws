/*
 * response_waiter.cpp
 *
 * 文件用途：
 * 1. 实现机器人 WebSocket 请求 guid 与响应 JSON 的线程安全缓存关系。
 * 2. 对齐现有机器人网关 response_events 的注册、响应写入和清理语义。
 * 3. 本模块不直接阻塞线程；真正等待/超时由 robot_ws_client 或调用方封装。
 * 4. 上游：robot_ws_client 收到的 response 消息。
 * 5. 下游：同步命令调用、动作执行、动作库同步和模式切换等待。
 */

#include "humanoid_robot_gateway_runtime/response_waiter.hpp"

#include <vector>

namespace humanoid_robot_gateway_runtime
{

std::string ResponseWaiter::name() const { return "response_waiter"; }

void ResponseWaiter::register_guid(
  const std::string & guid,
  const double created_at_sec)
{
  std::lock_guard<std::mutex> lock(mutex_);
  responses_[guid] = PendingResponse{false, "", created_at_sec};
}

bool ResponseWaiter::complete_response(
  const std::string & guid,
  const std::string & response_json)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = responses_.find(guid);
    if (it == responses_.end()) {
      return false;
    }
    it->second.completed = true;
    it->second.response_json = response_json;
  }
  condition_.notify_all();
  return true;
}

bool ResponseWaiter::is_completed(const std::string & guid) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = responses_.find(guid);
  return it != responses_.end() && it->second.completed;
}

std::optional<std::string> ResponseWaiter::get_response(const std::string & guid) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = responses_.find(guid);
  if (it == responses_.end() || !it->second.completed) {
    return std::nullopt;
  }
  return it->second.response_json;
}

std::optional<std::string> ResponseWaiter::wait_for_response(
  const std::string & guid,
  const std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lock(mutex_);
  const auto ready = [this, &guid]() {
      const auto it = responses_.find(guid);
      return it == responses_.end() || it->second.completed;
    };
  if (!condition_.wait_for(lock, timeout, ready)) {
    return std::nullopt;
  }
  const auto it = responses_.find(guid);
  if (it == responses_.end() || !it->second.completed) {
    return std::nullopt;
  }
  return it->second.response_json;
}

bool ResponseWaiter::remove_guid(const std::string & guid)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return responses_.erase(guid) > 0;
}

std::size_t ResponseWaiter::cleanup_expired(
  const double now_sec,
  const double timeout_sec)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::string> expired;
  for (const auto & [guid, pending] : responses_) {
    if (!pending.completed && now_sec - pending.created_at_sec > timeout_sec) {
      expired.push_back(guid);
    }
  }
  for (const auto & guid : expired) {
    responses_.erase(guid);
  }
  return expired.size();
}

std::size_t ResponseWaiter::pending_count() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return responses_.size();
}

}  // namespace humanoid_robot_gateway_runtime
