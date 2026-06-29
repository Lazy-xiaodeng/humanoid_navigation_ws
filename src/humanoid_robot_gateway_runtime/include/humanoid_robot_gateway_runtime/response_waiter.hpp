/*
 * response_waiter.hpp
 *
 * 文件用途：
 * 1. 管理机器人 WebSocket 请求 guid 与响应等待关系。
 * 2. 对齐现有请求响应等待关系：注册 guid、接收响应、读取结果和超时清理。
 * 3. 本模块是线程/异步安全重点，后续实现必须明确锁和超时行为。
 * 4. 上游：robot_ws_client 收到的 response 消息。
 * 5. 下游：同步命令调用、动作执行、动作库同步和模式切换等待。
 */

#pragma once

#include <condition_variable>
#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

namespace humanoid_robot_gateway_runtime
{

struct PendingResponse
{
  bool completed{false};
  std::string response_json;
  double created_at_sec{0.0};
};

class ResponseWaiter
{
public:
  std::string name() const;

  // 注册一个等待响应的 guid，保持线上协议 self.response_events[guid] = {"event": event, "response": None}。
  void register_guid(const std::string & guid, double created_at_sec);

  // 收到机器人消息后按 guid 写入响应；返回是否命中已注册 guid。
  bool complete_response(const std::string & guid, const std::string & response_json);

  // 查询 guid 是否完成。
  bool is_completed(const std::string & guid) const;

  // 获取响应 JSON；未完成或不存在时为空。
  std::optional<std::string> get_response(const std::string & guid) const;

  // 等待指定 guid 的响应直到超时；收到响应会立即唤醒，避免调用方 sleep 轮询。
  std::optional<std::string> wait_for_response(const std::string & guid, std::chrono::milliseconds timeout);

  // 清理一个 guid，保持线上协议 finally 删除 response_events[guid]。
  bool remove_guid(const std::string & guid);

  // 清理超时未完成的等待项，返回清理数量。
  std::size_t cleanup_expired(double now_sec, double timeout_sec);

  std::size_t pending_count() const;

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::unordered_map<std::string, PendingResponse> responses_;
};

}  // namespace humanoid_robot_gateway_runtime
