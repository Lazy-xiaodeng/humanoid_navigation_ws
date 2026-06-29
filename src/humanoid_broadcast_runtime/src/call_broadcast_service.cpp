/*
 * call_broadcast_service.cpp
 *
 * 文件作用：
 * 1. 提供播报服务命令行客户端，便于不依赖历史播报包时仍可手动测试播报链路。
 * 2. 从环境变量读取播报文本、播报 ID、点位 ID、音量和等待超时时间。
 * 3. 调用 /xiaorui_broadcast/play，并把服务返回结果打印到终端。
 *
 * 上游：
 * - 现场调试人员或 shell 脚本。
 *
 * 下游：
 * - xiaorui_broadcast_service，也就是 broadcast_service_node_cpp。
 */

#include <rclcpp/rclcpp.hpp>

#include <humanoid_interfaces/srv/play_broadcast.hpp>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

namespace
{

std::string getenv_or(const char * name, const std::string & fallback)
{
  const char * value = std::getenv(name);
  if (value == nullptr) {
    return fallback;
  }
  return std::string(value);
}

int getenv_int_or(const char * name, int fallback)
{
  const char * value = std::getenv(name);
  if (value == nullptr) {
    return fallback;
  }
  try {
    return std::stoi(value);
  } catch (...) {
    return fallback;
  }
}

double getenv_double_or(const char * name, double fallback)
{
  const char * value = std::getenv(name);
  if (value == nullptr) {
    return fallback;
  }
  try {
    return std::stod(value);
  } catch (...) {
    return fallback;
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("xiaorui_broadcast_service_client");
  auto client = node->create_client<humanoid_interfaces::srv::PlayBroadcast>("/xiaorui_broadcast/play");

  const double timeout_sec = getenv_double_or("XIAORUI_BROADCAST_SERVICE_TIMEOUT_SEC", 120.0);
  if (!client->wait_for_service(std::chrono::duration<double>(timeout_sec))) {
    std::cerr << "/xiaorui_broadcast/play service is not available" << std::endl;
    rclcpp::shutdown();
    return 3;
  }

  auto request = std::make_shared<humanoid_interfaces::srv::PlayBroadcast::Request>();
  request->text = getenv_or("XIAORUI_BROADCAST_TEXT", "");
  request->broadcast_id = getenv_or("XIAORUI_BROADCAST_ID", "");
  request->waypoint_id = getenv_or("XIAORUI_BROADCAST_WAYPOINT_ID", "");
  request->volume_percent = getenv_int_or("XIAORUI_BROADCAST_VOLUME", 72);
  request->use_request_volume = true;

  if (request->text.empty()) {
    std::cerr << "XIAORUI_BROADCAST_TEXT is empty" << std::endl;
    rclcpp::shutdown();
    return 2;
  }

  auto future = client->async_send_request(request);
  const auto result = rclcpp::spin_until_future_complete(
    node, future, std::chrono::duration<double>(timeout_sec));
  if (result != rclcpp::FutureReturnCode::SUCCESS) {
    std::cerr << "broadcast service call timed out" << std::endl;
    rclcpp::shutdown();
    return 4;
  }

  const auto response = future.get();
  if (!response->success) {
    std::cerr << (response->message.empty() ? "broadcast playback failed" : response->message) << std::endl;
    rclcpp::shutdown();
    return 5;
  }

  std::cout << "broadcast completed: duration=" << response->duration_sec
            << "s backend=" << response->backend
            << " device=" << response->selected_device << std::endl;
  rclcpp::shutdown();
  return 0;
}
