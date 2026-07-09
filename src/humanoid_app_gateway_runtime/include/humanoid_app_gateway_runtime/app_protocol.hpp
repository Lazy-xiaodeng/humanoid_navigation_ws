/*
 * app_protocol.hpp
 *
 * 文件用途：
 * 1. 定义 APP 协议解析、校验和基础消息字段处理边界。
 * 2. 对齐现有 APP 协议校验、基础消息字段和订阅请求校验语义。
 * 3. 本模块只处理协议语义，不直接发布 ROS topic，也不持有 WebSocket 连接。
 * 4. 上游：APP / 导航页 WebSocket JSON 消息。
 * 5. 下游：business_command_router、integration_forwarder 和 data request 转发链路。
 */

#pragma once

#include <string>

namespace humanoid_app_gateway_runtime
{

struct ProtocolValidationResult
{
  // 是否通过校验。
  bool valid{false};

  // 失败原因；成功时为空。
  std::string error;

  // 非致命警告，例如协议版本不一致但仍允许兼容。
  std::string warning;
};

class AppProtocol
{
public:
  std::string name() const;

  // 保持线上协议 validate_request_message()。
  ProtocolValidationResult validate_request_message(const std::string & message_json) const;

  // 保持线上协议 validate_subscription_message()。
  ProtocolValidationResult validate_subscription_message(const std::string & message_json) const;
};

}  // namespace humanoid_app_gateway_runtime
