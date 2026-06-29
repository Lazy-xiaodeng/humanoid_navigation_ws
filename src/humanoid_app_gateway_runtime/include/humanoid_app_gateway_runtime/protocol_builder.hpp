/*
 * protocol_builder.hpp
 *
 * 文件用途：
 * 1. 统一生成 data_response、push_message、subscription_response 和 error_response。
 * 2. 保证 APP 协议字段、message_id、metadata 和 timestamp 的结构一致。
 * 3. 对齐现有 integration response、push、subscription_response 和 error 组包语义。
 * 4. 上游：data_store、adapter 模块和 APP request/subscription 请求。
 * 5. 下游：/integration/data_responses、/integration/push_messages、/integration/subscription_responses。
 */

#pragma once

#include <string>

namespace humanoid_app_gateway_runtime
{

struct ProtocolBuildResult
{
  // 组包是否成功；输入 JSON 无法解析时为 false。
  bool ok{false};

  // 失败原因；成功时为空。
  std::string error;

  // 组包后的紧凑 JSON 字符串，保持线上协议 json.dumps(..., separators=(',', ':')) 的结构。
  std::string json;
};

class ProtocolBuilder
{
public:
  std::string protocol_version() const;

  // 创建协议基础结构。message_id 为空时自动生成，测试时可传固定值消除随机差异。
  ProtocolBuildResult create_base_message(
    const std::string & message_type,
    const std::string & data_type,
    const std::string & source,
    const std::string & destination,
    double timestamp_sec,
    const std::string & message_id = "") const;

  // 创建数据响应，保持线上协议 send_data_response() 组包语义。
  ProtocolBuildResult create_data_response(
    const std::string & client_id,
    const std::string & request_id,
    const std::string & data_type,
    const std::string & response_data_json,
    double timestamp_sec,
    double last_update_time_sec,
    const std::string & message_id = "") const;

  // 创建订阅响应，保持线上协议 send_subscription_response() 组包语义。
  ProtocolBuildResult create_subscription_response(
    const std::string & original_message_json,
    bool success,
    const std::string & message,
    double timestamp_sec,
    const std::string & message_id = "") const;

  // 创建通用错误响应，保持线上协议 send_error_response() 组包语义。
  ProtocolBuildResult create_error_response(
    const std::string & original_message_json,
    const std::string & error_message,
    const std::string & error_code,
    double timestamp_sec,
    const std::string & message_id = "") const;

  // 创建指定 client/request 的错误响应，保持线上协议 send_specific_error() 组包语义。
  ProtocolBuildResult create_specific_error(
    const std::string & client_id,
    const std::string & request_id,
    const std::string & data_type,
    const std::string & error_code,
    const std::string & error_message,
    double timestamp_sec,
    const std::string & message_id = "") const;

  // 创建订阅推送消息，保持线上协议 create_push_message() 组包语义。
  ProtocolBuildResult create_push_message(
    const std::string & data_type,
    const std::string & data_json,
    const std::string & client_id,
    double timestamp_sec,
    double last_update_time_sec,
    int subscription_count,
    const std::string & message_id = "") const;
};

}  // namespace humanoid_app_gateway_runtime
