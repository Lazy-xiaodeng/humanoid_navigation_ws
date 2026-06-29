/*
 * app_protocol_probe.cpp
 *
 * 文件用途：
 * 1. 作为开发期 APP 协议校验工具，通过命令行选择 request/subscription 校验模式。
 * 2. 从 stdin 读取 APP JSON 消息，输出 C++ 校验结果。
 * 3. 上游：协议一致性验证脚本或人工输入的 APP 消息。
 * 4. 下游：stdout，供脚本验证校验语义是否符合线上协议。
 */

#include <iostream>
#include <iterator>
#include <string>

#include "humanoid_app_gateway_runtime/app_protocol.hpp"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace
{

std::string stdin_to_string()
{
  return std::string{
    std::istreambuf_iterator<char>(std::cin),
    std::istreambuf_iterator<char>()};
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: app_protocol_probe <request|subscription>" << std::endl;
    return 2;
  }

  const std::string mode = argv[1];
  humanoid_app_gateway_runtime::AppProtocol protocol;
  humanoid_app_gateway_runtime::ProtocolValidationResult result;
  if (mode == "request") {
    result = protocol.validate_request_message(stdin_to_string());
  } else if (mode == "subscription") {
    result = protocol.validate_subscription_message(stdin_to_string());
  } else {
    std::cerr << "unknown mode: " << mode << std::endl;
    return 2;
  }

  rapidjson::Document document;
  document.SetObject();
  auto & allocator = document.GetAllocator();
  document.AddMember("valid", result.valid, allocator);
  document.AddMember(
    "error",
    rapidjson::Value(result.error.c_str(), allocator).Move(),
    allocator);
  document.AddMember(
    "warning",
    rapidjson::Value(result.warning.c_str(), allocator).Move(),
    allocator);

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  std::cout << buffer.GetString() << std::endl;
  return 0;
}
