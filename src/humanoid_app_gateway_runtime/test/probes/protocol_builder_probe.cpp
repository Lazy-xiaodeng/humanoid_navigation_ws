/*
 * protocol_builder_probe.cpp
 *
 * 文件用途：
 * 1. 作为开发期协议组包验证工具，通过命令行选择要验证的组包模式。
 * 2. 从 stdin 读取 JSON 业务数据或原始 APP 请求，调用 ProtocolBuilder 输出 C++ 组包结果。
 * 3. 上游：协议一致性验证脚本或人工输入的测试 JSON。
 * 4. 下游：stdout，供自动化脚本验证协议字段是否符合线上协议语义。
 */

#include <iostream>
#include <iterator>
#include <string>

#include "humanoid_app_gateway_runtime/protocol_builder.hpp"

namespace
{

std::string stdin_to_string()
{
  return std::string{
    std::istreambuf_iterator<char>(std::cin),
    std::istreambuf_iterator<char>()};
}

void print_usage()
{
  std::cerr
    << "usage:\n"
    << "  protocol_builder_probe base <message_type> <data_type> <source> <destination> <timestamp> <message_id>\n"
    << "  protocol_builder_probe data_response <client_id> <request_id> <data_type> <timestamp> <last_update> <message_id>\n"
    << "  protocol_builder_probe subscription_response <success:0|1> <message> <timestamp> <message_id>\n"
    << "  protocol_builder_probe error_response <error_message> <error_code> <timestamp> <message_id>\n"
    << "  protocol_builder_probe specific_error <client_id> <request_id> <data_type> <error_code> <error_message> <timestamp> <message_id>\n"
    << "  protocol_builder_probe push <data_type> <client_id> <timestamp> <last_update> <subscription_count> <message_id>\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  humanoid_app_gateway_runtime::ProtocolBuilder builder;
  humanoid_app_gateway_runtime::ProtocolBuildResult result;

  if (argc < 2) {
    print_usage();
    return 2;
  }

  const std::string mode = argv[1];
  try {
    if (mode == "base" && argc == 8) {
      result = builder.create_base_message(
        argv[2], argv[3], argv[4], argv[5], std::stod(argv[6]), argv[7]);
    } else if (mode == "data_response" && argc == 8) {
      result = builder.create_data_response(
        argv[2], argv[3], argv[4], stdin_to_string(), std::stod(argv[5]), std::stod(argv[6]), argv[7]);
    } else if (mode == "subscription_response" && argc == 6) {
      result = builder.create_subscription_response(
        stdin_to_string(), std::string(argv[2]) == "1", argv[3], std::stod(argv[4]), argv[5]);
    } else if (mode == "error_response" && argc == 6) {
      result = builder.create_error_response(
        stdin_to_string(), argv[2], argv[3], std::stod(argv[4]), argv[5]);
    } else if (mode == "specific_error" && argc == 9) {
      result = builder.create_specific_error(
        argv[2], argv[3], argv[4], argv[5], argv[6], std::stod(argv[7]), argv[8]);
    } else if (mode == "push" && argc == 8) {
      result = builder.create_push_message(
        argv[2], stdin_to_string(), argv[3], std::stod(argv[4]), std::stod(argv[5]), std::stoi(argv[6]), argv[7]);
    } else {
      print_usage();
      return 2;
    }
  } catch (const std::exception & exc) {
    std::cerr << exc.what() << std::endl;
    return 2;
  }

  if (!result.ok) {
    std::cerr << result.error << std::endl;
    return 3;
  }

  std::cout << result.json << std::endl;
  return 0;
}
