/*
 * robot_status_adapter_probe.cpp
 *
 * 文件用途：
 * 1. 作为开发期验证工具，从 stdin 读取一段 /robot_status_raw JSON。
 * 2. 调用 RobotStatusAdapter 输出 C++ 转换后的 system_status JSON。
 * 3. 上游：一致性验证脚本或人工输入的 raw 状态。
 * 4. 下游：stdout，供 字段一致性验证脚本读取。
 */

#include <iostream>
#include <iterator>
#include <string>

#include "humanoid_app_gateway_runtime/robot_status_adapter.hpp"

int main(int argc, char ** argv)
{
  double timestamp = 123456.0;
  if (argc > 1) {
    timestamp = std::stod(argv[1]);
  }

  const std::string input{
    std::istreambuf_iterator<char>(std::cin),
    std::istreambuf_iterator<char>()};

  humanoid_app_gateway_runtime::RobotStatusAdapter adapter;
  const auto result = adapter.convert_raw_to_system_status(input, timestamp);
  if (!result.ok) {
    std::cerr << result.error << std::endl;
    return 2;
  }
  std::cout << result.system_status_json << std::endl;
  return 0;
}
