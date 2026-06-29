/*
 * robot_status_parser_probe.cpp
 *
 * 文件用途：
 * 1. 作为开发期机器人状态解析验证工具，构造固定 notify_robot_info 和 notify_joy_data。
 * 2. 验证单位转换、身份字段补齐、控制可导航状态、通信质量估算和 Joy 数组解析。
 * 3. 上游：协议一致性验证脚本。
 * 4. 下游：stdout，供脚本比较状态解析结果是否与 既有机器人网关协议一致。
 */

#include <iostream>

#include "humanoid_robot_gateway_runtime/robot_status_parser.hpp"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

int main()
{
  humanoid_robot_gateway_runtime::RobotStatusParser parser;
  const std::string info =
    R"({"result":[{"name":"system_info","message":"ok","values":[{"key":"robot_status","value":"Walk"},{"key":"battery","value":"89"},{"key":"bat_vol","value":"51234"},{"key":"bat_cur","value":"-1234"},{"key":"bat_temp0","value":"321"},{"key":"mode","value":"navigation"}]},{"name":"peripheral","message":"ready","values":[{"key":"motor_vol","value":"24000"},{"key":"robot_sn","value":"SN_FROM_VALUES"}]}]})";
  const auto status = parser.parse_notify_robot_info(
    info,
    "HU_D04_01_289",
    "SN_MAIN",
    "notify_robot_info",
    "Walk",
    false,
    "",
    101.0,
    100.4,
    {500.0, 520.0});
  const auto joy = parser.parse_notify_joy_data(R"({"axes":[0,"1.5",-2],"buttons":[1,"0",true]})");

  if (!status.ok || !joy.ok) {
    std::cerr << status.error << joy.error << std::endl;
    return 2;
  }

  rapidjson::Document status_doc;
  status_doc.Parse(status.robot_status_raw_json.c_str());

  rapidjson::Document document;
  document.SetObject();
  auto & allocator = document.GetAllocator();
  document.AddMember("status", rapidjson::Value(status_doc, allocator), allocator);

  rapidjson::Value joy_doc(rapidjson::kObjectType);
  rapidjson::Value axes(rapidjson::kArrayType);
  for (const auto item : joy.axes) {
    axes.PushBack(item, allocator);
  }
  rapidjson::Value buttons(rapidjson::kArrayType);
  for (const auto item : joy.buttons) {
    buttons.PushBack(item, allocator);
  }
  joy_doc.AddMember("axes", axes, allocator);
  joy_doc.AddMember("buttons", buttons, allocator);
  joy_doc.AddMember("frame_id", rapidjson::Value(joy.frame_id.c_str(), allocator).Move(), allocator);
  document.AddMember("joy", joy_doc, allocator);

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  std::cout << buffer.GetString() << std::endl;
  return 0;
}
