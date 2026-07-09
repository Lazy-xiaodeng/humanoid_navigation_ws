/*
 * robot_protocol_probe.cpp
 *
 * 文件用途：
 * 1. 作为开发期机器人底层协议验证工具，构造固定 command 和机器人返回消息。
 * 2. 验证 accid/title/timestamp/guid/data 组包、title/guid/data 解析和身份字段清洗。
 * 3. 上游：协议一致性验证脚本。
 * 4. 下游：stdout，供脚本比较协议处理结果是否与 既有机器人网关协议一致。
 */

#include <iostream>

#include "humanoid_robot_gateway_runtime/robot_protocol.hpp"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace
{

void add_string(
  rapidjson::Value & object,
  rapidjson::Document::AllocatorType & allocator,
  const char * key,
  const std::string & value)
{
  object.AddMember(
    rapidjson::Value(key, allocator).Move(),
    rapidjson::Value(value.c_str(), allocator).Move(),
    allocator);
}

}  // namespace

int main()
{
  humanoid_robot_gateway_runtime::RobotProtocol protocol;
  const auto command = protocol.build_command_message(
    "HU_D04_01_289",
    "request_prepare",
    "guid-fixed",
    1234567890,
    R"({"foo":1,"bar":"二"})");
  const auto parsed = protocol.parse_robot_message(
    R"({"accid":" HU_D04_01_289 ","sn":" none ","title":"response_prepare","guid":"guid-fixed","data":{"ok":true}})");

  if (!command.ok || !parsed.ok) {
    std::cerr << command.error << parsed.error << std::endl;
    return 2;
  }

  rapidjson::Document command_doc;
  command_doc.Parse(command.json.c_str());
  rapidjson::Document parsed_data_doc;
  parsed_data_doc.Parse(parsed.data_json.c_str());

  rapidjson::Document document;
  document.SetObject();
  auto & allocator = document.GetAllocator();
  document.AddMember("command", rapidjson::Value(command_doc, allocator), allocator);

  rapidjson::Value parsed_json(rapidjson::kObjectType);
  add_string(parsed_json, allocator, "accid", parsed.accid);
  add_string(parsed_json, allocator, "sn", parsed.sn);
  add_string(parsed_json, allocator, "title", parsed.title);
  add_string(parsed_json, allocator, "guid", parsed.guid);
  parsed_json.AddMember("data", rapidjson::Value(parsed_data_doc, allocator), allocator);
  document.AddMember("parsed", parsed_json, allocator);

  add_string(document, allocator, "normalized_none", protocol.normalize_identity_value(" none "));
  add_string(document, allocator, "normalized_accid", protocol.normalize_identity_value(" HU_TEST "));

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  std::cout << buffer.GetString() << std::endl;
  return 0;
}
