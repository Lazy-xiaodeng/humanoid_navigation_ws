/*
 * gesture_sync_probe.cpp
 *
 * 文件用途：
 * 1. 作为开发期动作库同步验证工具，构造固定机器人动作库响应和旧 gestures.yaml。
 * 2. 验证动作列表提取、已有动作元数据保留、新动作默认字段生成、reload 通知组包。
 * 3. 上游：协议一致性验证脚本。
 * 4. 下游：stdout，供脚本比较动作库同步纯逻辑是否与 既有机器人网关协议一致。
 */

#include <iostream>

#include "humanoid_robot_gateway_runtime/gesture_sync.hpp"
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
  humanoid_robot_gateway_runtime::GestureSync sync;
  const std::string response_data = R"({
    "result": "success",
    "motion_list": [
      {"motion_name_en": "wave_greet_bye", "motion_name_cn": "挥手问候", "motion_index": 7},
      {"name": "point_forward", "name_cn": "向前指", "motion_index": 8},
      "simple_motion"
    ]
  })";
  const std::string existing_yaml = R"(
metadata:
  version: old
actions:
  wave_greet_bye:
    id: 1
    name: 旧挥手
    type: upper_body
    description: 保留旧描述
    duration_sec: 12.5
)";

  const auto items = sync.extract_motion_items(response_data);
  const auto yaml_result = sync.build_gestures_yaml(existing_yaml, items);
  const auto reload_result = sync.build_reload_message(123.456);
  if (!yaml_result.ok || !reload_result.ok) {
    std::cerr << yaml_result.error << reload_result.error << std::endl;
    return 2;
  }

  rapidjson::Document document;
  document.SetObject();
  auto & allocator = document.GetAllocator();

  rapidjson::Value motions(rapidjson::kArrayType);
  for (const auto & item : items) {
    rapidjson::Value object(rapidjson::kObjectType);
    add_string(object, allocator, "en_name", item.en_name);
    add_string(object, allocator, "cn_name", item.cn_name);
    object.AddMember("index", item.index, allocator);
    motions.PushBack(object, allocator);
  }
  document.AddMember("motion_items", motions, allocator);
  add_string(document, allocator, "yaml", yaml_result.text);

  rapidjson::Document reload_doc;
  reload_doc.Parse(reload_result.text.c_str());
  document.AddMember("reload", rapidjson::Value(reload_doc, allocator), allocator);

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  std::cout << buffer.GetString() << std::endl;
  return 0;
}
