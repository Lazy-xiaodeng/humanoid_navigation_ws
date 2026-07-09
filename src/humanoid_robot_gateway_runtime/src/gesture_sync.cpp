/*
 * gesture_sync.cpp
 *
 * 文件用途：
 * 1. 实现机器人动作库响应解析、gestures.yaml 内容生成和热重载通知组包。
 * 2. 上游：机器人本体动作库接口。
 * 3. 下游：humanoid_expression_runtime gestures.yaml、/system/gesture_list_updated 和 data_integration_node。
 * 4. 本模块当前只提供纯逻辑能力，不直接请求机器人、不直接写真实配置文件。
 */

#include "humanoid_robot_gateway_runtime/gesture_sync.hpp"

#include <sstream>

#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "yaml-cpp/yaml.h"

namespace humanoid_robot_gateway_runtime
{

namespace
{

const char * const kMotionListKeys[] = {
  "motion_list",
  "motions",
  "atomic_motions",
  "atomic_motion_list",
  "list"};

std::string get_string_field(
  const rapidjson::Value & object,
  const char * key)
{
  if (!object.IsObject() || !object.HasMember(key)) {
    return "";
  }
  const auto & value = object[key];
  if (value.IsString()) {
    return value.GetString();
  }
  if (value.IsInt()) {
    return std::to_string(value.GetInt());
  }
  if (value.IsUint()) {
    return std::to_string(value.GetUint());
  }
  if (value.IsInt64()) {
    return std::to_string(value.GetInt64());
  }
  if (value.IsUint64()) {
    return std::to_string(value.GetUint64());
  }
  if (value.IsDouble()) {
    std::ostringstream stream;
    stream << value.GetDouble();
    return stream.str();
  }
  return "";
}

int get_index_field(
  const rapidjson::Value & object,
  const char * key,
  const int fallback)
{
  if (!object.IsObject() || !object.HasMember(key)) {
    return fallback;
  }
  const auto & value = object[key];
  if (value.IsInt()) {
    return value.GetInt();
  }
  if (value.IsUint()) {
    return static_cast<int>(value.GetUint());
  }
  if (value.IsString()) {
    try {
      return std::stoi(value.GetString());
    } catch (...) {
      return fallback;
    }
  }
  return fallback;
}

void append_motion_items_from_array(
  const rapidjson::Value & motions,
  std::vector<GestureMotionItem> & items,
  const bool dict_uses_motion_name_fallback)
{
  if (!motions.IsArray()) {
    return;
  }

  for (rapidjson::SizeType i = 0; i < motions.Size(); ++i) {
    const auto & motion = motions[i];
    if (motion.IsString()) {
      const std::string name = motion.GetString();
      items.push_back(GestureMotionItem{name, name, static_cast<int>(items.size())});
      continue;
    }

    if (!motion.IsObject()) {
      continue;
    }

    std::string en_name = get_string_field(motion, "motion_name_en");
    if (en_name.empty()) {
      en_name = get_string_field(motion, "name");
    }
    if (en_name.empty() && dict_uses_motion_name_fallback) {
      en_name = get_string_field(motion, "motion_name");
    }
    if (en_name.empty()) {
      continue;
    }

    std::string cn_name = get_string_field(motion, "motion_name_cn");
    if (cn_name.empty()) {
      cn_name = get_string_field(motion, "name_cn");
    }
    if (cn_name.empty()) {
      cn_name = en_name;
    }

    const int index = get_index_field(motion, "motion_index", static_cast<int>(items.size()));
    items.push_back(GestureMotionItem{en_name, cn_name, index});
  }
}

std::string node_to_yaml_string(const YAML::Node & node)
{
  YAML::Emitter emitter;
  emitter.SetIndent(2);
  emitter << node;
  return std::string(emitter.c_str()) + "\n";
}

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

std::string document_to_json(const rapidjson::Document & document)
{
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  document.Accept(writer);
  return buffer.GetString();
}

}  // namespace

std::string GestureSync::name() const { return "gesture_sync"; }

std::vector<GestureMotionItem> GestureSync::extract_motion_items(const std::string & data_json) const
{
  rapidjson::Document document;
  document.Parse(data_json.c_str());
  if (document.HasParseError()) {
    return {};
  }

  if (document.IsObject()) {
    for (const char * key : kMotionListKeys) {
      if (!document.HasMember(key)) {
        continue;
      }
      std::vector<GestureMotionItem> items;
      append_motion_items_from_array(document[key], items, true);
      if (!items.empty()) {
        return items;
      }
    }
    return {};
  }

  if (document.IsArray()) {
    std::vector<GestureMotionItem> items;
    append_motion_items_from_array(document, items, false);
    return items;
  }

  return {};
}

GestureSyncResult GestureSync::build_gestures_yaml(
  const std::string & existing_yaml,
  const std::vector<GestureMotionItem> & motion_items) const
{
  GestureSyncResult result;
  if (motion_items.empty()) {
    result.error = "empty_motion_items";
    return result;
  }

  YAML::Node existing_actions(YAML::NodeType::Map);
  if (!existing_yaml.empty()) {
    try {
      const YAML::Node existing = YAML::Load(existing_yaml);
      if (existing["actions"] && existing["actions"].IsMap()) {
        existing_actions = existing["actions"];
      }
    } catch (const std::exception &) {
      // 读取旧 YAML 失败时忽略旧数据，保持动作库同步链路的容错策略。
      existing_actions = YAML::Node(YAML::NodeType::Map);
    }
  }

  YAML::Node actions(YAML::NodeType::Map);
  for (const auto & item : motion_items) {
    if (item.en_name.empty()) {
      continue;
    }

    YAML::Node entry;
    if (existing_actions[item.en_name] && existing_actions[item.en_name].IsMap()) {
      entry = YAML::Clone(existing_actions[item.en_name]);
      entry["id"] = item.index;
      if (!item.cn_name.empty()) {
        entry["name"] = item.cn_name;
      }
    } else {
      entry["id"] = item.index;
      entry["name"] = item.cn_name.empty() ? item.en_name : item.cn_name;
      entry["type"] = "upper_body";
      entry["description"] = "";
    }
    actions[item.en_name] = entry;
  }

  YAML::Node output;
  output["metadata"]["version"] = "ota";
  output["metadata"]["description"] = "逐际机器人上半身动作库 (OTA 自动更新)";
  output["metadata"]["author"] = "OTA Sync";
  output["actions"] = actions;

  result.ok = true;
  result.text = node_to_yaml_string(output);
  return result;
}

GestureSyncResult GestureSync::build_reload_message(const double timestamp_sec) const
{
  rapidjson::Document document;
  document.SetObject();
  auto & allocator = document.GetAllocator();
  add_string(document, allocator, "action", "reload");
  document.AddMember("timestamp", timestamp_sec, allocator);

  GestureSyncResult result;
  result.ok = true;
  result.text = document_to_json(document);
  return result;
}

}  // namespace humanoid_robot_gateway_runtime
