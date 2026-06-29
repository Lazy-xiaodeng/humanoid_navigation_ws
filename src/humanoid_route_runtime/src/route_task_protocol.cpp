/*
 * route_task_protocol.cpp
 *
 * 文件用途：
 * 1. 实现路线任务运行层的 JSON 协议工具函数。
 * 2. 所有函数都保持无 ROS 依赖，便于后续写纯单元测试，对比输入 payload 和输出事件是否一致。
 * 3. 这里不维护导航状态，也不决定能否启动导航，只负责把上游消息读准确、把下游字段组准确。
 */

#include "humanoid_route_runtime/route_task_protocol.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "humanoid_route_runtime/route_runtime_types.hpp"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace humanoid_route_runtime
{

namespace
{

const rapidjson::Value * object_member(const rapidjson::Value & object, const char * key)
{
  if (!object.IsObject() || !object.HasMember(key)) {
    return nullptr;
  }
  return &object[key];
}

const rapidjson::Value * object_member_object(const rapidjson::Value & object, const char * key)
{
  const auto * value = object_member(object, key);
  return value != nullptr && value->IsObject() ? value : nullptr;
}

const rapidjson::Value * prefer_member(
  const rapidjson::Value & object,
  const rapidjson::Value * properties,
  const char * key)
{
  if (object.IsObject() && object.HasMember(key)) {
    return &object[key];
  }
  if (properties != nullptr && properties->IsObject() && properties->HasMember(key)) {
    return &(*properties)[key];
  }
  return nullptr;
}

std::string read_route_waypoint_walk_direction(
  const rapidjson::Value & waypoint,
  const rapidjson::Value * properties)
{
  const std::array<const char *, 4> property_keys{
    "walk_direction", "navigation_direction", "drive_direction", "motion_direction"};
  const auto * direction = object_member(waypoint, "walk_direction");
  if (direction == nullptr && properties != nullptr && properties->IsObject()) {
    for (const auto * key : property_keys) {
      direction = object_member(*properties, key);
      if (direction != nullptr) {
        break;
      }
    }
  }
  if (direction == nullptr || direction->IsNull()) {
    return "forward";
  }
  if (direction->IsBool()) {
    return direction->GetBool() ? "backward" : "forward";
  }
  const std::string text = route_task_id(*direction);
  return text.empty() ? "forward" : text;
}

std::map<std::string, std::string> extract_revisions_by_map(
  const rapidjson::Value & message,
  const rapidjson::Value & legacy_data)
{
  const std::array<const rapidjson::Value *, 6> candidates{
    object_member(legacy_data, "waypoints_revisions_by_map"),
    object_member_object(legacy_data, "metadata") != nullptr ?
      object_member(*object_member_object(legacy_data, "metadata"), "waypoints_revisions_by_map") : nullptr,
    object_member_object(legacy_data, "data") != nullptr ?
      object_member(*object_member_object(legacy_data, "data"), "waypoints_revisions_by_map") : nullptr,
    object_member_object(legacy_data, "data") != nullptr &&
      object_member_object(*object_member_object(legacy_data, "data"), "metadata") != nullptr ?
      object_member(*object_member_object(*object_member_object(legacy_data, "data"), "metadata"), "waypoints_revisions_by_map") : nullptr,
    object_member(message, "waypoints_revisions_by_map"),
    object_member_object(message, "metadata") != nullptr ?
      object_member(*object_member_object(message, "metadata"), "waypoints_revisions_by_map") : nullptr,
  };

  for (const auto * candidate : candidates) {
    if (candidate == nullptr || !candidate->IsObject() || candidate->ObjectEmpty()) {
      continue;
    }
    std::map<std::string, std::string> revisions;
    for (auto it = candidate->MemberBegin(); it != candidate->MemberEnd(); ++it) {
      const std::string map_id = route_task_id(it->name);
      const std::string revision = route_task_id(it->value);
      if (!map_id.empty()) {
        revisions[map_id] = revision;
      }
    }
    return revisions;
  }
  return {};
}

}  // namespace

StoredWaypoint stored_waypoint_from_json(
  const std::string & waypoint_id,
  const rapidjson::Value & waypoint,
  const std::string & fallback_map_id,
  const RouteRuntimeConfig & config)
{
  StoredWaypoint stored;
  stored.waypoint_id = waypoint_id;
  const auto * properties = object_member_object(waypoint, "properties");
  stored.waypoint_name = read_string_member(waypoint, "waypoint_name", "");
  if (stored.waypoint_name.empty()) {
    stored.waypoint_name = read_string_member(waypoint, "name", waypoint_id);
  }
  stored.type = read_string_member(waypoint, "type", "");
  stored.map_id = read_route_task_id_member(waypoint, "map_id");
  if (stored.map_id.empty() && properties != nullptr) {
    stored.map_id = read_route_task_id_member(*properties, "map_id");
  }
  if (stored.map_id.empty()) {
    stored.map_id = fallback_map_id;
  }
  const auto * frame_value = prefer_member(waypoint, properties, "frame_id");
  stored.frame_id = normalize_route_task_frame_id(frame_value, config.default_frame_id);

  const auto position = normalize_route_task_position(object_member(waypoint, "position"));
  if (position.error.empty()) {
    stored.position = position.position;
    stored.has_position = true;
  }
  const auto orientation = normalize_route_task_orientation(object_member(waypoint, "orientation"));
  if (orientation.error.empty()) {
    stored.orientation = orientation.orientation;
    stored.has_orientation = true;
  }

  stored.waypoint_role = read_string_member(waypoint, "waypoint_role", "");
  if (stored.waypoint_role.empty() && properties != nullptr) {
    stored.waypoint_role = read_string_member(*properties, "waypoint_role", "");
  }
  std::transform(
    stored.waypoint_role.begin(), stored.waypoint_role.end(), stored.waypoint_role.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  stored.need_broadcast = route_task_bool(prefer_member(waypoint, properties, "need_broadcast"), false);
  stored.broadcast_id = read_route_task_id_member(waypoint, "broadcast_id");
  if (stored.broadcast_id.empty() && properties != nullptr) {
    stored.broadcast_id = read_route_task_id_member(*properties, "broadcast_id");
  }
  stored.broadcast_text = read_string_member(waypoint, "broadcast_text", "");
  if (stored.broadcast_text.empty() && properties != nullptr) {
    stored.broadcast_text = read_string_member(*properties, "broadcast_text", "");
  }
  stored.broadcast_blocking = route_task_bool(prefer_member(waypoint, properties, "broadcast_blocking"), true);
  stored.stop_and_align = route_task_bool(prefer_member(waypoint, properties, "stop_and_align"), true);
  stored.walk_direction = read_route_waypoint_walk_direction(waypoint, properties);
  return stored;
}

void collect_waypoints_from_bucket(
  const rapidjson::Value & bucket,
  const std::string & fallback_map_id,
  const RouteRuntimeConfig & config,
  std::map<std::string, StoredWaypoint> & output)
{
  if (!bucket.IsObject()) {
    return;
  }
  for (auto it = bucket.MemberBegin(); it != bucket.MemberEnd(); ++it) {
    if (!it->value.IsObject()) {
      continue;
    }
    const std::string key = route_task_id(it->name);
    const std::string direct_id = read_route_task_id_member(it->value, "id");
    if (!direct_id.empty() || it->value.HasMember("position")) {
      const std::string waypoint_id = !direct_id.empty() ? direct_id : key;
      output[waypoint_id] = stored_waypoint_from_json(waypoint_id, it->value, fallback_map_id, config);
      continue;
    }
    collect_waypoints_from_bucket(it->value, fallback_map_id, config, output);
  }
}

std::size_t count_cached_waypoints(const std::map<std::string, std::map<std::string, StoredWaypoint>> & by_map)
{
  std::size_t total = 0;
  for (const auto & item : by_map) {
    total += item.second.size();
  }
  return total;
}

std::string json_to_string(const rapidjson::Value & value)
{
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  value.Accept(writer);
  return buffer.GetString();
}

std::string read_string_member(
  const rapidjson::Value & object,
  const char * key,
  const std::string & fallback)
{
  if (!object.IsObject() || !object.HasMember(key)) {
    return fallback;
  }
  const auto & value = object[key];
  if (value.IsString()) {
    return trim_copy(value.GetString());
  }
  if (value.IsInt64()) {
    return std::to_string(value.GetInt64());
  }
  if (value.IsUint64()) {
    return std::to_string(value.GetUint64());
  }
  return fallback;
}

bool read_bool_member(
  const rapidjson::Value & object,
  const char * key,
  const bool fallback)
{
  if (!object.IsObject() || !object.HasMember(key)) {
    return fallback;
  }
  const auto & value = object[key];
  if (value.IsBool()) {
    return value.GetBool();
  }
  if (value.IsInt()) {
    return value.GetInt() != 0;
  }
  if (value.IsString()) {
    std::string text = trim_copy(value.GetString());
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return text == "true" || text == "1" || text == "yes" || text == "y" || text == "on";
  }
  return fallback;
}

std::string extract_revision(
  const rapidjson::Value & data,
  const rapidjson::Value & message)
{
  const std::array<const rapidjson::Value *, 6> candidates{
    object_member(data, "waypoints_revision"),
    object_member_object(data, "metadata") != nullptr ?
      object_member(*object_member_object(data, "metadata"), "waypoints_revision") : nullptr,
    object_member_object(data, "data") != nullptr ?
      object_member(*object_member_object(data, "data"), "waypoints_revision") : nullptr,
    object_member_object(data, "data") != nullptr &&
      object_member_object(*object_member_object(data, "data"), "metadata") != nullptr ?
      object_member(*object_member_object(*object_member_object(data, "data"), "metadata"), "waypoints_revision") :
      nullptr,
    object_member(message, "waypoints_revision"),
    object_member_object(message, "metadata") != nullptr ?
      object_member(*object_member_object(message, "metadata"), "waypoints_revision") : nullptr,
  };

  for (const auto * candidate : candidates) {
    if (candidate == nullptr) {
      continue;
    }
    const std::string revision = route_task_id(*candidate);
    if (!revision.empty()) {
      return revision;
    }
  }
  return "";
}

std::size_t count_waypoints_payload(const rapidjson::Value & data)
{
  if (!data.IsObject()) {
    return 0;
  }
  const rapidjson::Value * waypoints = nullptr;
  if (data.HasMember("waypoints") && data["waypoints"].IsObject()) {
    waypoints = &data["waypoints"];
  }
  if (waypoints == nullptr) {
    return 0;
  }
  std::size_t count = 0;
  for (auto it = waypoints->MemberBegin(); it != waypoints->MemberEnd(); ++it) {
    if (!it->value.IsObject()) {
      continue;
    }
    if (it->value.HasMember("id")) {
      ++count;
    } else {
      count += it->value.MemberCount();
    }
  }
  return count;
}

bool is_route_task_command(const std::string & command_type)
{
  return command_type == "start_route_task" ||
         command_type == "pause_route_task" ||
         command_type == "resume_route_task" ||
         command_type == "stop_route_task" ||
         command_type == "jump_to_waypoint" ||
         command_type == "broadcast_finished";
}

rapidjson::Document make_extra_data(
  const std::string & error_code,
  const std::string & request_type)
{
  rapidjson::Document extra;
  extra.SetObject();
  auto & allocator = extra.GetAllocator();
  extra.AddMember("error_code", rapidjson::Value(error_code.c_str(), allocator).Move(), allocator);
  extra.AddMember("request_type", rapidjson::Value(request_type.c_str(), allocator).Move(), allocator);
  return extra;
}

std::string route_task_id(const rapidjson::Value & value)
{
  if (value.IsNull()) {
    return "";
  }
  if (value.IsString()) {
    return trim_copy(value.GetString());
  }
  if (value.IsInt64()) {
    return std::to_string(value.GetInt64());
  }
  if (value.IsUint64()) {
    return std::to_string(value.GetUint64());
  }
  if (value.IsDouble() && std::isfinite(value.GetDouble())) {
    const double number = value.GetDouble();
    const auto integer_value = static_cast<int64_t>(number);
    if (std::abs(number - static_cast<double>(integer_value)) < 1e-9) {
      return std::to_string(integer_value);
    }
    return std::to_string(number);
  }
  return "";
}

std::string read_route_task_id_member(
  const rapidjson::Value & object,
  const char * key)
{
  if (!object.IsObject() || !object.HasMember(key)) {
    return "";
  }
  return route_task_id(object[key]);
}

bool route_task_bool(const rapidjson::Value * value, const bool default_value)
{
  if (value == nullptr || value->IsNull()) {
    return default_value;
  }
  if (value->IsBool()) {
    return value->GetBool();
  }
  if (value->IsNumber()) {
    return value->GetDouble() != 0.0;
  }
  if (value->IsString()) {
    std::string normalized = trim_copy(value->GetString());
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    if (normalized == "true" || normalized == "1" || normalized == "yes" ||
      normalized == "y" || normalized == "on")
    {
      return true;
    }
    if (normalized == "false" || normalized == "0" || normalized == "no" ||
      normalized == "n" || normalized == "off" || normalized.empty())
    {
      return false;
    }
  }
  return default_value;
}

std::string normalize_route_task_frame_id(
  const rapidjson::Value * value,
  const std::string & default_frame_id)
{
  if (value == nullptr) {
    return default_frame_id;
  }
  const std::string frame_id = route_task_id(*value);
  return frame_id.empty() ? default_frame_id : frame_id;
}

RouteTaskPositionResult normalize_route_task_position(const rapidjson::Value * value)
{
  RouteTaskPositionResult result;
  if (value == nullptr) {
    result.error = "position must be an array";
    return result;
  }

  std::array<const rapidjson::Value *, 3> components{nullptr, nullptr, nullptr};
  if (value->IsObject()) {
    if (value->HasMember("x")) {
      components[0] = &(*value)["x"];
    }
    if (value->HasMember("y")) {
      components[1] = &(*value)["y"];
    }
    if (value->HasMember("z")) {
      components[2] = &(*value)["z"];
    }
  } else if (value->IsArray()) {
    if (value->Size() < 2) {
      result.error = "position must contain at least x and y";
      return result;
    }
    components[0] = &(*value)[0];
    components[1] = &(*value)[1];
    if (value->Size() >= 3) {
      components[2] = &(*value)[2];
    }
  } else {
    result.error = "position must be an array";
    return result;
  }

  if (components[0] == nullptr || components[1] == nullptr) {
    result.error = "position must contain at least x and y";
    return result;
  }

  for (std::size_t i = 0; i < components.size(); ++i) {
    if (components[i] == nullptr) {
      result.position[i] = 0.0;
      continue;
    }
    if (!components[i]->IsNumber()) {
      result.error = "position values must be numbers";
      return result;
    }
    result.position[i] = components[i]->GetDouble();
    if (!std::isfinite(result.position[i])) {
      result.error = "position values must be finite numbers";
      return result;
    }
  }
  return result;
}

RouteTaskOrientationResult normalize_route_task_orientation(const rapidjson::Value * value)
{
  RouteTaskOrientationResult result;
  if (value == nullptr) {
    result.error = "orientation must be a quaternion array";
    return result;
  }

  std::array<const rapidjson::Value *, 4> components{nullptr, nullptr, nullptr, nullptr};
  if (value->IsObject()) {
    const std::array<const char *, 4> keys{"x", "y", "z", "w"};
    for (std::size_t i = 0; i < keys.size(); ++i) {
      if (value->HasMember(keys[i])) {
        components[i] = &(*value)[keys[i]];
      }
    }
  } else if (value->IsArray()) {
    if (value->Size() < 4) {
      result.error = "orientation must contain x, y, z and w";
      return result;
    }
    for (std::size_t i = 0; i < components.size(); ++i) {
      components[i] = &(*value)[static_cast<rapidjson::SizeType>(i)];
    }
  } else {
    result.error = "orientation must be a quaternion array";
    return result;
  }

  double norm_sq = 0.0;
  for (std::size_t i = 0; i < components.size(); ++i) {
    if (components[i] == nullptr) {
      result.error = "orientation must contain x, y, z and w";
      return result;
    }
    if (!components[i]->IsNumber()) {
      result.error = "orientation values must be numbers";
      return result;
    }
    result.orientation[i] = components[i]->GetDouble();
    if (!std::isfinite(result.orientation[i])) {
      result.error = "orientation values must be finite numbers";
      return result;
    }
    norm_sq += result.orientation[i] * result.orientation[i];
  }

  const double norm = std::sqrt(norm_sq);
  if (norm <= 1e-6) {
    result.error = "orientation quaternion norm must be greater than zero";
    return result;
  }
  for (double & component : result.orientation) {
    component /= norm;
  }
  return result;
}

RouteWaypointSourceResult validate_route_waypoint_source(const rapidjson::Value & command_data)
{
  RouteWaypointSourceResult result;
  const bool has_route_waypoints =
    command_data.IsObject() && command_data.HasMember("route_waypoints") &&
    command_data["route_waypoints"].IsArray() && !command_data["route_waypoints"].Empty();
  const bool has_route_waypoint_ids =
    command_data.IsObject() && command_data.HasMember("route_waypoint_ids") &&
    command_data["route_waypoint_ids"].IsArray() && !command_data["route_waypoint_ids"].Empty();

  if (has_route_waypoints && has_route_waypoint_ids) {
    result.error_code = "ambiguous_route_waypoint_source";
    result.message = "route_waypoints and route_waypoint_ids cannot both be provided";
    return result;
  }
  if (has_route_waypoints) {
    result.source = "inline_route_waypoints";
    return result;
  }
  if (has_route_waypoint_ids) {
    result.source = "stored_waypoint_ids";
    return result;
  }
  if (command_data.IsObject() && command_data.HasMember("route_waypoint_ids") &&
    command_data["route_waypoint_ids"].IsArray())
  {
    result.error_code = "invalid_route_waypoint_ids";
    result.message = "route_waypoint_ids must not be empty";
    return result;
  }
  result.error_code = "invalid_route_waypoints";
  result.message = "route_waypoints or route_waypoint_ids is required";
  return result;
}

RouteWaypointNormalizeResult normalize_route_task_waypoints(
  const rapidjson::Value & route_waypoints,
  const std::string & route_map_id,
  const RouteRuntimeConfig & config)
{
  RouteWaypointNormalizeResult result;
  if (!route_waypoints.IsArray()) {
    result.error_code = "invalid_route_waypoints";
    result.message = "route_waypoints must be an array";
    return result;
  }

  std::vector<std::string> seen_ids;
  int task_count = 0;
  const std::string normalized_route_map_id = trim_copy(route_map_id);
  for (rapidjson::SizeType index = 0; index < route_waypoints.Size(); ++index) {
    const auto & waypoint = route_waypoints[index];
    if (!waypoint.IsObject()) {
      result.error_code = "invalid_route_waypoints";
      result.message = "route_waypoints[" + std::to_string(index) + "] must be an object";
      return result;
    }

    RouteWaypoint normalized;
    normalized.waypoint_id = read_route_task_id_member(waypoint, "waypoint_id");
    if (normalized.waypoint_id.empty()) {
      result.error_code = "invalid_route_waypoints";
      result.message = "route_waypoints[" + std::to_string(index) + "] missing waypoint_id";
      return result;
    }
    if (std::find(seen_ids.begin(), seen_ids.end(), normalized.waypoint_id) != seen_ids.end()) {
      result.error_code = "duplicate_waypoint_id";
      result.message = "route_waypoints contains duplicate waypoint_id: " + normalized.waypoint_id;
      return result;
    }
    seen_ids.push_back(normalized.waypoint_id);

    const rapidjson::Value * properties = nullptr;
    if (waypoint.HasMember("properties") && waypoint["properties"].IsObject()) {
      properties = &waypoint["properties"];
    }

    normalized.map_id = read_route_task_id_member(waypoint, "map_id");
    if (normalized.map_id.empty() && properties != nullptr) {
      normalized.map_id = read_route_task_id_member(*properties, "map_id");
    }
    if (!normalized_route_map_id.empty()) {
      if (!normalized.map_id.empty() && normalized.map_id != normalized_route_map_id) {
        result.error_code = "route_map_mismatch";
        result.message =
          "waypoint " + normalized.waypoint_id + " map_id mismatch: route=" +
          normalized_route_map_id + ", waypoint=" + normalized.map_id;
        return result;
      }
      normalized.map_id = normalized_route_map_id;
    }

    const rapidjson::Value * frame_value = nullptr;
    if (waypoint.HasMember("frame_id")) {
      frame_value = &waypoint["frame_id"];
    } else if (properties != nullptr && properties->HasMember("frame_id")) {
      frame_value = &(*properties)["frame_id"];
    }
    normalized.frame_id = normalize_route_task_frame_id(frame_value, config.default_frame_id);

    const auto position = normalize_route_task_position(waypoint.HasMember("position") ? &waypoint["position"] : nullptr);
    if (!position.error.empty()) {
      result.error_code = "missing_waypoint_pose";
      result.message = "waypoint " + normalized.waypoint_id + " invalid position: " + position.error;
      return result;
    }
    normalized.position = position.position;

    const auto orientation =
      normalize_route_task_orientation(waypoint.HasMember("orientation") ? &waypoint["orientation"] : nullptr);
    if (!orientation.error.empty()) {
      result.error_code = "missing_waypoint_pose";
      result.message = "waypoint " + normalized.waypoint_id + " invalid orientation: " + orientation.error;
      return result;
    }
    normalized.orientation = orientation.orientation;

    normalized.waypoint_role = read_string_member(waypoint, "waypoint_role", "");
    if (normalized.waypoint_role.empty() && properties != nullptr) {
      normalized.waypoint_role = read_string_member(*properties, "waypoint_role", "");
    }
    std::transform(
      normalized.waypoint_role.begin(), normalized.waypoint_role.end(),
      normalized.waypoint_role.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (normalized.waypoint_role != "task" && normalized.waypoint_role != "transit") {
      result.error_code = "invalid_waypoint_role";
      result.message = "waypoint " + normalized.waypoint_id + " missing valid waypoint_role";
      return result;
    }

    normalized.source_index = static_cast<int>(index);
    normalized.waypoint_name = read_string_member(waypoint, "waypoint_name", "");
    if (normalized.waypoint_name.empty()) {
      normalized.waypoint_name = read_string_member(waypoint, "name", "");
    }

    if (normalized.waypoint_role == "task") {
      const rapidjson::Value * need_broadcast = waypoint.HasMember("need_broadcast") ? &waypoint["need_broadcast"] : nullptr;
      if (need_broadcast == nullptr && properties != nullptr && properties->HasMember("need_broadcast")) {
        need_broadcast = &(*properties)["need_broadcast"];
      }
      const rapidjson::Value * broadcast_blocking =
        waypoint.HasMember("broadcast_blocking") ? &waypoint["broadcast_blocking"] : nullptr;
      if (broadcast_blocking == nullptr && properties != nullptr && properties->HasMember("broadcast_blocking")) {
        broadcast_blocking = &(*properties)["broadcast_blocking"];
      }
      const rapidjson::Value * stop_and_align =
        waypoint.HasMember("stop_and_align") ? &waypoint["stop_and_align"] : nullptr;
      if (stop_and_align == nullptr && properties != nullptr && properties->HasMember("stop_and_align")) {
        stop_and_align = &(*properties)["stop_and_align"];
      }

      normalized.need_broadcast = route_task_bool(need_broadcast, false);
      normalized.broadcast_id = read_route_task_id_member(waypoint, "broadcast_id");
      if (normalized.broadcast_id.empty() && properties != nullptr) {
        normalized.broadcast_id = read_route_task_id_member(*properties, "broadcast_id");
      }
      if (normalized.need_broadcast && normalized.broadcast_id.empty()) {
        result.error_code = "missing_broadcast_id";
        result.message =
          "waypoint " + normalized.waypoint_id + " need_broadcast=true but missing broadcast_id";
        return result;
      }
      normalized.broadcast_text = read_string_member(waypoint, "broadcast_text", "");
      if (normalized.broadcast_text.empty() && properties != nullptr) {
        normalized.broadcast_text = read_string_member(*properties, "broadcast_text", "");
      }
      normalized.broadcast_blocking = route_task_bool(broadcast_blocking, true);
      normalized.stop_and_align = route_task_bool(stop_and_align, true);
      normalized.walk_direction = read_route_waypoint_walk_direction(waypoint, properties);
      ++task_count;
    } else {
      normalized.need_broadcast = false;
      normalized.broadcast_id.clear();
      normalized.broadcast_blocking = false;
      normalized.stop_and_align = false;
      normalized.walk_direction = "forward";
    }

    result.waypoints.push_back(normalized);
  }

  if (task_count == 0) {
    result.waypoints.clear();
    result.error_code = "missing_task_waypoints";
    result.message = "route must contain at least one task waypoint";
    return result;
  }

  return result;
}

std::vector<std::string> build_master_route_task_ids(const std::vector<RouteWaypoint> & route_waypoints)
{
  std::vector<std::string> ids;
  for (const auto & waypoint : route_waypoints) {
    if (waypoint.waypoint_role == "task") {
      ids.push_back(waypoint.waypoint_id);
    }
  }
  return ids;
}

WaypointCacheParseResult update_waypoint_cache_from_message(
  const rapidjson::Value & message,
  WaypointCacheState & cache,
  const RouteRuntimeConfig & config)
{
  WaypointCacheParseResult result;
  if (!message.IsObject()) {
    result.error = "waypoints message must be object";
    return result;
  }

  const rapidjson::Value * legacy_data = &message;
  if (message.HasMember("protocol_version") && message.HasMember("data") && message["data"].IsObject()) {
    const std::string data_type = read_string_member(message, "data_type", "");
    if (data_type != "waypoints_data") {
      result.accepted = false;
      return result;
    }
    legacy_data = &message["data"];
  }

  const rapidjson::Value * waypoints_data = object_member_object(*legacy_data, "data");
  if (waypoints_data == nullptr || !waypoints_data->HasMember("waypoints")) {
    waypoints_data = legacy_data;
  }

  const auto * legacy_metadata = object_member_object(*legacy_data, "metadata");
  std::string map_id = read_route_task_id_member(*legacy_data, "map_id");
  if (map_id.empty() && waypoints_data != nullptr) {
    map_id = read_route_task_id_member(*waypoints_data, "map_id");
  }
  if (map_id.empty() && legacy_metadata != nullptr) {
    map_id = read_route_task_id_member(*legacy_metadata, "map_id");
  }

  std::map<std::string, StoredWaypoint> current_map_waypoints;
  std::map<std::string, std::map<std::string, StoredWaypoint>> by_map;

  const auto * waypoints = waypoints_data != nullptr ? object_member_object(*waypoints_data, "waypoints") : nullptr;
  if (waypoints != nullptr) {
    collect_waypoints_from_bucket(*waypoints, map_id, config, current_map_waypoints);
  }

  const auto * incoming_by_map =
    waypoints_data != nullptr ? object_member_object(*waypoints_data, "waypoints_by_map") : nullptr;
  if (incoming_by_map != nullptr && !incoming_by_map->ObjectEmpty()) {
    for (auto map_it = incoming_by_map->MemberBegin(); map_it != incoming_by_map->MemberEnd(); ++map_it) {
      const std::string bucket_map_id = route_task_id(map_it->name);
      if (bucket_map_id.empty() || !map_it->value.IsObject()) {
        continue;
      }
      std::map<std::string, StoredWaypoint> bucket_waypoints;
      collect_waypoints_from_bucket(map_it->value, bucket_map_id, config, bucket_waypoints);
      if (!bucket_waypoints.empty()) {
        by_map[bucket_map_id] = std::move(bucket_waypoints);
      }
    }
  } else if (!map_id.empty() && !current_map_waypoints.empty()) {
    by_map[map_id] = current_map_waypoints;
  } else if (!current_map_waypoints.empty()) {
    by_map["default"] = current_map_waypoints;
  }

  cache.map_id = map_id;
  cache.revision = extract_revision(*legacy_data, message);
  cache.revisions_by_map = extract_revisions_by_map(message, *legacy_data);
  cache.waypoints = std::move(current_map_waypoints);
  cache.waypoints_by_map = std::move(by_map);
  cache.count = count_cached_waypoints(cache.waypoints_by_map);
  if (cache.count == 0 && !cache.waypoints.empty()) {
    cache.count = cache.waypoints.size();
  }
  cache.last_update = now_seconds();
  result.accepted = true;
  return result;
}

}  // namespace humanoid_route_runtime
