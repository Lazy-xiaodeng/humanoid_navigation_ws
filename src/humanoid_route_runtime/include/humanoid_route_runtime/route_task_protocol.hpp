/*
 * route_task_protocol.hpp
 *
 * 文件用途：
 * 1. 负责路线任务运行层所有 JSON 协议的轻量解析和组包辅助。
 * 2. 把 request_type、command_type、waypoints_revision、布尔/字符串归一化等协议细节从 ROS 节点外壳中拆出来。
 * 3. 后续如果 Web/任务层协议字段扩展，只需要优先检查这个模块，减少业务状态机被协议细节污染。
 *
 * 代码块顺序：
 * 1. JSON 序列化。
 * 2. 字段读取和类型归一化。
 * 3. 点位库 payload 辅助解析。
 * 4. 路线任务命令识别和错误附加字段构造。
 */

#pragma once

#include <cstddef>
#include <array>
#include <map>
#include <string>
#include <tuple>
#include <vector>

#include "humanoid_route_runtime/route_runtime_types.hpp"
#include "rapidjson/document.h"

namespace humanoid_route_runtime
{

std::string json_to_string(const rapidjson::Value & value);

std::string read_string_member(
  const rapidjson::Value & object,
  const char * key,
  const std::string & fallback);

bool read_bool_member(
  const rapidjson::Value & object,
  const char * key,
  bool fallback);

std::string extract_revision(
  const rapidjson::Value & data,
  const rapidjson::Value & message);

std::size_t count_waypoints_payload(const rapidjson::Value & data);

bool is_route_task_command(const std::string & command_type);

rapidjson::Document make_extra_data(
  const std::string & error_code,
  const std::string & request_type);

std::string route_task_id(const rapidjson::Value & value);

std::string read_route_task_id_member(
  const rapidjson::Value & object,
  const char * key);

bool route_task_bool(const rapidjson::Value * value, bool default_value);

std::string normalize_route_task_frame_id(
  const rapidjson::Value * value,
  const std::string & default_frame_id);

struct RouteTaskPositionResult
{
  std::array<double, 3> position{0.0, 0.0, 0.0};
  std::string error;
};

struct RouteTaskOrientationResult
{
  std::array<double, 4> orientation{0.0, 0.0, 0.0, 1.0};
  std::string error;
};

RouteTaskPositionResult normalize_route_task_position(const rapidjson::Value * value);

RouteTaskOrientationResult normalize_route_task_orientation(const rapidjson::Value * value);

struct RouteWaypointSourceResult
{
  std::string source;
  std::string error_code;
  std::string message;
};

RouteWaypointSourceResult validate_route_waypoint_source(const rapidjson::Value & command_data);

struct RouteWaypointNormalizeResult
{
  std::vector<RouteWaypoint> waypoints;
  std::string error_code;
  std::string message;
};

RouteWaypointNormalizeResult normalize_route_task_waypoints(
  const rapidjson::Value & route_waypoints,
  const std::string & route_map_id,
  const RouteRuntimeConfig & config);

std::vector<std::string> build_master_route_task_ids(const std::vector<RouteWaypoint> & route_waypoints);

StoredWaypoint stored_waypoint_from_json(
  const std::string & waypoint_id,
  const rapidjson::Value & waypoint,
  const std::string & fallback_map_id,
  const RouteRuntimeConfig & config);

void collect_waypoints_from_bucket(
  const rapidjson::Value & bucket,
  const std::string & fallback_map_id,
  const RouteRuntimeConfig & config,
  std::map<std::string, StoredWaypoint> & output);

std::size_t count_cached_waypoints(
  const std::map<std::string, std::map<std::string, StoredWaypoint>> & by_map);

struct WaypointCacheParseResult
{
  bool accepted{false};
  std::string error;
};

WaypointCacheParseResult update_waypoint_cache_from_message(
  const rapidjson::Value & message,
  WaypointCacheState & cache,
  const RouteRuntimeConfig & config);

}  // namespace humanoid_route_runtime
