/*
 * route_task_state_machine.hpp
 *
 * 文件用途：
 * 1. 负责路线任务核心状态机的模块边界。
 * 2. 处理 start、pause、resume、stop、jump、broadcast_finished 等路线任务命令。
 * 3. 只负责路线任务运行态推进和事件数据组装，Nav2 goal 的发送/取消由 ROS 节点外壳统一处理。
 *
 * 代码块顺序：
 * 1. 命令处理结果结构。
 * 2. 状态机类接口。
 */

#pragma once

#include <map>
#include <string>
#include <vector>

#include "humanoid_route_runtime/route_runtime_types.hpp"
#include "humanoid_route_runtime/route_task_protocol.hpp"
#include "rapidjson/document.h"

namespace humanoid_route_runtime
{

struct RouteTaskEventData
{
  std::string event_type;
  std::map<std::string, std::string> event_string_fields;
  std::map<std::string, bool> event_bool_fields;
  std::map<std::string, double> event_double_fields;
  std::map<std::string, int> event_int_fields;
  std::map<std::string, std::vector<std::string>> event_string_array_fields;
  std::map<std::string, std::map<std::string, std::string>> event_string_object_fields;
};

struct RouteTaskCommandResult : public RouteTaskEventData
{
  std::string status{"error"};
  std::string message{"路线任务命令处理失败"};
  std::string error_code{"route_task_command_failed"};
  std::string result_reason;
  std::string implementation_stage{"protocol_layer_only"};
  bool route_state_changed{false};
  bool reset_route_after_publish{false};
  std::vector<RouteTaskEventData> followup_events;
};

class RouteTaskStateMachine
{
public:
  RouteTaskCommandResult handle_command(
    const std::string & command_type,
    const rapidjson::Value & command_data,
    const std::string & start_block_reason,
    const std::string & resume_localization_block_reason,
    const RouteRuntimeConfig & config,
    const MapRuntimeState & map,
    const WaypointCacheState & waypoints,
    RouteTaskRuntimeState & route) const;

  void reset_route_task_state(RouteTaskRuntimeState & route) const;

  RouteTaskCommandResult handle_target_task_arrived(RouteTaskRuntimeState & route) const;

private:
  RouteTaskCommandResult handle_start_route_task(
    const rapidjson::Value & command_data,
    const std::string & start_block_reason,
    const RouteRuntimeConfig & config,
    const MapRuntimeState & map,
    const WaypointCacheState & waypoints,
    RouteTaskRuntimeState & route) const;

  RouteTaskCommandResult handle_pause_route_task(
    const rapidjson::Value & command_data,
    RouteTaskRuntimeState & route) const;

  RouteTaskCommandResult handle_resume_route_task(
    const rapidjson::Value & command_data,
    const std::string & resume_localization_block_reason,
    RouteTaskRuntimeState & route) const;

  RouteTaskCommandResult handle_stop_route_task(
    const rapidjson::Value & command_data,
    RouteTaskRuntimeState & route) const;

  RouteTaskCommandResult handle_jump_to_waypoint(
    const rapidjson::Value & command_data,
    const RouteRuntimeConfig & config,
    RouteTaskRuntimeState & route) const;

  RouteTaskCommandResult handle_broadcast_finished(
    const rapidjson::Value & command_data,
    RouteTaskRuntimeState & route) const;

  RouteTaskCommandResult unsupported_or_deferred(
    const std::string & command_type,
    const std::string & message,
    const std::string & error_code) const;

  std::string validate_active_map_ready(
    const std::string & route_map_id,
    const MapRuntimeState & map,
    std::string & error_code) const;

  ActiveRouteSegment build_first_active_segment(RouteTaskRuntimeState & route) const;

  const RouteWaypoint * find_route_waypoint_by_id(
    const RouteTaskRuntimeState & route,
    const std::string & waypoint_id) const;

  int resolve_current_progress_source_index(const RouteTaskRuntimeState & route) const;

  std::string resolve_current_progress_anchor_task_id(const RouteTaskRuntimeState & route) const;

  int resolve_route_task_index(
    const RouteTaskRuntimeState & route,
    const std::string & task_id) const;

  std::string compute_segment_direction(
    int start_source_index,
    int target_source_index) const;

  std::vector<RouteWaypoint> collect_route_interval_waypoints(
    const RouteTaskRuntimeState & route,
    int start_source_index,
    int target_source_index) const;

  bool rebuild_segment_from_current_progress(
    const std::string & target_waypoint_id,
    const RouteTaskRuntimeState & route,
    ActiveRouteSegment & segment,
    std::vector<std::string> & skipped_task_ids,
    std::string & error_code,
    std::string & message) const;

  bool build_next_active_segment(
    const RouteTaskRuntimeState & route,
    ActiveRouteSegment & segment,
    std::string & error_code,
    std::string & message) const;

  RouteTaskCommandResult finalize_task_waypoint_completion(
    const std::string & waypoint_id,
    const std::string & implementation_stage,
    RouteTaskRuntimeState & route) const;

  void fill_common_route_event_fields(
    RouteTaskEventData & event,
    const RouteTaskRuntimeState & route) const;

  std::string validate_waypoints_revision_for_id_mode(
    const rapidjson::Value & command_data,
    const std::string & map_id,
    const WaypointCacheState & waypoints,
    std::string & error_code) const;

  std::vector<std::string> normalize_route_waypoint_ids(
    const rapidjson::Value & route_waypoint_ids,
    std::string & error_code,
    std::string & message) const;

  RouteWaypointNormalizeResult build_route_waypoints_from_ids(
    const rapidjson::Value & route_waypoint_ids,
    const std::string & map_id,
    const WaypointCacheState & waypoints,
    const RouteRuntimeConfig & config) const;

  bool validate_active_route_task_control(
    const std::string & command_type,
    const rapidjson::Value & command_data,
    const RouteTaskRuntimeState & route,
    RouteTaskCommandResult & result) const;

  void fill_common_route_event_fields(
    RouteTaskCommandResult & result,
    const RouteTaskRuntimeState & route) const;
};

}  // namespace humanoid_route_runtime
