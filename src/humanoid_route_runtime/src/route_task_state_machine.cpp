/*
 * route_task_state_machine.cpp
 *
 * 文件用途：
 * 1. 实现路线任务核心状态机：启动、暂停、恢复、停止、跳点和播报完成。
 * 2. 负责构建 active segment、推进 task 完成状态、维护 completed/skipped 列表。
 * 3. 不直接创建 Nav2 action goal，让 ROS 节点外壳统一处理 action 生命周期和旧 goal 隔离。
 */

#include "humanoid_route_runtime/route_task_state_machine.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

#include "humanoid_route_runtime/route_task_protocol.hpp"
#include "rapidjson/document.h"

namespace humanoid_route_runtime
{

RouteTaskCommandResult RouteTaskStateMachine::handle_command(
  const std::string & command_type,
  const rapidjson::Value & command_data,
  const std::string & start_block_reason,
  const std::string & resume_localization_block_reason,
  const RouteRuntimeConfig & config,
  const MapRuntimeState & map,
  const WaypointCacheState & waypoints,
  RouteTaskRuntimeState & route) const
{
  if (command_type == "start_route_task") {
    return handle_start_route_task(command_data, start_block_reason, config, map, waypoints, route);
  }
  if (command_type == "pause_route_task") {
    return handle_pause_route_task(command_data, route);
  }
  if (command_type == "resume_route_task") {
    return handle_resume_route_task(command_data, resume_localization_block_reason, route);
  }
  if (command_type == "stop_route_task") {
    return handle_stop_route_task(command_data, route);
  }
  if (command_type == "jump_to_waypoint") {
    return handle_jump_to_waypoint(command_data, config, route);
  }
  if (command_type == "broadcast_finished") {
    return handle_broadcast_finished(command_data, route);
  }
  return unsupported_or_deferred(command_type, "unsupported navigation command", "unsupported_command");
}

void RouteTaskStateMachine::reset_route_task_state(RouteTaskRuntimeState & route) const
{
  route.current_goal_generation = -1;
  route.active_goal_generation = 0;
  route.active = false;
  route.task_session_id.clear();
  route.route_id.clear();
  route.map_id.clear();
  route.request_message_id.clear();
  route.route_waypoint_source.clear();
  route.waypoints_revision.clear();
  route.started_at = 0.0;
  route.route_waypoints.clear();
  route.master_route_task_ids.clear();
  route.completed_task_ids.clear();
  route.skipped_task_ids.clear();
  route.current_anchor_task_id.clear();
  route.current_anchor_task_index = -1;
  route.current_target_task_id.clear();
  route.current_target_task_index = -1;
  route.active_segment.reset();
  route.awaiting_broadcast = false;
  route.waiting_broadcast_waypoint_id.clear();
  route.waiting_broadcast_id.clear();
  route.jump_interrupts_broadcast = false;
  route.last_completed_task_id.clear();
  route.last_completed_broadcast_task_session_id.clear();
  route.last_completed_broadcast_waypoint_id.clear();
  route.last_completed_broadcast_id.clear();
  route.current_state = NavigationState::Idle;
  route.detailed_state = "IDLE";
  route.navigation_mode.clear();
  route.current_waypoint_index = 0;
  route.total_waypoints = 0;
  route.navigation_start_time = 0.0;
  route.pause_time = 0.0;
  route.pause_duration_limit = 0.0;
  route.pause_source.clear();
  route.pause_reason.clear();
  route.resume_mode.clear();
}

RouteTaskCommandResult RouteTaskStateMachine::handle_start_route_task(
  const rapidjson::Value & command_data,
  const std::string & start_block_reason,
  const RouteRuntimeConfig & config,
  const MapRuntimeState & map,
  const WaypointCacheState & waypoints,
  RouteTaskRuntimeState & route) const
{
  (void)waypoints;
  if (route.active) {
    return unsupported_or_deferred(
      "start_route_task", "route task already running", "route_task_already_running");
  }
  if (route.current_state == NavigationState::Planning ||
    route.current_state == NavigationState::Executing ||
    route.current_state == NavigationState::Paused)
  {
    return unsupported_or_deferred("start_route_task", "navigation is busy", "navigation_busy");
  }
  if (!start_block_reason.empty()) {
    return unsupported_or_deferred("start_route_task", start_block_reason, "navigation_start_blocked");
  }

  const std::string task_session_id = read_route_task_id_member(command_data, "task_session_id");
  const std::string route_id = read_route_task_id_member(command_data, "route_id");
  const std::string request_message_id = read_route_task_id_member(command_data, "request_message_id");
  const std::string map_id = read_route_task_id_member(command_data, "map_id");

  if (task_session_id.empty()) {
    return unsupported_or_deferred("start_route_task", "task_session_id is required", "missing_task_session_id");
  }
  if (route_id.empty()) {
    return unsupported_or_deferred("start_route_task", "route_id is required", "missing_route_id");
  }
  if (map_id.empty()) {
    return unsupported_or_deferred("start_route_task", "map_id is required", "missing_map_id");
  }

  std::string map_error_code;
  const std::string map_error = validate_active_map_ready(map_id, map, map_error_code);
  if (!map_error.empty()) {
    return unsupported_or_deferred("start_route_task", map_error, map_error_code);
  }

  const auto source = validate_route_waypoint_source(command_data);
  if (!source.error_code.empty()) {
    return unsupported_or_deferred("start_route_task", source.message, source.error_code);
  }
  RouteWaypointNormalizeResult normalized;
  if (source.source == "stored_waypoint_ids") {
    std::string revision_error_code;
    const std::string revision_error =
      validate_waypoints_revision_for_id_mode(command_data, map_id, waypoints, revision_error_code);
    if (!revision_error.empty()) {
      return unsupported_or_deferred("start_route_task", revision_error, revision_error_code);
    }
    normalized =
      build_route_waypoints_from_ids(command_data["route_waypoint_ids"], map_id, waypoints, config);
    if (!normalized.error_code.empty()) {
      return unsupported_or_deferred("start_route_task", normalized.message, normalized.error_code);
    }
  } else {
    normalized = normalize_route_task_waypoints(command_data["route_waypoints"], map_id, config);
    if (!normalized.error_code.empty()) {
      return unsupported_or_deferred("start_route_task", normalized.message, normalized.error_code);
    }
  }

  route.route_task_version += 1;
  route.active_goal_generation = 0;
  route.current_goal_generation = 0;
  route.active = true;
  route.task_session_id = task_session_id;
  route.route_id = route_id;
  route.map_id = map_id;
  route.request_message_id = request_message_id;
  route.route_waypoints = normalized.waypoints;
  route.route_waypoint_source = source.source;
  route.waypoints_revision = read_route_task_id_member(command_data, "waypoints_revision");
  route.started_at = now_seconds();
  route.master_route_task_ids = build_master_route_task_ids(route.route_waypoints);
  route.completed_task_ids.clear();
  route.skipped_task_ids.clear();
  route.current_anchor_task_id.clear();
  route.current_anchor_task_index = -1;
  route.current_target_task_index = 0;
  route.current_target_task_id = route.master_route_task_ids.empty() ? "" : route.master_route_task_ids.front();
  route.awaiting_broadcast = false;
  route.waiting_broadcast_waypoint_id.clear();
  route.waiting_broadcast_id.clear();
  route.last_completed_task_id.clear();
  route.active_segment = build_first_active_segment(route);

  route.current_state = NavigationState::Executing;
  route.detailed_state = "ROUTE_TASK_SEGMENT_READY";
  route.navigation_mode = "route_task";
  route.navigation_start_time = now_seconds();
  route.total_waypoints = static_cast<int>(route.master_route_task_ids.size());
  route.current_waypoint_index = 0;

  RouteTaskCommandResult result;
  result.status = "success";
  result.message = "route task accepted and first segment started";
  result.error_code.clear();
  result.implementation_stage = "route_task_pure_state_ready";
  result.route_state_changed = true;
  return result;
}

RouteTaskCommandResult RouteTaskStateMachine::handle_pause_route_task(
  const rapidjson::Value & command_data,
  RouteTaskRuntimeState & route) const
{
  RouteTaskCommandResult result;
  if (!validate_active_route_task_control("pause_route_task", command_data, route, result)) {
    return result;
  }

  if (route.current_state == NavigationState::Paused) {
    result.status = "success";
    result.message = "route task already paused";
    result.error_code.clear();
    result.result_reason = "route_task_already_paused";
    result.implementation_stage = "route_task_control_state_ready";
    result.event_type = "navigation_paused";
    fill_common_route_event_fields(result, route);
    return result;
  }

  if (route.current_state != NavigationState::Executing && !route.awaiting_broadcast) {
    return unsupported_or_deferred(
      "pause_route_task", "route task is not executing", "invalid_route_task_state");
  }

  double pause_duration = 0.0;
  if (command_data.HasMember("pause_parameters") && command_data["pause_parameters"].IsObject() &&
    command_data["pause_parameters"].HasMember("pause_duration") &&
    command_data["pause_parameters"]["pause_duration"].IsNumber())
  {
    pause_duration = command_data["pause_parameters"]["pause_duration"].GetDouble();
  }
  route.current_state = NavigationState::Paused;
  route.detailed_state = "PAUSED";
  route.pause_time = now_seconds();
  route.pause_duration_limit = pause_duration;
  route.pause_source = "route_task_user_request";
  route.pause_reason = read_string_member(command_data, "reason", "用户手动暂停路线任务");
  route.resume_mode = "manual";

  result.status = "success";
  result.message = "route task paused";
  result.error_code.clear();
  result.result_reason = "route_task_paused";
  result.implementation_stage = "route_task_control_state_ready";
  result.route_state_changed = true;
  result.event_type = "navigation_paused";
  fill_common_route_event_fields(result, route);
  result.event_string_fields["pause_source"] = route.pause_source;
  result.event_string_fields["reason"] = route.pause_reason;
  result.event_string_fields["resume_mode"] = route.resume_mode;
  result.event_bool_fields["route_task"] = true;
  result.event_bool_fields["waiting_for_obstacle_clear"] = false;
  result.event_double_fields["pause_time"] = route.pause_time;
  result.event_double_fields["pause_duration"] = route.pause_duration_limit;
  return result;
}

RouteTaskCommandResult RouteTaskStateMachine::handle_resume_route_task(
  const rapidjson::Value & command_data,
  const std::string & resume_localization_block_reason,
  RouteTaskRuntimeState & route) const
{
  RouteTaskCommandResult result;
  if (!validate_active_route_task_control("resume_route_task", command_data, route, result)) {
    return result;
  }
  if (route.current_state != NavigationState::Paused) {
    return unsupported_or_deferred(
      "resume_route_task", "route task is not paused", "route_task_not_paused");
  }
  if (!resume_localization_block_reason.empty()) {
    (void)resume_localization_block_reason;
    return unsupported_or_deferred(
      "resume_route_task",
      "localization recovery is active, route task resume is blocked",
      "localization_resume_blocked");
  }
  if (!route.active_segment.has_value() && !route.awaiting_broadcast) {
    return unsupported_or_deferred(
      "resume_route_task", "route task segment is missing", "missing_active_segment");
  }

  const double pause_elapsed = route.pause_time > 0.0 ? now_seconds() - route.pause_time : 0.0;
  const std::string resume_source =
    route.pause_source.empty() ? "route_task_user_request" : route.pause_source;
  route.current_state = NavigationState::Executing;
  route.detailed_state = route.awaiting_broadcast ? "WAITING_BROADCAST" : "ROUTE_TASK_SEGMENT_READY";
  route.pause_source.clear();
  route.pause_reason.clear();
  route.resume_mode.clear();
  route.pause_time = 0.0;
  route.pause_duration_limit = 0.0;

  result.status = "success";
  result.message = route.awaiting_broadcast ? "route task broadcast wait resumed" : "route task resumed";
  result.error_code.clear();
  result.result_reason =
    route.awaiting_broadcast ? "route_task_broadcast_wait_resumed" : "route_task_resumed";
  result.implementation_stage = "route_task_control_state_ready";
  result.route_state_changed = true;
  result.event_type = "navigation_resumed";
  fill_common_route_event_fields(result, route);
  result.event_string_fields["resume_reason"] =
    read_string_member(command_data, "reason", "route_task_user_request");
  result.event_string_fields["resume_source"] = resume_source;
  result.event_bool_fields["route_task"] = true;
  result.event_bool_fields["awaiting_broadcast"] = route.awaiting_broadcast;
  result.event_double_fields["pause_duration_actual"] = std::round(pause_elapsed * 10.0) / 10.0;
  return result;
}

RouteTaskCommandResult RouteTaskStateMachine::handle_stop_route_task(
  const rapidjson::Value & command_data,
  RouteTaskRuntimeState & route) const
{
  RouteTaskCommandResult result;
  if (!validate_active_route_task_control("stop_route_task", command_data, route, result)) {
    return result;
  }

  bool emergency_stop = false;
  std::string stop_reason = read_string_member(command_data, "reason", "");
  if (command_data.HasMember("stop_parameters") && command_data["stop_parameters"].IsObject()) {
    const auto & stop_parameters = command_data["stop_parameters"];
    if (stop_parameters.HasMember("emergency_stop")) {
      emergency_stop = route_task_bool(&stop_parameters["emergency_stop"], false);
    }
    if (stop_reason.empty()) {
      stop_reason = read_string_member(stop_parameters, "reason", "route_task_user_stop");
    }
  }
  if (stop_reason.empty()) {
    stop_reason = "route_task_user_stop";
  }

  route.current_state = NavigationState::Cancelled;
  route.detailed_state = "CANCELLED";

  result.status = "success";
  result.message = "route task stopped";
  result.error_code.clear();
  result.result_reason = "route_task_stopped";
  result.implementation_stage = "route_task_control_state_ready";
  result.route_state_changed = true;
  result.event_type = "navigation_stopped";
  result.reset_route_after_publish = true;
  fill_common_route_event_fields(result, route);
  result.event_string_fields["reason"] = stop_reason;
  result.event_bool_fields["route_task"] = true;
  result.event_bool_fields["emergency_stop"] = emergency_stop;
  result.event_string_array_fields["completed_task_ids"] = route.completed_task_ids;
  result.event_string_array_fields["skipped_task_ids"] = route.skipped_task_ids;
  result.event_int_fields["completed_count"] = static_cast<int>(route.completed_task_ids.size());
  result.event_int_fields["skipped_count"] = static_cast<int>(route.skipped_task_ids.size());
  result.event_int_fields["task_count"] = static_cast<int>(route.master_route_task_ids.size());
  result.event_double_fields["stopped_at"] = now_seconds();
  return result;
}

RouteTaskCommandResult RouteTaskStateMachine::handle_jump_to_waypoint(
  const rapidjson::Value & command_data,
  const RouteRuntimeConfig & config,
  RouteTaskRuntimeState & route) const
{
  RouteTaskCommandResult result;
  if (!validate_active_route_task_control("jump_to_waypoint", command_data, route, result)) {
    return result;
  }

  const std::string target_waypoint_id = read_route_task_id_member(command_data, "target_waypoint_id");
  if (target_waypoint_id.empty()) {
    return unsupported_or_deferred(
      "jump_to_waypoint", "target waypoint not found", "invalid_target_waypoint");
  }

  const RouteWaypoint * target_waypoint = find_route_waypoint_by_id(route, target_waypoint_id);
  if (target_waypoint == nullptr) {
    return unsupported_or_deferred(
      "jump_to_waypoint", "target waypoint not found", "invalid_target_waypoint");
  }
  if (target_waypoint->waypoint_role != "task") {
    return unsupported_or_deferred(
      "jump_to_waypoint", "target waypoint is not task", "target_waypoint_not_task");
  }

  if (target_waypoint_id == route.current_target_task_id) {
    result.status = "success";
    result.message = "target is already current route task target";
    result.error_code.clear();
    result.result_reason = "already_current_target";
    result.implementation_stage = "route_task_jump_state_ready";
    result.event_type = "jump_updated";
    fill_common_route_event_fields(result, route);
    result.event_string_fields["target_waypoint_id"] = target_waypoint_id;
    result.event_bool_fields["already_current_target"] = true;
    return result;
  }

  const bool interrupt_broadcast = route_task_bool(
    command_data.HasMember("interrupt_broadcast") ? &command_data["interrupt_broadcast"] : nullptr,
    config.route_task_default_interrupt_broadcast);
  if (route.awaiting_broadcast && !interrupt_broadcast) {
    return unsupported_or_deferred(
      "jump_to_waypoint",
      "interrupt_broadcast=false is not supported",
      "interrupt_broadcast_false_not_supported");
  }

  ActiveRouteSegment new_segment;
  std::vector<std::string> skipped_task_ids;
  std::string error_code;
  std::string message;
  if (!rebuild_segment_from_current_progress(
      target_waypoint_id, route, new_segment, skipped_task_ids, error_code, message))
  {
    return unsupported_or_deferred("jump_to_waypoint", message, error_code);
  }

  std::map<std::string, std::string> interrupted_broadcast;
  if (route.awaiting_broadcast) {
    interrupted_broadcast["waypoint_id"] = route.waiting_broadcast_waypoint_id;
    interrupted_broadcast["broadcast_id"] = route.waiting_broadcast_id;
    if (route.waiting_broadcast_waypoint_id != target_waypoint_id &&
      std::find(
        route.completed_task_ids.begin(), route.completed_task_ids.end(),
        route.waiting_broadcast_waypoint_id) == route.completed_task_ids.end() &&
      std::find(
        skipped_task_ids.begin(), skipped_task_ids.end(),
        route.waiting_broadcast_waypoint_id) == skipped_task_ids.end())
    {
      skipped_task_ids.push_back(route.waiting_broadcast_waypoint_id);
    }
    route.awaiting_broadcast = false;
    route.waiting_broadcast_waypoint_id.clear();
    route.waiting_broadcast_id.clear();
  }

  for (const auto & skipped_id : skipped_task_ids) {
    if (std::find(route.completed_task_ids.begin(), route.completed_task_ids.end(), skipped_id) !=
      route.completed_task_ids.end())
    {
      continue;
    }
    if (std::find(route.skipped_task_ids.begin(), route.skipped_task_ids.end(), skipped_id) ==
      route.skipped_task_ids.end())
    {
      route.skipped_task_ids.push_back(skipped_id);
    }
  }
  route.skipped_task_ids.erase(
    std::remove(route.skipped_task_ids.begin(), route.skipped_task_ids.end(), target_waypoint_id),
    route.skipped_task_ids.end());

  route.current_target_task_id = target_waypoint_id;
  route.current_target_task_index = resolve_route_task_index(route, target_waypoint_id);
  route.current_anchor_task_id = new_segment.segment_start_task_id;
  route.current_anchor_task_index = resolve_route_task_index(route, route.current_anchor_task_id);
  route.active_segment = new_segment;
  route.current_state = NavigationState::Executing;
  route.detailed_state = "JUMP_REPLANNING";
  route.navigation_mode = "route_task";
  route.jump_interrupts_broadcast = interrupt_broadcast;

  result.status = "success";
  result.message = "jump request accepted";
  result.error_code.clear();
  result.result_reason = "jump_segment_rebuilt";
  result.implementation_stage = "route_task_jump_state_ready";
  result.route_state_changed = true;
  result.event_type = "jump_updated";
  fill_common_route_event_fields(result, route);
  result.event_string_fields["target_waypoint_id"] = target_waypoint_id;
  result.event_bool_fields["interrupt_broadcast"] = interrupt_broadcast;
  result.event_bool_fields["interrupted_broadcast_active"] = !interrupted_broadcast.empty();
  result.event_string_array_fields["execution_waypoint_ids"] = new_segment.execution_waypoint_ids;
  result.event_string_array_fields["skipped_task_ids"] = route.skipped_task_ids;
  result.event_string_object_fields["interrupted_broadcast"] = interrupted_broadcast;
  return result;
}

RouteTaskCommandResult RouteTaskStateMachine::handle_broadcast_finished(
  const rapidjson::Value & command_data,
  RouteTaskRuntimeState & route) const
{
  RouteTaskCommandResult result;
  if (!validate_active_route_task_control("broadcast_finished", command_data, route, result)) {
    return result;
  }

  const std::string waypoint_id = read_route_task_id_member(command_data, "waypoint_id");
  const std::string broadcast_id = read_route_task_id_member(command_data, "broadcast_id");
  std::string broadcast_result = read_string_member(command_data, "broadcast_result", "completed");
  std::transform(
    broadcast_result.begin(), broadcast_result.end(), broadcast_result.begin(),
    [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  broadcast_result = trim_copy(broadcast_result);
  if (broadcast_result.empty()) {
    broadcast_result = "";
  }
  if (broadcast_result != "completed") {
    return unsupported_or_deferred(
      "broadcast_finished", "unsupported broadcast result", "unsupported_broadcast_result");
  }

  if (route.task_session_id == route.last_completed_broadcast_task_session_id &&
    waypoint_id == route.last_completed_broadcast_waypoint_id &&
    broadcast_id == route.last_completed_broadcast_id)
  {
    result.status = "success";
    result.message = "duplicate broadcast_finished ignored";
    result.error_code.clear();
    result.result_reason = "duplicate_broadcast_finished";
    result.implementation_stage = "route_task_broadcast_state_ready";
    result.event_type = "broadcast_finished_ignored";
    fill_common_route_event_fields(result, route);
    result.event_string_fields["waypoint_id"] = waypoint_id;
    result.event_string_fields["broadcast_id"] = broadcast_id;
    return result;
  }

  if (!route.awaiting_broadcast) {
    return unsupported_or_deferred(
      "broadcast_finished", "route task is not waiting for broadcast", "broadcast_not_waiting");
  }
  if (waypoint_id != route.waiting_broadcast_waypoint_id || broadcast_id != route.waiting_broadcast_id) {
    return unsupported_or_deferred(
      "broadcast_finished", "broadcast context mismatch", "broadcast_context_mismatch");
  }

  route.last_completed_broadcast_task_session_id = route.task_session_id;
  route.last_completed_broadcast_waypoint_id = waypoint_id;
  route.last_completed_broadcast_id = broadcast_id;

  result = finalize_task_waypoint_completion(waypoint_id, "route_task_broadcast_state_ready", route);
  result.message = "broadcast completion accepted";
  result.result_reason.clear();
  result.implementation_stage = "route_task_broadcast_state_ready";
  return result;
}

RouteTaskCommandResult RouteTaskStateMachine::handle_target_task_arrived(
  RouteTaskRuntimeState & route) const
{
  if (!route.active) {
    return unsupported_or_deferred(
      "target_task_arrived", "route task is not running", "route_task_not_running");
  }
  if (route.current_target_task_id.empty()) {
    return unsupported_or_deferred(
      "target_task_arrived", "current target task is empty", "missing_current_target_task");
  }

  const RouteWaypoint * target_task = find_route_waypoint_by_id(route, route.current_target_task_id);
  if (target_task == nullptr) {
    return unsupported_or_deferred(
      "target_task_arrived", "target task missing", "target_task_missing");
  }
  if (target_task->waypoint_role != "task") {
    return unsupported_or_deferred(
      "target_task_arrived", "current target waypoint is not task", "target_waypoint_not_task");
  }

  if (target_task->need_broadcast) {
    route.awaiting_broadcast = true;
    route.waiting_broadcast_waypoint_id = route.current_target_task_id;
    route.waiting_broadcast_id = target_task->broadcast_id;
    route.current_state = NavigationState::Executing;
    route.detailed_state = "WAITING_BROADCAST";
    route.navigation_mode = "route_task";

    RouteTaskCommandResult result;
    result.status = "success";
    result.message = "target task arrived and broadcast requested";
    result.error_code.clear();
    result.result_reason = "broadcast_requested";
    result.implementation_stage = "route_task_arrival_state_ready";
    result.route_state_changed = true;
    result.event_type = "broadcast_requested";
    fill_common_route_event_fields(result, route);
    result.event_string_fields["waypoint_id"] = route.current_target_task_id;
    result.event_string_fields["broadcast_id"] = target_task->broadcast_id;
    return result;
  }

  auto result = finalize_task_waypoint_completion(
    route.current_target_task_id, "route_task_arrival_state_ready", route);
  result.message = "target task arrived and completed";
  result.result_reason = "target_task_completed_without_broadcast";
  result.implementation_stage = "route_task_arrival_state_ready";
  return result;
}

RouteTaskCommandResult RouteTaskStateMachine::unsupported_or_deferred(
  const std::string & command_type,
  const std::string & message,
  const std::string & error_code) const
{
  (void)command_type;
  RouteTaskCommandResult result;
  result.status = "error";
  result.message = message;
  result.error_code = error_code;
  result.result_reason.clear();
  result.implementation_stage = "route_task_pure_state_ready";
  result.route_state_changed = false;
  result.event_type = "navigation_start_rejected";
  return result;
}

std::string RouteTaskStateMachine::validate_active_map_ready(
  const std::string & route_map_id,
  const MapRuntimeState & map,
  std::string & error_code) const
{
  if (route_map_id.empty()) {
    error_code = "missing_map_id";
    return "map_id is required";
  }
  if (map.active_map_id.empty()) {
    error_code = "map_status_not_ready";
    return "map status is not ready; call switch_map/get_current_map first";
  }
  if (map.active_map_id != route_map_id) {
    error_code = "active_map_mismatch";
    return "active map mismatch: active=" + map.active_map_id + ", route=" + route_map_id;
  }
  if (map.map_state != "ready") {
    error_code = "map_not_ready";
    return "map is not ready: map_id=" + route_map_id + ", map_state=" + map.map_state +
           ", localization_state=" + map.localization_state;
  }
  error_code.clear();
  return "";
}

ActiveRouteSegment RouteTaskStateMachine::build_first_active_segment(RouteTaskRuntimeState & route) const
{
  ActiveRouteSegment segment;
  if (route.master_route_task_ids.empty()) {
    return segment;
  }
  const std::string first_task_id = route.master_route_task_ids.front();
  int target_index = -1;
  for (const auto & waypoint : route.route_waypoints) {
    if (waypoint.waypoint_id == first_task_id) {
      target_index = waypoint.source_index;
      break;
    }
  }

  segment.segment_id =
    "seg_" + std::to_string(route.route_task_version) + "_" +
    std::to_string(route.active_goal_generation + 1);
  segment.segment_direction = "forward";
  segment.segment_start_task_id = "";
  segment.segment_target_task_id = first_task_id;
  segment.segment_start_source_index = -1;
  segment.segment_target_source_index = target_index;
  segment.segment_goal_generation = route.active_goal_generation + 1;

  for (const auto & waypoint : route.route_waypoints) {
    if (waypoint.source_index > target_index || target_index < 0) {
      continue;
    }
    if (waypoint.waypoint_role == "transit") {
      segment.transit_waypoint_ids.push_back(waypoint.waypoint_id);
      segment.execution_waypoint_ids.push_back(waypoint.waypoint_id);
    }
    if (waypoint.waypoint_id == first_task_id) {
      segment.execution_waypoint_ids.push_back(first_task_id);
      break;
    }
  }
  return segment;
}

const RouteWaypoint * RouteTaskStateMachine::find_route_waypoint_by_id(
  const RouteTaskRuntimeState & route,
  const std::string & waypoint_id) const
{
  for (const auto & waypoint : route.route_waypoints) {
    if (waypoint.waypoint_id == waypoint_id) {
      return &waypoint;
    }
  }
  return nullptr;
}

int RouteTaskStateMachine::resolve_current_progress_source_index(
  const RouteTaskRuntimeState & route) const
{
  if (route.awaiting_broadcast && !route.current_target_task_id.empty()) {
    const RouteWaypoint * current_target = find_route_waypoint_by_id(route, route.current_target_task_id);
    return current_target != nullptr ? current_target->source_index : -1;
  }
  if (!route.active_segment.has_value()) {
    return -1;
  }

  const auto & segment = route.active_segment.value();
  if (!segment.passed_transit_waypoint_ids.empty()) {
    const RouteWaypoint * passed =
      find_route_waypoint_by_id(route, segment.passed_transit_waypoint_ids.back());
    if (passed != nullptr) {
      return passed->source_index;
    }
  }
  return segment.segment_start_source_index;
}

std::string RouteTaskStateMachine::resolve_current_progress_anchor_task_id(
  const RouteTaskRuntimeState & route) const
{
  if (route.awaiting_broadcast && !route.current_target_task_id.empty()) {
    return route.current_target_task_id;
  }
  if (!route.current_anchor_task_id.empty()) {
    return route.current_anchor_task_id;
  }
  if (!route.last_completed_task_id.empty()) {
    return route.last_completed_task_id;
  }
  if (!route.completed_task_ids.empty()) {
    return route.completed_task_ids.back();
  }
  return "";
}

int RouteTaskStateMachine::resolve_route_task_index(
  const RouteTaskRuntimeState & route,
  const std::string & task_id) const
{
  if (task_id.empty()) {
    return -1;
  }
  const auto it = std::find(route.master_route_task_ids.begin(), route.master_route_task_ids.end(), task_id);
  if (it == route.master_route_task_ids.end()) {
    return -1;
  }
  return static_cast<int>(std::distance(route.master_route_task_ids.begin(), it));
}

std::string RouteTaskStateMachine::compute_segment_direction(
  const int start_source_index,
  const int target_source_index) const
{
  return target_source_index >= start_source_index ? "forward" : "backward";
}

std::vector<RouteWaypoint> RouteTaskStateMachine::collect_route_interval_waypoints(
  const RouteTaskRuntimeState & route,
  const int start_source_index,
  const int target_source_index) const
{
  std::vector<RouteWaypoint> interval;
  if (target_source_index >= start_source_index) {
    for (const auto & waypoint : route.route_waypoints) {
      if (waypoint.source_index > start_source_index && waypoint.source_index <= target_source_index) {
        interval.push_back(waypoint);
      }
    }
    return interval;
  }

  for (auto it = route.route_waypoints.rbegin(); it != route.route_waypoints.rend(); ++it) {
    if (it->source_index >= target_source_index && it->source_index < start_source_index) {
      interval.push_back(*it);
    }
  }
  return interval;
}

bool RouteTaskStateMachine::rebuild_segment_from_current_progress(
  const std::string & target_waypoint_id,
  const RouteTaskRuntimeState & route,
  ActiveRouteSegment & segment,
  std::vector<std::string> & skipped_task_ids,
  std::string & error_code,
  std::string & message) const
{
  const RouteWaypoint * target = find_route_waypoint_by_id(route, target_waypoint_id);
  if (target == nullptr) {
    error_code = "invalid_target_waypoint";
    message = "target waypoint not found";
    return false;
  }
  if (target->waypoint_role != "task") {
    error_code = "target_waypoint_not_task";
    message = "target waypoint must be a task waypoint";
    return false;
  }

  const int start_source_index = resolve_current_progress_source_index(route);
  const int target_source_index = target->source_index;
  const std::string anchor_task_id = resolve_current_progress_anchor_task_id(route);
  const auto interval = collect_route_interval_waypoints(route, start_source_index, target_source_index);

  segment.segment_id =
    "seg_" + std::to_string(route.route_task_version) + "_" +
    std::to_string(route.active_goal_generation + 1);
  segment.segment_direction = compute_segment_direction(start_source_index, target_source_index);
  segment.segment_start_task_id = anchor_task_id;
  segment.segment_target_task_id = target_waypoint_id;
  segment.segment_start_source_index = start_source_index;
  segment.segment_target_source_index = target_source_index;
  segment.current_segment_progress_index = 0;
  segment.segment_goal_generation = route.active_goal_generation + 1;

  for (const auto & waypoint : interval) {
    if (waypoint.waypoint_role == "transit") {
      if (route.active_segment.has_value()) {
        const auto & passed = route.active_segment->passed_transit_waypoint_ids;
        if (std::find(passed.begin(), passed.end(), waypoint.waypoint_id) != passed.end()) {
          continue;
        }
      }
      segment.transit_waypoint_ids.push_back(waypoint.waypoint_id);
      segment.execution_waypoint_ids.push_back(waypoint.waypoint_id);
      continue;
    }

    if (waypoint.waypoint_role == "task") {
      if (waypoint.waypoint_id == target_waypoint_id) {
        segment.execution_waypoint_ids.push_back(waypoint.waypoint_id);
        continue;
      }
      if (std::find(
          route.completed_task_ids.begin(), route.completed_task_ids.end(), waypoint.waypoint_id) ==
        route.completed_task_ids.end())
      {
        skipped_task_ids.push_back(waypoint.waypoint_id);
      }
    }
  }

  if (segment.execution_waypoint_ids.empty()) {
    error_code = "empty_jump_segment";
    message = "jump target produced empty execution segment";
    return false;
  }
  error_code.clear();
  message.clear();
  return true;
}

bool RouteTaskStateMachine::build_next_active_segment(
  const RouteTaskRuntimeState & route,
  ActiveRouteSegment & segment,
  std::string & error_code,
  std::string & message) const
{
  const int next_task_index = route.current_target_task_index + 1;
  if (next_task_index >= static_cast<int>(route.master_route_task_ids.size())) {
    error_code = "route_task_complete";
    message.clear();
    return false;
  }

  const std::string anchor_task_id = route.current_target_task_id;
  const std::string next_task_id = route.master_route_task_ids[next_task_index];
  const RouteWaypoint * anchor_task = find_route_waypoint_by_id(route, anchor_task_id);
  const RouteWaypoint * next_task = find_route_waypoint_by_id(route, next_task_id);
  if (anchor_task == nullptr || next_task == nullptr) {
    error_code = "next_segment_waypoint_missing";
    message = "next segment waypoint missing";
    return false;
  }

  const auto interval =
    collect_route_interval_waypoints(route, anchor_task->source_index, next_task->source_index);
  segment.segment_id =
    "seg_" + std::to_string(route.route_task_version) + "_" +
    std::to_string(route.active_goal_generation + 1);
  segment.segment_direction =
    compute_segment_direction(anchor_task->source_index, next_task->source_index);
  segment.segment_start_task_id = anchor_task_id;
  segment.segment_target_task_id = next_task_id;
  segment.segment_start_source_index = anchor_task->source_index;
  segment.segment_target_source_index = next_task->source_index;
  segment.current_segment_progress_index = 0;
  segment.segment_goal_generation = route.active_goal_generation + 1;

  for (const auto & waypoint : interval) {
    if (waypoint.waypoint_role == "transit") {
      segment.transit_waypoint_ids.push_back(waypoint.waypoint_id);
      segment.execution_waypoint_ids.push_back(waypoint.waypoint_id);
    } else if (waypoint.waypoint_id == next_task_id) {
      segment.execution_waypoint_ids.push_back(waypoint.waypoint_id);
    }
  }

  if (segment.execution_waypoint_ids.empty()) {
    error_code = "empty_next_segment";
    message = "next route segment is empty";
    return false;
  }
  error_code.clear();
  message.clear();
  return true;
}

RouteTaskCommandResult RouteTaskStateMachine::finalize_task_waypoint_completion(
  const std::string & waypoint_id,
  const std::string & implementation_stage,
  RouteTaskRuntimeState & route) const
{
  if (std::find(route.completed_task_ids.begin(), route.completed_task_ids.end(), waypoint_id) ==
    route.completed_task_ids.end())
  {
    route.completed_task_ids.push_back(waypoint_id);
  }
  route.skipped_task_ids.erase(
    std::remove(route.skipped_task_ids.begin(), route.skipped_task_ids.end(), waypoint_id),
    route.skipped_task_ids.end());
  route.last_completed_task_id = waypoint_id;
  route.awaiting_broadcast = false;
  route.waiting_broadcast_waypoint_id.clear();
  route.waiting_broadcast_id.clear();

  RouteTaskCommandResult result;
  result.status = "success";
  result.message = "task waypoint completed";
  result.error_code.clear();
  result.result_reason = "task_waypoint_completed";
  result.implementation_stage = implementation_stage;
  result.route_state_changed = true;
  result.event_type = "task_waypoint_completed";
  fill_common_route_event_fields(result, route);
  result.event_string_fields["waypoint_id"] = waypoint_id;
  result.event_string_array_fields["completed_task_ids"] = route.completed_task_ids;
  result.event_string_array_fields["skipped_task_ids"] = route.skipped_task_ids;

  const int next_task_index = route.current_target_task_index + 1;
  const std::string next_task_id =
    next_task_index < static_cast<int>(route.master_route_task_ids.size()) ?
    route.master_route_task_ids[next_task_index] : "";
  result.event_string_fields["next_target_task_id"] = next_task_id;

  ActiveRouteSegment next_segment;
  std::string error_code;
  std::string message;
  if (!build_next_active_segment(route, next_segment, error_code, message)) {
    if (error_code == "route_task_complete") {
      RouteTaskEventData complete_event;
      complete_event.event_type = "route_task_completed";
      complete_event.event_string_fields["task_session_id"] = route.task_session_id;
      complete_event.event_string_fields["route_id"] = route.route_id;
      complete_event.event_string_fields["map_id"] = route.map_id;
      complete_event.event_string_fields["completed_waypoint_id"] = route.last_completed_task_id;
      complete_event.event_string_fields["result"] = "success";
      complete_event.event_string_array_fields["completed_task_ids"] = route.completed_task_ids;
      complete_event.event_string_array_fields["skipped_task_ids"] = route.skipped_task_ids;
      complete_event.event_double_fields["completed_at"] = now_seconds();
      complete_event.event_int_fields["task_count"] = static_cast<int>(route.master_route_task_ids.size());
      complete_event.event_int_fields["completed_count"] = static_cast<int>(route.completed_task_ids.size());
      complete_event.event_int_fields["skipped_count"] = static_cast<int>(route.skipped_task_ids.size());
      result.followup_events.push_back(complete_event);
      route.current_state = NavigationState::Completed;
      route.detailed_state = "COMPLETED";
      result.reset_route_after_publish = true;
      return result;
    }
    result.status = "error";
    result.message = message;
    result.error_code = error_code;
    result.result_reason.clear();
    result.event_type = "navigation_start_rejected";
    return result;
  }

  route.current_anchor_task_id = route.current_target_task_id;
  route.current_anchor_task_index = route.current_target_task_index;
  route.current_target_task_index = next_task_index;
  route.current_target_task_id = next_task_id;
  route.active_segment = next_segment;
  route.current_state = NavigationState::Executing;
  route.detailed_state = "ROUTE_TASK_SEGMENT_READY";
  route.navigation_mode = "route_task";

  RouteTaskEventData next_event;
  next_event.event_type = "next_route_segment_prepared";
  fill_common_route_event_fields(next_event, route);
  next_event.event_string_array_fields["execution_waypoint_ids"] = route.active_segment->execution_waypoint_ids;
  result.followup_events.push_back(next_event);
  return result;
}

std::string RouteTaskStateMachine::validate_waypoints_revision_for_id_mode(
  const rapidjson::Value & command_data,
  const std::string & map_id,
  const WaypointCacheState & waypoints,
  std::string & error_code) const
{
  const std::string requested_revision = read_route_task_id_member(command_data, "waypoints_revision");
  if (requested_revision.empty()) {
    error_code = "missing_waypoints_revision";
    return "route_waypoint_ids mode requires waypoints_revision";
  }
  if (map_id.empty()) {
    error_code = "missing_map_id";
    return "route_waypoint_ids mode requires map_id";
  }

  std::string current_revision;
  const auto revision_it = waypoints.revisions_by_map.find(map_id);
  if (revision_it != waypoints.revisions_by_map.end()) {
    current_revision = revision_it->second;
  }
  if (current_revision.empty() && map_id == waypoints.map_id) {
    current_revision = waypoints.revision;
  }
  if (current_revision.empty()) {
    error_code = "waypoints_cache_not_ready";
    return "waypoints cache revision is not ready";
  }
  if (requested_revision != current_revision) {
    error_code = "waypoints_revision_mismatch";
    return "waypoints_revision mismatch: map_id=" + map_id + ", app=" + requested_revision +
           ", ros=" + current_revision;
  }
  error_code.clear();
  return "";
}

std::vector<std::string> RouteTaskStateMachine::normalize_route_waypoint_ids(
  const rapidjson::Value & route_waypoint_ids,
  std::string & error_code,
  std::string & message) const
{
  std::vector<std::string> ids;
  if (!route_waypoint_ids.IsArray()) {
    error_code = "invalid_route_waypoint_ids";
    message = "route_waypoint_ids must be an array";
    return ids;
  }
  for (rapidjson::SizeType index = 0; index < route_waypoint_ids.Size(); ++index) {
    const std::string waypoint_id = route_task_id(route_waypoint_ids[index]);
    if (waypoint_id.empty()) {
      error_code = "invalid_route_waypoint_ids";
      message = "route_waypoint_ids[" + std::to_string(index) + "] is empty";
      ids.clear();
      return ids;
    }
    if (std::find(ids.begin(), ids.end(), waypoint_id) != ids.end()) {
      error_code = "duplicate_waypoint_id";
      message = "route_waypoint_ids contains duplicate waypoint_id: " + waypoint_id;
      ids.clear();
      return ids;
    }
    ids.push_back(waypoint_id);
  }
  if (ids.empty()) {
    error_code = "invalid_route_waypoint_ids";
    message = "route_waypoint_ids must not be empty";
    return ids;
  }
  error_code.clear();
  message.clear();
  return ids;
}

RouteWaypointNormalizeResult RouteTaskStateMachine::build_route_waypoints_from_ids(
  const rapidjson::Value & route_waypoint_ids,
  const std::string & map_id,
  const WaypointCacheState & waypoints,
  const RouteRuntimeConfig & config) const
{
  RouteWaypointNormalizeResult result;
  if (map_id.empty()) {
    result.error_code = "missing_map_id";
    result.message = "route_waypoint_ids mode requires map_id";
    return result;
  }
  const auto bucket_it = waypoints.waypoints_by_map.find(map_id);
  if (bucket_it == waypoints.waypoints_by_map.end() || bucket_it->second.empty()) {
    result.error_code = "waypoints_cache_not_ready";
    result.message = "waypoints cache is empty for map_id=" + map_id;
    return result;
  }

  std::string id_error_code;
  std::string id_message;
  const auto ids = normalize_route_waypoint_ids(route_waypoint_ids, id_error_code, id_message);
  if (!id_error_code.empty()) {
    result.error_code = id_error_code;
    result.message = id_message;
    return result;
  }

  int task_count = 0;
  for (std::size_t index = 0; index < ids.size(); ++index) {
    const auto stored_it = bucket_it->second.find(ids[index]);
    if (stored_it == bucket_it->second.end()) {
      result.error_code = "waypoint_id_not_found";
      result.message =
        "route_waypoint_ids[" + std::to_string(index) + "] waypoint_id not found: " +
        ids[index] + " in map_id=" + map_id;
      result.waypoints.clear();
      return result;
    }

    const auto & stored = stored_it->second;
    RouteWaypoint waypoint;
    waypoint.waypoint_id = ids[index];
    waypoint.map_id = map_id;
    waypoint.waypoint_name = stored.waypoint_name.empty() ? ids[index] : stored.waypoint_name;
    waypoint.waypoint_role = stored.waypoint_role;
    waypoint.frame_id = stored.frame_id.empty() ? config.default_frame_id : stored.frame_id;
    waypoint.source_index = static_cast<int>(index);
    waypoint.position = stored.position;
    waypoint.orientation = stored.orientation;
    waypoint.need_broadcast = stored.need_broadcast;
    waypoint.broadcast_id = stored.broadcast_id;
    waypoint.broadcast_text = stored.broadcast_text;
    waypoint.broadcast_blocking = stored.broadcast_blocking;
    waypoint.stop_and_align = stored.stop_and_align;
    waypoint.walk_direction = stored.walk_direction.empty() ? "forward" : stored.walk_direction;

    if (!stored.has_position) {
      result.error_code = "missing_waypoint_pose";
      result.message = "waypoint " + waypoint.waypoint_id + " invalid position: position must be an array";
      result.waypoints.clear();
      return result;
    }
    if (!stored.has_orientation) {
      result.error_code = "missing_waypoint_pose";
      result.message =
        "waypoint " + waypoint.waypoint_id + " invalid orientation: orientation must be a quaternion array";
      result.waypoints.clear();
      return result;
    }
    if (waypoint.waypoint_role != "task" && waypoint.waypoint_role != "transit") {
      result.error_code = "invalid_waypoint_role";
      result.message = "waypoint " + waypoint.waypoint_id + " missing valid waypoint_role";
      result.waypoints.clear();
      return result;
    }
    if (waypoint.waypoint_role == "task") {
      if (waypoint.need_broadcast && waypoint.broadcast_id.empty()) {
        result.error_code = "missing_broadcast_id";
        result.message =
          "waypoint " + waypoint.waypoint_id + " need_broadcast=true but missing broadcast_id";
        result.waypoints.clear();
        return result;
      }
      ++task_count;
    } else {
      waypoint.need_broadcast = false;
      waypoint.broadcast_id.clear();
      waypoint.broadcast_blocking = false;
      waypoint.stop_and_align = false;
      waypoint.walk_direction = "forward";
    }
    result.waypoints.push_back(waypoint);
  }
  if (task_count == 0) {
    result.waypoints.clear();
    result.error_code = "missing_task_waypoints";
    result.message = "route must contain at least one task waypoint";
  }
  return result;
}

bool RouteTaskStateMachine::validate_active_route_task_control(
  const std::string & command_type,
  const rapidjson::Value & command_data,
  const RouteTaskRuntimeState & route,
  RouteTaskCommandResult & result) const
{
  if (!route.active) {
    result = unsupported_or_deferred(command_type, "route task is not running", "route_task_not_running");
    return false;
  }

  const std::string task_session_id = read_route_task_id_member(command_data, "task_session_id");
  const std::string route_id = read_route_task_id_member(command_data, "route_id");
  if (task_session_id != route.task_session_id) {
    result = unsupported_or_deferred(command_type, "invalid task session", "invalid_task_session");
    return false;
  }
  if (route_id != route.route_id) {
    result = unsupported_or_deferred(command_type, "invalid route id", "invalid_route_id");
    return false;
  }
  return true;
}

void RouteTaskStateMachine::fill_common_route_event_fields(
  RouteTaskEventData & result,
  const RouteTaskRuntimeState & route) const
{
  result.event_string_fields["task_session_id"] = route.task_session_id;
  result.event_string_fields["route_id"] = route.route_id;
  result.event_string_fields["map_id"] = route.map_id;
  result.event_string_fields["current_target_task_id"] = route.current_target_task_id;
  result.event_string_fields["waiting_broadcast_waypoint_id"] = route.waiting_broadcast_waypoint_id;
  result.event_string_fields["waiting_broadcast_id"] = route.waiting_broadcast_id;
  if (route.active_segment.has_value()) {
    result.event_string_fields["segment_id"] = route.active_segment->segment_id;
    result.event_string_fields["segment_direction"] = route.active_segment->segment_direction;
  } else {
    result.event_string_fields["segment_id"] = "";
    result.event_string_fields["segment_direction"] = "";
  }
  result.event_int_fields["current_target_task_index"] = route.current_target_task_index;
  result.event_int_fields["completed_count"] = static_cast<int>(route.completed_task_ids.size());
  result.event_int_fields["skipped_count"] = static_cast<int>(route.skipped_task_ids.size());
  result.event_int_fields["task_count"] = static_cast<int>(route.master_route_task_ids.size());
  result.event_bool_fields["awaiting_broadcast"] = route.awaiting_broadcast;
}

void RouteTaskStateMachine::fill_common_route_event_fields(
  RouteTaskCommandResult & result,
  const RouteTaskRuntimeState & route) const
{
  fill_common_route_event_fields(static_cast<RouteTaskEventData &>(result), route);
}

}  // namespace humanoid_route_runtime
