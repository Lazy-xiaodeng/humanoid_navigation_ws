#!/bin/bash
set -eo pipefail
set +u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WORKSPACE:-$SCRIPT_DIR}"
PID_FILE="$WORKSPACE/.navigation_stack.pid"
OLD_PIDS=()

add_unique_pid() {
  local pid="$1"
  local existing
  [ -n "$pid" ] || return 0
  [[ "$pid" =~ ^[0-9]+$ ]] || return 0
  [ "$pid" -gt 1 ] || return 0
  [ "$pid" != "$$" ] || return 0
  for existing in "${OLD_PIDS[@]}"; do
    [ "$existing" != "$pid" ] || return 0
  done
  OLD_PIDS+=("$pid")
}

collect_navigation_stack_pids() {
  OLD_PIDS=()
  if [ -f "$PID_FILE" ]; then
    local old_pid
    old_pid="$(tr -d '[:space:]' < "$PID_FILE" || true)"
    if [ -n "$old_pid" ] && kill -0 "$old_pid" >/dev/null 2>&1; then
      add_unique_pid "$old_pid"
    fi
  fi
  if ! command -v pgrep >/dev/null 2>&1; then
    return 0
  fi
  local patterns=(
    "ros2 launch humanoid_bringup robot_navigation_stack.launch.py"
    "ros2 launch humanoid_description display.launch.py"
    "ros2 launch humanoid_navigation2"
    "ros2 launch humanoid_navigation navigation_route_runtime.launch.py"
    "robot_state_publisher"
    "joint_state_publisher"
    "rslidar_sdk_node"
    "fastlio_mapping"
    "point_cloud_filter_node"
    "static_transform_publisher.*tf_odom_to_camera_init"
    "static_transform_publisher.*tf_map_to_ground"
    "static_transform_publisher.*tf_body_to_base_footprint"
    "static_transform_publisher.*tf_base_footprint_to_clearing_lidar"
    "dynamic_odom_ground_publisher"
    "nav2_map_server.*/map_server"
    "nav2_lifecycle_manager.*/lifecycle_manager"
    "robosense_lidar_localization_node"
    "open3d_loc"
    "fastlio_open3d_axis_adapter"
    "prior_map_odom_bridge"
    "robot_realpose_publisher"
    "periodic_clearing_publisher"
    "periodic_clearing_3d_publisher"
    "wait_for_tf.*wait_for_localization_tf"
    "nav2_planner.*/planner_server"
    "nav2_controller.*/controller_server"
    "nav2_behaviors.*/behavior_server"
    "nav2_bt_navigator.*/bt_navigator"
    "navigation_state_manager"
    "rviz2.*humanoid_navigation2.*navigation.rviz"
  )
  local pattern pid
  for pattern in "${patterns[@]}"; do
    while IFS= read -r pid; do
      add_unique_pid "$pid"
    done < <(pgrep -f "$pattern" || true)
  done
}

wait_for_exit() {
  local timeout="$1"
  shift
  local pids=("$@")
  local elapsed=0
  local pid alive
  while [ "$elapsed" -lt "$timeout" ]; do
    alive=0
    for pid in "${pids[@]}"; do
      if kill -0 "$pid" >/dev/null 2>&1; then
        alive=1
        break
      fi
    done
    [ "$alive" -eq 0 ] && return 0
    sleep 1
    elapsed=$((elapsed + 1))
  done
  return 1
}

collect_navigation_stack_pids
if [ "${#OLD_PIDS[@]}" -eq 0 ]; then
  echo "No navigation stack processes detected."
  rm -f "$PID_FILE"
  exit 0
fi

echo "Stopping navigation stack processes: ${OLD_PIDS[*]}"
kill -INT "${OLD_PIDS[@]}" >/dev/null 2>&1 || true
if wait_for_exit 10 "${OLD_PIDS[@]}"; then
  echo "Navigation stack stopped cleanly."
  rm -f "$PID_FILE"
  exit 0
fi

kill -TERM "${OLD_PIDS[@]}" >/dev/null 2>&1 || true
if wait_for_exit 5 "${OLD_PIDS[@]}"; then
  echo "Navigation stack stopped after SIGTERM."
  rm -f "$PID_FILE"
  exit 0
fi

kill -KILL "${OLD_PIDS[@]}" >/dev/null 2>&1 || true
sleep 1
rm -f "$PID_FILE"
echo "Navigation stack cleanup complete."
