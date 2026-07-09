#!/bin/bash
set -eo pipefail
set +u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WORKSPACE:-$SCRIPT_DIR}"
PID_FILE="$WORKSPACE/.start_navigation.pid"

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

wait_for_processes_to_exit() {
  local timeout_sec="$1"
  shift
  local pids=("$@")
  local elapsed=0
  local pid
  local any_alive

  while [ "$elapsed" -lt "$timeout_sec" ]; do
    any_alive=0
    for pid in "${pids[@]}"; do
      if kill -0 "$pid" > /dev/null 2>&1; then
        any_alive=1
        break
      fi
    done

    [ "$any_alive" -eq 1 ] || return 0
    sleep 1
    elapsed=$((elapsed + 1))
  done

  return 1
}

collect_navigation_pids() {
  OLD_PIDS=()

  if [ -f "$PID_FILE" ]; then
    local old_pid
    old_pid="$(tr -d '[:space:]' < "$PID_FILE" || true)"
    if [ -n "$old_pid" ] && kill -0 "$old_pid" > /dev/null 2>&1; then
      add_unique_pid "$old_pid"
    fi
  fi

  if ! command -v pgrep > /dev/null 2>&1; then
    echo "WARN: pgrep not found, can only stop PID from $PID_FILE."
    return 0
  fi

  local patterns=(
    "ros2 launch humanoid_bringup robot_real.launch.py"
    "ros2 launch humanoid_description display.launch.py"
    "ros2 launch humanoid_navigation2"
    "ros2 launch humanoid_control_runtime"
    "ros2 launch humanoid_route_runtime"
    "ros2 launch humanoid_app_gateway_runtime"
    "ros2 launch humanoid_robot_gateway_runtime"
    "ros2 launch humanoid_expression_runtime"
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
    "hdl_global_localization_node"
    "component_container_mt.*hdl_bootstrap_container"
    "scancontext_global_localizer"
    "lidar_localization_node"
    "robosense_lidar_localization_node"
    "hdl_bootstrap_to_initialpose"
    "prior_map_odom_bridge"
    "robot_realpose_publisher"
    "periodic_clearing_publisher"
    "periodic_clearing_3d_publisher"
    "wait_for_tf.*wait_for_localization_tf"
    "nav2_planner.*/planner_server"
    "nav2_controller.*/controller_server"
    "nav2_behaviors.*/behavior_server"
    "nav2_bt_navigator.*/bt_navigator"
    "dynamic_waypoints_manager_cpp"
    "navigation_state_manager"
    "navigation_state_manager_cpp"
    "app_gateway_node"
    "data_integration_node"
    "robot_gateway_node"
    "facial_driver"
    "rviz2.*humanoid_navigation2.*navigation.rviz"
  )

  local pattern
  local pid
  for pattern in "${patterns[@]}"; do
    while IFS= read -r pid; do
      add_unique_pid "$pid"
    done < <(pgrep -f "$pattern" || true)
  done
}

stop_ros_daemon() {
  if [ -f /opt/ros/jazzy/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
  fi
  if [ -f "$WORKSPACE/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "$WORKSPACE/install/setup.bash"
  fi

  if command -v ros2 > /dev/null 2>&1; then
    ros2 daemon stop > /dev/null 2>&1 || true
  fi
}

cd "$WORKSPACE"

collect_navigation_pids

if [ "${#OLD_PIDS[@]}" -eq 0 ]; then
  echo "No navigation processes detected."
  rm -f "$PID_FILE"
  stop_ros_daemon
  exit 0
fi

echo "Detected navigation processes: ${OLD_PIDS[*]}"
echo "Process details before cleanup:"
ps -o pid,ppid,stat,cmd -p "$(IFS=,; echo "${OLD_PIDS[*]}")" || true

echo "Stopping navigation processes with SIGINT..."
kill -INT "${OLD_PIDS[@]}" > /dev/null 2>&1 || true
if wait_for_processes_to_exit 10 "${OLD_PIDS[@]}"; then
  echo "Navigation processes stopped cleanly."
  rm -f "$PID_FILE"
  stop_ros_daemon
  exit 0
fi

echo "Some processes are still alive, sending SIGTERM..."
kill -TERM "${OLD_PIDS[@]}" > /dev/null 2>&1 || true
if wait_for_processes_to_exit 5 "${OLD_PIDS[@]}"; then
  echo "Navigation processes stopped after SIGTERM."
  rm -f "$PID_FILE"
  stop_ros_daemon
  exit 0
fi

echo "Some processes ignored SIGTERM, sending SIGKILL..."
kill -KILL "${OLD_PIDS[@]}" > /dev/null 2>&1 || true
sleep 1
rm -f "$PID_FILE"
stop_ros_daemon

collect_navigation_pids
if [ "${#OLD_PIDS[@]}" -eq 0 ]; then
  echo "Navigation cleanup complete."
else
  echo "WARN: processes still detected after cleanup: ${OLD_PIDS[*]}"
  ps -o pid,ppid,stat,cmd -p "$(IFS=,; echo "${OLD_PIDS[*]}")" || true
fi
