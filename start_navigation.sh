#!/bin/bash
set -eo pipefail
set +u

WORKSPACE="${WORKSPACE:-$HOME/humanoid_ws}"
START_TIME="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$WORKSPACE/debug_logs"
LOG_FILE="$LOG_DIR/start_navigation_${START_TIME}.log"
ROS_LOG_DIR="$LOG_DIR/ros_${START_TIME}"
PID_FILE="$WORKSPACE/.start_navigation.pid"

mkdir -p "$LOG_DIR" "$ROS_LOG_DIR"
export ROS_LOG_DIR

exec > >(tee -a "$LOG_FILE") 2>&1

echo "Starting Humanoid Navigation at $(date '+%F %T %z')"
echo "Debug log: $LOG_FILE"
echo "ROS log dir: $ROS_LOG_DIR"

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

collect_old_navigation_pids() {
  OLD_PIDS=()

  if [ -f "$PID_FILE" ]; then
    local old_pid
    old_pid="$(tr -d '[:space:]' < "$PID_FILE" || true)"
    if [ -n "$old_pid" ] && kill -0 "$old_pid" > /dev/null 2>&1; then
      add_unique_pid "$old_pid"
    fi
  fi

  if ! command -v pgrep > /dev/null 2>&1; then
    echo "WARN: pgrep not found, skip process pattern cleanup."
    return 0
  fi

  local patterns=(
    "ros2 launch humanoid_bringup robot_real.launch.py"
    "ros2 launch humanoid_description display.launch.py"
    "ros2 launch humanoid_navigation2 navigation_stack.launch.py"
    "ros2 launch humanoid_navigation navigation.launch.py"
    "ros2 launch humanoid_websocket websocket_server.launch.py"
    "ros2 launch humanoid_locomotion locomotion.launch.py"
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
    "nav2_lifecycle_manager.*/lifecycle_manager.*lifecycle_manager_(map|ndt|navigation)"
    "hdl_global_localization_node"
    "component_container_mt.*hdl_bootstrap_container"
    "scancontext_global_localizer"
    "lidar_localization_node"
    "hdl_bootstrap_to_initialpose"
    "robot_realpose_publisher"
    "periodic_clearing_publisher"
    "periodic_clearing_3d_publisher"
    "wait_for_tf.*wait_for_localization_tf"
    "nav2_planner.*/planner_server"
    "nav2_controller.*/controller_server"
    "nav2_behaviors.*/behavior_server"
    "nav2_bt_navigator.*/bt_navigator"
    "dynamic_waypoints_manager"
    "navigation_state_manager_recoverable"
    "websocket_server_node"
    "data_integration_node_recoverable"
    "websocket_client_node"
    "message_bridge_node"
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

cleanup_old_navigation_processes() {
  if [ -x "$WORKSPACE/stop_navigation.sh" ]; then
    "$WORKSPACE/stop_navigation.sh"
    return 0
  fi

  collect_old_navigation_pids

  if [ "${#OLD_PIDS[@]}" -eq 0 ]; then
    echo "No previous navigation processes detected."
    rm -f "$PID_FILE"
    return 0
  fi

  echo "Detected previous navigation processes: ${OLD_PIDS[*]}"
  echo "Process details before cleanup:"
  ps -o pid,ppid,stat,cmd -p "$(IFS=,; echo "${OLD_PIDS[*]}")" || true

  echo "Stopping previous navigation processes with SIGINT..."
  kill -INT "${OLD_PIDS[@]}" > /dev/null 2>&1 || true
  if wait_for_processes_to_exit 10 "${OLD_PIDS[@]}"; then
    echo "Previous navigation processes stopped cleanly."
    rm -f "$PID_FILE"
    return 0
  fi

  echo "Some processes are still alive, sending SIGTERM..."
  kill -TERM "${OLD_PIDS[@]}" > /dev/null 2>&1 || true
  if wait_for_processes_to_exit 5 "${OLD_PIDS[@]}"; then
    echo "Previous navigation processes stopped after SIGTERM."
    rm -f "$PID_FILE"
    return 0
  fi

  echo "Some processes ignored SIGTERM, sending SIGKILL..."
  kill -KILL "${OLD_PIDS[@]}" > /dev/null 2>&1 || true
  sleep 1
  rm -f "$PID_FILE"
}

cd "$WORKSPACE"

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "ERROR: /opt/ros/jazzy/setup.bash not found"
  exit 1
fi

source /opt/ros/jazzy/setup.bash

# Keep the parent launch process and every child node on the same DDS profile.
# Large PointCloud2 topics such as /airy_points need this profile to avoid
# falling back to slow inter-process transport during navigation startup.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.config/fastdds_shm.xml}"
export RMW_FASTRTPS_USE_QOS_FROM_XML="${RMW_FASTRTPS_USE_QOS_FROM_XML:-1}"

echo "Building workspace..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ ! -f "$WORKSPACE/install/setup.bash" ]; then
  echo "ERROR: $WORKSPACE/install/setup.bash not found after build."
  exit 1
fi

source "$WORKSPACE/install/setup.bash"

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.config/fastdds_shm.xml}"
export RMW_FASTRTPS_USE_QOS_FROM_XML="${RMW_FASTRTPS_USE_QOS_FROM_XML:-1}"

python3 - <<'PY'
import importlib.metadata as metadata

required = ("humanoid-navigation", "humanoid-navigation2", "humanoid-websocket")
missing = []
for package_name in required:
    try:
        metadata.distribution(package_name)
    except metadata.PackageNotFoundError:
        missing.append(package_name)

if missing:
    raise SystemExit(
        "ERROR: missing Python package metadata after sourcing install/setup.bash: "
        + ", ".join(missing)
    )
PY

echo "ROS environment loaded from $WORKSPACE/install/setup.bash"

cleanup_old_navigation_processes

ros2 launch humanoid_bringup robot_real.launch.py use_sim_time:=false &

NAV_PID=$!
echo "$NAV_PID" > "$PID_FILE"

sleep 2

if kill -0 "$NAV_PID" > /dev/null 2>&1; then
    echo "NAVIGATION_STARTED_SUCCESSFULLY_PID_$NAV_PID"
    echo "Debug log: $LOG_FILE"
    exit 0
else
    echo "ERROR: Navigation process crashed immediately."
    rm -f "$PID_FILE"
    exit 1
fi
