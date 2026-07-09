#!/bin/bash
set -eo pipefail
set +u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WORKSPACE:-$SCRIPT_DIR}"
START_TIME="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$WORKSPACE/debug_logs"
LOG_FILE="$LOG_DIR/start_navigation_${START_TIME}.log"
ROS_LOG_DIR="$LOG_DIR/ros_${START_TIME}"
PID_FILE="$WORKSPACE/.start_navigation.pid"

mkdir -p "$LOG_DIR" "$ROS_LOG_DIR"
export ROS_LOG_DIR
export HUMANOID_WS="$WORKSPACE"

OPEN3D_ARCH="$(uname -m)"
case "$OPEN3D_ARCH" in
  x86_64|amd64|AMD64) OPEN3D_ARCH_DIR="x86_64" ;;
  aarch64|arm64|ARM64) OPEN3D_ARCH_DIR="aarch64" ;;
  *) OPEN3D_ARCH_DIR="" ;;
esac

if [ -n "$OPEN3D_ARCH_DIR" ] && [ -d "$WORKSPACE/src/humanoid_prior_localization_runtime/third_party/open3d/$OPEN3D_ARCH_DIR/open3d-0.18.0/lib/cmake/Open3D" ]; then
  export Open3D_DIR="${Open3D_DIR:-$WORKSPACE/src/humanoid_prior_localization_runtime/third_party/open3d/$OPEN3D_ARCH_DIR/open3d-0.18.0/lib/cmake/Open3D}"
  export LD_LIBRARY_PATH="$WORKSPACE/src/humanoid_prior_localization_runtime/third_party/open3d/$OPEN3D_ARCH_DIR/open3d-0.18.0/lib:${LD_LIBRARY_PATH:-}"
elif [ -d "$WORKSPACE/third_party/open3d-0.18.0/lib/cmake/Open3D" ]; then
  export Open3D_DIR="${Open3D_DIR:-$WORKSPACE/third_party/open3d-0.18.0/lib/cmake/Open3D}"
  export LD_LIBRARY_PATH="$WORKSPACE/third_party/open3d-0.18.0/lib:${LD_LIBRARY_PATH:-}"
fi

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
    "ros2 launch humanoid_control_runtime control_runtime.launch.py"
    "ros2 launch humanoid_route_runtime route_runtime.launch.py"
    "ros2 launch humanoid_app_gateway_runtime app_gateway_runtime.launch.py"
    "ros2 launch humanoid_robot_gateway_runtime robot_gateway_runtime.launch.py"
    "ros2 launch humanoid_expression_runtime expression_runtime.launch.py"
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
    "robot_realpose_publisher"
    "periodic_clearing_publisher"
    "periodic_clearing_3d_publisher"
    "wait_for_tf.*wait_for_localization_tf"
    "nav2_planner.*/planner_server"
    "nav2_controller.*/controller_server"
    "nav2_behaviors.*/behavior_server"
    "nav2_bt_navigator.*/bt_navigator"
    "dynamic_waypoints_manager_cpp"
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

cleanup_control_console_simulators() {
  if ! command -v pgrep > /dev/null 2>&1; then
    echo "WARN: pgrep not found, skip control-console simulator cleanup."
    return 0
  fi

  local sim_pids=()
  local patterns=(
    "tools/ros/robot_body_ws_simulator.py"
    "tools/ros/navigation_activity_simulator.py"
  )
  local pattern
  local pid

  for pattern in "${patterns[@]}"; do
    while IFS= read -r pid; do
      [ -n "$pid" ] || continue
      [ "$pid" != "$$" ] || continue
      sim_pids+=("$pid")
    done < <(pgrep -f "$pattern" || true)
  done

  if [ "${#sim_pids[@]}" -eq 0 ]; then
    echo "No control-console ROS simulators detected."
    return 0
  fi

  echo "Stopping control-console ROS simulators before real navigation: ${sim_pids[*]}"
  ps -o pid,ppid,stat,cmd -p "$(IFS=,; echo "${sim_pids[*]}")" || true
  kill -TERM "${sim_pids[@]}" > /dev/null 2>&1 || true
  if wait_for_processes_to_exit 5 "${sim_pids[@]}"; then
    echo "Control-console ROS simulators stopped."
    return 0
  fi

  echo "Some control-console ROS simulators are still alive, sending SIGKILL..."
  kill -KILL "${sim_pids[@]}" > /dev/null 2>&1 || true
  sleep 1
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

if [ "${SKIP_COLCON_BUILD:-0}" = "1" ]; then
  echo "SKIP_COLCON_BUILD=1, skip runtime colcon build."
else
  echo "Building workspace..."
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

if [ ! -f "$WORKSPACE/install/setup.bash" ]; then
  echo "ERROR: $WORKSPACE/install/setup.bash not found after build."
  exit 1
fi

source "$WORKSPACE/install/setup.bash"

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.config/fastdds_shm.xml}"
export RMW_FASTRTPS_USE_QOS_FROM_XML="${RMW_FASTRTPS_USE_QOS_FROM_XML:-1}"

for required_pkg in humanoid_bringup humanoid_navigation2 humanoid_control_runtime humanoid_route_runtime humanoid_app_gateway_runtime humanoid_robot_gateway_runtime humanoid_expression_runtime; do
  if ! ros2 pkg prefix "$required_pkg" > /dev/null 2>&1; then
    echo "ERROR: missing ROS package after sourcing install/setup.bash: $required_pkg"
    exit 1
  fi
done

echo "ROS environment loaded from $WORKSPACE/install/setup.bash"

cleanup_control_console_simulators
cleanup_old_navigation_processes

nohup ros2 launch humanoid_bringup robot_real.launch.py use_sim_time:=false >> "$LOG_FILE" 2>&1 < /dev/null &

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
