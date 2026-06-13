#!/bin/bash
set -eo pipefail
set +u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WORKSPACE:-$SCRIPT_DIR}"
START_TIME="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$WORKSPACE/debug_logs"
LOG_FILE="$LOG_DIR/start_ros_control_plane_${START_TIME}.log"
ROS_LOG_DIR="${ROS_LOG_DIR:-$LOG_DIR/ros_control_${START_TIME}}"
PID_FILE="$WORKSPACE/.ros_control_plane.pid"

mkdir -p "$LOG_DIR" "$ROS_LOG_DIR"
export ROS_LOG_DIR
exec > >(tee -a "$LOG_FILE") 2>&1

source_ros_env() {
  cd "$WORKSPACE"
  unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
  source /opt/ros/jazzy/setup.bash
  source "$WORKSPACE/install/local_setup.bash"
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.config/fastdds_shm.xml}"
  export RMW_FASTRTPS_USE_QOS_FROM_XML="${RMW_FASTRTPS_USE_QOS_FROM_XML:-1}"
}

stop_existing_control_plane() {
  local pids=()
  local old_pid=""
  if [ -f "$PID_FILE" ]; then
    old_pid="$(tr -d '[:space:]' < "$PID_FILE" || true)"
    if [ -n "$old_pid" ] && kill -0 "$old_pid" >/dev/null 2>&1; then
      pids+=("$old_pid")
    fi
  fi
  if command -v pgrep >/dev/null 2>&1; then
    local patterns=(
      "ros2 launch humanoid_bringup robot_control_plane.launch.py"
      "dynamic_waypoints_manager"
      "map_context_manager"
      "websocket_server_node"
      "data_integration_node"
      "websocket_client_node"
      "message_bridge_node"
      "facial_driver"
    )
    local pattern pid
    for pattern in "${patterns[@]}"; do
      while IFS= read -r pid; do
        [ -n "$pid" ] && [ "$pid" != "$$" ] && pids+=("$pid")
      done < <(pgrep -f "$pattern" || true)
    done
  fi
  if [ "${#pids[@]}" -gt 0 ]; then
    echo "Stopping existing control plane processes: ${pids[*]}"
    kill -INT "${pids[@]}" >/dev/null 2>&1 || true
    sleep 3
    kill -TERM "${pids[@]}" >/dev/null 2>&1 || true
    sleep 1
  fi
  rm -f "$PID_FILE"
}

source_ros_env
stop_existing_control_plane

ros2 launch humanoid_bringup robot_control_plane.launch.py use_sim_time:=false &
CONTROL_PID=$!
echo "$CONTROL_PID" > "$PID_FILE"

sleep 2
if kill -0 "$CONTROL_PID" >/dev/null 2>&1; then
  echo "ROS_CONTROL_PLANE_STARTED_SUCCESSFULLY_PID_$CONTROL_PID"
  echo "Debug log: $LOG_FILE"
  exit 0
fi

echo "ERROR: ROS control plane crashed immediately."
rm -f "$PID_FILE"
exit 1
