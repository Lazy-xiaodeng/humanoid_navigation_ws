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
      "app_gateway_node"
      "robot_gateway_node"
      "websocket_client_node"
      "facial_driver"
      "facial_driver_cpp"
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
    # 部分 C++ WebSocket 进程在端口占用或后台线程阻塞时可能没有及时退出。
    # 只对前面已经收集到的旧控制面 PID 做兜底清理，避免 8765 被残留进程占用。
    local alive_pids=()
    local pid
    for pid in "${pids[@]}"; do
      if [ -n "$pid" ] && [ "$pid" != "$$" ] && kill -0 "$pid" >/dev/null 2>&1; then
        alive_pids+=("$pid")
      fi
    done
    if [ "${#alive_pids[@]}" -gt 0 ]; then
      echo "Force killing lingering control plane processes: ${alive_pids[*]}"
      kill -KILL "${alive_pids[@]}" >/dev/null 2>&1 || true
      sleep 1
    fi
  fi
  rm -f "$PID_FILE"
}

source_ros_env
stop_existing_control_plane

setsid nohup ros2 launch humanoid_bringup robot_control_plane.launch.py \
  use_sim_time:=false \
  use_cpp_control_runtime:="${USE_CPP_CONTROL_RUNTIME:-true}" \
  use_cpp_app_gateway:="${USE_CPP_APP_GATEWAY:-true}" \
  cpp_app_websocket_server_enable:="${CPP_APP_WEBSOCKET_SERVER_ENABLE:-true}" \
  cpp_app_websocket_host:="${CPP_APP_WEBSOCKET_HOST:-0.0.0.0}" \
  cpp_app_websocket_port:="${CPP_APP_WEBSOCKET_PORT:-8765}" \
  cpp_data_integration_enable:="${CPP_DATA_INTEGRATION_ENABLE:-true}" \
  use_cpp_robot_gateway:="${USE_CPP_ROBOT_GATEWAY:-true}" \
  cpp_robot_ws_enable:="${CPP_ROBOT_WS_ENABLE:-true}" \
  cpp_robot_walk_velocity_send_enable:="${CPP_ROBOT_WALK_VELOCITY_SEND_ENABLE:-false}" \
  cpp_robot_motion_execution_enable:="${CPP_ROBOT_MOTION_EXECUTION_ENABLE:-false}" \
  cpp_robot_motion_allow_enter_menu:="${CPP_ROBOT_MOTION_ALLOW_ENTER_MENU:-false}" \
  cpp_robot_motion_allow_return_walk:="${CPP_ROBOT_MOTION_ALLOW_RETURN_WALK:-false}" \
  cpp_robot_gesture_sync_enable:="${CPP_ROBOT_GESTURE_SYNC_ENABLE:-false}" \
  use_cpp_expression_runtime:="${USE_CPP_EXPRESSION_RUNTIME:-true}" \
  cpp_expression_config_file:="${CPP_EXPRESSION_CONFIG_FILE:-/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_expression_runtime/config/expression_runtime.yaml}" \
  >> "$LOG_FILE" 2>&1 < /dev/null &
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
