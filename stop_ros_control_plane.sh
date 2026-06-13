#!/bin/bash
set -eo pipefail
set +u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WORKSPACE:-$SCRIPT_DIR}"
PID_FILE="$WORKSPACE/.ros_control_plane.pid"
PIDS=()

add_pid() {
  local pid="$1"
  [ -n "$pid" ] || return 0
  [[ "$pid" =~ ^[0-9]+$ ]] || return 0
  [ "$pid" -gt 1 ] || return 0
  [ "$pid" != "$$" ] || return 0
  PIDS+=("$pid")
}

if [ -f "$PID_FILE" ]; then
  old_pid="$(tr -d '[:space:]' < "$PID_FILE" || true)"
  if [ -n "$old_pid" ] && kill -0 "$old_pid" >/dev/null 2>&1; then
    add_pid "$old_pid"
  fi
fi

if command -v pgrep >/dev/null 2>&1; then
  for pattern in \
    "ros2 launch humanoid_bringup robot_control_plane.launch.py" \
    "dynamic_waypoints_manager" \
    "map_context_manager" \
    "websocket_server_node" \
    "data_integration_node" \
    "websocket_client_node" \
    "message_bridge_node" \
    "facial_driver"; do
    while IFS= read -r pid; do
      add_pid "$pid"
    done < <(pgrep -f "$pattern" || true)
  done
fi

if [ "${#PIDS[@]}" -eq 0 ]; then
  echo "No ROS control plane processes detected."
  rm -f "$PID_FILE"
  exit 0
fi

echo "Stopping ROS control plane processes: ${PIDS[*]}"
kill -INT "${PIDS[@]}" >/dev/null 2>&1 || true
sleep 3
kill -TERM "${PIDS[@]}" >/dev/null 2>&1 || true
sleep 1
rm -f "$PID_FILE"
echo "ROS control plane cleanup complete."
