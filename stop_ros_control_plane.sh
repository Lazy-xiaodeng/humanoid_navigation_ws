#!/bin/bash
set -eo pipefail
set +u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${WORKSPACE:-$SCRIPT_DIR}"
PID_FILE="$WORKSPACE/.ros_control_plane.pid"
PIDS=()

add_pid() {
  local pid="$1"
  local existing
  [ -n "$pid" ] || return 0
  [[ "$pid" =~ ^[0-9]+$ ]] || return 0
  [ "$pid" -gt 1 ] || return 0
  [ "$pid" != "$$" ] || return 0
  for existing in "${PIDS[@]}"; do
    [ "$existing" != "$pid" ] || return 0
  done
  PIDS+=("$pid")
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

force_stop_residual_control_plane() {
  if ! command -v pgrep >/dev/null 2>&1; then
    return 0
  fi

  local residual_pids=()
  local seen=""
  local pattern pid
  for pattern in \
    "dynamic_waypoints_manager_cpp" \
    "map_context_manager_cpp" \
    "app_gateway_node" \
    "data_integration_node" \
    "robot_gateway_node" \
    "facial_driver"; do
    while IFS= read -r pid; do
      [ -n "$pid" ] || continue
      [[ "$pid" =~ ^[0-9]+$ ]] || continue
      [ "$pid" != "$$" ] || continue
      case " $seen " in
        *" $pid "*) continue ;;
      esac
      seen="$seen $pid"
      residual_pids+=("$pid")
    done < <(pgrep -f "$pattern" || true)
  done

  if [ "${#residual_pids[@]}" -gt 0 ]; then
    echo "Force stopping residual ROS control plane processes: ${residual_pids[*]}"
    kill -KILL "${residual_pids[@]}" >/dev/null 2>&1 || true
    sleep 1
  fi
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
    "ros2 launch humanoid_bringup robot_real.launch.py" \
    "dynamic_waypoints_manager_cpp" \
    "map_context_manager_cpp" \
    "app_gateway_node" \
    "data_integration_node" \
    "robot_gateway_node" \
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
if wait_for_exit 8 "${PIDS[@]}"; then
  rm -f "$PID_FILE"
  sleep 3
  force_stop_residual_control_plane
  echo "ROS control plane stopped cleanly."
  exit 0
fi

kill -TERM "${PIDS[@]}" >/dev/null 2>&1 || true
if wait_for_exit 5 "${PIDS[@]}"; then
  rm -f "$PID_FILE"
  sleep 3
  force_stop_residual_control_plane
  echo "ROS control plane stopped after SIGTERM."
  exit 0
fi

kill -KILL "${PIDS[@]}" >/dev/null 2>&1 || true
sleep 1
rm -f "$PID_FILE"
sleep 3
force_stop_residual_control_plane
echo "ROS control plane cleanup complete."
