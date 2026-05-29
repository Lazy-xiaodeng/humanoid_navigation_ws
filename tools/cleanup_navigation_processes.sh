#!/usr/bin/env bash
set -uo pipefail

DRY_RUN=0
CLEAN_SHM=1
CHECK_PORT=8765

usage() {
  cat <<'EOF'
Usage: tools/cleanup_navigation_processes.sh [--dry-run] [--no-shm] [--port PORT]

Stops leftover humanoid navigation ROS processes and removes stale FastDDS
shared-memory files that can cause fastrtps_port lock errors on restart.

Options:
  --dry-run   Print matched processes and files without killing/removing.
  --no-shm    Do not remove /dev/shm FastDDS/FastRTPS files.
  --port N    Check and clear websocket port N. Default: 8765.
  -h, --help  Show this help.
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --dry-run)
      DRY_RUN=1
      ;;
    --no-shm)
      CLEAN_SHM=0
      ;;
    --port)
      shift
      if [ "$#" -eq 0 ] || ! [[ "$1" =~ ^[0-9]+$ ]]; then
        echo "ERROR: --port requires a numeric value" >&2
        exit 2
      fi
      CHECK_PORT="$1"
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

log() {
  echo "[$(date '+%F %T')] $*"
}

run_or_print() {
  if [ "$DRY_RUN" -eq 1 ]; then
    printf 'DRY-RUN:'
    printf ' %q' "$@"
    printf '\n'
    return 0
  fi

  "$@"
}

add_pid() {
  local pid="$1"
  local existing

  [ -n "$pid" ] || return 0
  [[ "$pid" =~ ^[0-9]+$ ]] || return 0
  [ "$pid" -gt 1 ] || return 0
  [ "$pid" != "$$" ] || return 0
  [ "$pid" != "$PPID" ] || return 0

  for existing in "${PIDS[@]:-}"; do
    [ "$existing" != "$pid" ] || return 0
  done

  PIDS+=("$pid")
}

collect_pids() {
  PIDS=()

  if ! command -v pgrep >/dev/null 2>&1; then
    echo "ERROR: pgrep not found; cannot collect processes safely." >&2
    exit 1
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
    "static_transform_publisher.*tf_body_to_base_footprint"
    "static_transform_publisher.*tf_base_footprint_to_clearing_lidar"
    "dynamic_odom_ground_publisher"
    "nav2_map_server.*/map_server"
    "nav2_lifecycle_manager.*/lifecycle_manager"
    "hdl_global_localization_node"
    "component_container_mt.*hdl_bootstrap_container"
    "scancontext_global_localizer"
    "scancontext_to_initialpose"
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
    "navigation_state_manager"
    "websocket_server_node"
    "data_integration_node"
    "websocket_client_node"
    "message_bridge_node"
    "facial_driver"
    "rviz2"
  )

  local pattern
  local pid
  for pattern in "${patterns[@]}"; do
    while IFS= read -r pid; do
      add_pid "$pid"
    done < <(pgrep -f "$pattern" || true)
  done
}

alive_pids() {
  local pid
  local alive=()

  for pid in "$@"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      alive+=("$pid")
    fi
  done

  printf '%s\n' "${alive[@]}"
}

wait_until_gone() {
  local timeout_sec="$1"
  shift
  local pids=("$@")
  local elapsed=0

  while [ "$elapsed" -lt "$timeout_sec" ]; do
    mapfile -t remaining < <(alive_pids "${pids[@]}")
    [ "${#remaining[@]}" -eq 0 ] && return 0
    sleep 1
    elapsed=$((elapsed + 1))
  done

  return 1
}

show_processes() {
  local pids=("$@")
  [ "${#pids[@]}" -gt 0 ] || return 0
  ps -o pid,ppid,stat,etime,cmd -p "$(IFS=,; echo "${pids[*]}")" || true
}

stop_processes() {
  collect_pids

  if [ "${#PIDS[@]}" -eq 0 ]; then
    log "No humanoid navigation processes matched."
    return 0
  fi

  log "Matched ${#PIDS[@]} process(es): ${PIDS[*]}"
  show_processes "${PIDS[@]}"

  if [ "$DRY_RUN" -eq 1 ]; then
    log "Dry run enabled; no signals sent."
    return 0
  fi

  log "Sending SIGINT..."
  kill -INT "${PIDS[@]}" >/dev/null 2>&1 || true
  if wait_until_gone 8 "${PIDS[@]}"; then
    log "All matched processes exited after SIGINT."
    return 0
  fi

  mapfile -t remaining < <(alive_pids "${PIDS[@]}")
  log "Still alive after SIGINT: ${remaining[*]}"
  show_processes "${remaining[@]}"

  log "Sending SIGTERM..."
  kill -TERM "${remaining[@]}" >/dev/null 2>&1 || true
  if wait_until_gone 5 "${remaining[@]}"; then
    log "Remaining processes exited after SIGTERM."
    return 0
  fi

  mapfile -t remaining < <(alive_pids "${remaining[@]}")
  log "Still alive after SIGTERM: ${remaining[*]}"
  show_processes "${remaining[@]}"

  log "Sending SIGKILL..."
  kill -KILL "${remaining[@]}" >/dev/null 2>&1 || true
  sleep 1
}

cleanup_port() {
  local port="$1"

  if ! command -v lsof >/dev/null 2>&1; then
    log "lsof not found; skipping port ${port} check."
    return 0
  fi

  local pids
  mapfile -t pids < <(lsof -ti :"$port" 2>/dev/null || true)
  if [ "${#pids[@]}" -eq 0 ]; then
    log "Port ${port} is free."
    return 0
  fi

  log "Port ${port} is still used by PID(s): ${pids[*]}"
  lsof -i :"$port" || true

  if [ "$DRY_RUN" -eq 1 ]; then
    log "Dry run enabled; not killing port owner(s)."
    return 0
  fi

  kill -TERM "${pids[@]}" >/dev/null 2>&1 || true
  sleep 2
  mapfile -t pids < <(lsof -ti :"$port" 2>/dev/null || true)
  if [ "${#pids[@]}" -gt 0 ]; then
    log "Port ${port} still busy; sending SIGKILL to PID(s): ${pids[*]}"
    kill -KILL "${pids[@]}" >/dev/null 2>&1 || true
  fi
}

cleanup_fastdds_shm() {
  [ "$CLEAN_SHM" -eq 1 ] || {
    log "Skipping FastDDS /dev/shm cleanup (--no-shm)."
    return 0
  }

  local files=()
  local pattern
  for pattern in /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* /dev/shm/fast_datasharing_*; do
    [ -e "$pattern" ] || continue
    files+=("$pattern")
  done

  if [ "${#files[@]}" -eq 0 ]; then
    log "No FastDDS/FastRTPS /dev/shm files found."
    return 0
  fi

  log "Removing ${#files[@]} FastDDS/FastRTPS /dev/shm file(s)."
  if [ "$DRY_RUN" -eq 1 ]; then
    printf '%s\n' "${files[@]}"
    return 0
  fi

  rm -f "${files[@]}"
}

log "Cleaning humanoid navigation leftovers..."
stop_processes
cleanup_port "$CHECK_PORT"
cleanup_fastdds_shm

collect_pids
if [ "${#PIDS[@]}" -eq 0 ]; then
  log "Done. No matched ROS navigation processes remain."
else
  log "WARNING: process(es) still matched after cleanup: ${PIDS[*]}"
  show_processes "${PIDS[@]}"
  exit 1
fi

