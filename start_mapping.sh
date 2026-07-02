#!/bin/bash
set -Eeo pipefail
set +u

WORKSPACE="${WORKSPACE:-$HOME/humanoid_ws}"
MAP_NAME="${1:-hall}"
START_TIME="$(date +%Y%m%d_%H%M%S)"

MAP_DIR="$WORKSPACE/src/humanoid_navigation2/maps"
PCD_DIR="$WORKSPACE/src/humanoid_navigation2/pcd"
MAP_REGISTRY_FILE="$WORKSPACE/data/maps/map_registry.json"
WAYPOINTS_DIR="$WORKSPACE/data/waypoints"
BAG_ROOT="${BAG_ROOT:-/home/ubuntu/fast-lio-bags}"
ROSBAG_STOP_TIMEOUT_SEC="${ROSBAG_STOP_TIMEOUT_SEC:-10}"
MAP_SAVE_ATTEMPTS="${MAP_SAVE_ATTEMPTS:-10}"
MAP_SAVE_RETRY_DELAY_SEC="${MAP_SAVE_RETRY_DELAY_SEC:-1}"
LOG_DIR="$WORKSPACE/debug_logs"
ROS_LOG_DIR="$LOG_DIR/ros_mapping_${START_TIME}"
RUN_LOG="$LOG_DIR/start_mapping_${START_TIME}.log"
COMMAND_FILE="$WORKSPACE/.start_mapping.command"

MAP_PREFIX="$MAP_DIR/$MAP_NAME"
PCD_FILE="$PCD_DIR/$MAP_NAME.pcd"
PCD_STANDARD_FILE="$PCD_DIR/${MAP_NAME}_standard.pcd"
PCD_OPEN3D_FILE="$PCD_DIR/${MAP_NAME}_open3d_grounded.pcd"
SC_DB_FILE="$MAP_DIR/${MAP_NAME}_sc.bin"
BAG_DIR="$BAG_ROOT/${MAP_NAME}_mapping_${START_TIME}"

MAPPING_PID=""
PCD_PID=""
BAG_PID=""
FINISHING=0
INPUT_FD=0
MAPPING_STARTED=0

mkdir -p "$MAP_DIR" "$PCD_DIR" "$WAYPOINTS_DIR" "$BAG_ROOT" "$LOG_DIR" "$ROS_LOG_DIR"
export ROS_LOG_DIR
rm -f "$COMMAND_FILE"

exec > >(tee -a "$RUN_LOG") 2>&1

log() {
  echo "[$(date '+%F %T')] $*"
}

is_alive() {
  local pid="${1:-}"
  [ -n "$pid" ] && kill -0 "$pid" > /dev/null 2>&1
}

get_process_group() {
  local pid="${1:-}"
  [ -n "$pid" ] || return 1
  ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ' || true
}

process_group_has_members() {
  local pgid="${1:-}"
  [ -n "$pgid" ] || return 1
  ps -eo pgid= 2>/dev/null | awk -v target="$pgid" '$1 == target { found = 1 } END { exit found ? 0 : 1 }'
}

process_target_alive() {
  local pid="${1:-}"
  local pgid="${2:-}"
  if [ -n "$pgid" ]; then
    process_group_has_members "$pgid"
  else
    is_alive "$pid"
  fi
}

normalize_command() {
  local raw_cmd="${1:-}"
  printf '%s' "$raw_cmd" | tr -d '\r' | xargs | tr '[:upper:]' '[:lower:]'
}

send_signal_to_process() {
  local signal_name="$1"
  local pid="${2:-}"
  local pgid
  local self_pgid

  if ! is_alive "$pid"; then
    return 0
  fi

  pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
  self_pgid="$(ps -o pgid= -p "$$" 2>/dev/null | tr -d ' ' || true)"
  if [ -n "$pgid" ] && [ "$pgid" != "$self_pgid" ]; then
    kill "-$signal_name" -- "-$pgid" > /dev/null 2>&1 || true
  else
    kill "-$signal_name" "$pid" > /dev/null 2>&1 || true
  fi
}

send_signal_to_target() {
  local signal_name="$1"
  local pid="${2:-}"
  local pgid="${3:-}"

  if [ -n "$pgid" ]; then
    kill "-$signal_name" -- "-$pgid" > /dev/null 2>&1 || true
  else
    send_signal_to_process "$signal_name" "$pid"
  fi
}

stop_process_int() {
  local name="$1"
  local pid="${2:-}"
  local timeout_sec="${3:-30}"
  local elapsed=0
  local pgid
  local wait_pgid=""
  local self_pgid

  if ! is_alive "$pid"; then
    return 0
  fi

  pgid="$(get_process_group "$pid")"
  self_pgid="$(get_process_group "$$")"
  if [ -n "$pgid" ] && [ "$pgid" != "$self_pgid" ]; then
    wait_pgid="$pgid"
  fi

  log "Stopping $name with SIGINT (pid=$pid)..."
  send_signal_to_target INT "$pid" "$wait_pgid"

  while process_target_alive "$pid" "$wait_pgid" && [ "$elapsed" -lt "$timeout_sec" ]; do
    sleep 1
    elapsed=$((elapsed + 1))
    if [ $((elapsed % 5)) -eq 0 ] && process_target_alive "$pid" "$wait_pgid"; then
      log "RUNNING: waiting for $name to stop (${elapsed}s elapsed, please wait...)"
    fi
  done

  if process_target_alive "$pid" "$wait_pgid"; then
    log "WARN: $name did not exit after ${timeout_sec}s, sending SIGTERM."
    send_signal_to_target TERM "$pid" "$wait_pgid"
    sleep 2
  fi

  if process_target_alive "$pid" "$wait_pgid"; then
    log "WARN: $name still alive, sending SIGKILL."
    send_signal_to_target KILL "$pid" "$wait_pgid"
    sleep 1
    if process_target_alive "$pid" "$wait_pgid"; then
      log "ERROR: $name still has live processes after SIGKILL."
    fi
  fi
}

stop_process_fast() {
  local name="$1"
  local pid="${2:-}"
  local timeout_sec="${3:-5}"
  local elapsed=0
  local pgid
  local wait_pgid=""
  local self_pgid

  if ! is_alive "$pid"; then
    return 0
  fi

  pgid="$(get_process_group "$pid")"
  self_pgid="$(get_process_group "$$")"
  if [ -n "$pgid" ] && [ "$pgid" != "$self_pgid" ]; then
    wait_pgid="$pgid"
  fi

  log "Stopping $name with SIGTERM (pid=$pid)..."
  send_signal_to_target TERM "$pid" "$wait_pgid"

  while process_target_alive "$pid" "$wait_pgid" && [ "$elapsed" -lt "$timeout_sec" ]; do
    sleep 1
    elapsed=$((elapsed + 1))
  done

  if process_target_alive "$pid" "$wait_pgid"; then
    log "WARN: $name still alive after ${timeout_sec}s, sending SIGKILL."
    send_signal_to_target KILL "$pid" "$wait_pgid"
    sleep 1
    if process_target_alive "$pid" "$wait_pgid"; then
      log "ERROR: $name still has live processes after SIGKILL."
    fi
  fi
}

run_ros() {
  bash -lc "cd '$WORKSPACE' && source /opt/ros/jazzy/setup.bash && source install/setup.bash && $*"
}

run_step() {
  local label="$1"
  shift
  local heartbeat_sec=5
  local elapsed=0
  local pid
  local status

  log "START: $label"
  "$@" &
  pid=$!

  while kill -0 "$pid" > /dev/null 2>&1; do
    sleep "$heartbeat_sec"
    elapsed=$((elapsed + heartbeat_sec))
    if kill -0 "$pid" > /dev/null 2>&1; then
      log "RUNNING: $label (${elapsed}s elapsed, please wait...)"
    fi
  done

  set +e
  wait "$pid"
  status=$?
  set -e
  if [ "$status" -eq 0 ]; then
    log "DONE: $label"
  else
    log "ERROR: $label failed with exit code $status"
  fi
  return "$status"
}

wait_for_service() {
  local service_name="$1"
  local timeout_sec="${2:-30}"
  local elapsed=0

  log "Waiting for service $service_name ..."
  while [ "$elapsed" -lt "$timeout_sec" ]; do
    if run_ros "ros2 service type '$service_name'" > /dev/null 2>&1; then
      log "Service ready: $service_name"
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done

  log "WARN: service not ready after ${timeout_sec}s: $service_name"
  return 1
}

check_required_file() {
  local path="$1"
  if [ -s "$path" ]; then
    log "OK: $path ($(du -h "$path" | awk '{print $1}'))"
    return 0
  fi
  log "ERROR: missing or empty file: $path"
  return 1
}

save_2d_occupancy_grid_map() {
  local attempt
  local marker="$LOG_DIR/.map_save_marker_${START_TIME}"

  if ! wait_for_service "/slam_toolbox/save_map" 20; then
    log "WARN: /slam_toolbox/save_map unavailable; 2D occupancy grid map will be missing."
    return 1
  fi

  for attempt in $(seq 1 "$MAP_SAVE_ATTEMPTS"); do
    rm -f "$marker"
    : > "$marker"

    log "START: 2D occupancy grid map save (attempt ${attempt}/${MAP_SAVE_ATTEMPTS})"
    run_ros "ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \"{name: {data: '$MAP_PREFIX'}}\"" || true

    if [ -s "${MAP_PREFIX}.yaml" ] && [ -s "${MAP_PREFIX}.pgm" ] && \
       [ "${MAP_PREFIX}.yaml" -nt "$marker" ] && [ "${MAP_PREFIX}.pgm" -nt "$marker" ]; then
      rm -f "$marker"
      log "DONE: 2D occupancy grid map save"
      return 0
    fi

    log "WARN: /slam_toolbox/save_map did not update ${MAP_PREFIX}.yaml/.pgm on attempt ${attempt}."
    if [ "$attempt" -lt "$MAP_SAVE_ATTEMPTS" ]; then
      sleep "$MAP_SAVE_RETRY_DELAY_SEC"
    fi
  done

  rm -f "$marker"
  log "ERROR: 2D occupancy grid map save failed after ${MAP_SAVE_ATTEMPTS} attempts."
  return 1
}

register_completed_map() {
  log "Registering completed map into registry: $MAP_REGISTRY_FILE"
  MAP_NAME_ENV="$MAP_NAME" \
  MAP_REGISTRY_FILE_ENV="$MAP_REGISTRY_FILE" \
  WAYPOINTS_FILE_ENV="$WAYPOINTS_DIR/${MAP_NAME}.json" \
  MAP_YAML_FILE_ENV="${MAP_PREFIX}.yaml" \
  MAP_PGM_FILE_ENV="${MAP_PREFIX}.pgm" \
  MAP_POSEGRAPH_FILE_ENV="${MAP_PREFIX}.posegraph" \
  MAP_DATA_FILE_ENV="${MAP_PREFIX}.data" \
  RAW_PCD_FILE_ENV="$PCD_FILE" \
  STANDARD_PCD_FILE_ENV="$PCD_STANDARD_FILE" \
  OPEN3D_PCD_FILE_ENV="$PCD_OPEN3D_FILE" \
  SC_DB_FILE_ENV="$SC_DB_FILE" \
  BAG_DIR_ENV="$BAG_DIR" \
  START_TIME_ENV="$START_TIME" \
  python3 - <<'PY'
import json
import os
import time
from pathlib import Path

map_id = os.environ["MAP_NAME_ENV"].strip()
registry_path = Path(os.environ["MAP_REGISTRY_FILE_ENV"])
waypoints_file = Path(os.environ["WAYPOINTS_FILE_ENV"])

registry_path.parent.mkdir(parents=True, exist_ok=True)
waypoints_file.parent.mkdir(parents=True, exist_ok=True)

if registry_path.exists():
    try:
        registry = json.loads(registry_path.read_text(encoding="utf-8"))
    except Exception:
        registry = {}
else:
    registry = {}

maps = registry.get("maps")
if not isinstance(maps, list):
    maps = []

now = time.time()
new_entry = {
    "map_id": map_id,
    "display_name": map_id,
    "enabled": True,
    "description": f"建图脚本自动注册地图，建图时间 {os.environ['START_TIME_ENV']}",
    "waypoints_file": str(waypoints_file),
    "map_yaml_file": os.environ["MAP_YAML_FILE_ENV"],
    "map_pgm_file": os.environ["MAP_PGM_FILE_ENV"],
    "map_posegraph_file": os.environ["MAP_POSEGRAPH_FILE_ENV"],
    "map_data_file": os.environ["MAP_DATA_FILE_ENV"],
    "raw_pcd_file": os.environ["RAW_PCD_FILE_ENV"],
    "standard_pcd_file": os.environ["STANDARD_PCD_FILE_ENV"],
    "open3d_prior_map_file": os.environ["OPEN3D_PCD_FILE_ENV"],
    "scancontext_database_file": os.environ["SC_DB_FILE_ENV"],
    "bag_dir": os.environ["BAG_DIR_ENV"],
    "initial_pose": {
        "frame_id": "map",
        "position": [0.0, 0.0, 0.0],
        "orientation": [0.0, 0.0, 0.0, 1.0],
    },
    "created_time": now,
    "last_modified": now,
}

updated = False
for index, item in enumerate(maps):
    if str(item.get("map_id", "")).strip() == map_id:
        merged = dict(item)
        merged.update(new_entry)
        if item.get("display_name"):
            merged["display_name"] = item.get("display_name")
        if isinstance(item.get("initial_pose"), dict):
            merged["initial_pose"] = item["initial_pose"]
        if item.get("created_time"):
            merged["created_time"] = item["created_time"]
        maps[index] = merged
        updated = True
        break

if not updated:
    maps.append(new_entry)

registry.setdefault("default_map_id", "hall")
registry.setdefault("current_map_id", registry.get("default_map_id", "hall"))
registry["maps"] = maps
registry_path.write_text(json.dumps(registry, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

if not waypoints_file.exists():
    empty_waypoints = {
        "map_id": map_id,
        "waypoints_revision": f"initial_empty_{map_id}_{int(now * 1000)}",
        "waypoints": {
            "navigation_target": {},
            "exhibition_point": {},
            "obstacle_point": {},
            "charging_point": {},
            "rest_point": {},
            "landmark_point": {},
        },
        "timestamp": now,
    }
    waypoints_file.write_text(json.dumps(empty_waypoints, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

print(f"registered map_id={map_id}")
print(f"registry={registry_path}")
print(f"waypoints={waypoints_file}")
PY
}

finish_mapping() {
  if [ "$FINISHING" -eq 1 ]; then
    return 0
  fi
  FINISHING=1
  trap - INT TERM EXIT
  rm -f "$COMMAND_FILE"

  log "Finish requested. Finishing mapping and saving outputs..."

  log "[1/8] Stopping recorders and flushing bag/PCD data..."

  # Stop data recorders first so bag and PCD are flushed to disk.
  # ros2 bag can ignore SIGINT when keyboard handling is disabled; TERM still
  # lets it flush metadata and write remaining cached messages.
  stop_process_int "rosbag recorder" "$BAG_PID" "$ROSBAG_STOP_TIMEOUT_SEC"
  stop_process_int "PCD saver" "$PCD_PID" 120

  log "[2/8] Saving 2D occupancy grid map: ${MAP_PREFIX}.yaml/.pgm"
  save_2d_occupancy_grid_map || true

  log "[3/8] Saving slam_toolbox posegraph: ${MAP_PREFIX}.posegraph/.data"
  if wait_for_service "/slam_toolbox/serialize_map" 20; then
    run_step "slam_toolbox posegraph save" run_ros "ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \"{filename: '$MAP_PREFIX'}\""
  else
    log "WARN: /slam_toolbox/serialize_map unavailable; posegraph will not be saved."
  fi

  log "[4/8] Building Scan Context database: $SC_DB_FILE"
  if [ -d "$BAG_DIR" ]; then
    run_step "Scan Context database build" run_ros "ros2 run humanoid_relocalization build_sc_database.py --bag '$BAG_DIR' --output '$SC_DB_FILE' --cloud-topic /fast_lio/cloud_registered --odom-topic /odom --interval 2.0"
  else
    log "ERROR: bag directory not found, skip Scan Context database: $BAG_DIR"
  fi

  log "[5/8] Generating derived PCD files..."
  if [ -s "$PCD_FILE" ]; then
    log "Generating standard-frame PCD: $PCD_STANDARD_FILE"
    run_step "standard-frame PCD generation" run_ros "python3 '$WORKSPACE/src/humanoid_navigation2/humanoid_navigation2/pcd_converter.py' '$PCD_FILE' '$PCD_STANDARD_FILE'" || \
      log "WARN: standard PCD conversion failed."

    log "Generating Open3D/RoboSense prior-map PCD: $PCD_OPEN3D_FILE"
    # 当前正式导航的 RoboSense/Open3D 定位链路使用 *_open3d_grounded.pcd。
    # 该口径只去掉地面以下点，保留墙面/天花板等高处结构，并使用 0.05m voxel。
    run_step "Open3D prior-map PCD generation" run_ros "python3 '$WORKSPACE/src/humanoid_navigation2/scripts/make_localization_pcd.py' '$PCD_FILE' '$PCD_OPEN3D_FILE' --min-z 0.0 --voxel-size 0.05 --keep-ceiling" || \
      log "WARN: Open3D prior-map PCD generation failed."
  else
    log "ERROR: base PCD is missing; skip PCD conversions: $PCD_FILE"
  fi

  log "[6/8] Stopping mapping launch..."
  stop_process_int "mapping launch" "$MAPPING_PID" 30

  log "[7/8] Checking output files..."
  local failed=0
  check_required_file "${MAP_PREFIX}.yaml" || failed=1
  check_required_file "${MAP_PREFIX}.pgm" || failed=1
  check_required_file "${MAP_PREFIX}.posegraph" || failed=1
  check_required_file "${MAP_PREFIX}.data" || failed=1
  check_required_file "$SC_DB_FILE" || failed=1
  check_required_file "$PCD_FILE" || failed=1
  check_required_file "$PCD_STANDARD_FILE" || failed=1
  check_required_file "$PCD_OPEN3D_FILE" || failed=1

  if [ -d "$BAG_DIR" ] && run_ros "ros2 bag info '$BAG_DIR'" > /dev/null 2>&1; then
    log "OK: rosbag recorded: $BAG_DIR"
  else
    log "ERROR: rosbag check failed: $BAG_DIR"
    failed=1
  fi

  log "Mapping log: $RUN_LOG"
  log "ROS log dir: $ROS_LOG_DIR"

  if [ "$failed" -eq 0 ]; then
    log "[8/8] Registering map for APP map switching..."
    register_completed_map
    log "Mapping completed successfully."
    exit 0
  fi

  log "Mapping completed with missing or invalid outputs. Check the warnings above."
  exit 1
}

abort_mapping() {
  if [ "$FINISHING" -eq 1 ]; then
    return 0
  fi
  FINISHING=1
  trap - INT TERM EXIT
  rm -f "$COMMAND_FILE"

  log "Abort requested. Stopping mapping without saving final map outputs..."

  log "[abort 1/3] Stopping mapping launch..."
  stop_process_fast "mapping launch" "$MAPPING_PID" 8

  log "[abort 2/3] Stopping rosbag recorder..."
  stop_process_fast "rosbag recorder" "$BAG_PID" 8

  log "[abort 3/3] Stopping PCD saver without saving PCD..."
  stop_process_fast "PCD saver" "$PCD_PID" 5

  log "Aborted mapping run. No final map save/check was performed."
  log "Partial bag, if any, is at: $BAG_DIR"
  log "Mapping log: $RUN_LOG"
  exit 2
}

handle_int() {
  log "Ctrl+C received. Starting finish/save flow..."
  finish_mapping
}

handle_exit() {
  local status="$1"

  # In some terminal/process-group cases Ctrl+C makes bash leave the main loop
  # before the INT trap runs. Treat exit status 130 as an interrupted mapping
  # run and finish it instead of leaving child processes orphaned.
  if [ "$FINISHING" -eq 0 ] && [ "$MAPPING_STARTED" -eq 1 ] && [ "$status" -eq 130 ]; then
    log "Interrupted exit detected. Starting finish/save flow..."
    finish_mapping
  fi
}

trap handle_int INT
trap abort_mapping TERM
trap 'handle_exit "$?"' EXIT

cd "$WORKSPACE"

if [ -r /dev/tty ]; then
  if exec 3</dev/tty; then
    INPUT_FD=3
  else
    INPUT_FD=0
  fi
fi

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  log "ERROR: /opt/ros/jazzy/setup.bash not found"
  exit 1
fi

if [ ! -f "$WORKSPACE/install/setup.bash" ]; then
  log "ERROR: $WORKSPACE/install/setup.bash not found. Run colcon build first."
  exit 1
fi

source /opt/ros/jazzy/setup.bash
source install/setup.bash

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.config/fastdds_shm.xml}"
export RMW_FASTRTPS_USE_QOS_FROM_XML="${RMW_FASTRTPS_USE_QOS_FROM_XML:-1}"

log "Starting Humanoid Mapping"
log "Map name: $MAP_NAME"
log "2D map prefix: $MAP_PREFIX"
log "PCD map: $PCD_FILE"
log "Open3D/RoboSense prior-map PCD: $PCD_OPEN3D_FILE"
log "Scan Context database: $SC_DB_FILE"
log "Map registry: $MAP_REGISTRY_FILE"
log "Bag directory: $BAG_DIR"
log "Run log: $RUN_LOG"

log "Starting mapping launch..."
setsid ros2 launch humanoid_bringup robot_mapping.launch.py \
  pcd_map_file:="$PCD_FILE" \
  rviz:=true \
  app_layer:=false &
MAPPING_PID=$!
MAPPING_STARTED=1

sleep 8
if ! is_alive "$MAPPING_PID"; then
  log "ERROR: mapping launch exited early."
  exit 1
fi

log "Starting PCD saver..."
python3 "$WORKSPACE/src/humanoid_navigation2/humanoid_navigation2/save_pcd_map.py" "$PCD_FILE" &
PCD_PID=$!

log "Starting rosbag recorder..."
setsid ros2 bag record -o "$BAG_DIR" \
  /rosout \
  /diagnostics \
  /tf \
  /tf_static \
  /airy_points \
  /airy_imu \
  /fast_lio/cloud_registered \
  /airy_points_filtered \
  /Odometry \
  /odom \
  /pcl_pose \
  /robot_realpose \
  /localization/ndt_status \
  /status \
  /hdl_bootstrap/odom \
  /localization/recovery_status \
  /localization/recovery_requests \
  /initialpose \
  /cmd_vel \
  /cmd_vel_nav \
  /cmd_vel_smoothed \
  /cmd_vel_teleop \
  /cmd_vel_raw \
  /robot_speed \
  /velocity_smoother/transition_event \
  /goal_pose \
  /cloud_registered \
  /plan \
  /local_plan \
  /transformed_global_plan \
  /global_costmap/costmap \
  /local_costmap/costmap \
  /global_costmap/published_footprint \
  /local_costmap/published_footprint \
  /navigate_to_pose/_action/status \
  /navigate_to_pose/_action/feedback \
  /navigation/status \
  /follow_path/_action/status \
  /follow_path/_action/feedback \
  /spin/_action/status \
  /spin/_action/feedback \
  /back_up/_action/status \
  /back_up/_action/feedback \
  /behavior_tree_log \
  /scancontext_global_localization/best_pose \
  /scancontext_global_localization/status \
  /scancontext_global_localization/candidate &
BAG_PID=$!

log "Mapping is running."
log "Walk the robot through the whole environment slowly and smoothly."
log "When mapping is complete, press Ctrl+C in this terminal, or type 'finish' and press Enter."
log "If this run is invalid and you want to restart without saving maps, type 'abort' and press Enter."
log "If terminal output is too noisy, use another terminal:"
log "  echo finish > $COMMAND_FILE"
log "  echo abort  > $COMMAND_FILE"
log "The script will then save maps, bag, PCD files, Scan Context database, and check output files."

while is_alive "$MAPPING_PID"; do
  user_cmd=""
  cmd_source=""
  if [ -f "$COMMAND_FILE" ]; then
    user_cmd="$(sed -n '1p' "$COMMAND_FILE" 2>/dev/null || true)"
    rm -f "$COMMAND_FILE"
    cmd_source="$COMMAND_FILE"
  elif IFS= read -r -t 1 -u "$INPUT_FD" user_cmd; then
    cmd_source="terminal input"
  fi

  if [ -n "$cmd_source" ]; then
    user_cmd="$(normalize_command "$user_cmd")"
    case "$user_cmd" in
      finish|done|q|quit|exit)
        log "Finish command received from $cmd_source."
        finish_mapping
        ;;
      abort|cancel|discard)
        log "Abort command received from $cmd_source."
        abort_mapping
        ;;
      "")
        ;;
      *)
        log "Mapping is still running. Type 'finish' to save, or 'abort' to stop without saving."
        ;;
    esac
  fi
done

log "Mapping launch exited before Ctrl+C."
finish_mapping
