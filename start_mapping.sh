#!/bin/bash
set -Eeo pipefail
set +u

WORKSPACE="${WORKSPACE:-$HOME/humanoid_ws}"
MAP_NAME="${1:-hall}"
START_TIME="$(date +%Y%m%d_%H%M%S)"

MAP_DIR="$WORKSPACE/src/humanoid_navigation2/maps"
PCD_DIR="$WORKSPACE/src/humanoid_navigation2/pcd"
BAG_ROOT="${BAG_ROOT:-/home/ubuntu/fast-lio-bags}"
LOG_DIR="$WORKSPACE/debug_logs"
ROS_LOG_DIR="$LOG_DIR/ros_mapping_${START_TIME}"
RUN_LOG="$LOG_DIR/start_mapping_${START_TIME}.log"
COMMAND_FILE="$WORKSPACE/.start_mapping.command"

MAP_PREFIX="$MAP_DIR/$MAP_NAME"
PCD_FILE="$PCD_DIR/$MAP_NAME.pcd"
PCD_STANDARD_FILE="$PCD_DIR/${MAP_NAME}_standard.pcd"
PCD_LOCALIZATION_FILE="$PCD_DIR/${MAP_NAME}_localization_grounded.pcd"
SC_DB_FILE="$MAP_DIR/${MAP_NAME}_sc.bin"
BAG_DIR="$BAG_ROOT/${MAP_NAME}_mapping_${START_TIME}"

MAPPING_PID=""
PCD_PID=""
BAG_PID=""
FINISHING=0
INPUT_FD=0
MAPPING_STARTED=0

mkdir -p "$MAP_DIR" "$PCD_DIR" "$BAG_ROOT" "$LOG_DIR" "$ROS_LOG_DIR"
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

stop_process_int() {
  local name="$1"
  local pid="${2:-}"
  local timeout_sec="${3:-30}"
  local elapsed=0

  if ! is_alive "$pid"; then
    return 0
  fi

  log "Stopping $name with SIGINT (pid=$pid)..."
  kill -INT "$pid" > /dev/null 2>&1 || true

  while is_alive "$pid" && [ "$elapsed" -lt "$timeout_sec" ]; do
    sleep 1
    elapsed=$((elapsed + 1))
    if [ $((elapsed % 5)) -eq 0 ] && is_alive "$pid"; then
      log "RUNNING: waiting for $name to stop (${elapsed}s elapsed, please wait...)"
    fi
  done

  if is_alive "$pid"; then
    log "WARN: $name did not exit after ${timeout_sec}s, sending SIGTERM."
    kill -TERM "$pid" > /dev/null 2>&1 || true
    sleep 2
  fi

  if is_alive "$pid"; then
    log "WARN: $name still alive, sending SIGKILL."
    kill -KILL "$pid" > /dev/null 2>&1 || true
    sleep 1
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
  stop_process_int "rosbag recorder" "$BAG_PID" 90
  stop_process_int "PCD saver" "$PCD_PID" 120

  log "[2/8] Saving Fast-LIO map through /map_save when available..."
  if wait_for_service "/map_save" 5; then
    run_step "Fast-LIO /map_save" run_ros "ros2 service call /map_save std_srvs/srv/Trigger '{}'" || \
      log "WARN: /map_save call failed; continuing with PCD saver output."
  else
    log "WARN: /map_save service unavailable; continuing with PCD saver output."
  fi

  log "[3/8] Saving 2D occupancy grid map: ${MAP_PREFIX}.yaml/.pgm"
  if wait_for_service "/slam_toolbox/save_map" 20; then
    run_step "2D occupancy grid map save" run_ros "ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \"{name: {data: '$MAP_PREFIX'}}\""
  else
    log "WARN: /slam_toolbox/save_map unavailable; trying map_saver_cli."
    run_step "2D occupancy grid map save via map_saver_cli" run_ros "ros2 run nav2_map_server map_saver_cli -f '$MAP_PREFIX'" || true
  fi

  log "[4/8] Saving slam_toolbox posegraph: ${MAP_PREFIX}.posegraph/.data"
  if wait_for_service "/slam_toolbox/serialize_map" 20; then
    run_step "slam_toolbox posegraph save" run_ros "ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \"{filename: '$MAP_PREFIX'}\""
  else
    log "WARN: /slam_toolbox/serialize_map unavailable; posegraph will not be saved."
  fi

  log "[5/8] Building Scan Context database: $SC_DB_FILE"
  if [ -d "$BAG_DIR" ]; then
    run_step "Scan Context database build" run_ros "ros2 run humanoid_relocalization build_sc_database.py --bag '$BAG_DIR' --output '$SC_DB_FILE' --cloud-topic /fast_lio/cloud_registered --odom-topic /odom --interval 2.0"
  else
    log "ERROR: bag directory not found, skip Scan Context database: $BAG_DIR"
  fi

  log "[6/8] Generating derived PCD files..."
  if [ -s "$PCD_FILE" ]; then
    log "Generating standard-frame PCD: $PCD_STANDARD_FILE"
    run_step "standard-frame PCD generation" run_ros "python3 '$WORKSPACE/src/humanoid_navigation2/humanoid_navigation2/pcd_converter.py' '$PCD_FILE' '$PCD_STANDARD_FILE'" || \
      log "WARN: standard PCD conversion failed."

    log "Generating localization PCD: $PCD_LOCALIZATION_FILE"
    run_step "localization PCD generation" run_ros "python3 '$WORKSPACE/src/humanoid_navigation2/scripts/make_localization_pcd.py' '$PCD_FILE' '$PCD_LOCALIZATION_FILE' --min-z 0.0 --max-z 2.30 --voxel-size 0.10" || \
      log "WARN: localization PCD generation failed."
  else
    log "ERROR: base PCD is missing; skip PCD conversions: $PCD_FILE"
  fi

  log "[7/8] Stopping mapping launch..."
  stop_process_int "mapping launch" "$MAPPING_PID" 30

  log "[8/8] Checking output files..."
  local failed=0
  check_required_file "${MAP_PREFIX}.yaml" || failed=1
  check_required_file "${MAP_PREFIX}.pgm" || failed=1
  check_required_file "${MAP_PREFIX}.posegraph" || failed=1
  check_required_file "${MAP_PREFIX}.data" || failed=1
  check_required_file "$SC_DB_FILE" || failed=1
  check_required_file "$PCD_FILE" || failed=1
  check_required_file "$PCD_STANDARD_FILE" || failed=1
  check_required_file "$PCD_LOCALIZATION_FILE" || failed=1

  if [ -d "$BAG_DIR" ] && run_ros "ros2 bag info '$BAG_DIR'" > /dev/null 2>&1; then
    log "OK: rosbag recorded: $BAG_DIR"
  else
    log "ERROR: rosbag check failed: $BAG_DIR"
    failed=1
  fi

  log "Mapping log: $RUN_LOG"
  log "ROS log dir: $ROS_LOG_DIR"

  if [ "$failed" -eq 0 ]; then
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

  # Keep rosbag SIGINT so rosbag metadata is closed cleanly, but do not build
  # databases or overwrite map files from this aborted run.
  stop_process_int "rosbag recorder" "$BAG_PID" 30

  if is_alive "$PCD_PID"; then
    log "Stopping PCD saver without Ctrl+C save (pid=$PCD_PID)..."
    kill -TERM "$PCD_PID" > /dev/null 2>&1 || true
    sleep 2
    if is_alive "$PCD_PID"; then
      kill -KILL "$PCD_PID" > /dev/null 2>&1 || true
    fi
  fi

  stop_process_int "mapping launch" "$MAPPING_PID" 30

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
  exec 3</dev/tty
  INPUT_FD=3
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
log "Scan Context database: $SC_DB_FILE"
log "Bag directory: $BAG_DIR"
log "Run log: $RUN_LOG"

log "Starting mapping launch..."
ros2 launch humanoid_bringup robot_mapping.launch.py &
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
ros2 bag record -o "$BAG_DIR" \
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
    case "$user_cmd" in
      finish|FINISH|done|DONE|q|Q|quit|QUIT|exit|EXIT)
        log "Finish command received from $cmd_source."
        finish_mapping
        ;;
      abort|ABORT|cancel|CANCEL|discard|DISCARD)
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
