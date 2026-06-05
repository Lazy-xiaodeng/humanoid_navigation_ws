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

mkdir -p "$MAP_DIR" "$PCD_DIR" "$BAG_ROOT" "$LOG_DIR" "$ROS_LOG_DIR"
export ROS_LOG_DIR

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
  trap - INT TERM

  log "Ctrl+C received. Finishing mapping and saving outputs..."

  # Stop data recorders first so bag and PCD are flushed to disk.
  stop_process_int "rosbag recorder" "$BAG_PID" 90
  stop_process_int "PCD saver" "$PCD_PID" 120

  log "Saving Fast-LIO map through /map_save when available..."
  if wait_for_service "/map_save" 5; then
    run_ros "ros2 service call /map_save std_srvs/srv/Trigger '{}'" || \
      log "WARN: /map_save call failed; continuing with PCD saver output."
  else
    log "WARN: /map_save service unavailable; continuing with PCD saver output."
  fi

  log "Saving 2D occupancy grid map: ${MAP_PREFIX}.yaml/.pgm"
  if wait_for_service "/slam_toolbox/save_map" 20; then
    run_ros "ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \"{name: {data: '$MAP_PREFIX'}}\""
  else
    log "WARN: /slam_toolbox/save_map unavailable; trying map_saver_cli."
    run_ros "ros2 run nav2_map_server map_saver_cli -f '$MAP_PREFIX'" || true
  fi

  log "Saving slam_toolbox posegraph: ${MAP_PREFIX}.posegraph/.data"
  if wait_for_service "/slam_toolbox/serialize_map" 20; then
    run_ros "ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \"{filename: '$MAP_PREFIX'}\""
  else
    log "WARN: /slam_toolbox/serialize_map unavailable; posegraph will not be saved."
  fi

  log "Building Scan Context database: $SC_DB_FILE"
  if [ -d "$BAG_DIR" ]; then
    run_ros "ros2 run humanoid_relocalization build_sc_database.py --bag '$BAG_DIR' --output '$SC_DB_FILE' --cloud-topic /fast_lio/cloud_registered --odom-topic /odom --interval 2.0"
  else
    log "ERROR: bag directory not found, skip Scan Context database: $BAG_DIR"
  fi

  if [ -s "$PCD_FILE" ]; then
    log "Generating standard-frame PCD: $PCD_STANDARD_FILE"
    run_ros "python3 '$WORKSPACE/src/humanoid_navigation2/humanoid_navigation2/pcd_converter.py' '$PCD_FILE' '$PCD_STANDARD_FILE'" || \
      log "WARN: standard PCD conversion failed."

    log "Generating localization PCD: $PCD_LOCALIZATION_FILE"
    run_ros "python3 '$WORKSPACE/src/humanoid_navigation2/scripts/make_localization_pcd.py' '$PCD_FILE' '$PCD_LOCALIZATION_FILE' --min-z 0.0 --max-z 2.30 --voxel-size 0.10" || \
      log "WARN: localization PCD generation failed."
  else
    log "ERROR: base PCD is missing; skip PCD conversions: $PCD_FILE"
  fi

  log "Stopping mapping launch..."
  stop_process_int "mapping launch" "$MAPPING_PID" 30

  log "Checking output files..."
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

trap finish_mapping INT TERM

cd "$WORKSPACE"

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
log "Press Ctrl+C in this terminal when mapping is complete. The script will save maps, bag, PCD files, Scan Context database, and then check output files."

while is_alive "$MAPPING_PID"; do
  sleep 2
done

log "Mapping launch exited before Ctrl+C."
finish_mapping
