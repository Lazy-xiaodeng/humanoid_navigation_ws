#!/usr/bin/env bash
set -euo pipefail

WS="${WS:-/home/ubuntu/humanoid_ws}"
BAG="${BAG:-/home/ubuntu/nav_drift_test/nav_drift_test44}"
RATE="${RATE:-1.0}"
DISTANCES="${DISTANCES:-2.0 1.0 0.8}"
PLAYBACK_DURATION="${PLAYBACK_DURATION:-}"
ROS_DOMAIN_BASE="${ROS_DOMAIN_BASE:-90}"
RUN_DIR="${1:-$WS/debug_monitor/robosense_blind_distance_sweep_$(date +%Y%m%d_%H%M%S)}"
BASE_CONFIG="$WS/src/robosense_lidar_localization/config/robosense_lidar_localization_bag.yaml"
RO_LAUNCH="$WS/src/humanoid_navigation2/launch/robosense_prior_map_bag_test_isolated.launch.py"

mkdir -p "$RUN_DIR"

cleanup() {
  local pids=("$@")
  for pid in "${pids[@]}"; do
    if [ -n "${pid:-}" ] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done
  sleep 3
  for pid in "${pids[@]}"; do
    if [ -n "${pid:-}" ] && kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done
  wait "${pids[@]}" 2>/dev/null || true
}

make_config() {
  local distance="$1"
  local out="$2"
  python3 - "$BASE_CONFIG" "$out" "$distance" <<'PY'
import re
import sys
src, dst, distance = sys.argv[1:4]
text = open(src, encoding="utf-8").read()
text = re.sub(
    r'(?m)^abs_pose_topic:\s*".*"\s*$',
    'abs_pose_topic: "/prior_localization/manual_initialpose"',
    text,
)
text = re.sub(r"(?m)^debug_print:\s*false\s*$", "debug_print: true", text)
text = re.sub(r"(?m)^(\s+debug_print:)\s*false\s*$", r"\1 true", text)
text = re.sub(r"(?m)^(\s+blind_distance:)\s*[-0-9.]+\s*$", rf"\1 {distance}", text)
open(dst, "w", encoding="utf-8").write(text)
PY
}

run_one() {
  local index="$1"
  local distance="$2"
  local label="blind_${distance//./p}"
  local out="$RUN_DIR/$label"
  mkdir -p "$out/ros_logs"
  make_config "$distance" "$out/robosense_lidar_localization_bag_${label}.yaml"

  {
    echo "run_label=$label"
    echo "blind_distance=$distance"
    echo "bag=$BAG"
    echo "rate=$RATE"
    echo "playback_duration=${PLAYBACK_DURATION:-full}"
    echo "ros_domain_id=$((ROS_DOMAIN_BASE + index))"
    date --iso-8601=seconds
  } > "$out/run_info.txt"

  (
    cd "$WS"
    set +u
    source install/setup.bash
    set -u
    export ROS_DOMAIN_ID="$((ROS_DOMAIN_BASE + index))"
    export ROS_LOG_DIR="$out/ros_logs"

    ros2 launch "$RO_LAUNCH" \
      config_file:="$out/robosense_lidar_localization_bag_${label}.yaml" \
      jump_protection_mode:=monitor \
      > "$out/launch.log" 2>&1 &
    local launch_pid=$!

    python3 "$WS/src/humanoid_navigation2/humanoid_navigation2/prior_map_bag_monitor.py" \
      --ros-args \
      -p output_dir:="$out" \
      -p robot_pose_topic:=/robot_realpose \
      -p prior_odom_topic:=/prior_localization/robosense_odom \
      > "$out/prior_map_bag_monitor.log" 2>&1 &
    local monitor_pid=$!

    sleep 6

    set +e
    local duration_args=()
    if [ -n "$PLAYBACK_DURATION" ]; then
      duration_args=(--playback-duration "$PLAYBACK_DURATION")
    fi

    ros2 bag play "$BAG" \
      --clock \
      --topics /odom /fast_lio/cloud_registered /robot_realpose /navigation/status \
      --read-ahead-queue-size 100 \
      --rate "$RATE" \
      "${duration_args[@]}" \
      --disable-keyboard-controls \
      > "$out/rosbag_play.log" 2>&1
    local play_rc=$?
    set -e

    sleep 3
    cleanup "$monitor_pid" "$launch_pid"
    echo "$play_rc" > "$out/rosbag_play.returncode"
    if grep -q "process has died" "$out/launch.log"; then
      echo "launch process died; see $out/launch.log" >&2
      return 20
    fi
    return "$play_rc"
  )
}

index=0
for distance in $DISTANCES; do
  echo "[blind-sweep] running blind_distance=$distance output=$RUN_DIR"
  run_one "$index" "$distance"
  index=$((index + 1))
done

python3 "$WS/tools/analyze_robosense_blind_sweep.py" "$RUN_DIR"
echo "$RUN_DIR"
