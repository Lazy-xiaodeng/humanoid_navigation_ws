#!/usr/bin/env bash
set -euo pipefail

WS="${WS:-/home/ubuntu/humanoid_ws}"
BAG="${BAG:-/home/ubuntu/nav_drift_test/nav_drift_test44}"
RATE="${RATE:-1.0}"
PLAYBACK_DURATION="${PLAYBACK_DURATION:-1512}"
START_OFFSET="${START_OFFSET:-0}"
ROS_DOMAIN_BASE="${ROS_DOMAIN_BASE:-180}"
VARIANTS="${VARIANTS:-registered_interpolated raw_airy_points_filtered}"
RUN_DIR="${1:-$WS/debug_monitor/robosense_input_chain_bag44_$(date +%Y%m%d_%H%M%S)}"
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
  local variant="$1"
  local out="$2"
  python3 - "$BASE_CONFIG" "$out" "$variant" <<'PY'
import re
import sys

src, dst, variant = sys.argv[1:4]
text = open(src, encoding="utf-8").read()
text = re.sub(r'(?m)^abs_pose_topic:\s*".*"\s*$', 'abs_pose_topic: "/prior_localization/manual_initialpose"', text)
text = re.sub(r"(?m)^debug_print:\s*false\s*$", "debug_print: true", text)
text = re.sub(r"(?m)^(\s+debug_print:)\s*false\s*$", r"\1 true", text)
text = re.sub(r"(?m)^map_voxel_leaf_size:\s*[-0-9.]+\s*$", "map_voxel_leaf_size: 0.30", text)
text = re.sub(r"(?m)^source_voxel_leaf_size:\s*[-0-9.]+\s*$", "source_voxel_leaf_size: 0.15", text)
text = re.sub(r"(?m)^(\s+neighbor_search_radius:)\s*[-0-9.]+\s*$", r"\1 0.30", text)
text = re.sub(r"(?m)^(\s+max_pair_size:)\s*[-0-9]+\s*$", r"\1 0", text)
text = re.sub(r"(?m)^(\s+blind_distance:)\s*[-0-9.]+\s*$", r"\1 1.0", text)
if variant == "raw_airy_points_filtered":
    text = re.sub(r'(?m)^lidar_topic:\s*".*"\s*$', 'lidar_topic: "/airy_points_filtered"', text)
    text = re.sub(r"(?m)^convert_registered_cloud_to_body:\s*(true|false)\s*$", "convert_registered_cloud_to_body: false", text)
elif variant == "raw_airy_points":
    text = re.sub(r'(?m)^lidar_topic:\s*".*"\s*$', 'lidar_topic: "/airy_points"', text)
    text = re.sub(r"(?m)^convert_registered_cloud_to_body:\s*(true|false)\s*$", "convert_registered_cloud_to_body: false", text)
elif variant == "registered_interpolated":
    text = re.sub(r'(?m)^lidar_topic:\s*".*"\s*$', 'lidar_topic: "/fast_lio/cloud_registered"', text)
    text = re.sub(r"(?m)^convert_registered_cloud_to_body:\s*(true|false)\s*$", "convert_registered_cloud_to_body: true", text)
else:
    raise SystemExit(f"unknown variant: {variant}")
open(dst, "w", encoding="utf-8").write(text)
PY
}

run_one() {
  local index="$1"
  local variant="$2"
  local topics="$3"
  local out="$RUN_DIR/$variant"
  mkdir -p "$out/ros_logs"
  make_config "$variant" "$out/robosense_lidar_localization_bag_${variant}.yaml"

  {
    echo "run_label=$variant"
    echo "variant=$variant"
    echo "bag=$BAG"
    echo "rate=$RATE"
    echo "start_offset=$START_OFFSET"
    echo "playback_duration=$PLAYBACK_DURATION"
    echo "topics=$topics"
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
      config_file:="$out/robosense_lidar_localization_bag_${variant}.yaml" \
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
    # shellcheck disable=SC2086
    ros2 bag play "$BAG" \
      --clock \
      --topics $topics \
      --read-ahead-queue-size 100 \
      --rate "$RATE" \
      --start-offset "$START_OFFSET" \
      --playback-duration "$PLAYBACK_DURATION" \
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
for variant in $VARIANTS; do
  case "$variant" in
    registered_interpolated)
      topics="/odom /fast_lio/cloud_registered /robot_realpose /navigation/status"
      ;;
    raw_airy_points_filtered)
      topics="/odom /airy_points_filtered /robot_realpose /navigation/status"
      ;;
    raw_airy_points)
      topics="/odom /airy_points /robot_realpose /navigation/status"
      ;;
    *)
      echo "unknown variant: $variant" >&2
      exit 2
      ;;
  esac
  run_one "$index" "$variant" "$topics"
  index=$((index + 1))
done

echo "$RUN_DIR"
