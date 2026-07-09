#!/usr/bin/env bash
set -euo pipefail

WS="${WS:-/home/ubuntu/humanoid_ws}"
BAG="${BAG:-/home/ubuntu/nav_drift_test/nav_drift_test44}"
RATE="${RATE:-1.0}"
PLAYBACK_DURATION="${PLAYBACK_DURATION:-240}"
START_OFFSET="${START_OFFSET:-0}"
ROS_DOMAIN_BASE="${ROS_DOMAIN_BASE:-170}"
BLIND_DISTANCE="${BLIND_DISTANCE:-1.0}"
MAP_VOXELS="${MAP_VOXELS:-0.10 0.15 0.30}"
SOURCE_VOXELS="${SOURCE_VOXELS:-0.20 0.15 0.10}"
NEIGHBOR_RADII="${NEIGHBOR_RADII:-0.30}"
MAX_PAIR_SIZES="${MAX_PAIR_SIZES:-0}"
RUN_DIR="${1:-$WS/debug_monitor/robosense_voxel_sweep_$(date +%Y%m%d_%H%M%S)}"
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

label_num() {
  echo "$1" | sed 's/\./p/g'
}

make_config() {
  local map_voxel="$1"
  local source_voxel="$2"
  local neighbor_radius="$3"
  local max_pair_size="$4"
  local out="$5"
  python3 - "$BASE_CONFIG" "$out" "$BLIND_DISTANCE" "$map_voxel" "$source_voxel" "$neighbor_radius" "$max_pair_size" <<'PY'
import re
import sys

src, dst, blind, map_voxel, source_voxel, neighbor_radius, max_pair_size = sys.argv[1:8]
text = open(src, encoding="utf-8").read()
text = re.sub(
    r'(?m)^abs_pose_topic:\s*".*"\s*$',
    'abs_pose_topic: "/prior_localization/manual_initialpose"',
    text,
)
text = re.sub(r"(?m)^debug_print:\s*false\s*$", "debug_print: true", text)
text = re.sub(r"(?m)^(\s+debug_print:)\s*false\s*$", r"\1 true", text)
text = re.sub(r"(?m)^(\s+blind_distance:)\s*[-0-9.]+\s*$", rf"\1 {blind}", text)
text = re.sub(r"(?m)^(\s+neighbor_search_radius:)\s*[-0-9.]+\s*$", rf"\1 {neighbor_radius}", text)
text = re.sub(r"(?m)^(\s+max_pair_size:)\s*[-0-9]+\s*$", rf"\1 {max_pair_size}", text)
text = re.sub(r"(?m)^map_voxel_leaf_size:\s*[-0-9.]+\s*$", f"map_voxel_leaf_size: {map_voxel}", text)
if re.search(r"(?m)^source_voxel_leaf_size:\s*[-0-9.]+\s*$", text):
    text = re.sub(r"(?m)^source_voxel_leaf_size:\s*[-0-9.]+\s*$", f"source_voxel_leaf_size: {source_voxel}", text)
else:
    text = re.sub(
        r"(?m)^(input_cloud_size_thr:\s*[-0-9.]+\s*)$",
        rf"\1\nsource_voxel_leaf_size: {source_voxel}",
        text,
    )
open(dst, "w", encoding="utf-8").write(text)
PY
}

run_one() {
  local index="$1"
  local map_voxel="$2"
  local source_voxel="$3"
  local neighbor_radius="$4"
  local max_pair_size="$5"
  local label="map_$(label_num "$map_voxel")_src_$(label_num "$source_voxel")_rad_$(label_num "$neighbor_radius")_max_${max_pair_size}"
  local out="$RUN_DIR/$label"
  mkdir -p "$out/ros_logs"
  make_config "$map_voxel" "$source_voxel" "$neighbor_radius" "$max_pair_size" "$out/robosense_lidar_localization_bag_${label}.yaml"

  {
    echo "run_label=$label"
    echo "blind_distance=$BLIND_DISTANCE"
    echo "map_voxel_leaf_size=$map_voxel"
    echo "source_voxel_leaf_size=$source_voxel"
    echo "neighbor_search_radius=$neighbor_radius"
    echo "max_pair_size=$max_pair_size"
    echo "bag=$BAG"
    echo "rate=$RATE"
    echo "start_offset=$START_OFFSET"
    echo "playback_duration=$PLAYBACK_DURATION"
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
    ros2 bag play "$BAG" \
      --clock \
      --topics /odom /fast_lio/cloud_registered /robot_realpose /navigation/status \
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
for map_voxel in $MAP_VOXELS; do
  for source_voxel in $SOURCE_VOXELS; do
    for neighbor_radius in $NEIGHBOR_RADII; do
      for max_pair_size in $MAX_PAIR_SIZES; do
        echo "[voxel-sweep] map_voxel=$map_voxel source_voxel=$source_voxel neighbor_radius=$neighbor_radius max_pair_size=$max_pair_size output=$RUN_DIR"
        run_one "$index" "$map_voxel" "$source_voxel" "$neighbor_radius" "$max_pair_size"
        index=$((index + 1))
      done
    done
  done
done

python3 "$WS/tools/analyze_robosense_voxel_sweep.py" "$RUN_DIR"
echo "$RUN_DIR"
