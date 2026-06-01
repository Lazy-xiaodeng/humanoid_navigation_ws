#!/usr/bin/env bash
set -euo pipefail

RUN_DIR="${1:-/home/ubuntu/humanoid_ws/workspace_archive/debug_monitor/robosense_open3d_validation_$(date +%Y%m%d_%H%M%S)}"
RATE="${RATE:-3.0}"

WS="/home/ubuntu/humanoid_ws"
RS_WS="/home/ubuntu/exp_code/robosense_localization"
MAP_PCD="$RS_WS/lidar_localization/map/hall_open3d_grounded.pcd"

BAGS=(
  "nav_drift_test23:1406:/home/ubuntu/nav_drift_test/nav_drift_test23"
  "nav_drift_test24:1258:/home/ubuntu/nav_drift_test/nav_drift_test24"
  "nav_drift_test25:1510:/home/ubuntu/nav_drift_test/nav_drift_test25"
)

mkdir -p "$RUN_DIR"/robosense "$RUN_DIR"/open3d

run_robosense() {
  local name="$1"
  local duration="$2"
  local bag="$3"
  local out="$RUN_DIR/robosense/$name"
  mkdir -p "$out"
  echo "[robosense] $name duration=$duration rate=$RATE"
  (
    cd "$RS_WS"
    ROS_DOMAIN_ID=83 ./tools/run_tj_lidar_localization_test.sh \
      "$duration" "$RATE" "$out" "$bag" registered
    python3 tools/make_odom_vs_lidar_html.py \
      "$out/poses_${duration}s.csv" \
      "$MAP_PCD" \
      "$name RoboSense" \
      "$out/odom_vs_robosense.html"
  )
}

cleanup_open3d() {
  local pids=("$@")
  for pid in "${pids[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done
  sleep 2
  for pid in "${pids[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done
  wait "${pids[@]}" 2>/dev/null || true
}

run_open3d() {
  local name="$1"
  local duration="$2"
  local bag="$3"
  local out="$RUN_DIR/open3d/$name"
  mkdir -p "$out"
  echo "[open3d] $name duration=$duration rate=$RATE"
  (
    cd "$WS"
    set +u
    source install/setup.bash
    set -u
    export ROS_DOMAIN_ID=84
    export ROS_LOG_DIR="$out/ros_logs"
    mkdir -p "$ROS_LOG_DIR"

    ros2 launch humanoid_navigation2 prior_map_bag_test.launch.py \
      >"$out/open3d_launch.log" 2>&1 &
    local launch_pid=$!

    python3 "$WS/src/humanoid_navigation2/humanoid_navigation2/prior_map_bag_monitor.py" \
      --ros-args \
      -p output_dir:="$out" \
      -p robot_pose_topic:=/robot_realpose \
      >"$out/prior_map_bag_monitor.log" 2>&1 &
    local monitor_pid=$!

    sleep 8

    set +e
    ros2 bag play "$bag" \
      --clock \
      --topics /odom /fast_lio/cloud_registered /robot_realpose /navigation/status \
      --read-ahead-queue-size 100 \
      --rate "$RATE" \
      --playback-duration "$duration" \
      --disable-keyboard-controls \
      >"$out/rosbag_play.log" 2>&1
    local play_rc=$?
    set -e

    sleep 3
    cleanup_open3d "$monitor_pid" "$launch_pid"
    return "$play_rc"
  )
}

if [[ "${SKIP_ROBOSENSE:-0}" != "1" ]]; then
  for spec in "${BAGS[@]}"; do
    IFS=: read -r name duration bag <<<"$spec"
    run_robosense "$name" "$duration" "$bag"
  done
fi

if [[ "${SKIP_OPEN3D:-0}" != "1" ]]; then
  for spec in "${BAGS[@]}"; do
    IFS=: read -r name duration bag <<<"$spec"
    run_open3d "$name" "$duration" "$bag"
  done
fi

python3 "$WS/workspace_archive/debug_monitor/analyze_robosense_open3d_validation.py" \
  "$RUN_DIR" nav_drift_test23 nav_drift_test24 nav_drift_test25

echo "$RUN_DIR"
