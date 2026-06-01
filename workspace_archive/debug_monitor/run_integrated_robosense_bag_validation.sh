#!/usr/bin/env bash
set -euo pipefail

RUN_DIR="${1:-/home/ubuntu/humanoid_ws/workspace_archive/debug_monitor/integrated_robosense_validation_$(date +%Y%m%d_%H%M%S)}"
RATE="${RATE:-3.0}"
OPEN3D_BASE="${OPEN3D_BASE:-/home/ubuntu/humanoid_ws/workspace_archive/debug_monitor/robosense_open3d_validation_20260531_full}"
JUMP_PROTECTION_MODE="${JUMP_PROTECTION_MODE:-monitor}"

WS="/home/ubuntu/humanoid_ws"
RO_LAUNCH="$WS/src/humanoid_navigation2/launch/robosense_prior_map_bag_test_isolated.launch.py"

BAGS=(
  "nav_drift_test23:1406:/home/ubuntu/nav_drift_test/nav_drift_test23"
  "nav_drift_test24:1258:/home/ubuntu/nav_drift_test/nav_drift_test24"
  "nav_drift_test25:1510:/home/ubuntu/nav_drift_test/nav_drift_test25"
)

mkdir -p "$RUN_DIR/robosense_integrated"

cleanup() {
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

run_ro() {
  local name="$1"
  local duration="$2"
  local bag="$3"
  local out="$RUN_DIR/robosense_integrated/$name"
  mkdir -p "$out"
  echo "[ro-integrated] $name duration=$duration rate=$RATE"
  (
    cd "$WS"
    set +u
    source install/setup.bash
    set -u
    export ROS_DOMAIN_ID=86
    export ROS_LOG_DIR="$out/ros_logs"
    mkdir -p "$ROS_LOG_DIR"

    ros2 launch "$RO_LAUNCH" \
      jump_protection_mode:="$JUMP_PROTECTION_MODE" \
      >"$out/robosense_integrated_launch.log" 2>&1 &
    local launch_pid=$!

    python3 "$WS/src/humanoid_navigation2/humanoid_navigation2/prior_map_bag_monitor.py" \
      --ros-args \
      -p output_dir:="$out" \
      -p robot_pose_topic:=/robot_realpose \
      -p prior_odom_topic:=/prior_localization/robosense_odom \
      >"$out/prior_map_bag_monitor.log" 2>&1 &
    local monitor_pid=$!

    sleep 6

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
    cleanup "$monitor_pid" "$launch_pid"
    return "$play_rc"
  )
}

for spec in "${BAGS[@]}"; do
  IFS=: read -r name duration bag <<<"$spec"
  run_ro "$name" "$duration" "$bag"
done

python3 "$WS/workspace_archive/debug_monitor/analyze_integrated_ro_vs_open3d.py" \
  "$RUN_DIR" "$OPEN3D_BASE" nav_drift_test23 nav_drift_test24 nav_drift_test25

echo "$RUN_DIR"
