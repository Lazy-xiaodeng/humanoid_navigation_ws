#!/usr/bin/env bash
set -euo pipefail

RUN_DIR="${1:-/home/ubuntu/humanoid_ws/debug_monitor/integrated_voxel01_validation_$(date +%Y%m%d_%H%M%S)}"
RATE="${RATE:-3.0}"
JUMP_PROTECTION_MODE="${JUMP_PROTECTION_MODE:-monitor}"
OPEN3D_VOXELSIZE_FINE="${OPEN3D_VOXELSIZE_FINE:-0.10}"

WS="/home/ubuntu/humanoid_ws"
RO_LAUNCH="$WS/src/humanoid_navigation2/launch/robosense_prior_map_bag_test_isolated.launch.py"
OP_LAUNCH="$WS/src/humanoid_navigation2/launch/prior_map_bag_test.launch.py"

BAGS=(
  "nav_drift_test23:1406:/home/ubuntu/nav_drift_test/nav_drift_test23"
  "nav_drift_test24:1258:/home/ubuntu/nav_drift_test/nav_drift_test24"
  "nav_drift_test25:1510:/home/ubuntu/nav_drift_test/nav_drift_test25"
)

mkdir -p "$RUN_DIR/robosense_integrated" "$RUN_DIR/open3d"

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

run_robosense() {
  local name="$1"
  local duration="$2"
  local bag="$3"
  local out="$RUN_DIR/robosense_integrated/$name"
  mkdir -p "$out"
  echo "[ro-integrated voxel=0.10] $name duration=$duration rate=$RATE"
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

run_open3d() {
  local name="$1"
  local duration="$2"
  local bag="$3"
  local out="$RUN_DIR/open3d/$name"
  mkdir -p "$out"
  echo "[op fine_voxel=$OPEN3D_VOXELSIZE_FINE] $name duration=$duration rate=$RATE"
  (
    cd "$WS"
    set +u
    source install/setup.bash
    set -u
    export ROS_DOMAIN_ID=87
    export ROS_LOG_DIR="$out/ros_logs"
    mkdir -p "$ROS_LOG_DIR"

    ros2 launch "$OP_LAUNCH" \
      voxelsize_fine:="$OPEN3D_VOXELSIZE_FINE" \
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
    cleanup "$monitor_pid" "$launch_pid"
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

python3 "$WS/debug_monitor/analyze_integrated_ro_vs_open3d.py" \
  "$RUN_DIR" "$RUN_DIR" nav_drift_test23 nav_drift_test24 nav_drift_test25

python3 "$WS/debug_monitor/make_integrated_ro_open3d_plots.py" \
  "$RUN_DIR" "$RUN_DIR"

echo "$RUN_DIR"
