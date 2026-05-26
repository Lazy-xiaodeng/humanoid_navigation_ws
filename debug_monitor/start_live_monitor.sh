#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/humanoid_ws}"
cd "$WORKSPACE"

set +u
if [ -f /opt/ros/jazzy/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi

if [ -f "$WORKSPACE/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$WORKSPACE/install/setup.bash"
fi
set -u

STAMP="$(date +%Y%m%d_%H%M%S)"
MON_DIR="$WORKSPACE/debug_monitor/live_$STAMP"
PIDS_FILE="$MON_DIR/pids.tsv"
META_FILE="$MON_DIR/meta.txt"

mkdir -p "$MON_DIR"
: > "$PIDS_FILE"

latest_nav_log="$(ls -t "$WORKSPACE"/debug_logs/start_navigation_*.log 2>/dev/null | head -1 || true)"
latest_ros_log_dir=""
if [ -n "$latest_nav_log" ]; then
  latest_ros_log_dir="$(sed -n 's/^ROS log dir: //p' "$latest_nav_log" | tail -1 || true)"
fi
if [ -z "$latest_ros_log_dir" ] || [ ! -d "$latest_ros_log_dir" ]; then
  latest_ros_log_dir="$(find "$WORKSPACE/debug_logs" -maxdepth 1 -type d -name 'ros_*' 2>/dev/null | sort | tail -1 || true)"
fi

{
  echo "start_time=$(date '+%F %T %z')"
  echo "workspace=$WORKSPACE"
  echo "monitor_dir=$MON_DIR"
  echo "ros_distro=${ROS_DISTRO:-unknown}"
  echo "ros_domain_id=${ROS_DOMAIN_ID:-unset}"
  echo "rmw_implementation=${RMW_IMPLEMENTATION:-unset}"
  echo "latest_nav_log=$latest_nav_log"
  echo "latest_ros_log_dir=$latest_ros_log_dir"
} > "$META_FILE"

start_cmd() {
  local name="$1"
  shift
  local logfile="$MON_DIR/${name}.log"

  nohup setsid bash -c '
    set -e
    echo "# command: $*"
    echo "# started: $(date "+%F %T %z")"
    exec "$@"
  ' _ "$@" > "$logfile" 2>&1 &

  printf '%s\t%s\t%s\n' "$!" "$name" "$logfile" >> "$PIDS_FILE"
}

start_cmd hz_airy_points ros2 topic hz /airy_points --window 100
start_cmd hz_airy_points_filtered ros2 topic hz /airy_points_filtered --window 100
start_cmd hz_airy_points_for_elevation ros2 topic hz /airy_points_for_elevation --window 100
start_cmd hz_fast_lio_cloud_registered ros2 topic hz /fast_lio/cloud_registered --window 100
start_cmd hz_odom ros2 topic hz /odom --window 100
start_cmd hz_hdl_bootstrap_odom ros2 topic hz /hdl_bootstrap/odom --window 100
start_cmd hz_pose ros2 topic hz /pose --window 100
start_cmd hz_pcl_pose ros2 topic hz /pcl_pose --window 100
start_cmd hz_robot_realpose ros2 topic hz /robot_realpose --window 100
start_cmd hz_tf ros2 topic hz /tf --window 200

start_cmd bw_airy_points ros2 topic bw /airy_points
start_cmd bw_airy_points_filtered ros2 topic bw /airy_points_filtered

start_cmd tf_echo_map_odom ros2 run tf2_ros tf2_echo map odom
start_cmd tf_echo_map_base_footprint ros2 run tf2_ros tf2_echo map base_footprint
start_cmd tf_echo_odom_base_footprint ros2 run tf2_ros tf2_echo odom base_footprint
start_cmd tf_echo_map_ground_base_footprint ros2 run tf2_ros tf2_echo map_ground base_footprint

start_cmd tf_monitor_map_odom ros2 run tf2_ros tf2_monitor map odom
start_cmd tf_monitor_map_base_footprint ros2 run tf2_ros tf2_monitor map base_footprint
start_cmd tf_monitor_odom_base_footprint ros2 run tf2_ros tf2_monitor odom base_footprint
start_cmd tf_monitor_map_ground_base_footprint ros2 run tf2_ros tf2_monitor map_ground base_footprint

start_cmd echo_rosout ros2 topic echo /rosout
start_cmd echo_initialpose ros2 topic echo /initialpose
start_cmd echo_hdl_relocalize_prior ros2 topic echo /hdl_relocalize_prior
start_cmd echo_diagnostics ros2 topic echo /diagnostics

if [ -n "$latest_nav_log" ] && [ -f "$latest_nav_log" ]; then
  start_cmd tail_start_navigation tail -n 0 -F "$latest_nav_log"
fi

if [ -n "$latest_ros_log_dir" ] && [ -d "$latest_ros_log_dir" ]; then
  start_cmd tail_ros_node_logs bash -lc 'tail -n 0 -F "$1"/*.log' _ "$latest_ros_log_dir"
fi

echo "$MON_DIR"
