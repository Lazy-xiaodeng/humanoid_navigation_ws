#!/usr/bin/env bash
set -eo pipefail

cd /home/ubuntu/humanoid_ws

set +u
source install/setup.bash
set -u

mkdir -p /tmp/ros_logs workspace_archive/debug_monitor
export ROS_LOG_DIR=/tmp/ros_logs
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-/home/ubuntu/.config/fastdds_shm.xml}"
export RMW_FASTRTPS_USE_QOS_FROM_XML="${RMW_FASTRTPS_USE_QOS_FROM_XML:-1}"

exec workspace_archive/debug_monitor/fastlio_drift_dashboard.py --host 127.0.0.1 --port 8766
