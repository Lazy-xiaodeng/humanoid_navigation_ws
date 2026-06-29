#!/usr/bin/env bash
set -eo pipefail
set +u

DURATION="${1:-8}"

echo "== LIO processes =="
pgrep -af 'run_mapping_online|fastlio_mapping|rslidar_sdk_node|point_cloud_filter_node|robosense_lidar_localization|prior_map_odom_bridge' || true

echo
echo "== Resource snapshot =="
ps -eo pid=,pcpu=,rss=,comm=,args= --sort=-pcpu \
  | awk '
    /run_mapping_online|fastlio_mapping|rslidar_sdk_node|point_cloud_filter_node|robosense_lidar_localization|prior_map_odom_bridge/ && !/awk/ {
      printf "pid=%s cpu=%s%% rss=%.1fMB comm=%s args=", $1, $2, $3 / 1024.0, $4;
      for (i = 5; i <= NF; ++i) {
        printf "%s%s", $i, (i == NF ? "" : " ");
      }
      printf "\n";
    }'

echo
echo "== Active LIO implementation =="
if pgrep -f 'run_mapping_online' >/dev/null 2>&1; then
  echo "Faster-LIO: active"
else
  echo "Faster-LIO: not detected"
fi

if pgrep -f 'fastlio_mapping' >/dev/null 2>&1; then
  echo "Fast-LIO: active"
else
  echo "Fast-LIO: not detected"
fi

echo
echo "== Topic hz (${DURATION}s each) =="
for topic in /airy_points /odom /fast_lio/cloud_registered /airy_points_filtered; do
  echo "-- ${topic}"
  timeout "${DURATION}" ros2 topic hz "${topic}" 2>&1 | tail -n 5 || true
done
