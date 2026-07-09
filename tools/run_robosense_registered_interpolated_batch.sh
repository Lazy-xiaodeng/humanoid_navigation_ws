#!/usr/bin/env bash
set -euo pipefail

WS="${WS:-/home/ubuntu/humanoid_ws}"
BAG_ROOT="${BAG_ROOT:-/home/ubuntu/nav_drift_test}"
RATE="${RATE:-1.0}"
ROS_DOMAIN_BASE="${ROS_DOMAIN_BASE:-180}"
RUN_DIR="${1:-$WS/debug_monitor/robosense_registered_interpolated_allbags_$(date +%Y%m%d_%H%M%S)}"

mkdir -p "$RUN_DIR"

duration_for_bag() {
  ros2 bag info "$1" | python3 -c 'import re,sys,math
text=sys.stdin.read()
m=re.search(r"Duration:\s+([0-9.]+)s", text)
if not m:
    raise SystemExit("Duration not found")
print(f"{float(m.group(1)) + 1.0:.3f}")'
}

index=0
for bag in "$BAG_ROOT"/nav_drift_test4{1..6}; do
  if [ ! -f "$bag/metadata.yaml" ]; then
    echo "[batch] skip missing bag: $bag"
    continue
  fi
  bag_name="$(basename "$bag")"
  out="$RUN_DIR/$bag_name"
  duration="$(duration_for_bag "$bag")"
  echo "[batch] running $bag_name duration=${duration}s output=$out"
  BAG="$bag" \
    PLAYBACK_DURATION="$duration" \
    RATE="$RATE" \
    ROS_DOMAIN_BASE="$((ROS_DOMAIN_BASE + index))" \
    VARIANTS=registered_interpolated \
    "$WS/tools/run_robosense_input_chain_experiment.sh" "$out"
  index=$((index + 1))
done

echo "$RUN_DIR"
