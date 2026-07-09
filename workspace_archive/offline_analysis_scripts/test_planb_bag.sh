#!/bin/bash
# Plan B bag replay 测试脚本
# 回放 nav_drift_test4 bag，用新 NDT 代码离线定位，对比新旧 ndt_status

set -e

BAG="/home/ubuntu/nav_drift_test4/nav_drift_test4_0.mcap"
LOC_PARAMS="/home/ubuntu/humanoid_ws/src/lidar_localization/param/localization.yaml"
OUTPUT_CSV="/tmp/planb_test_$(date +%Y%m%d_%H%M%S).csv"
OUTPUT_LOG="/tmp/planb_test_$(date +%Y%m%d_%H%M%S).log"

echo "=== Plan B Bag Replay Test ==="
echo "Bag: $BAG"
echo "Output CSV: $OUTPUT_CSV"
echo "Output Log: $OUTPUT_LOG"

# Cleanup any leftover processes on exit
cleanup() {
  echo "Cleaning up..."
  kill %1 2>/dev/null || true
  kill %2 2>/dev/null || true
}
trap cleanup EXIT

# Step 1: Start NDT node with use_sim_time
echo "[1/4] Starting NDT node..."
ros2 run lidar_localization_ros2 lidar_localization_node --ros-args \
  --params-file "$LOC_PARAMS" \
  -p use_sim_time:=true \
  -p set_initial_pose:=True \
  -p initial_pose_x:=0.0 \
  -p initial_pose_y:=0.0 \
  -p initial_pose_z:=0.0 \
  -p score_threshold:=0.3 \
  -p reject_pose_jump:=True \
  -p max_pose_jump_translation:=0.80 \
  -p max_pose_jump_yaw:=0.60 \
  -p initialpose_relax_duration_sec:=4.0 \
  -p initialpose_max_pose_jump_translation:=2.00 \
  -p initialpose_max_pose_jump_yaw:=3.00 \
  -p pose_jump_reacquire_enabled:=True \
  -p pose_jump_reacquire_max_translation:=0.80 \
  -p pose_jump_reacquire_max_yaw:=0.30 \
  -p pose_jump_reacquire_max_fitness:=0.08 \
  -p pose_jump_reacquire_required_frames:=2 \
  -p pose_jump_reacquire_xy_tolerance:=0.50 \
  -p pose_jump_reacquire_yaw_tolerance:=0.25 \
  -p use_fastlio_delta_guess:=True \
  -p fastlio_camera_frame:=camera_init \
  -p fastlio_body_frame:=body \
  -p tf_max_stamp_mismatch_sec:=0.2 \
  -p fastlio_max_delta_translation:=0.20 \
  -p fastlio_max_delta_yaw:=0.25 \
  -p fastlio_max_dead_reckon_sec:=2.0 \
  -p republish_last_good_tf_on_failure:=True \
  -p max_last_good_tf_age_sec:=5.0 \
  -p ndt_outlier_ratio:=0.30 \
  -p ndt_max_corr_dist:=2.0 \
  -p ndt_rotation_prior_enabled:=True \
  -p ndt_rotation_prior_weight:=10.0 \
  -p ndt_rotation_prior_roll_pitch_only:=True \
  -p use_odom:=false \
  -p enable_debug:=true \
  --log-level warn > "$OUTPUT_LOG" 2>&1 &

NDT_PID=$!
sleep 3

# Step 2: Configure and activate lifecycle
echo "[2/4] Activating lifecycle..."
ros2 lifecycle set /lidar_localization configure 2>/dev/null || {
  echo "WARNING: configure failed (node may not be lifecycle-managed, trying alternative...)"
}
sleep 2
ros2 lifecycle set /lidar_localization activate 2>/dev/null || {
  echo "WARNING: activate failed"
}
sleep 2

# Step 3: Start recording ndt_status
echo "[3/4] Recording ndt_status to CSV..."
ros2 topic echo /localization/ndt_status --csv > "$OUTPUT_CSV" 2>/dev/null &
RECORD_PID=$!
sleep 1

# Step 4: Play bag
echo "[4/4] Playing bag..."
ros2 bag play "$BAG" --clock 100 \
  --topics /fast_lio/cloud_registered /tf /tf_static /initialpose 2>&1 | tail -5

# Wait for bag to finish
wait $RECORD_PID 2>/dev/null || true
sleep 2

echo "=== Done ==="
echo "CSV output: $OUTPUT_CSV"
echo "Lines: $(wc -l < "$OUTPUT_CSV")"
echo "Log: $OUTPUT_LOG"
