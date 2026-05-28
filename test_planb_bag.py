#!/usr/bin/env python3
"""Plan B bag replay: 订阅 ndt_status 并写入 JSONL"""
import subprocess
import signal
import sys
import time
import json
import os

BAG = "/home/ubuntu/nav_drift_test4/nav_drift_test4_0.mcap"
OUTPUT = f"/tmp/planb_test_{time.strftime('%Y%m%d_%H%M%S')}.jsonl"

# Build ros2 run command
cmd = [
    "ros2", "run", "lidar_localization_ros2", "lidar_localization_node",
    "--ros-args",
    "--params-file", "/home/ubuntu/humanoid_ws/src/lidar_localization/param/localization.yaml",
    "-p", "use_sim_time:=true",
    "-p", "set_initial_pose:=True",
    "-p", "initial_pose_x:=0.0",
    "-p", "initial_pose_y:=0.0",
    "-p", "initial_pose_z:=0.0",
    "-p", "score_threshold:=0.3",
    "-p", "max_pose_jump_translation:=0.80",
    "-p", "max_pose_jump_yaw:=0.60",
    "-p", "initialpose_relax_duration_sec:=4.0",
    "-p", "initialpose_max_pose_jump_translation:=2.00",
    "-p", "initialpose_max_pose_jump_yaw:=3.00",
    "-p", "pose_jump_reacquire_enabled:=True",
    "-p", "pose_jump_reacquire_max_translation:=0.80",
    "-p", "pose_jump_reacquire_max_yaw:=0.30",
    "-p", "pose_jump_reacquire_max_fitness:=0.08",
    "-p", "use_fastlio_delta_guess:=True",
    "-p", "fastlio_camera_frame:=camera_init",
    "-p", "fastlio_body_frame:=body",
    "-p", "fastlio_max_delta_translation:=0.20",
    "-p", "fastlio_max_delta_yaw:=0.25",
    "-p", "fastlio_max_dead_reckon_sec:=2.0",
    "-p", "republish_last_good_tf_on_failure:=True",
    "-p", "max_last_good_tf_age_sec:=5.0",
    "-p", "ndt_outlier_ratio:=0.30",
    "-p", "ndt_max_corr_dist:=2.0",
    "-p", "ndt_rotation_prior_enabled:=True",
    "-p", "ndt_rotation_prior_weight:=10.0",
    "-p", "use_odom:=false",
    "-p", "enable_debug:=true",
    "--log-level", "warn",
]

print(f"Output: {OUTPUT}")

# Kill everything on exit
procs = []

def cleanup():
    for p in procs:
        try:
            p.terminate()
            p.wait(timeout=3)
        except:
            p.kill()

signal.signal(signal.SIGINT, lambda s, f: (cleanup(), sys.exit(0)))
signal.signal(signal.SIGTERM, lambda s, f: (cleanup(), sys.exit(0)))

try:
    # Step 1: Start NDT node
    print("[1/4] Starting NDT node...")
    ndt_proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    procs.append(ndt_proc)
    time.sleep(4)

    # Step 2: Lifecycle transitions
    print("[2/4] Activating lifecycle...")
    subprocess.run(["ros2", "lifecycle", "set", "/lidar_localization", "configure"],
                   capture_output=True, timeout=10)
    time.sleep(2)
    subprocess.run(["ros2", "lifecycle", "set", "/lidar_localization", "activate"],
                   capture_output=True, timeout=10)
    time.sleep(2)

    # Step 3: Start recording ndt_status (json format via --field data)
    print("[3/4] Starting recorder...")
    with open(OUTPUT, 'w') as f:
        rec_proc = subprocess.Popen(
            ["ros2", "topic", "echo", "/localization/ndt_status", "--field", "data"],
            stdout=f, stderr=subprocess.DEVNULL)
        procs.append(rec_proc)
        time.sleep(1)

        # Step 4: Play bag
        print("[4/4] Playing bag...")
        bag_proc = subprocess.Popen(
            ["ros2", "bag", "play", BAG, "--clock", "100",
             "--topics", "/fast_lio/cloud_registered", "/tf", "/tf_static", "/initialpose"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        procs.append(bag_proc)
        bag_proc.wait(timeout=600)  # 10 min timeout for 8 min bag

        # Wait for a few more seconds for late messages
        time.sleep(5)

except Exception as e:
    print(f"ERROR: {e}")
finally:
    cleanup()

# Report
if os.path.exists(OUTPUT):
    with open(OUTPUT) as f:
        lines = [l for l in f if l.strip()]
    print(f"\nDone! {len(lines)} ndt_status messages written to {OUTPUT}")
    # Show first 3 lines as sample
    for l in lines[:3]:
        try:
            d = json.loads(l)
            print(f"  state={d.get('state')} fitness={d.get('fitness_score'):.4f} "
                  f"corr={d.get('correction_translation'):.4f} "
                  f"fastlio_applied={d.get('fastlio_delta_applied')} "
                  f"fastlio_reason={d.get('fastlio_delta_reject_reason', '')}")
        except:
            print(f"  {l[:120]}")
else:
    print("ERROR: No output file generated")
