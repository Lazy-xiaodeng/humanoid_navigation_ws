#!/usr/bin/env python3
"""NDT bag replay test with proper lifecycle management via Python API."""
import subprocess
import signal
import sys
import time
import json
import os
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State
from std_msgs.msg import String

BAG = "/home/ubuntu/nav_drift_test4/nav_drift_test4_0.mcap"
OUTPUT = f"/tmp/ndt_replay_planb_{time.strftime('%Y%m%d_%H%M%S')}.jsonl"

# Launch NDT node as subprocess
ndt_proc = None
recorder = None
recorded = []

def cleanup():
    global ndt_proc
    if ndt_proc:
        ndt_proc.terminate()
        try:
            ndt_proc.wait(timeout=5)
        except:
            ndt_proc.kill()

signal.signal(signal.SIGINT, lambda s, f: (cleanup(), sys.exit(0)))
signal.signal(signal.SIGTERM, lambda s, f: (cleanup(), sys.exit(0)))

class Recorder(Node):
    def __init__(self):
        super().__init__('ndt_recorder')
        self.sub = self.create_subscription(
            String, '/localization/ndt_status', self.cb, 1000)
    def cb(self, msg):
        recorded.append(msg.data)

def call_service(node, srv_name, srv_type, request, timeout=30.0):
    """Call a ROS2 service with timeout."""
    client = node.create_client(srv_type, srv_name)
    if not client.wait_for_service(timeout_sec=5.0):
        print(f"  WARNING: Service {srv_name} not available")
        return None
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    node.destroy_client(client)
    if future.done() and future.result() is not None:
        return future.result()
    return None

def transition_to(node, target_state_id, timeout=60.0):
    """Transition lifecycle node to target state, with polling."""
    state_names = {1: 'UNCONFIGURED', 2: 'INACTIVE', 3: 'ACTIVE',
                   4: 'FINALIZED', 5: 'UNCONFIGURED', 6: 'INACTIVE'}

    # First get current state
    get_req = GetState.Request()
    get_result = call_service(node, '/lidar_localization/get_state', GetState, get_req)
    if get_result is None:
        print(f"  Cannot get current state")
        return False
    current_id = get_result.current_state.id
    print(f"  Current: {state_names.get(current_id, current_id)} → target: {state_names.get(target_state_id, target_state_id)}")

    if current_id == target_state_id:
        print(f"  Already at target state")
        return True

    # Request transition
    req = ChangeState.Request()
    req.transition = Transition()
    req.transition.id = target_state_id - current_id + 1  # approximate
    # Actually, just try the standard transitions
    if target_state_id == State.PRIMARY_STATE_INACTIVE and current_id == State.PRIMARY_STATE_UNCONFIGURED:
        req.transition.id = Transition.TRANSITION_CONFIGURE
    elif target_state_id == State.PRIMARY_STATE_ACTIVE and current_id == State.PRIMARY_STATE_INACTIVE:
        req.transition.id = Transition.TRANSITION_ACTIVATE

    result = call_service(node, '/lidar_localization/change_state', ChangeState, req, timeout=timeout)
    if result is not None and result.success:
        # Wait for state to stabilize
        for _ in range(int(timeout)):
            time.sleep(1)
            get_req2 = GetState.Request()
            r = call_service(node, '/lidar_localization/get_state', GetState, get_req2)
            if r is not None and r.current_state.id == target_state_id:
                print(f"  Success: reached {state_names.get(target_state_id, target_state_id)}")
                return True
        print(f"  Timeout waiting for target state")
        return False
    else:
        print(f"  Transition failed: {result}")
        return False


def main():
    global ndt_proc

    rclpy.init()
    node = Node('test_controller')

    # Step 1: Start NDT node
    print("[1/4] Starting NDT node...")
    ndt_proc = subprocess.Popen([
        'ros2', 'run', 'lidar_localization_ros2', 'lidar_localization_node',
        '--ros-args',
        '--params-file', '/home/ubuntu/humanoid_ws/src/lidar_localization/param/localization.yaml',
        '-p', 'use_sim_time:=true',
        '-p', 'set_initial_pose:=True',
        '-p', 'initial_pose_x:=0.0',
        '-p', 'initial_pose_y:=0.0',
        '-p', 'initial_pose_z:=0.0',
        '-p', 'score_threshold:=0.3',
        '-p', 'max_pose_jump_translation:=0.50',
        '-p', 'max_pose_jump_yaw:=0.40',
        '-p', 'initialpose_relax_duration_sec:=4.0',
        '-p', 'initialpose_max_pose_jump_translation:=2.00',
        '-p', 'initialpose_max_pose_jump_yaw:=3.00',
        '-p', 'pose_jump_reacquire_enabled:=True',
        '-p', 'pose_jump_reacquire_max_translation:=0.50',
        '-p', 'pose_jump_reacquire_max_fitness:=0.08',
        '-p', 'use_fastlio_delta_guess:=True',
        '-p', 'fastlio_max_delta_translation:=0.20',
        '-p', 'fastlio_max_delta_yaw:=0.25',
        '-p', 'fastlio_max_dead_reckon_sec:=2.0',
        '-p', 'republish_last_good_tf_on_failure:=True',
        '-p', 'max_last_good_tf_age_sec:=5.0',
        '-p', 'ndt_outlier_ratio:=0.30',
        '-p', 'ndt_max_corr_dist:=2.0',
        '-p', 'ndt_rotation_prior_enabled:=True',
        '-p', 'ndt_rotation_prior_weight:=10.0',
        '-p', 'use_odom:=false',
        '-p', 'enable_debug:=true',
        '--log-level', 'warn',
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(5)

    # Step 2: Lifecycle transitions
    print("[2/4] Lifecycle transitions...")
    ok = transition_to(node, State.PRIMARY_STATE_INACTIVE, timeout=30)
    if ok:
        ok = transition_to(node, State.PRIMARY_STATE_ACTIVE, timeout=60)  # long timeout for PCD load
    if not ok:
        print("ERROR: Failed to activate NDT node")
        cleanup()
        return 1

    # Step 3: Start recorder
    print("[3/4] Starting recorder...")
    rec = Recorder()
    executor = SingleThreadedExecutor()
    executor.add_node(rec)
    rec_thread = threading.Thread(target=lambda: executor.spin(), daemon=True)
    rec_thread.start()
    time.sleep(1)

    # Step 4: Play bag
    print(f"[4/4] Playing bag (~8 min)...")
    result = subprocess.run([
        'ros2', 'bag', 'play', BAG, '--clock', '100',
        '--topics', '/fast_lio/cloud_registered', '/tf', '/tf_static', '/initialpose',
    ], capture_output=True, text=True, timeout=600)
    print(result.stdout[-500:] if len(result.stdout) > 500 else result.stdout)

    time.sleep(10)
    executor.shutdown()
    rec_thread.join(timeout=5)

    # ── Results ──
    print(f"\n{'='*60}")
    print(f"  RESULTS")
    print(f"{'='*60}")
    print(f"  Recorded: {len(recorded)} ndt_status messages")

    if recorded:
        with open(OUTPUT, 'w') as f:
            for line in recorded:
                f.write(line + '\n')

        lines = [json.loads(l) for l in recorded if l.strip()]
        acc = [l for l in lines if l.get('state') == 'accepted']
        rej = [l for l in lines if l.get('state') == 'rejected']
        total = len(lines)

        print(f"  Total: {total}  Accepted: {len(acc)} ({len(acc)/total*100:.1f}%)  "
              f"Rejected: {len(rej)} ({len(rej)/total*100:.1f}%)")

        if acc:
            corrs = sorted([l['correction_translation'] for l in acc if 'correction_translation' in l])
            fits = sorted([l['fitness_score'] for l in acc if 'fitness_score' in l and l['fitness_score'] >= 0])
            da = sum(1 for l in lines if l.get('fastlio_delta_applied'))
            n = len(corrs)
            print(f"  Correction (m): p50={corrs[n//2]:.4f} p90={corrs[int(n*0.9)]:.4f} "
                  f"p99={corrs[int(n*0.99)]:.4f}")
            if fits:
                fn = len(fits)
                print(f"  Fitness: p50={fits[fn//2]:.4f} p90={fits[int(fn*0.9)]:.4f}")
            print(f"  Delta applied: {da} ({da/total*100:.1f}%)")

            for reason in ['dead_reckon_timeout', 'tf_lookup_failed', 'max_delta_exceeded',
                           'tf_stamp_mismatch', 'tf_clock_mismatch', 'no_prev_pose']:
                cnt = sum(1 for l in lines if l.get('fastlio_delta_reject_reason') == reason)
                if cnt > 0:
                    print(f"    {reason}: {cnt}")

            # Compare with old: accept rate timeline
            window = 100
            print(f"\n  Accept rate timeline (window={window}):")
            for i in range(0, len(lines), window):
                chunk = lines[i:i+window]
                a = sum(1 for l in chunk if l.get('state') == 'accepted')
                rate = a/len(chunk)*100
                t_rel = chunk[0].get('stamp_sec',0) - lines[0].get('stamp_sec',0) if lines else 0
                bar = '#' * int(rate/100*30)
                if rate < 90 or i % (window*5) == 0:
                    print(f"    t={t_rel:6.0f}s  {a:3d}/{len(chunk)} ({rate:5.1f}%)  {bar}")

        print(f"\n  Output: {OUTPUT}")

    cleanup()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
