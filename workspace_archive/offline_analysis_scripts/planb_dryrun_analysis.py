#!/usr/bin/env python3
"""
Plan B dry-run analysis: from bag replay data, simulate delta guess frame by frame.

用法: python3 planb_dryrun_analysis.py [bag_path]
"""

import sys
import math
import json
from collections import defaultdict
from dataclasses import dataclass
from typing import Optional, List, Dict, Tuple

# Need ROS2 for deserialization
import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

BAG_PATH = "/home/ubuntu/nav_drift_test4/nav_drift_test4_0.mcap"

# Plan B parameters (matching launch file)
MAX_DELTA_TRANSLATION = 0.20
MAX_DELTA_YAW = 0.25
MAX_DEAD_RECKON_SEC = 2.0


def quat_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2)


def quat_inv(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def quat_to_yaw(q):
    w, x, y, z = q
    return math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))


R = (0.5, -0.5, -0.5, 0.5)   # R_cam_to_ros
R_inv = quat_inv(R)


def rot_to_ros(q_cam):
    return quat_mult(quat_mult(R, q_cam), R_inv)


def load_bag(path):
    """Load cloud, TF, initialpose, ndt_status from bag."""
    storage_options = StorageOptions(uri=path, storage_id='mcap')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    tf_entries: Dict[Tuple[str, str], List[dict]] = defaultdict(list)
    cloud_stamps: List[float] = []
    initialpose_stamps: List[float] = []
    ndt_statuses: List[dict] = []
    ndt_accept_stamps: List[float] = []
    pcl_poses: List[dict] = []  # pcl_pose snapshots

    count = 0
    last_report = 0
    while reader.has_next():
        (topic, msg_bytes, stamp_ns) = reader.read_next()
        stamp_sec = stamp_ns * 1e-9
        count += 1

        if count - last_report >= 500000:
            print(f"  ... {count/1e6:.1f}M messages, {len(cloud_stamps)} clouds, "
                  f"{len(ndt_statuses)} ndt_status")
            last_report = count

        if topic == '/tf':
            msg = deserialize_message(msg_bytes, TFMessage)
            for t in msg.transforms:
                ts = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
                tf_entries[(t.header.frame_id, t.child_frame_id)].append({
                    'stamp': ts,
                    'tx': t.transform.translation.x,
                    'ty': t.transform.translation.y,
                    'tz': t.transform.translation.z,
                    'qx': t.transform.rotation.x,
                    'qy': t.transform.rotation.y,
                    'qz': t.transform.rotation.z,
                    'qw': t.transform.rotation.w,
                })

        elif topic == '/fast_lio/cloud_registered':
            msg = deserialize_message(msg_bytes, PointCloud2)
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            cloud_stamps.append(ts)

        elif topic == '/initialpose':
            msg = deserialize_message(msg_bytes, PoseWithCovarianceStamped)
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            initialpose_stamps.append(ts)

        elif topic == '/localization/ndt_status':
            msg = deserialize_message(msg_bytes, String)
            try:
                d = json.loads(msg.data)
                ndt_statuses.append(d)
                if d.get('state') == 'accepted' and 'stamp_sec' in d:
                    ndt_accept_stamps.append(d['stamp_sec'])
            except json.JSONDecodeError:
                pass

        elif topic == '/pcl_pose':
            msg = deserialize_message(msg_bytes, PoseWithCovarianceStamped)
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            pcl_poses.append({
                'stamp': ts,
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
            })

    cloud_stamps.sort()
    initialpose_stamps.sort()
    ndt_accept_stamps.sort()

    print(f"Loaded: {len(cloud_stamps)} clouds, {len(initialpose_stamps)} initialposes, "
          f"{len(ndt_statuses)} ndt_status ({len(ndt_accept_stamps)} accepted), "
          f"{len(pcl_poses)} pcl_poses, "
          f"{sum(len(v) for v in tf_entries.values())} TFs")

    return tf_entries, cloud_stamps, initialpose_stamps, ndt_statuses, ndt_accept_stamps, pcl_poses


def lookup_tf(tf_entries, parent, child, stamp):
    """Find closest TF entry at or before stamp."""
    key = (parent, child)
    if key not in tf_entries:
        return None
    entries = tf_entries[key]
    best = None
    for e in entries:
        if e['stamp'] <= stamp:
            if best is None or e['stamp'] > best['stamp']:
                best = e
    if best is None and entries:
        best = entries[0]
    if best and abs(best['stamp'] - stamp) > 0.2:
        return None
    return best


def run_dryrun(tf_entries, cloud_stamps, initialpose_stamps, ndt_accept_stamps):
    """Simulate Plan B delta guess using recorded NDT accept times for dead reckoning."""
    has_prev_body = False
    prev_body = None
    last_accept_time = cloud_stamps[0] if cloud_stamps else 0.0

    total = len(cloud_stamps)
    stats = defaultdict(int)
    deltas = []

    ip_idx = 0
    accept_idx = 0
    for i, cloud_stamp in enumerate(cloud_stamps):
        # Check initialpose reset
        while ip_idx < len(initialpose_stamps) and initialpose_stamps[ip_idx] <= cloud_stamp:
            has_prev_body = False
            ip_idx += 1

        # Update last_accept_time from recorded NDT accepts (simulating real NDT behavior)
        while accept_idx < len(ndt_accept_stamps) and ndt_accept_stamps[accept_idx] <= cloud_stamp:
            last_accept_time = max(last_accept_time, ndt_accept_stamps[accept_idx])
            accept_idx += 1

        tf = lookup_tf(tf_entries, "camera_init", "body", cloud_stamp)
        if tf is None:
            stats['tf_lookup_failed'] += 1
            continue

        if not has_prev_body:
            prev_body = tf
            has_prev_body = True
            stats['no_prev_pose'] += 1
            continue

        dead_reckon_age = cloud_stamp - last_accept_time
        if dead_reckon_age > MAX_DEAD_RECKON_SEC:
            stats['dead_reckon_timeout'] += 1
            continue

        # Delta in camera_init frame
        dx_cam = tf['tx'] - prev_body['tx']
        dy_cam = tf['ty'] - prev_body['ty']
        dz_cam = tf['tz'] - prev_body['tz']

        # Rotate to ROS: R_cam_to_ros = [[0,0,-1],[1,0,0],[0,-1,0]]
        dx_ros = -dz_cam
        dy_ros = dx_cam

        # Yaw delta in ROS frame
        q_prev_cam = (prev_body['qw'], prev_body['qx'], prev_body['qy'], prev_body['qz'])
        q_curr_cam = (tf['qw'], tf['qx'], tf['qy'], tf['qz'])
        q_prev_ros = rot_to_ros(q_prev_cam)
        q_curr_ros = rot_to_ros(q_curr_cam)
        delta_q = quat_mult(quat_inv(q_prev_ros), q_curr_ros)
        dyaw = quat_to_yaw(delta_q)

        # Anomaly check
        if abs(dx_ros) > MAX_DELTA_TRANSLATION or \
           abs(dy_ros) > MAX_DELTA_TRANSLATION or \
           abs(dyaw) > MAX_DELTA_YAW:
            stats['max_delta_exceeded'] += 1
            has_prev_body = False
            continue

        stats['delta_applied'] += 1
        trans = math.hypot(dx_ros, dy_ros)
        deltas.append({
            'stamp': cloud_stamp,
            'dx': dx_ros, 'dy': dy_ros, 'dyaw': dyaw,
            'trans': trans,
            'dead_reckon_age': dead_reckon_age,
        })

        # Store for next frame
        prev_body = tf

    return {
        'total': total,
        'stats': dict(stats),
        'coverage_pct': stats['delta_applied'] / total * 100 if total else 0,
        'deltas': deltas,
    }


def percentile(vals, p):
    if not vals:
        return 0
    vals = sorted(vals)
    idx = int(len(vals) * p / 100.0)
    return vals[min(idx, len(vals)-1)]


def main():
    rclpy.init(args=[])

    path = sys.argv[1] if len(sys.argv) > 1 else BAG_PATH
    print(f"=== Plan B Dry-Run Analysis ===")
    print(f"Bag: {path}\n")

    # ── Load ──
    print("[1/2] Loading bag...")
    tf_entries, cloud_stamps, initialpose_stamps, ndt_statuses, ndt_accept_stamps, pcl_poses = load_bag(path)

    # ── Dry-run ──
    print(f"\n[2/2] Simulating delta guess...")
    result = run_dryrun(tf_entries, cloud_stamps, initialpose_stamps, ndt_accept_stamps)

    # ── Report ──
    print(f"\n{'='*65}")
    print(f"  PLAN B DELTA GUESS DRY-RUN RESULTS")
    print(f"{'='*65}")

    total = result['total']
    s = result['stats']
    print(f"  Total cloud frames:        {total:>6d}")
    print(f"  Delta APPLIED:             {s.get('delta_applied',0):>6d}  "
          f"({result['coverage_pct']:.1f}%)")
    print(f"  TF lookup failed:          {s.get('tf_lookup_failed',0):>6d}")
    print(f"  No prev pose (first/reset):{s.get('no_prev_pose',0):>6d}")
    print(f"  Dead reckon timeout:       {s.get('dead_reckon_timeout',0):>6d}")
    print(f"  Max delta exceeded:        {s.get('max_delta_exceeded',0):>6d}")

    # Delta magnitude stats
    deltas = result['deltas']
    if deltas:
        trans = [d['trans'] for d in deltas]
        yaws = [abs(d['dyaw']) for d in deltas]
        dts = [deltas[i+1]['stamp'] - deltas[i]['stamp'] for i in range(len(deltas)-1)]
        ages = [d['dead_reckon_age'] for d in deltas]

        print(f"\n  Delta magnitude (per-frame, ROS frame):")
        print(f"    {'Metric':<20s} {'p50':>8s} {'p90':>8s} {'p99':>8s} {'max':>8s}")
        print(f"    {'-'*20} {'-'*8} {'-'*8} {'-'*8} {'-'*8}")
        for label, vals in [('translation (m)', trans), ('|dyaw| (rad)', yaws),
                             ('dt (s)', dts), ('dead_reckon_age (s)', ages)]:
            if vals:
                print(f"    {label:<20s} {percentile(vals,50):>8.4f} {percentile(vals,90):>8.4f} "
                      f"{percentile(vals,99):>8.4f} {max(vals):>8.4f}")

        # Check: any delta over max_delta that wasn't caught?
        over = sum(1 for d in deltas if d['trans'] > MAX_DELTA_TRANSLATION or
                   abs(d['dyaw']) > MAX_DELTA_YAW)
        if over > 0:
            print(f"\n  WARNING: {over} frames with delta > max_delta not caught")
        else:
            print(f"\n  All applied deltas within limits. OK.")

    # ── Old NDT status comparison ──
    if ndt_statuses:
        print(f"\n{'='*65}")
        print(f"  RECORDED NDT STATUS (old code, from bag)")
        print(f"{'='*65}")

        accepted = [e for e in ndt_statuses if e.get('state') == 'accepted']
        rejected = [e for e in ndt_statuses if e.get('state') == 'rejected']
        confirming = [e for e in ndt_statuses if e.get('state') == 'confirming']

        print(f"  Total: {len(ndt_statuses)}  Accepted: {len(accepted)}  "
              f"Rejected: {len(rejected)}  Confirming: {len(confirming)}")
        accept_rate = len(accepted) / len(ndt_statuses) * 100 if ndt_statuses else 0
        print(f"  Accept rate: {accept_rate:.1f}%")

        # Correction stats by state
        for label, subset in [('ALL', ndt_statuses), ('accepted', accepted),
                               ('rejected', rejected)]:
            corr = [e['correction_translation'] for e in subset
                    if e.get('correction_translation', -1) >= 0]
            fit = [e['fitness_score'] for e in subset
                   if e.get('fitness_score', -1) >= 0]
            if corr:
                print(f"  [{label}] correction: p50={percentile(corr,50):.4f} "
                      f"p90={percentile(corr,90):.4f} p99={percentile(corr,99):.4f} "
                      f"max={max(corr):.4f}  |  fitness: p50={percentile(fit,50):.4f}")

        # Temporal accept rate
        if ndt_statuses:
            window = 50
            rates = []
            for i in range(0, len(ndt_statuses) - window, window):
                w = ndt_statuses[i:i+window]
                a = sum(1 for e in w if e.get('state') == 'accepted')
                rates.append(a / len(w) * 100)
            if rates:
                print(f"  Accept rate range (window={window}): min={min(rates):.1f}% "
                      f"max={max(rates):.1f}% avg={sum(rates)/len(rates):.1f}%")

    # ── Verdict ──
    print(f"\n{'='*65}")
    print(f"  VERDICT")
    print(f"{'='*65}")

    coverage = result['coverage_pct']
    tf_fail_rate = s.get('tf_lookup_failed', 0) / total * 100 if total else 0
    maxd_rate = s.get('max_delta_exceeded', 0) / total * 100 if total else 0

    issues = []
    if coverage < 80:
        issues.append(f"Delta coverage only {coverage:.1f}%")
    if tf_fail_rate > 5:
        issues.append(f"TF failure rate {tf_fail_rate:.1f}%")
    if maxd_rate > 1:
        issues.append(f"Max delta exceeded {maxd_rate:.1f}% of frames")
    if s.get('dead_reckon_timeout', 0) > total * 0.05:
        issues.append(f"Dead reckon timeout {s['dead_reckon_timeout']} frames")

    if not issues:
        print(f"  PASS — Delta guess feasible for {coverage:.1f}% of frames")
        print(f"  TF available, deltas within limits, dead reckoning ok")
        print(f"  Plan B should provide reliable init_guess advancement")
        print(f"  Expected improvement: correction from p50~{percentile([e.get('correction_translation',0) for e in ndt_statuses if e.get('correction_translation',-1)>=0], 50) if ndt_statuses else '?'}m "
              f"to <0.05m, accept rate from {accept_rate:.1f}% to >90%")
    else:
        print(f"  ISSUES FOUND:")
        for issue in issues:
            print(f"    - {issue}")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
