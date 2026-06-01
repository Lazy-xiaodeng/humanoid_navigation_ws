#!/usr/bin/env python3
"""
Bag replay analysis v2: track localization vs waypoints using /odom + map->odom TF.
"""
import sys, json, struct, math, time
from collections import defaultdict
sys.path.insert(0, '/usr/lib/python3/dist-packages')
from mcap.reader import make_reader

BAG_PATH = '/home/ubuntu/nav_drift_test2/nav_drift_test2_0.mcap'
WAYPOINTS_PATH = '/home/ubuntu/humanoid_ws/data/dynamic_waypoints.json'

# ── Load waypoints ──────────────────────────────────────────────
with open(WAYPOINTS_PATH) as f:
    wp_data = json.load(f)
waypoints = []
for k, v in sorted(wp_data['waypoints']['navigation_target'].items(),
                    key=lambda x: int(x[0])):
    p = v['position']
    waypoints.append({'id': v['id'], 'name': v['name'], 'x': p[0], 'y': p[1]})
print(f"Loaded {len(waypoints)} waypoints")

# ── CDR helpers ──────────────────────────────────────────────────
def parse_cdr_string(data, offset):
    strlen = struct.unpack_from('<I', data, offset)[0]
    offset += 4
    if strlen > 10000:
        return None, offset
    s = data[offset:offset+strlen].decode('utf-8', errors='replace').rstrip('\x00')
    offset += strlen
    offset += (4 - (strlen % 4)) % 4
    return s, offset

def skip_rtps(data):
    """Return offset after RTPS encapsulation header."""
    if len(data) < 4:
        return 0
    encap = struct.unpack_from('<H', data, 0)[0]
    return 4 if encap in (0x0001, 0x0100) else 0

def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

# ── Apply static body->base_footprint transform ──────────────────
# From launch config: translation=(0.004, 1.215, 0.072), quat=(0.5, 0.5, -0.5, 0.5)
# This quaternion rotates body (X-forward) to base_footprint (standard ROS)
# For 2D analysis we care about the horizontal offset.
# The body->base_footprint horizontal offset: approx (-1.215, 0.004) in body frame
# But since this is a static offset that affects both map and odom equally,
# we can work in the "body" frame and compare odometry vs localization directly.
# Let's use camera_init->body as the odometry (Fast-LIO output).

# ── Parse odometry from /odom topic ──────────────────────────────
def parse_odom_message(data):
    """Parse nav_msgs/Odometry from CDR, return (ros_t, x, y, z, qx, qy, qz, qw)."""
    if len(data) < 8:
        return None
    try:
        offset = skip_rtps(data)
        # Header: sec, nanosec, frame_id
        sec = struct.unpack_from('<i', data, offset)[0]; offset += 4
        nanosec = struct.unpack_from('<I', data, offset)[0]; offset += 4
        frame_id, offset = parse_cdr_string(data, offset)
        if frame_id is None or len(frame_id) > 60: return None
        # child_frame_id string
        child_id, offset = parse_cdr_string(data, offset)
        if child_id is None or len(child_id) > 60: return None
        # Pose: position (x,y,z double) + orientation (x,y,z,w double) + covariance[36]
        if offset + 56 > len(data): return None
        px = struct.unpack_from('<d', data, offset)[0]; offset += 8
        py = struct.unpack_from('<d', data, offset)[0]; offset += 8
        pz = struct.unpack_from('<d', data, offset)[0]; offset += 8
        ox = struct.unpack_from('<d', data, offset)[0]; offset += 8
        oy = struct.unpack_from('<d', data, offset)[0]; offset += 8
        oz = struct.unpack_from('<d', data, offset)[0]; offset += 8
        ow = struct.unpack_from('<d', data, offset)[0]; offset += 8
        if not all(math.isfinite(v) for v in (px, py, pz, ox, oy, oz, ow)):
            return None
        ts = sec + nanosec * 1e-9
        return (ts, px, py, pz, ox, oy, oz, ow)
    except (struct.error, IndexError):
        return None

def parse_status_message(data):
    """Parse ScanMatchingStatus from CDR."""
    if len(data) < 8:
        return None
    try:
        offset = skip_rtps(data)
        camera_idx = data.find(b'camera_init', offset)
        if camera_idx < 0:
            return None
        strlen_at = camera_idx - 4
        strlen_val = struct.unpack_from('<I', data, strlen_at)[0]
        if strlen_val > 100:
            return None
        after = camera_idx + strlen_val
        after += (4 - (strlen_val % 4)) % 4
        if after + 12 > len(data):
            return None
        hc = struct.unpack_from('<I', data, after)[0] != 0
        me = struct.unpack_from('<f', data, after + 4)[0]
        inf = struct.unpack_from('<f', data, after + 8)[0]
        if not math.isfinite(me) or not math.isfinite(inf):
            return None
        return {'has_converged': hc, 'matching_error': me, 'inlier_fraction': inf}
    except (struct.error, IndexError):
        return None

def parse_string_message(data):
    """Parse std_msgs/String."""
    if len(data) < 4:
        return None
    try:
        offset = skip_rtps(data)
        s, _ = parse_cdr_string(data, offset)
        return s
    except (struct.error, IndexError):
        return None

# ── Stream bag efficiently ───────────────────────────────────────
print("Streaming bag...")
t0 = time.time()

# Data stores
map_odom_samples = []    # [(ros_t, map_odom_x, map_odom_y, map_odom_yaw)]
odom_samples = []        # [(ros_t, odom_x, odom_y, odom_z, odom_yaw)]
# camera_init->body (Fast-LIO) stored as body_odom for odometry reference
body_in_odom = []        # [(ros_t, x, y, z, yaw)]  # camera_init->body
status_samples = []      # [(ros_t, status_dict)]
recovery_requests = []   # [(ros_t, text)]
recovery_status = []     # [(ros_t, text)]
health_events = []       # [(ros_t, "healthy"|"degraded"|"drifted"|"lost")]

with open(BAG_PATH, 'rb') as f:
    reader = make_reader(f)
    msg_count = 0

    for schema, channel, msg in reader.iter_messages(
        topics=['/tf', '/odom', '/status',
                '/localization/recovery_requests', '/localization/recovery_status']):
        ros_t = msg.log_time / 1e9
        msg_count += 1

        if channel.topic == '/tf':
            data = msg.data
            if len(data) < 8:
                continue
            try:
                offset = skip_rtps(data)
                n = struct.unpack_from('<I', data, offset)[0]; offset += 4
                for _ in range(min(n, 20)):
                    if offset + 16 > len(data): break
                    sec = struct.unpack_from('<i', data, offset)[0]; offset += 4
                    nanosec = struct.unpack_from('<I', data, offset)[0]; offset += 4
                    frame_id, offset = parse_cdr_string(data, offset)
                    child_id, offset = parse_cdr_string(data, offset)
                    if frame_id is None or child_id is None: break
                    if len(frame_id) > 60 or len(child_id) > 60: break
                    if offset + 56 > len(data): break
                    tx = struct.unpack_from('<d', data, offset)[0]; offset += 8
                    ty = struct.unpack_from('<d', data, offset)[0]; offset += 8
                    tz = struct.unpack_from('<d', data, offset)[0]; offset += 8
                    qx = struct.unpack_from('<d', data, offset)[0]; offset += 8
                    qy = struct.unpack_from('<d', data, offset)[0]; offset += 8
                    qz = struct.unpack_from('<d', data, offset)[0]; offset += 8
                    qw = struct.unpack_from('<d', data, offset)[0]; offset += 8
                    if not all(math.isfinite(v) for v in (tx, ty, tz, qx, qy, qz, qw)):
                        continue
                    yaw = quat_to_yaw(qx, qy, qz, qw)
                    if (frame_id, child_id) == ('map', 'odom'):
                        map_odom_samples.append((ros_t, tx, ty, tz, yaw))
                    elif (frame_id, child_id) == ('camera_init', 'body'):
                        body_in_odom.append((ros_t, tx, ty, tz, yaw))
            except (struct.error, IndexError):
                continue

        elif channel.topic == '/odom':
            parsed = parse_odom_message(msg.data)
            if parsed:
                ts, px, py, pz, ox, oy, oz, ow = parsed
                yaw = quat_to_yaw(ox, oy, oz, ow)
                odom_samples.append((ts, px, py, pz, yaw))

        elif channel.topic == '/status':
            s = parse_status_message(msg.data)
            if s:
                status_samples.append((ros_t, s))

        elif channel.topic == '/localization/recovery_requests':
            text = parse_string_message(msg.data)
            if text:
                recovery_requests.append((ros_t, text))

        elif channel.topic == '/localization/recovery_status':
            text = parse_string_message(msg.data)
            if text:
                recovery_status.append((ros_t, text))

        if msg_count % 50000 == 0:
            print(f"  ... {msg_count/1000:.0f}k msgs, {time.time()-t0:.0f}s")

elapsed = time.time() - t0
print(f"Done: {msg_count} msgs in {elapsed:.0f}s")
print(f"  map->odom: {len(map_odom_samples)}")
print(f"  /odom msgs: {len(odom_samples)}")
print(f"  camera_init->body: {len(body_in_odom)}")
print(f"  status: {len(status_samples)}")
print(f"  recovery requests: {len(recovery_requests)}")
print(f"  recovery status: {len(recovery_status)}")

# ── Build aligned trajectory ─────────────────────────────────────
# For each timestamp with map->odom, find closest odometry
# map->base = map->odom * odom->base (where odom->base comes from /odom topic)

def find_nearest(samples, target_t, max_dt=0.5):
    """Binary search for nearest sample within max_dt."""
    if not samples:
        return None
    lo, hi = 0, len(samples) - 1
    best_idx, best_dt = None, max_dt
    while lo <= hi:
        mid = (lo + hi) // 2
        dt = abs(samples[mid][0] - target_t)
        if dt < best_dt:
            best_dt = dt
            best_idx = mid
        if samples[mid][0] < target_t:
            lo = mid + 1
        else:
            hi = mid - 1
    return samples[best_idx] if best_idx is not None else None

print("\nBuilding aligned trajectory...")
trajectory = []  # [(ros_t, map_base_x, map_base_y, odom_base_x, odom_base_y, map_odom_x, map_odom_y)]

for mo_t, mo_x, mo_y, mo_z, mo_yaw in map_odom_samples:
    odom = find_nearest(odom_samples, mo_t)
    if odom is None:
        continue
    _, ob_x, ob_y, ob_z, ob_yaw = odom

    # Compute map->base: map_T_base = map_T_odom * odom_T_base
    # map_T_odom = (mo_x, mo_y, mo_yaw)
    # odom_T_base = (ob_x, ob_y, ob_yaw)
    # Simplified 2D: map_base_x = mo_x + ob_x*cos(mo_yaw) - ob_y*sin(mo_yaw)
    #              map_base_y = mo_y + ob_x*sin(mo_yaw) + ob_y*cos(mo_yaw)
    cos_mo = math.cos(mo_yaw)
    sin_mo = math.sin(mo_yaw)
    mb_x = mo_x + ob_x * cos_mo - ob_y * sin_mo
    mb_y = mo_y + ob_x * sin_mo + ob_y * cos_mo

    trajectory.append((mo_t, mb_x, mb_y, ob_x, ob_y, mo_x, mo_y))

print(f"Aligned trajectory: {len(trajectory)} points")

# ── Per-waypoint analysis ────────────────────────────────────────
print("\n" + "="*85)
print("WAYPOINT-BY-WAYPOINT LOCALIZATION ANALYSIS")
print("="*85)

waypoint_results = []
prev_near_wp = None

for wp_idx, wp in enumerate(waypoints):
    wp_x, wp_y = wp['x'], wp['y']

    # Find times when robot is near this waypoint
    min_dist = float('inf')
    min_dist_t = 0
    min_dist_mb = (0, 0)
    near_samples = []  # (ros_t, mb_x, mb_y)

    for ros_t, mb_x, mb_y, ob_x, ob_y, mo_x, mo_y in trajectory:
        d = math.hypot(mb_x - wp_x, mb_y - wp_y)
        if d < min_dist:
            min_dist = d
            min_dist_t = ros_t
            min_dist_mb = (mb_x, mb_y)
        if d < 5.0:
            near_samples.append((ros_t, mb_x, mb_y, ob_x, ob_y, mo_x, mo_y))

    # Time window around closest approach
    t_window_start = min_dist_t - 30
    t_window_end = min_dist_t + 30

    # NDT status in window
    ndt_errors = []
    ndt_inliers = []
    for ros_t, s in status_samples:
        if t_window_start <= ros_t <= t_window_end:
            ndt_errors.append(s['matching_error'])
            ndt_inliers.append(s['inlier_fraction'])

    # Recovery events in window (wider window: 60s)
    recov_events = []
    for ros_t, text in recovery_requests:
        if abs(ros_t - min_dist_t) < 60:
            recov_events.append((ros_t, text))

    recov_status_events = []
    for ros_t, text in recovery_status:
        if abs(ros_t - min_dist_t) < 60:
            recov_status_events.append((ros_t, text))

    avg_ndt = sum(ndt_errors) / len(ndt_errors) if ndt_errors else float('nan')
    max_ndt = max(ndt_errors) if ndt_errors else float('nan')
    min_ndt = min(ndt_errors) if ndt_errors else float('nan')

    # Determine drift status
    if not ndt_errors or max_ndt <= 0.3:
        health = "OK"
    elif max_ndt <= 0.5:
        health = "DEGRADED"
    elif max_ndt <= 2.0:
        health = "DRIFT"
    else:
        health = "SEVERE"

    # Check if robot actually visited this waypoint
    visited = min_dist < 3.0

    # Check odometry: compare odom path vs map path near waypoint
    odom_displacement = 0.0
    map_displacement = 0.0
    for i in range(1, len(near_samples)):
        _, mb_x1, mb_y1, ob_x1, ob_y1, _, _ = near_samples[i-1]
        _, mb_x2, mb_y2, ob_x2, ob_y2, _, _ = near_samples[i]
        map_displacement += math.hypot(mb_x2 - mb_x1, mb_y2 - mb_y1)
        odom_displacement += math.hypot(ob_x2 - ob_x1, ob_y2 - ob_y1)

    waypoint_results.append({
        'wp': wp,
        'min_dist': min_dist,
        'min_dist_t': min_dist_t,
        'min_dist_mb': min_dist_mb,
        'avg_ndt': avg_ndt,
        'max_ndt': max_ndt,
        'min_ndt': min_ndt,
        'num_ndt': len(ndt_errors),
        'health': health,
        'visited': visited,
        'recov_count': len(recov_events),
        'recov_events': recov_events,
        'recov_status': recov_status_events,
        'near_count': len(near_samples),
        'odom_disp': odom_displacement,
        'map_disp': map_displacement,
        'odom_map_ratio': odom_displacement / map_displacement if map_displacement > 0.01 else float('nan'),
    })

    health_icon = {'OK': '✓', 'DEGRADED': '⚠', 'DRIFT': '✗', 'SEVERE': '☠'}[health]
    print(f"  {wp['name']:<6} ({wp['id']}): "
          f"dist={min_dist:.2f}m @ map=({min_dist_mb[0]:.1f},{min_dist_mb[1]:.1f}) | "
          f"NDT=[{min_ndt:.2f}~{max_ndt:.2f} avg={avg_ndt:.2f}] n={len(ndt_errors)} | "
          f"{health_icon} {health}"
          f"{' | '+str(len(recov_events))+' recoveries' if recov_events else ''}"
          f"{' | NOT VISITED' if not visited else ''}")

# ── Find where drift starts ──────────────────────────────────────
print("\n" + "="*85)
print("DRIFT ONSET DETECTION")
print("="*85)

# Find the first waypoint where NDT degrades
first_drift_wp = None
for r in waypoint_results:
    if r['health'] in ('DRIFT', 'SEVERE'):
        first_drift_wp = r
        break

if first_drift_wp:
    wp = first_drift_wp['wp']
    print(f"Drift starts at: {wp['name']} ({wp['id']})")
    print(f"  Waypoint map position: ({wp['x']:.2f}, {wp['y']:.2f})")
    print(f"  Robot was at map: ({first_drift_wp['min_dist_mb'][0]:.2f}, {first_drift_wp['min_dist_mb'][1]:.2f})")
    print(f"  NDT error: {first_drift_wp['min_ndt']:.4f} ~ {first_drift_wp['max_ndt']:.4f}")
    print(f"  Distance to waypoint: {first_drift_wp['min_dist']:.2f}m")
else:
    print("No drift detected in this bag!")

# Show full drift timeline
print("\nNDT error over time (status samples):")
prev_sev = None
for ros_t, s in status_samples:
    me = s['matching_error']
    sev = 'OK' if me <= 0.3 else ('DEG' if me <= 0.5 else ('DRIFT' if me <= 2.0 else 'SEV'))
    if sev != prev_sev:
        print(f"  t={ros_t:.1f}: NDT error={me:.4f} -> {sev}")
        prev_sev = sev

# ── Recovery analysis ────────────────────────────────────────────
print("\n" + "="*85)
print("RECOVERY EFFECTIVENESS")
print("="*85)

if recovery_requests:
    for ros_t, text in recovery_requests:
        # Find NDT status before and after
        before = []
        after = []
        for st, s in status_samples:
            if st < ros_t and ros_t - st < 10:
                before.append(s['matching_error'])
            elif st > ros_t and st - ros_t < 10:
                after.append(s['matching_error'])
        before_avg = sum(before)/len(before) if before else float('nan')
        after_avg = sum(after)/len(after) if after else float('nan')
        improved = after_avg < before_avg if before and after else None
        icon = '✓' if improved else ('✗' if improved is False else '?')
        print(f"  {icon} t={ros_t:.1f}: {text[:80]}")
        if before and after:
            print(f"     NDT: {before_avg:.4f} -> {after_avg:.4f}")

if recovery_status:
    print("\nRecovery status messages:")
    for ros_t, text in recovery_status[:20]:
        print(f"  t={ros_t:.1f}: {text[:100]}")

# ── Odometry reliability analysis ────────────────────────────────
print("\n" + "="*85)
print("ODOMETRY RELIABILITY ANALYSIS")
print("="*85)

# Compare odom total displacement vs map total displacement
total_odom_disp = 0
total_map_disp = 0
for i in range(1, len(trajectory)):
    _, mb_x1, mb_y1, ob_x1, ob_y1, _, _ = trajectory[i-1]
    _, mb_x2, mb_y2, ob_x2, ob_y2, _, _ = trajectory[i]
    total_odom_disp += math.hypot(ob_x2 - ob_x1, ob_y2 - ob_y1)
    total_map_disp += math.hypot(mb_x2 - mb_x1, mb_y2 - mb_y1)

print(f"Total map path length:  {total_map_disp:.1f}m")
print(f"Total odom path length: {total_odom_disp:.1f}m")

# Check odom drift rate: during healthy periods, how much does odom accumulate error?
healthy_traj = [(t, mbx, mby, obx, oby)
                for t, mbx, mby, obx, oby, _, _ in trajectory
                if not any(abs(st - t) < 0.5 and s['matching_error'] > 0.3
                          for st, s in status_samples)]

if len(healthy_traj) > 10:
    # Start and end of healthy trajectory
    t0_h, mbx0, mby0, obx0, oby0 = healthy_traj[0]
    t1_h, mbx1, mby1, obx1, oby1 = healthy_traj[-1]
    dt_healthy = t1_h - t0_h

    odom_start_end_disp = math.hypot(obx1 - obx0, oby1 - oby0)
    map_start_end_disp = math.hypot(mbx1 - mbx0, mby1 - mby0)
    odom_drift = abs(odom_start_end_disp - map_start_end_disp)

    print(f"\nHealthy period: {dt_healthy:.0f}s")
    print(f"  Odom displacement: {odom_start_end_disp:.2f}m")
    print(f"  Map displacement:  {map_start_end_disp:.2f}m")
    print(f"  Odom drift: {odom_drift:.2f}m ({odom_drift/dt_healthy*100:.2f} cm/s)")

    # Check during drift: does odom stay near waypoints while map drifts?
    drift_traj = [(t, mbx, mby, obx, oby)
                  for t, mbx, mby, obx, oby, _, _ in trajectory
                  if any(abs(st - t) < 0.5 and s['matching_error'] > 0.5
                        for st, s in status_samples)]

    if drift_traj:
        print(f"\nDuring drift ({len(drift_traj)} points):")
        # Check if odom stays within reasonable bounds of nearby waypoints
        drifted_odom_ok = 0
        drifted_odom_far = 0
        for t, mbx, mby, obx, oby in drift_traj:
            # Find nearest waypoint
            nearest_wp_dist = min(math.hypot(obx - w['x'], oby - w['y']) for w in waypoints)
            if nearest_wp_dist < 5.0:
                drifted_odom_ok += 1
            else:
                drifted_odom_far += 1

        odom_near_wp_pct = drifted_odom_ok / len(drift_traj) * 100 if drift_traj else 0
        print(f"  Odom near waypoints (<5m): {odom_near_wp_pct:.0f}%")

# Compare odom vs map path lengths per waypoint visit
print("\nOdom vs Map displacement per waypoint visit:")
for r in waypoint_results:
    if r['visited'] and r['near_count'] > 5:
        wp = r['wp']
        ratio = r['odom_map_ratio']
        status = '✓ match' if 0.8 < ratio < 1.2 else ('⚠ ' + ('>' if ratio > 1.2 else '<') + ' map')
        print(f"  {wp['name']} ({wp['id']}): odom={r['odom_disp']:.2f}m map={r['map_disp']:.2f}m ratio={ratio:.2f} {status}")

# ── Odometry feasibility for correction ──────────────────────────
print("\n" + "="*85)
print("ODOMETRY CORRECTION FEASIBILITY ASSESSMENT")
print("="*85)

if len(healthy_traj) > 100:
    odom_drift_rate = odom_drift / dt_healthy if dt_healthy > 0 else 0

    # Also check: during drift periods, what happens to map->odom?
    print(f"Odom drift rate: {odom_drift_rate*100:.2f} cm/s")

    # Compute map->odom stability during healthy periods
    mo_x_vals = [mo_x for _, _, _, _, _, mo_x, _ in trajectory
                 if not any(abs(st - t) < 0.5 and s['matching_error'] > 0.3
                          for st, s in status_samples)]
    if len(mo_x_vals) > 10:
        mo_x_std = (sum((x - sum(mo_x_vals)/len(mo_x_vals))**2 for x in mo_x_vals) / len(mo_x_vals))**0.5

    # During drift, how much does map->odom jump?
    drift_mo = [(t, mo_x, mo_y) for t, _, _, _, _, mo_x, mo_y in trajectory
                if any(abs(st - t) < 0.5 and s['matching_error'] > 0.5
                      for st, s in status_samples)]
    if drift_mo:
        mo_jumps = []
        for i in range(1, min(100, len(drift_mo))):
            d = math.hypot(drift_mo[i][1] - drift_mo[i-1][1], drift_mo[i][2] - drift_mo[i-1][2])
            mo_jumps.append(d)
        max_mo_jump = max(mo_jumps) if mo_jumps else 0
        avg_mo_jump = sum(mo_jumps)/len(mo_jumps) if mo_jumps else 0
        print(f"During drift: max map->odom jump={max_mo_jump:.2f}m, avg={avg_mo_jump:.2f}m")

    # Final assessment
    print("\nAssessment:")
    if odom_drift_rate < 0.0005:  # < 0.05 cm/s
        print("  ODOM DRIFT: VERY LOW - highly reliable for localization correction")
    elif odom_drift_rate < 0.002:  # < 0.2 cm/s
        print("  ODOM DRIFT: LOW - can be used for short-term correction")
    elif odom_drift_rate < 0.01:  # < 1 cm/s
        print("  ODOM DRIFT: MODERATE - usable with frequent map->odom resets")
    else:
        print("  ODOM DRIFT: HIGH - not reliable for correction without external reference")

    odom_path_error_pct = abs(total_odom_disp - total_map_disp) / total_map_disp * 100 if total_map_disp > 0 else 0
    print(f"  Odom path error: {odom_path_error_pct:.1f}%")
    if odom_path_error_pct < 2:
        print("  ✓ ODOM IS SUFFICIENTLY ACCURATE to correct localization during NDT drift!")
        print("    Strategy: when NDT matching_error > 0.5, use odom-only propagation")
        print("    with periodic map->odom reset when NDT recovers.")
    elif odom_path_error_pct < 5:
        print("  ~ ODOM IS MARGINALLY ACCURATE: can provide soft correction")
    else:
        print("  ✗ ODOM DRIFTS TOO MUCH: not suitable as primary fallback")

# ── Final data table ─────────────────────────────────────────────
print("\n" + "="*85)
print("DETAILED DATA TABLE")
print("="*85)
header = (f"{'WP':<6} {'ID':<5} {'Waypoint(map)':<22} {'Robot(map)':<22} "
          f"{'Dist':<7} {'NDT_avg':<9} {'NDT_max':<9} {'Health':<10} "
          f"{'Recov':<6} {'Visited':<8} {'O/M_ratio':<10}")
print(header)
print("-"*107)

for r in waypoint_results:
    wp = r['wp']
    wp_pos = f"({wp['x']:.1f},{wp['y']:.1f})"
    rb_pos = f"({r['min_dist_mb'][0]:.1f},{r['min_dist_mb'][1]:.1f})"
    dist_str = f"{r['min_dist']:.2f}m" if r['min_dist'] < 100 else "N/A"
    avg_str = f"{r['avg_ndt']:.4f}" if math.isfinite(r['avg_ndt']) else "N/A"
    max_str = f"{r['max_ndt']:.4f}" if math.isfinite(r['max_ndt']) else "N/A"
    ratio_str = f"{r['odom_map_ratio']:.2f}" if math.isfinite(r['odom_map_ratio']) else "N/A"
    print(f"{wp['name']:<6} {wp['id']:<5} {wp_pos:<22} {rb_pos:<22} "
          f"{dist_str:<7} {avg_str:<9} {max_str:<9} {r['health']:<10} "
          f"{r['recov_count']:<6} {'YES' if r['visited'] else 'NO':<8} {ratio_str:<10}")

# ── Key conclusions ──────────────────────────────────────────────
print("\n" + "="*85)
print("KEY CONCLUSIONS")
print("="*85)

drifted_wps = [r for r in waypoint_results if r['health'] in ('DRIFT', 'SEVERE')]
degraded_wps = [r for r in waypoint_results if r['health'] == 'DEGRADED']

print(f"1. Total waypoints: {len(waypoints)}")
print(f"2. Waypoints with drift (NDT > 0.5): {len(drifted_wps)}")
for r in drifted_wps:
    print(f"   - {r['wp']['name']} ({r['wp']['id']}) at map({r['wp']['x']:.1f},{r['wp']['y']:.1f}): "
          f"NDT [{r['min_ndt']:.2f}~{r['max_ndt']:.2f}], {r['recov_count']} recoveries")

print(f"3. Waypoints degraded (NDT 0.3-0.5): {len(degraded_wps)}")
for r in degraded_wps:
    print(f"   - {r['wp']['name']} ({r['wp']['id']}): NDT [{r['min_ndt']:.2f}~{r['max_ndt']:.2f}]")

print(f"4. Recovery attempts total: {len(recovery_requests)}")
successful = 0
for ros_t, text in recovery_requests:
    # Check if NDT improved after recovery
    before = [s['matching_error'] for st, s in status_samples if st < ros_t and ros_t - st < 10]
    after = [s['matching_error'] for st, s in status_samples if st > ros_t and st - ros_t < 10]
    if before and after and sum(after)/len(after) < sum(before)/len(before):
        successful += 1
print(f"   Successful (NDT improved): {successful}/{len(recovery_requests)}")

print(f"\n5. Odom total path: {total_odom_disp:.1f}m vs Map total path: {total_map_disp:.1f}m")
print(f"   Odom path error: {abs(total_odom_disp - total_map_disp):.1f}m "
      f"({abs(total_odom_disp - total_map_disp)/total_map_disp*100:.1f}%)" if total_map_disp > 0 else "")

print(f"\nTotal wall time: {time.time() - t0:.0f}s")
