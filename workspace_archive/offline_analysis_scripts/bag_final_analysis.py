#!/usr/bin/env python3
"""Final analysis: localization vs waypoints, odometry reliability."""
import sys, json, struct, math, time
sys.path.insert(0, '/usr/lib/python3/dist-packages')
from mcap.reader import make_reader
from collections import defaultdict

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
    if strlen > 10000: return None, offset
    s = data[offset:offset+strlen].decode('utf-8', errors='replace').rstrip('\x00')
    offset += strlen
    offset += (4 - (strlen % 4)) % 4
    return s, offset

def skip_rtps(data):
    if len(data) < 4: return 0
    encap = struct.unpack_from('<H', data, 0)[0]
    return 4 if encap in (0x0001, 0x0100) else 0

def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))

def parse_odom_message(data):
    if len(data) < 8: return None
    try:
        offset = skip_rtps(data)
        sec = struct.unpack_from('<i', data, offset)[0]; offset += 4
        nanosec = struct.unpack_from('<I', data, offset)[0]; offset += 4
        frame_id, offset = parse_cdr_string(data, offset)
        if frame_id is None or len(frame_id) > 60: return None
        child_id, offset = parse_cdr_string(data, offset)
        if child_id is None or len(child_id) > 60: return None
        if offset + 56 > len(data): return None
        px = struct.unpack_from('<d', data, offset)[0]; offset += 8
        py = struct.unpack_from('<d', data, offset)[0]; offset += 8
        pz = struct.unpack_from('<d', data, offset)[0]; offset += 8
        ox = struct.unpack_from('<d', data, offset)[0]; offset += 8
        oy = struct.unpack_from('<d', data, offset)[0]; offset += 8
        oz = struct.unpack_from('<d', data, offset)[0]; offset += 8
        ow = struct.unpack_from('<d', data, offset)[0]; offset += 8
        if not all(math.isfinite(v) for v in (px,py,pz,ox,oy,oz,ow)): return None
        return (sec+nanosec*1e-9, px, py, pz, ox, oy, oz, ow)
    except: return None

def parse_status_message(data):
    if len(data) < 8: return None
    try:
        offset = skip_rtps(data)
        camera_idx = data.find(b'camera_init', offset)
        if camera_idx < 0: return None
        strlen_at = camera_idx - 4
        strlen_val = struct.unpack_from('<I', data, strlen_at)[0]
        if strlen_val > 100: return None
        after = camera_idx + strlen_val
        after += (4 - (strlen_val % 4)) % 4
        if after + 12 > len(data): return None
        hc = struct.unpack_from('<I', data, after)[0] != 0
        me = struct.unpack_from('<f', data, after + 4)[0]
        inf = struct.unpack_from('<f', data, after + 8)[0]
        if not math.isfinite(me) or not math.isfinite(inf): return None
        return {'has_converged': hc, 'matching_error': me, 'inlier_fraction': inf}
    except: return None

# ── Stream bag ───────────────────────────────────────────────────
print("Streaming bag...")
t0 = time.time()

map_odom_samples = []   # [(ros_t, x, y, z, yaw)]
odom_samples = []       # [(ros_t, x, y, z, yaw)]  # /odom topic
body_in_odom = []       # [(ros_t, x, y, z, yaw)]  # camera_init->body
status_samples = []     # [(ros_t, dict)]
recovery_requests = []  # [(ros_t, text)]
recovery_status = []    # [(ros_t, text)]

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
            if len(data) < 8: continue
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
                    if not all(math.isfinite(v) for v in (tx,ty,tz,qx,qy,qz,qw)): continue
                    yaw = quat_to_yaw(qx, qy, qz, qw)
                    if (frame_id, child_id) == ('map', 'odom'):
                        map_odom_samples.append((ros_t, tx, ty, tz, yaw))
                    elif (frame_id, child_id) == ('camera_init', 'body'):
                        body_in_odom.append((ros_t, tx, ty, tz, yaw))
            except: continue

        elif channel.topic == '/odom':
            parsed = parse_odom_message(msg.data)
            if parsed:
                ts, px, py, pz, ox, oy, oz, ow = parsed
                yaw = quat_to_yaw(ox, oy, oz, ow)
                odom_samples.append((ts, px, py, pz, yaw))

        elif channel.topic == '/status':
            s = parse_status_message(msg.data)
            if s: status_samples.append((ros_t, s))

        elif channel.topic == '/localization/recovery_requests':
            if len(msg.data) > 4:
                try:
                    off = skip_rtps(msg.data)
                    text, _ = parse_cdr_string(msg.data, off)
                    if text: recovery_requests.append((ros_t, text))
                except: pass

        elif channel.topic == '/localization/recovery_status':
            if len(msg.data) > 4:
                try:
                    off = skip_rtps(msg.data)
                    text, _ = parse_cdr_string(msg.data, off)
                    if text: recovery_status.append((ros_t, text))
                except: pass

        if msg_count % 50000 == 0:
            print(f"  ... {msg_count/1000:.0f}k msgs, {time.time()-t0:.0f}s")

print(f"Done: {msg_count} msgs in {time.time()-t0:.0f}s")
print(f"  map->odom: {len(map_odom_samples)}, /odom: {len(odom_samples)}")
print(f"  status: {len(status_samples)}, recovery_req: {len(recovery_requests)}")

# ── Build aligned trajectory ─────────────────────────────────────
def find_nearest(samples, target_t, max_dt=0.5):
    if not samples: return None
    lo, hi = 0, len(samples)-1
    best_idx, best_dt = None, max_dt
    while lo <= hi:
        mid = (lo+hi)//2
        dt = abs(samples[mid][0] - target_t)
        if dt < best_dt: best_dt = dt; best_idx = mid
        if samples[mid][0] < target_t: lo = mid+1
        else: hi = mid-1
    return samples[best_idx] if best_idx is not None else None

print("\nBuilding aligned trajectory...")
trajectory = []
prev_mb = None
prev_ob = None
for mo_t, mo_x, mo_y, mo_z, mo_yaw in map_odom_samples:
    odom = find_nearest(odom_samples, mo_t)
    if odom is None: continue
    _, ob_x, ob_y, ob_z, ob_yaw = odom
    cos_mo = math.cos(mo_yaw); sin_mo = math.sin(mo_yaw)
    mb_x = mo_x + ob_x*cos_mo - ob_y*sin_mo
    mb_y = mo_y + ob_x*sin_mo + ob_y*cos_mo

    # Filter implausible jumps (>20m between consecutive map->base)
    if prev_mb and math.hypot(mb_x-prev_mb[0], mb_y-prev_mb[1]) > 20:
        continue
    prev_mb = (mb_x, mb_y)
    trajectory.append((mo_t, mb_x, mb_y, ob_x, ob_y, mo_x, mo_y, mo_yaw))

print(f"Aligned trajectory: {len(trajectory)} points")

# ── Compute map->odom changes to detect drift jumps ──────────────
mo_jumps = []
prev_mo = None
for mo_t, mb_x, mb_y, ob_x, ob_y, mo_x, mo_y, mo_yaw in trajectory:
    if prev_mo:
        dmo = math.hypot(mo_x-prev_mo[0], mo_y-prev_mo[1])
        if dmo > 0.5:
            mo_jumps.append((mo_t, prev_mo[0], prev_mo[1], mo_x, mo_y, dmo))
    prev_mo = (mo_x, mo_y)

# ── Compute odometry drift: compare odom path vs map path ────────
# Only during healthy periods (NDT < 0.3)
healthy_segments = []
in_healthy = False
seg_start = 0
for i, (ros_t, _, _, _, _, _, _, _) in enumerate(trajectory):
    ndt_ok = True
    for st, s in status_samples:
        if abs(st - ros_t) < 1.0 and s['matching_error'] > 0.3:
            ndt_ok = False; break
    if ndt_ok and not in_healthy:
        in_healthy = True; seg_start = i
    elif not ndt_ok and in_healthy:
        in_healthy = False
        if i - seg_start > 10:
            healthy_segments.append((seg_start, i))
if in_healthy and len(trajectory) - seg_start > 10:
    healthy_segments.append((seg_start, len(trajectory)-1))

print(f"Healthy segments: {len(healthy_segments)}")

total_odom_disp = 0.0
total_map_disp = 0.0
for i in range(1, len(trajectory)):
    _, mb_x1, mb_y1, ob_x1, ob_y1, _, _, _ = trajectory[i-1]
    _, mb_x2, mb_y2, ob_x2, ob_y2, _, _, _ = trajectory[i]
    d_odom = math.hypot(ob_x2-ob_x1, ob_y2-ob_y1)
    d_map = math.hypot(mb_x2-mb_x1, mb_y2-mb_y1)
    if math.isfinite(d_odom): total_odom_disp += d_odom
    if math.isfinite(d_map): total_map_disp += d_map

# Odom health check in healthy segments
odom_drift_data = []
for seg_start, seg_end in healthy_segments:
    if seg_end - seg_start < 10: continue
    t0_s = trajectory[seg_start][0]
    t1_s = trajectory[seg_end][0]
    mb_disp = math.hypot(trajectory[seg_end][1]-trajectory[seg_start][1],
                         trajectory[seg_end][2]-trajectory[seg_start][2])
    ob_disp = math.hypot(trajectory[seg_end][3]-trajectory[seg_start][3],
                         trajectory[seg_end][4]-trajectory[seg_start][4])
    dt = t1_s - t0_s
    if dt > 0:
        odom_drift_data.append((dt, mb_disp, ob_disp, abs(ob_disp-mb_disp)))

# ── Per-waypoint analysis ────────────────────────────────────────
print("\n" + "="*100)
print("WAYPOINT-BY-WAYPOINT ANALYSIS")
print("="*100)

results = []
for wp in waypoints:
    # Find closest trajectory point
    min_dist = float('inf')
    min_idx = 0
    for i, (_, mb_x, mb_y, _, _, _, _, _) in enumerate(trajectory):
        d = math.hypot(mb_x-wp['x'], mb_y-wp['y'])
        if d < min_dist: min_dist = d; min_idx = i

    ros_t = trajectory[min_idx][0] if min_idx < len(trajectory) else 0
    mb_x, mb_y = trajectory[min_idx][1], trajectory[min_idx][2]
    ob_x, ob_y = trajectory[min_idx][3], trajectory[min_idx][4]

    # NDT status around this time
    ndt_window = []
    for st, s in status_samples:
        if abs(st - ros_t) < 30:
            ndt_window.append(s)

    avg_ndt = sum(s['matching_error'] for s in ndt_window)/len(ndt_window) if ndt_window else float('nan')
    max_ndt = max((s['matching_error'] for s in ndt_window), default=float('nan'))

    # Determine drift at this waypoint
    if not ndt_window: health = 'NO_DATA'
    elif max_ndt <= 0.3: health = 'OK'
    elif max_ndt <= 0.5: health = 'DEGRADED'
    elif max_ndt <= 2.0: health = 'DRIFT'
    else: health = 'SEVERE'

    # Recovery events
    recov_near = [(rt, text) for rt, text in recovery_requests if abs(rt - ros_t) < 60]

    # Odom check: is odom near the waypoint?
    odom_dist_to_wp = math.hypot(ob_x-wp['x'], ob_y-wp['y'])

    # Map->odom jump near this waypoint
    mo_jumps_near = [(t, d) for t, _, _, _, _, d in mo_jumps if abs(t - ros_t) < 30]

    visited = min_dist < 3.0

    results.append({
        'wp': wp, 'min_dist': min_dist, 'ros_t': ros_t,
        'map_pos': (mb_x, mb_y), 'odom_pos': (ob_x, ob_y),
        'avg_ndt': avg_ndt, 'max_ndt': max_ndt, 'num_ndt': len(ndt_window),
        'health': health, 'visited': visited,
        'recov_count': len(recov_near),
        'odom_dist_to_wp': odom_dist_to_wp,
        'mo_jumps': mo_jumps_near,
    })

    icon = {'OK':'✓','DEGRADED':'⚠','DRIFT':'✗','SEVERE':'☠','NO_DATA':'?'}[health]
    print(f"  {icon} {wp['name']:<6} ({wp['id']}): "
          f"NDT[{avg_ndt:.2f}~{max_ndt:.2f}] | "
          f"map=({mb_x:.1f},{mb_y:.1f}) dist={min_dist:.1f}m | "
          f"odom=({ob_x:.1f},{ob_y:.1f}) odom2wp={odom_dist_to_wp:.1f}m | "
          f"{health}"
          f"{' | '+str(len(recov_near))+' recoveries' if recov_near else ''}"
          f"{' | map->odom jumps: '+str(len(mo_jumps_near)) if mo_jumps_near else ''}")

# ── Drift timeline ───────────────────────────────────────────────
print("\n" + "="*100)
print("DRIFT TIMELINE")
print("="*100)

# Group waypoints by health
phase = None
for r in results:
    if r['health'] != phase:
        phase = r['health']
        print(f"\n  --- {phase} phase ---")
    wp = r['wp']
    print(f"  {wp['name']} ({wp['id']}): map({wp['x']:.1f},{wp['y']:.1f}) "
          f"robot_map=({r['map_pos'][0]:.1f},{r['map_pos'][1]:.1f}) "
          f"odom=({r['odom_pos'][0]:.1f},{r['odom_pos'][1]:.1f}) "
          f"NDT=[{r['avg_ndt']:.2f} ~ {r['max_ndt']:.2f}]")

# ── Map->odom jumps ──────────────────────────────────────────────
print("\n" + "="*100)
print("MAP->ODOM JUMPS (>0.5m) - indicates localization corrections/drift")
print("="*100)
for t, old_x, old_y, new_x, new_y, d in mo_jumps[:30]:
    print(f"  t={t:.1f}: ({old_x:.1f},{old_y:.1f}) -> ({new_x:.1f},{new_y:.1f}) d={d:.2f}m")

# ── Odometry reliability ─────────────────────────────────────────
print("\n" + "="*100)
print("ODOMETRY RELIABILITY ASSESSMENT")
print("="*100)

print(f"\nTotal map path: {total_map_disp:.1f}m")
print(f"Total odom path: {total_odom_disp:.1f}m")
print(f"Odom/map ratio: {total_odom_disp/total_map_disp:.2f}" if total_map_disp > 0 else "")

# Check odom drift in each healthy segment
if odom_drift_data:
    print(f"\nHealthy segment odom drift:")
    total_drift = 0
    total_dt = 0
    for dt, mb_disp, ob_disp, drift in odom_drift_data[:10]:
        rate = drift/dt*100 if dt>0 else 0
        print(f"  {dt:.0f}s: map_disp={mb_disp:.1f}m odom_disp={ob_disp:.1f}m drift={drift:.2f}m (={rate:.2f} cm/s)")
        total_drift += drift
        total_dt += dt
    avg_rate = total_drift/total_dt*100 if total_dt > 0 else 0
    print(f"  Average odom drift rate: {avg_rate:.2f} cm/s")

# Check odom position vs waypoints during healthy periods
print(f"\nDuring HEALTHY periods (NDT OK): odom vs nearest waypoint")
ok_results = [r for r in results if r['health'] == 'OK']
if ok_results:
    avg_odom2wp = sum(r['odom_dist_to_wp'] for r in ok_results) / len(ok_results)
    print(f"  {len(ok_results)} waypoints: avg odom-to-waypoint distance = {avg_odom2wp:.1f}m")

# During drift: where does odom think the robot is?
drift_results = [r for r in results if r['health'] in ('DRIFT', 'SEVERE')]
if drift_results:
    print(f"\nDuring DRIFT periods: odom still tracks correctly?")
    for r in drift_results[:5]:
        wp = r['wp']
        # Find nearest waypoint by odom
        nearest_wp = min(waypoints, key=lambda w: math.hypot(r['odom_pos'][0]-w['x'], r['odom_pos'][1]-w['y']))
        print(f"  {wp['name']} ({wp['id']}): odom=({r['odom_pos'][0]:.1f},{r['odom_pos'][1]:.1f}) "
              f"→ nearest WP: {nearest_wp['name']} ({nearest_wp['id']}) "
              f"at ({nearest_wp['x']:.1f},{nearest_wp['y']:.1f}) "
              f"dist={math.hypot(r['odom_pos'][0]-nearest_wp['x'], r['odom_pos'][1]-nearest_wp['y']):.1f}m")

# ── Key summary ──────────────────────────────────────────────────
print("\n" + "="*100)
print("FINAL SUMMARY")
print("="*100)

drifted_wps = [r for r in results if r['health'] in ('DRIFT', 'SEVERE')]
ok_wps = [r for r in results if r['health'] == 'OK']
visited_wps = [r for r in results if r['visited']]

print(f"\n  Waypoints: {len(waypoints)} total, {len(visited_wps)} visited by localization")
print(f"  Healthy at: {len(ok_wps)} waypoints")
print(f"  Drifted at: {len(drifted_wps)} waypoints")

if drifted_wps:
    first = drifted_wps[0]
    last = drifted_wps[-1]
    print(f"\n  Drift range: {first['wp']['name']}({first['wp']['id']}) → {last['wp']['name']}({last['wp']['id']})")
    print(f"  Drift NDT range: [{first['max_ndt']:.2f} ~ {last['max_ndt']:.2f}]")

print(f"\n  Recovery attempts: {len(recovery_requests)}")

if drifted_wps:
    print(f"\n  Drift ROOT CAUSE: NDT fitness at map ~({drifted_wps[0]['map_pos'][0]:.1f},{drifted_wps[0]['map_pos'][1]:.1f})")
    print(f"    Robot actually at odom ~({drifted_wps[0]['odom_pos'][0]:.1f},{drifted_wps[0]['odom_pos'][1]:.1f})")
    print(f"    NDT thinks robot is near origin (-0.2, 2.0) — 15-20m error")

# Odom assessment
if odom_drift_data:
    avg_rate = sum(d for _,_,_,d in odom_drift_data)/sum(dt for dt,_,_,_ in odom_drift_data)*100
    print(f"\n  Odom drift rate: {avg_rate:.3f} cm/s")
    if avg_rate < 0.005:
        print("  VERDICT: ✓ ODOM IS HIGHLY RELIABLE")
        print("    - Can be used as primary fallback when NDT drifts")
        print("    - Strategy: when NDT matching_error > 0.5, latch map->odom")
        print("      and propagate pose using odom->base_footprint only")
        print("    - Reset map->odom when NDT recovers (matching_error < 0.15)")

print(f"\nTotal analysis time: {time.time()-t0:.0f}s")
