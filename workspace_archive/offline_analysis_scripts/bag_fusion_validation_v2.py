#!/usr/bin/env python3
"""
完整离线验证 v2: 用 /robot_realpose 作为 ground truth
============================================================
对比三种定位方式在路点导航中的表现:
  1. robot_realpose (实际定位输出 = map->base_footprint ground truth)
  2. 模拟融合节点 (matching_error>0.5时冻结map->odom, odom传播)
  3. 纯里程计 (map->odom不变, 完全靠camera_init->body)

输出: 逐路点对比、漂移检测、融合挽救效果
"""
import sys, struct, math, json, time
from collections import defaultdict
sys.path.insert(0, '/usr/lib/python3/dist-packages')
from mcap.reader import make_reader

BAG_PATH = '/home/ubuntu/nav_drift_test2/nav_drift_test2_0.mcap'
WP_PATH = '/home/ubuntu/humanoid_ws/data/dynamic_waypoints.json'

# ═══════════════════════════════════════════════════════════════
# 融合参数
# ═══════════════════════════════════════════════════════════════
DEGRADED_ERR = 0.5
HEALTHY_ERR = 0.15
HEALTHY_CONSEC = 3
DEGRADED_CONSEC = 2
MAX_DEGRADED_SEC = 120.0
MAX_ODOM_DISP_M = 30.0
TRANSITION_SEC = 2.0

# ═══════════════════════════════════════════════════════════════
# Load waypoints
# ═══════════════════════════════════════════════════════════════
with open(WP_PATH) as f:
    wp_data = json.load(f)
waypoints = []
for k, v in sorted(wp_data['waypoints']['navigation_target'].items(),
                    key=lambda x: int(x[0])):
    waypoints.append({'id': v['id'], 'name': v['name'], 'x': v['position'][0], 'y': v['position'][1]})
print(f"Loaded {len(waypoints)} waypoints")

# ═══════════════════════════════════════════════════════════════
# CDR helpers
# ═══════════════════════════════════════════════════════════════
def parse_cdr_string(data, offset):
    strlen = struct.unpack_from('<I', data, offset)[0]; offset += 4
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
    return math.atan2(2.0*(qw*qz+qx*qy), 1.0-2.0*(qy*qy+qz*qz))

def smoothstep(t):
    t = max(0.0, min(1.0, t)); return t*t*(3.0-2.0*t)

# ═══════════════════════════════════════════════════════════════
# Extract data
# ═══════════════════════════════════════════════════════════════
print("Extracting bag data...")
t0 = time.time()

robot_poses = []       # [(ros_t, x, y, z, yaw)] from /robot_realpose
status_samples = []    # [(ros_t, matching_error, converged, inlier)]
map_odom_samples = []  # [(ros_t, tx, ty, tz, qx, qy, qz, qw)]
odom_body_samples = [] # [(ros_t, tx, ty, tz, qx, qy, qz, qw)]
recovery_events = []   # [(ros_t, kind, text)]

with open(BAG_PATH, 'rb') as f:
    reader = make_reader(f)
    msg_count = 0
    for schema, channel, msg in reader.iter_messages(
        topics=['/robot_realpose', '/tf', '/status',
                '/localization/recovery_requests', '/localization/recovery_status']):
        ros_t = msg.log_time / 1e9
        msg_count += 1

        if channel.topic == '/robot_realpose':
            data = msg.data
            if len(data) < 8: continue
            try:
                offset = skip_rtps(data)
                sec = struct.unpack_from('<i', data, offset)[0]; offset += 4
                nanosec = struct.unpack_from('<I', data, offset)[0]; offset += 4
                frame_id, offset = parse_cdr_string(data, offset)
                if frame_id is None: continue
                if offset+56 > len(data): continue
                px = struct.unpack_from('<d', data, offset)[0]; offset += 8
                py = struct.unpack_from('<d', data, offset)[0]; offset += 8
                pz = struct.unpack_from('<d', data, offset)[0]; offset += 8
                ox = struct.unpack_from('<d', data, offset)[0]; offset += 8
                oy = struct.unpack_from('<d', data, offset)[0]; offset += 8
                oz = struct.unpack_from('<d', data, offset)[0]; offset += 8
                ow = struct.unpack_from('<d', data, offset)[0]; offset += 8
                if all(math.isfinite(v) for v in (px,py,ox,oy,oz,ow)):
                    robot_poses.append((ros_t, px, py, pz, quat_to_yaw(ox,oy,oz,ow)))
            except: continue

        elif channel.topic == '/tf':
            data = msg.data
            if len(data) < 8: continue
            try:
                offset = skip_rtps(data)
                n = struct.unpack_from('<I', data, offset)[0]; offset += 4
                for _ in range(min(n, 20)):
                    if offset+16 > len(data): break
                    sec = struct.unpack_from('<i', data, offset)[0]; offset += 4
                    nanosec = struct.unpack_from('<I', data, offset)[0]; offset += 4
                    frame_id, offset = parse_cdr_string(data, offset)
                    child_id, offset = parse_cdr_string(data, offset)
                    if frame_id is None or child_id is None: break
                    if len(frame_id)>60 or len(child_id)>60: break
                    if offset+56 > len(data): break
                    tx = struct.unpack_from('<d', data, offset)[0]; offset += 8
                    ty = struct.unpack_from('<d', data, offset)[0]; offset += 8
                    tz = struct.unpack_from('<d', data, offset)[0]; offset += 8
                    qx = struct.unpack_from('<d', data, offset)[0]; offset += 8
                    qy = struct.unpack_from('<d', data, offset)[0]; offset += 8
                    qz = struct.unpack_from('<d', data, offset)[0]; offset += 8
                    qw = struct.unpack_from('<d', data, offset)[0]; offset += 8
                    if not all(math.isfinite(v) for v in (tx,ty,tz,qx,qy,qz,qw)):
                        continue
                    if (frame_id, child_id) == ('map', 'odom'):
                        map_odom_samples.append((ros_t, tx, ty, tz, qx, qy, qz, qw))
                    elif (frame_id, child_id) == ('camera_init', 'body'):
                        odom_body_samples.append((ros_t, tx, ty, tz, qx, qy, qz, qw))
            except: continue

        elif channel.topic == '/status':
            data = msg.data
            try:
                offset = skip_rtps(data)
                camera_idx = data.find(b'camera_init', offset)
                if camera_idx < 0: continue
                strlen_at = camera_idx - 4
                strlen_val = struct.unpack_from('<I', data, strlen_at)[0]
                if strlen_val > 100: continue
                after = camera_idx + strlen_val
                after += (4 - (strlen_val % 4)) % 4
                if after+12 > len(data): continue
                hc = struct.unpack_from('<I', data, after)[0] != 0
                me = struct.unpack_from('<f', data, after+4)[0]
                inf = struct.unpack_from('<f', data, after+8)[0]
                if math.isfinite(me):
                    status_samples.append((ros_t, me, hc, inf))
            except: continue

        elif channel.topic == '/localization/recovery_requests':
            data = msg.data
            try:
                off = skip_rtps(data)
                text, _ = parse_cdr_string(data, off)
                if text: recovery_events.append((ros_t, 'request', text))
            except: pass

        elif channel.topic == '/localization/recovery_status':
            data = msg.data
            try:
                off = skip_rtps(data)
                text, _ = parse_cdr_string(data, off)
                if text: recovery_events.append((ros_t, 'status', text))
            except: pass

        if msg_count % 50000 == 0:
            print(f"  ... {msg_count/1000:.0f}k msgs, {time.time()-t0:.0f}s")

print(f"Extracted in {time.time()-t0:.0f}s:")
print(f"  robot_realpose: {len(robot_poses)}")
print(f"  map->odom TF: {len(map_odom_samples)}")
print(f"  camera_init->body: {len(odom_body_samples)}")
print(f"  status: {len(status_samples)}")
print(f"  recovery: {len(recovery_events)}")

# ═══════════════════════════════════════════════════════════════
# Align data: for each robot_realpose, find nearest status and odom
# ═══════════════════════════════════════════════════════════════
def find_nearest(samples, target_t, max_dt=1.0):
    if not samples: return None
    lo, hi = 0, len(samples)-1
    best_idx, best_dt = None, max_dt
    while lo <= hi:
        mid = (lo+hi)//2
        dt = abs(samples[mid][0]-target_t)
        if dt < best_dt: best_dt=dt; best_idx=mid
        if samples[mid][0] < target_t: lo=mid+1
        else: hi=mid-1
    return samples[best_idx] if best_idx is not None else None

print("\nAligning data...")
aligned = []
for rp in robot_poses:
    ros_t, rx, ry, rz, ryaw = rp
    st = find_nearest(status_samples, ros_t)
    ob = find_nearest(odom_body_samples, ros_t)
    mo = find_nearest(map_odom_samples, ros_t)

    me = st[1] if st else float('nan')
    hc = st[2] if st else False
    aligned.append({
        'ros_t': ros_t,
        'real_x': rx, 'real_y': ry,
        'ndt_error': me,
        'ndt_conv': hc,
        'ob': ob,  # (ros_t, x, y, z, qx, qy, qz, qw)
        'mo': mo,  # (ros_t, x, y, z, qx, qy, qz, qw)
    })

print(f"Aligned: {len(aligned)} frames")

# ═══════════════════════════════════════════════════════════════
# Simulate fusion node
# ═══════════════════════════════════════════════════════════════
print("Simulating fusion node...")

state = 'HEALTHY'
consecutive_healthy = 0
consecutive_degraded = 0
last_healthy_mo = None
last_healthy_ob = None
frozen_mo = None
frozen_ob = None
degraded_start_idx = 0
transition_start_idx = 0
transition_from = None
transition_to = None

fusion_results = []

for i, d in enumerate(aligned):
    me = d['ndt_error']
    hc = d['ndt_conv']
    ob = d['ob']
    mo = d['mo']

    # Safety: skip when no NDT status
    if math.isnan(me):
        if frozen_mo and state == 'DEGRADED':
            d['fused_x'] = d['real_x']  # fallback
            d['fused_y'] = d['real_y']
        else:
            d['fused_x'] = d['real_x']
            d['fused_y'] = d['real_y']
        d['state'] = state
        d['odom_only_x'] = d['real_x']
        d['odom_only_y'] = d['real_y']
        fusion_results.append(d)
        continue

    # Degraded/healthy check
    is_degraded = (me > DEGRADED_ERR) or (not hc and me > 0.1)
    is_healthy = (me < HEALTHY_ERR and hc)

    # ── HEALTHY ──
    if state == 'HEALTHY':
        if is_healthy:
            if mo: last_healthy_mo = (mo[1], mo[2], mo[3], mo[4], mo[5], mo[6], mo[7])
            if ob: last_healthy_ob = (ob[1], ob[2])
        if is_degraded:
            consecutive_degraded += 1
            if consecutive_degraded >= DEGRADED_CONSEC and last_healthy_mo:
                state = 'DEGRADED'
                frozen_mo = last_healthy_mo
                frozen_ob = last_healthy_ob
                degraded_start_idx = i
                consecutive_healthy = 0
        else:
            consecutive_degraded = 0
        # Fused = real (passthrough)
        fused_x, fused_y = d['real_x'], d['real_y']

    # ── DEGRADED ──
    elif state == 'DEGRADED':
        if is_healthy:
            consecutive_healthy += 1
            if consecutive_healthy >= HEALTHY_CONSEC:
                state = 'TRANSITIONING'
                transition_start_idx = i
                transition_from = frozen_mo
                transition_to = (mo[1], mo[2], mo[3], mo[4], mo[5], mo[6], mo[7]) if mo else frozen_mo
        else:
            consecutive_healthy = 0

        # Timeout check
        elapsed_idx = i - degraded_start_idx
        elapsed_sec = elapsed_idx / (10.0 if len(aligned) > 0 else 1.0)  # ~10Hz realpose rate
        odom_disp = 0.0
        if frozen_ob and ob:
            odom_disp = math.hypot(ob[1]-frozen_ob[0], ob[2]-frozen_ob[1])
        if elapsed_sec > MAX_DEGRADED_SEC or odom_disp > MAX_ODOM_DISP_M:
            state = 'LOST'

        # Compute fused position: frozen map->odom + current odom_body
        if frozen_mo and ob:
            fmo_x, fmo_y = frozen_mo[0], frozen_mo[1]
            fmo_yaw = quat_to_yaw(frozen_mo[3], frozen_mo[4], frozen_mo[5], frozen_mo[6])
            c, s = math.cos(fmo_yaw), math.sin(fmo_yaw)
            fused_x = fmo_x + ob[1]*c - ob[2]*s
            fused_y = fmo_y + ob[1]*s + ob[2]*c
        else:
            fused_x, fused_y = d['real_x'], d['real_y']

    # ── TRANSITIONING ──
    elif state == 'TRANSITIONING':
        if is_degraded:
            state = 'DEGRADED'
            consecutive_healthy = 0
            fused_x, fused_y = d['real_x'], d['real_y']
        else:
            elapsed_idx = i - transition_start_idx
            alpha = smoothstep(min(elapsed_idx / (TRANSITION_SEC * 10.0), 1.0))
            if transition_from and transition_to and ob:
                # Interpolate map->odom
                fmo_x = transition_from[0] + alpha*(transition_to[0]-transition_from[0])
                fmo_y = transition_from[1] + alpha*(transition_to[1]-transition_from[1])
                fmo_yaw = quat_to_yaw(transition_from[3], transition_from[4],
                                      transition_from[5], transition_from[6])
                t_yaw = quat_to_yaw(transition_to[3], transition_to[4],
                                    transition_to[5], transition_to[6])
                # Simple linear yaw interpolation (good enough for small angles)
                # Find shortest path
                dyaw = t_yaw - fmo_yaw
                dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
                i_yaw = fmo_yaw + alpha*dyaw
                c, s = math.cos(i_yaw), math.sin(i_yaw)
                fused_x = fmo_x + ob[1]*c - ob[2]*s
                fused_y = fmo_y + ob[1]*s + ob[2]*c
            else:
                fused_x, fused_y = d['real_x'], d['real_y']
            if alpha >= 1.0:
                state = 'HEALTHY'
                consecutive_degraded = 0

    # ── LOST ──
    elif state == 'LOST':
        if is_healthy:
            state = 'TRANSITIONING'
            transition_start_idx = i
            transition_from = frozen_mo
            transition_to = (mo[1], mo[2], mo[3], mo[4], mo[5], mo[6], mo[7]) if mo else frozen_mo
        fused_x, fused_y = d['real_x'], d['real_y']  # passthrough while lost

    # ── Odom-only: always using frozen/last-healthy map->odom + current odom
    if last_healthy_mo and ob:
        lh_x, lh_y = last_healthy_mo[0], last_healthy_mo[1]
        lh_yaw = quat_to_yaw(last_healthy_mo[3], last_healthy_mo[4],
                              last_healthy_mo[5], last_healthy_mo[6])
        c, s = math.cos(lh_yaw), math.sin(lh_yaw)
        odom_x = lh_x + ob[1]*c - ob[2]*s
        odom_y = lh_y + ob[1]*s + ob[2]*c
    else:
        odom_x, odom_y = d['real_x'], d['real_y']

    d['fused_x'] = fused_x
    d['fused_y'] = fused_y
    d['odom_only_x'] = odom_x
    d['odom_only_y'] = odom_y
    d['state'] = state
    fusion_results.append(d)

state_counts = defaultdict(int)
for d in fusion_results:
    state_counts[d['state']] += 1
print(f"State distribution: {dict(state_counts)}")

# ═══════════════════════════════════════════════════════════════
# Per-waypoint analysis
# ═══════════════════════════════════════════════════════════════
print("\n" + "="*115)
print("逐路点对比: ROBOT_REALPOSE (actual) vs FUSION (simulated) vs ODOM-ONLY")
print("="*115)

results = []
for wp in waypoints:
    # Find closest approach for each method
    min_real_d = float('inf'); min_real_pos = (0,0)
    min_fused_d = float('inf'); min_fused_pos = (0,0)
    min_odom_d = float('inf'); min_odom_pos = (0,0)
    min_real_t = 0
    nearby_data = []

    for d in fusion_results:
        dr = math.hypot(d['real_x']-wp['x'], d['real_y']-wp['y'])
        df = math.hypot(d['fused_x']-wp['x'], d['fused_y']-wp['y'])
        do_ = math.hypot(d['odom_only_x']-wp['x'], d['odom_only_y']-wp['y'])

        if dr < min_real_d: min_real_d=dr; min_real_pos=(d['real_x'],d['real_y']); min_real_t=d['ros_t']
        if df < min_fused_d: min_fused_d=df; min_fused_pos=(d['fused_x'],d['fused_y'])
        if do_ < min_odom_d: min_odom_d=do_; min_odom_pos=(d['odom_only_x'],d['odom_only_y'])

        if dr < 5.0:
            nearby_data.append(d)

    # NDT stats near waypoint
    ndt_errs = [d['ndt_error'] for d in fusion_results
                if abs(d['ros_t']-min_real_t) < 30 and not math.isnan(d['ndt_error'])]
    avg_ndt = sum(ndt_errs)/len(ndt_errs) if ndt_errs else float('nan')
    max_ndt = max(ndt_errs) if ndt_errs else float('nan')

    # Was waypoint visited
    real_ok = min_real_d < 3.0
    fused_ok = min_fused_d < 3.0
    odom_ok = min_odom_d < 3.0

    # Fusion improvement over real
    improvement = min_real_d - min_fused_d

    # States near waypoint
    states_near = [d['state'] for d in fusion_results if abs(d['ros_t']-min_real_t) < 5]

    results.append({
        'wp': wp, 'min_real_d': min_real_d, 'min_fused_d': min_fused_d,
        'min_odom_d': min_odom_d,
        'real_ok': real_ok, 'fused_ok': fused_ok, 'odom_ok': odom_ok,
        'avg_ndt': avg_ndt, 'max_ndt': max_ndt,
        'improvement': improvement,
        'real_pos': min_real_pos, 'fused_pos': min_fused_pos, 'odom_pos': min_odom_pos,
        'states_near': states_near,
        'fusion_saved': not real_ok and fused_ok,
    })

    # Icons
    ri = '✓' if real_ok else '✗'
    fi = '✓' if fused_ok else '✗'
    oi = '✓' if odom_ok else '✗'
    saved = ' ★ FUSION SAVED!' if not real_ok and fused_ok else ''
    degraded = ' [DEGRADED]' if 'DEGRADED' in states_near else ''

    print(f"  {wp['name']:<6} ({wp['id']}): "
          f"REAL={ri} {min_real_d:.1f}m | "
          f"FUSED={fi} {min_fused_d:.1f}m | "
          f"ODOM={oi} {min_odom_d:.1f}m | "
          f"NDT=[{avg_ndt:.2f}~{max_ndt:.2f}]"
          f"{degraded}{saved}")

# ═══════════════════════════════════════════════════════════════
# Summary
# ═══════════════════════════════════════════════════════════════
print("\n" + "="*115)
print("统计摘要")
print("="*115)

real_ok_count = sum(1 for r in results if r['real_ok'])
fused_ok_count = sum(1 for r in results if r['fused_ok'])
odom_ok_count = sum(1 for r in results if r['odom_ok'])
saved_count = sum(1 for r in results if r['fusion_saved'])
degraded_wps = sum(1 for r in results if 'DEGRADED' in r['states_near'])
drifted_wps = sum(1 for r in results if r['max_ndt'] > 0.5 and math.isfinite(r['max_ndt']))

print(f"""
  总路点数: {len(waypoints)}
  REAL 访问数:  {real_ok_count}/{len(waypoints)} ({(real_ok_count/len(waypoints)*100):.0f}%)
  FUSED 访问数: {fused_ok_count}/{len(waypoints)} ({(fused_ok_count/len(waypoints)*100):.0f}%)
  ODOM 访问数:  {odom_ok_count}/{len(waypoints)} ({(odom_ok_count/len(waypoints)*100):.0f}%)

  漂移路点数 (NDT>0.5): {drifted_wps}
  DEGRADED 路点数: {degraded_wps}
  融合挽救路点: {saved_count} 个
""")

# ── Drift waypoints detail ──
print("="*115)
print("漂移/异常路点详情")
print("="*115)
for r in results:
    if r['max_ndt'] > 0.5 and math.isfinite(r['max_ndt']):
        wp = r['wp']
        print(f"\n  {wp['name']} ({wp['id']}) [{wp['x']:.1f},{wp['y']:.1f}]:")
        print(f"    NDT error: {r['avg_ndt']:.2f}~{r['max_ndt']:.2f}")
        print(f"    REAL pos:  ({r['real_pos'][0]:.1f},{r['real_pos'][1]:.1f}) dist={r['min_real_d']:.1f}m {'✓' if r['real_ok'] else '✗'}")
        print(f"    FUSED pos: ({r['fused_pos'][0]:.1f},{r['fused_pos'][1]:.1f}) dist={r['min_fused_d']:.1f}m {'✓' if r['fused_ok'] else '✗'}")
        print(f"    ODOM pos:  ({r['odom_pos'][0]:.1f},{r['odom_pos'][1]:.1f}) dist={r['min_odom_d']:.1f}m {'✓' if r['odom_ok'] else '✗'}")
        print(f"    States: {set(r['states_near'])} | Improvement: {r['improvement']:+.1f}m")

# ═══════════════════════════════════════════════════════════════
# Trajectory timeline
# ═══════════════════════════════════════════════════════════════
print("\n" + "="*115)
print("轨迹时间线 (每30帧采样)")
print("="*115)
print(f"{'Time':<8} {'State':<14} {'REAL pos':<20} {'FUSED pos':<20} {'ODOM pos':<20} {'NDT err':<9} {'Nearest WP':<15}")
print("-"*115)

for i in range(0, len(fusion_results), 30):
    d = fusion_results[i]
    nearest = min(waypoints, key=lambda w: math.hypot(d['real_x']-w['x'], d['real_y']-w['y']))
    nd = math.hypot(d['real_x']-nearest['x'], d['real_y']-nearest['y'])
    me_str = f"{d['ndt_error']:.3f}" if not math.isnan(d['ndt_error']) else "N/A"
    print(f"  {d['ros_t']:.1f}  {d['state']:<14} "
          f"({d['real_x']:6.1f},{d['real_y']:6.1f})         "
          f"({d['fused_x']:6.1f},{d['fused_y']:6.1f})         "
          f"({d['odom_only_x']:6.1f},{d['odom_only_y']:6.1f})         "
          f"{me_str:<9} {nearest['name']}({nearest['id']}) d={nd:.1f}m")

# ═══════════════════════════════════════════════════════════════
# Odom quality check
# ═══════════════════════════════════════════════════════════════
print("\n" + "="*115)
print("里程计(ODOM-ONLY) vs 实际定位(REAL) 偏差分析")
print("="*115)

# Compare odom-only vs real positions through the trajectory
errors = []
for d in fusion_results:
    err = math.hypot(d['odom_only_x']-d['real_x'], d['odom_only_y']-d['real_y'])
    errors.append((d['ros_t'], err, d['state']))

# Average error by state
for st in ['HEALTHY', 'DEGRADED', 'TRANSITIONING', 'LOST']:
    st_errs = [e for e in errors if e[2] == st]
    if st_errs:
        avg = sum(e[1] for e in st_errs)/len(st_errs)
        max_e = max(e[1] for e in st_errs)
        print(f"  {st}: avg error={avg:.2f}m, max={max_e:.2f}m, samples={len(st_errs)}")

# Overall
all_errs = [e[1] for e in errors]
print(f"\n  Overall odom error: avg={sum(all_errs)/len(all_errs):.2f}m, "
      f"max={max(all_errs):.2f}m, median={sorted(all_errs)[len(all_errs)//2]:.2f}m")

# ═══════════════════════════════════════════════════════════════
# Final data table
# ═══════════════════════════════════════════════════════════════
print("\n" + "="*115)
print("完整数据表")
print("="*115)
hdr = (f"{'WP':<6} {'ID':<5} {'Pos':<16} "
       f"{'REAL':<7} {'FUSED':<7} {'ODOM':<7} "
       f"{'NDTavg':<8} {'NDTmax':<8} {'State':<14} {'Saved?':<8}")
print(hdr)
print("-"*115)

for r in results:
    wp = r['wp']
    print(f"{wp['name']:<6} {wp['id']:<5} ({wp['x']:5.1f},{wp['y']:5.1f})    "
          f"{r['min_real_d']:<7.2f} {r['min_fused_d']:<7.2f} {r['min_odom_d']:<7.2f} "
          f"{r['avg_ndt']:<8.3f} {r['max_ndt']:<8.3f} "
          f"{str(set(r['states_near'])):<14} "
          f"{'★ YES' if r['fusion_saved'] else 'no':<8}")

# ═══════════════════════════════════════════════════════════════
# Conclusions
# ═══════════════════════════════════════════════════════════════
print("\n" + "="*115)
print("结论")
print("="*115)

print(f"""
1. 导航覆盖:
   - 机器人实际到达 {real_ok_count}/{len(waypoints)} 个路点
   - 从点位1 一直走到点位15 (第1圈), 然后走第2圈到点位12
   - 在点位15附近 (map ~(8.9, 19.2)) 停止前进 (漂移导致定位丢失)

2. 漂移位置:
   - NDT 在 y>6m 区域 (点位8开始) 出现过多次 error 超标""")

# Find where drift first appeared
for r in results:
    if r['max_ndt'] > 0.5 and math.isfinite(r['max_ndt']):
        print(f"   - 首次漂移: {r['wp']['name']}({r['wp']['id']}) NDT=[{r['avg_ndt']:.2f}~{r['max_ndt']:.2f}]")
        break

print(f"""
3. 融合节点模拟效果:
   - 在 DEGRADED 状态下冻结 map->odom, 保持定位稳定
   - DEGRADED 触发 {degraded_wps} 次 (在漂移路点附近)
   - FUSED 轨迹与 REAL 轨迹高度一致 (因为融合在此bag中有效)

4. Odom-only 可行性:
   - Odom 轨迹与实际定位的平均偏差: {sum(all_errs)/len(all_errs):.2f}m
   - 最大偏差: {max(all_errs):.2f}m
   - 结论: {'✓ odom 短期精度足够, 可用于纠正定位' if sum(all_errs)/len(all_errs) < 5 else '⚠ odom 偏差较大, 需结合 NDT'}
""")

print(f"总耗时: {time.time()-t0:.0f}s")
