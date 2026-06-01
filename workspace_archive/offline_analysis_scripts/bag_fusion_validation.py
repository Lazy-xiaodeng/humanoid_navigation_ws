#!/usr/bin/env python3
"""
离线验证 NDT + 里程计融合节点的定位效果
============================================================
模拟 fusion 节点状态机，对比:
  1. NDT-only 轨迹 (bag 中实际发生的)
  2. Fused 轨迹 (如果有融合节点会怎样)
  3. Odom 轨迹 (纯里程计)

输出: 逐点位对比表、漂移检测、odom 可靠性评估
"""
import sys, json, struct, math, time
from collections import defaultdict, deque
sys.path.insert(0, '/usr/lib/python3/dist-packages')
from mcap.reader import make_reader

BAG_PATH = '/home/ubuntu/nav_drift_test2/nav_drift_test2_0.mcap'
WP_PATH = '/home/ubuntu/humanoid_ws/data/dynamic_waypoints.json'

# ═══════════════════════════════════════════════════════════════
# 参数 (与 localization_odom_fusion.py 保持一致)
# ═══════════════════════════════════════════════════════════════
DEGRADED_ERROR_THRESHOLD = 0.5
HEALTHY_ERROR_THRESHOLD = 0.15
HEALTHY_CONSECUTIVE_FRAMES = 3
DEGRADED_CONSECUTIVE_FRAMES = 2
MAX_DEGRADED_DURATION_SEC = 120.0
MAX_ODOM_DISPLACEMENT_M = 30.0
TRANSITION_DURATION_SEC = 2.0

# ═══════════════════════════════════════════════════════════════
# 加载路点
# ═══════════════════════════════════════════════════════════════
with open(WP_PATH) as f:
    wp_data = json.load(f)
waypoints = []
for k, v in sorted(wp_data['waypoints']['navigation_target'].items(),
                    key=lambda x: int(x[0])):
    p = v['position']
    waypoints.append({'id': v['id'], 'name': v['name'], 'x': p[0], 'y': p[1]})
print(f"加载 {len(waypoints)} 个路点")

# ═══════════════════════════════════════════════════════════════
# CDR 解析
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
    t = max(0.0, min(1.0, t))
    return t*t*(3.0-2.0*t)

def quat_slerp(q1, q2, t):
    """四元数球面线性插值"""
    import numpy as np
    q1, q2 = np.array(q1), np.array(q2)
    dot = np.dot(q1, q2)
    if dot < 0.0: q2 = -q2; dot = -dot
    dot = np.clip(dot, -1.0, 1.0)
    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)
    if sin_theta_0 < 1e-6:
        result = q1*(1.0-t)+q2*t
        return tuple(result/np.linalg.norm(result))
    s0 = math.sin((1.0-t)*theta_0)/sin_theta_0
    s1 = math.sin(t*theta_0)/sin_theta_0
    return tuple(q1*s0+q2*s1)

# ═══════════════════════════════════════════════════════════════
# 流式读取 bag 数据
# ═══════════════════════════════════════════════════════════════
print("读取 bag 数据...")
t0 = time.time()

map_odom_samples = []    # [(ros_t, tx, ty, tz, qx, qy, qz, qw)]
odom_body_samples = []   # [(ros_t, tx, ty, tz, qx, qy, qz, qw)]
status_samples = []      # [(ros_t, matching_error, has_converged, inlier)]
recovery_events = []     # [(ros_t, kind, detail)]

with open(BAG_PATH, 'rb') as f:
    reader = make_reader(f)
    msg_count = 0
    for schema, channel, msg in reader.iter_messages(
        topics=['/tf', '/status', '/localization/recovery_requests',
                '/localization/recovery_status']):
        ros_t = msg.log_time / 1e9
        msg_count += 1

        if channel.topic == '/tf':
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

        elif channel.topic in ('/localization/recovery_requests',
                               '/localization/recovery_status'):
            data = msg.data
            try:
                off = skip_rtps(data)
                text, _ = parse_cdr_string(data, off)
                if text:
                    kind = 'request' if 'requests' in channel.topic else 'status'
                    recovery_events.append((ros_t, kind, text))
            except: pass

        if msg_count % 50000 == 0:
            print(f"  ... {msg_count/1000:.0f}k msgs, {time.time()-t0:.0f}s")

print(f"读取完成: {msg_count} 条消息, {time.time()-t0:.0f}s")
print(f"  map->odom: {len(map_odom_samples)}")
print(f"  camera_init->body: {len(odom_body_samples)}")
print(f"  status: {len(status_samples)}")
print(f"  recovery: {len(recovery_events)}")

# ═══════════════════════════════════════════════════════════════
# 辅助: 查找最近样本
# ═══════════════════════════════════════════════════════════════
def find_nearest(samples, target_t, max_dt=0.5):
    if not samples: return None
    lo, hi = 0, len(samples)-1
    best_idx, best_dt = None, max_dt
    while lo <= hi:
        mid = (lo+hi)//2
        dt = abs(samples[mid][0]-target_t)
        if dt < best_dt: best_dt = dt; best_idx = mid
        if samples[mid][0] < target_t: lo = mid+1
        else: hi = mid-1
    return samples[best_idx] if best_idx is not None else None

def compute_map_base(mo_x, mo_y, mo_yaw, ob_x, ob_y):
    """map_T_base = map_T_odom * odom_T_body (简化2D)"""
    cos_mo = math.cos(mo_yaw); sin_mo = math.sin(mo_yaw)
    mb_x = mo_x + ob_x*cos_mo - ob_y*sin_mo
    mb_y = mo_y + ob_x*sin_mo + ob_y*cos_mo
    return mb_x, mb_y

# ═══════════════════════════════════════════════════════════════
# 模拟融合节点状态机
# ═══════════════════════════════════════════════════════════════
print("\n模拟融合节点状态机...")

# 对齐数据: 对每个 status 时间点，找到最近的 map_odom 和 odom_body
aligned = []
for st, me, hc, inf in status_samples:
    mo = find_nearest(map_odom_samples, st)
    ob = find_nearest(odom_body_samples, st)
    if mo and ob:
        aligned.append({
            'ros_t': st,
            'ndt_error': me,
            'ndt_converged': hc,
            'ndt_inlier': inf,
            'mo_x': mo[1], 'mo_y': mo[2], 'mo_z': mo[3],
            'mo_qx': mo[4], 'mo_qy': mo[5], 'mo_qz': mo[6], 'mo_qw': mo[7],
            'ob_x': ob[1], 'ob_y': ob[2],
        })

print(f"对齐数据点: {len(aligned)}")

# 模拟状态机
state = 'HEALTHY'
consecutive_healthy = 0
consecutive_degraded = 0
last_healthy_mo = None  # {x,y,z,qx,qy,qz,qw}
last_healthy_ob = None  # {x,y}
frozen_mo = None
frozen_ob = None
degraded_start_idx = 0
transition_start_idx = 0
transition_from = None
transition_to = None

fusion_results = []  # [(ros_t, state, fused_mo_x, fused_mo_y, fused_mo_yaw,
                     #   ndt_mb_x, ndt_mb_y, fused_mb_x, fused_mb_y, odom_mb_x, odom_mb_y)]

for i, d in enumerate(aligned):
    ros_t = d['ros_t']
    me = d['ndt_error']
    hc = d['ndt_converged']
    mo_yaw = quat_to_yaw(d['mo_qx'], d['mo_qy'], d['mo_qz'], d['mo_qw'])

    # 当前 NDT 的 map->odom
    ndt_mo = {'x': d['mo_x'], 'y': d['mo_y'], 'z': d['mo_z'],
              'qx': d['mo_qx'], 'qy': d['mo_qy'], 'qz': d['mo_qz'], 'qw': d['mo_qw']}
    ob_xy = {'x': d['ob_x'], 'y': d['ob_y']}

    # NDT-only 的 map->base
    ndt_mb_x, ndt_mb_y = compute_map_base(d['mo_x'], d['mo_y'], mo_yaw, d['ob_x'], d['ob_y'])

    # ── 退化/健康判断 ──
    is_degraded = (me > DEGRADED_ERROR_THRESHOLD) or (not hc and me > 0.1)
    is_healthy = (me < HEALTHY_ERROR_THRESHOLD and hc)

    # ── 状态机 ──
    if state == 'HEALTHY':
        if is_healthy and me < HEALTHY_ERROR_THRESHOLD:
            last_healthy_mo = ndt_mo.copy()
            last_healthy_ob = ob_xy.copy()

        if is_degraded:
            consecutive_degraded += 1
            if consecutive_degraded >= DEGRADED_CONSECUTIVE_FRAMES:
                if last_healthy_mo is not None:
                    state = 'DEGRADED'
                    frozen_mo = last_healthy_mo.copy()
                    frozen_ob = last_healthy_ob.copy() if last_healthy_ob else None
                    degraded_start_idx = i
                    consecutive_healthy = 0
        else:
            consecutive_degraded = 0

        # 发布: NDT 直通
        fused_mo = ndt_mo

    elif state == 'DEGRADED':
        if is_healthy:
            consecutive_healthy += 1
            if consecutive_healthy >= HEALTHY_CONSECUTIVE_FRAMES:
                state = 'TRANSITIONING'
                transition_start_idx = i
                transition_from = frozen_mo.copy()
                transition_to = ndt_mo.copy()
        else:
            consecutive_healthy = 0

        # 检查超时 (基于 sample 索引差 × 平均采样间隔)
        elapsed_idx = i - degraded_start_idx
        # status 采样率约 0.24Hz, 所以 elapsed_sec ≈ elapsed_idx / 0.24
        elapsed_sec = elapsed_idx * 4.0  # 近似

        # 检查 odom 位移
        if frozen_ob and ob_xy:
            odom_disp = math.hypot(ob_xy['x']-frozen_ob['x'], ob_xy['y']-frozen_ob['y'])
        else:
            odom_disp = 0.0

        if elapsed_sec > MAX_DEGRADED_DURATION_SEC or odom_disp > MAX_ODOM_DISPLACEMENT_M:
            state = 'LOST'
            frozen_mo = None  # 停止发布

        # 发布: 冻结值
        fused_mo = frozen_mo if frozen_mo else ndt_mo

    elif state == 'TRANSITIONING':
        # 过渡中断检查: 如果 NDT 又变差了，回到 DEGRADED
        if is_degraded:
            state = 'DEGRADED'
            consecutive_healthy = 0
            fused_mo = frozen_mo if frozen_mo else ndt_mo
        else:
            elapsed_idx = i - transition_start_idx
            alpha = smoothstep(min(elapsed_idx * 4.0 / TRANSITION_DURATION_SEC, 1.0))

            if transition_from and transition_to:
                # 平移线性插值
                fx = transition_from['x'] + alpha*(transition_to['x']-transition_from['x'])
                fy = transition_from['y'] + alpha*(transition_to['y']-transition_from['y'])
                fz = transition_from['z'] + alpha*(transition_to['z']-transition_from['z'])
                # 旋转 slerp
                qf = (transition_from['qx'], transition_from['qy'],
                      transition_from['qz'], transition_from['qw'])
                qt = (transition_to['qx'], transition_to['qy'],
                      transition_to['qz'], transition_to['qw'])
                qi = quat_slerp(qf, qt, alpha)
                fused_mo = {'x': fx, 'y': fy, 'z': fz,
                           'qx': qi[0], 'qy': qi[1], 'qz': qi[2], 'qw': qi[3]}
            else:
                fused_mo = ndt_mo

            if alpha >= 1.0:
                state = 'HEALTHY'
                last_healthy_mo = ndt_mo.copy()
                last_healthy_ob = ob_xy.copy()
                consecutive_degraded = 0

    elif state == 'LOST':
        # 等待 recovery (检查 recovery_status)
        # 在 bag 数据中，当 recovery 成功时会更新 map->odom
        # 我们用 is_healthy 作为 recovery 成功的信号
        if is_healthy:
            state = 'TRANSITIONING'
            transition_start_idx = i
            transition_from = frozen_mo.copy() if frozen_mo else ndt_mo.copy()
            transition_to = ndt_mo.copy()
        fused_mo = ndt_mo  # 让 NDT 的结果通过(期望 recovery 已纠正)

    # 计算 fused map->base
    fused_mo_yaw = quat_to_yaw(fused_mo['qx'], fused_mo['qy'], fused_mo['qz'], fused_mo['qw'])
    fused_mb_x, fused_mb_y = compute_map_base(
        fused_mo['x'], fused_mo['y'], fused_mo_yaw, d['ob_x'], d['ob_y'])

    # 纯 odom 的 map->base (用 frozen 的 map->odom, 本次用第一个 healthy 值)
    # odom-only: 假设 map->odom 保持不变，只有 odom 提供运动
    if last_healthy_mo:
        lh_yaw = quat_to_yaw(last_healthy_mo['qx'], last_healthy_mo['qy'],
                              last_healthy_mo['qz'], last_healthy_mo['qw'])
        odom_mb_x, odom_mb_y = compute_map_base(
            last_healthy_mo['x'], last_healthy_mo['y'], lh_yaw, d['ob_x'], d['ob_y'])
    else:
        odom_mb_x, odom_mb_y = ndt_mb_x, ndt_mb_y

    fusion_results.append({
        'ros_t': ros_t,
        'state': state,
        'ndt_error': me,
        'ndt_mb_x': ndt_mb_x, 'ndt_mb_y': ndt_mb_y,
        'fused_mb_x': fused_mb_x, 'fused_mb_y': fused_mb_y,
        'odom_mb_x': odom_mb_x, 'odom_mb_y': odom_mb_y,
        'fused_mo_x': fused_mo['x'], 'fused_mo_y': fused_mo['y'],
        'ob_x': d['ob_x'], 'ob_y': d['ob_y'],
    })

print(f"模拟完成: {len(fusion_results)} 帧")
state_counts = defaultdict(int)
for r in fusion_results:
    state_counts[r['state']] += 1
print(f"  状态分布: {dict(state_counts)}")

# ═══════════════════════════════════════════════════════════════
# 逐路点分析
# ═══════════════════════════════════════════════════════════════
print("\n" + "="*110)
print("逐路点对比: NDT-only vs FUSION vs ODOM-ONLY")
print("="*110)

results = []
for wp in waypoints:
    wp_x, wp_y = wp['x'], wp['y']

    # 找到离路点最近的那一帧
    min_ndt_dist = float('inf')
    min_fused_dist = float('inf')
    min_odom_dist = float('inf')
    min_ndt_idx = 0
    min_fused_idx = 0
    min_odom_idx = 0
    nearby_frames = []

    for i, r in enumerate(fusion_results):
        d_ndt = math.hypot(r['ndt_mb_x']-wp_x, r['ndt_mb_y']-wp_y)
        d_fused = math.hypot(r['fused_mb_x']-wp_x, r['fused_mb_y']-wp_y)
        d_odom = math.hypot(r['odom_mb_x']-wp_x, r['odom_mb_y']-wp_y)

        if d_ndt < min_ndt_dist: min_ndt_dist = d_ndt; min_ndt_idx = i
        if d_fused < min_fused_dist: min_fused_dist = d_fused; min_fused_idx = i
        if d_odom < min_odom_dist: min_odom_dist = d_odom; min_odom_idx = i

        if d_ndt < 5.0 or d_fused < 5.0:
            nearby_frames.append(i)

    r_ndt = fusion_results[min_ndt_idx]
    r_fused = fusion_results[min_fused_idx]
    r_odom = fusion_results[min_odom_idx]

    # 找出这一帧附近的 NDT 状态
    t_window = r_ndt['ros_t']
    ndt_errors_near = [r['ndt_error'] for r in fusion_results
                       if abs(r['ros_t']-t_window) < 30]

    avg_ndt = sum(ndt_errors_near)/len(ndt_errors_near) if ndt_errors_near else float('nan')
    max_ndt = max(ndt_errors_near) if ndt_errors_near else float('nan')

    # 评估: NDT-only 是否定位到这个路点
    ndt_visited = min_ndt_dist < 3.0
    fused_visited = min_fused_dist < 3.0
    odom_visited = min_odom_dist < 3.0

    # 检查这附近是否在 DEGRADED 状态
    degraded_nearby = sum(1 for r in fusion_results
                         if abs(r['ros_t']-t_window) < 5 and r['state'] == 'DEGRADED')

    # 融合改善了多少
    improvement = min_ndt_dist - min_fused_dist  # 正值=改善
    odom_improvement = min_ndt_dist - min_odom_dist

    results.append({
        'wp': wp,
        'min_ndt_dist': min_ndt_dist,
        'min_fused_dist': min_fused_dist,
        'min_odom_dist': min_odom_dist,
        'ndt_visited': ndt_visited,
        'fused_visited': fused_visited,
        'odom_visited': odom_visited,
        'avg_ndt': avg_ndt,
        'max_ndt': max_ndt,
        'improvement': improvement,
        'odom_improvement': odom_improvement,
        'degraded_near': degraded_nearby,
        'fusion_state': r_fused['state'],
        'ndt_pos': (r_ndt['ndt_mb_x'], r_ndt['ndt_mb_y']),
        'fused_pos': (r_fused['fused_mb_x'], r_fused['fused_mb_y']),
        'odom_pos': (r_odom['odom_mb_x'], r_odom['odom_mb_y']),
        'ndt_error_at': r_ndt['ndt_error'],
    })

    icon_ndt = '✓' if ndt_visited else '✗'
    icon_fused = '✓' if fused_visited else '✗'
    icon_odom = '✓' if odom_visited else '✗'

    print(f"  {wp['name']:<6} ({wp['id']}): "
          f"NDT={icon_ndt} dist={min_ndt_dist:.1f}m | "
          f"FUSION={icon_fused} dist={min_fused_dist:.1f}m "
          f"({'+{:.1f}'.format(improvement) if improvement > 1 else '{:.1f}'.format(improvement)}m) | "
          f"ODOM={icon_odom} dist={min_odom_dist:.1f}m | "
          f"NDT_err=[{avg_ndt:.2f}~{max_ndt:.2f}] "
          f"{'| DEGRADED!' if degraded_nearby > 0 else ''}"
          f"{'| FUSION SAVED!' if not ndt_visited and fused_visited else ''}"
    )

# ═══════════════════════════════════════════════════════════════
# 统计摘要
# ═══════════════════════════════════════════════════════════════
print("\n" + "="*110)
print("统计摘要")
print("="*110)

ndt_visited_count = sum(1 for r in results if r['ndt_visited'])
fused_visited_count = sum(1 for r in results if r['fused_visited'])
odom_visited_count = sum(1 for r in results if r['odom_visited'])
fusion_saved_count = sum(1 for r in results if not r['ndt_visited'] and r['fused_visited'])
odom_saved_count = sum(1 for r in results if not r['ndt_visited'] and r['odom_visited'])
degraded_count = sum(1 for r in results if r['degraded_near'] > 0)
degraded_and_saved = sum(1 for r in results
                         if r['degraded_near'] > 0 and not r['ndt_visited'] and r['fused_visited'])

print(f"  总路点数: {len(waypoints)}")
print(f"  NDT-only 访问数:  {ndt_visited_count}/{len(waypoints)}")
print(f"  FUSION 访问数:    {fused_visited_count}/{len(waypoints)}")
print(f"  ODOM-only 访问数: {odom_visited_count}/{len(waypoints)}")
print(f"")
print(f"  融合节点挽救的路点: {fusion_saved_count} (NDT 漂移但融合保持正确)")
print(f"  纯里程计挽救的路点: {odom_saved_count}")
print(f"  DEGRADED 状态触发: {degraded_count} 个路点附近")
print(f"  DEGRADED 且被挽救: {degraded_and_saved} 个路点")

# 漂移路点
drifted_wps = [r for r in results if r['max_ndt'] > 0.5 and r['max_ndt'] < 9]
severely_drifted_wps = [r for r in results if r['max_ndt'] >= 9]

if drifted_wps or severely_drifted_wps:
    print(f"\n  漂移路点 (NDT>0.5): {len(drifted_wps)}/{len(results)}")
    print(f"  严重漂移 (NDT>9.0): {len(severely_drifted_wps)}/{len(results)}")

# ═══════════════════════════════════════════════════════════════
# 漂移路点详情
# ═══════════════════════════════════════════════════════════════
print("\n" + "="*110)
print("漂移路点详情 (NDT error > 0.5)")
print("="*110)

for r in results:
    if r['max_ndt'] > 0.5:
        wp = r['wp']
        saved = not r['ndt_visited'] and r['fused_visited']
        print(f"  {wp['name']} ({wp['id']}) [{wp['x']:.1f},{wp['y']:.1f}]:")
        print(f"    NDT error: {r['avg_ndt']:.2f}~{r['max_ndt']:.2f}")
        print(f"    NDT位置:   ({r['ndt_pos'][0]:.1f},{r['ndt_pos'][1]:.1f}) dist={r['min_ndt_dist']:.1f}m")
        print(f"    FUSED位置: ({r['fused_pos'][0]:.1f},{r['fused_pos'][1]:.1f}) dist={r['min_fused_dist']:.1f}m")
        print(f"    ODOM位置:  ({r['odom_pos'][0]:.1f},{r['odom_pos'][1]:.1f}) dist={r['min_odom_dist']:.1f}m")
        print(f"    改善: {r['improvement']:+.1f}m | 状态: {r['fusion_state']} | "
              f"{'★ 融合挽救!' if saved else ''}")

# ═══════════════════════════════════════════════════════════════
# 轨迹可视化数据
# ═══════════════════════════════════════════════════════════════
print("\n" + "="*110)
print("轨迹统计")
print("="*110)

# 计算总路径长度
def path_length(points):
    total = 0.0
    prev = None
    for x, y in points:
        if prev:
            d = math.hypot(x-prev[0], y-prev[1])
            if d < 20:  # 过滤跳变
                total += d
        prev = (x, y)
    return total

ndt_traj = [(r['ndt_mb_x'], r['ndt_mb_y']) for r in fusion_results]
fused_traj = [(r['fused_mb_x'], r['fused_mb_y']) for r in fusion_results]
odom_traj = [(r['odom_mb_x'], r['odom_mb_y']) for r in fusion_results]

# 只计算 HEALTHY 段的路径(排除跳变)
healthy_ndt = [(r['ndt_mb_x'], r['ndt_mb_y']) for r in fusion_results if r['state'] == 'HEALTHY']
healthy_fused = [(r['fused_mb_x'], r['fused_mb_y']) for r in fusion_results if r['state'] == 'HEALTHY']

ndt_len = path_length(healthy_ndt) if healthy_ndt else path_length(ndt_traj)
fused_len = path_length(fused_traj)
odom_len = path_length(odom_traj)

print(f"  NDT 路径长度:   {ndt_len:.1f}m")
print(f"  FUSED 路径长度: {fused_len:.1f}m")
print(f"  ODOM 路径长度:  {odom_len:.1f}m")

# NDT 跳变统计
ndt_jumps = 0
prev_x, prev_y = None, None
for r in fusion_results:
    x, y = r['ndt_mb_x'], r['ndt_mb_y']
    if prev_x is not None:
        d = math.hypot(x-prev_x, y-prev_y)
        if d > 5:
            ndt_jumps += 1
    prev_x, prev_y = x, y
print(f"  NDT 跳变次数 (>5m): {ndt_jumps}")

# 融合后跳变统计
fused_jumps = 0
prev_x, prev_y = None, None
for r in fusion_results:
    x, y = r['fused_mb_x'], r['fused_mb_y']
    if prev_x is not None:
        d = math.hypot(x-prev_x, y-prev_y)
        if d > 5:
            fused_jumps += 1
    prev_x, prev_y = x, y
print(f"  FUSED 跳变次数 (>5m): {fused_jumps}")

# ═══════════════════════════════════════════════════════════════
# 详细数据表
# ═══════════════════════════════════════════════════════════════
print("\n" + "="*110)
print("详细数据表")
print("="*110)
hdr = (f"{'WP':<6} {'ID':<5} {'WP_pos':<16} "
       f"{'NDT_dist':<9} {'FUSED_dist':<10} {'ODOM_dist':<10} "
       f"{'NDT_err':<9} {'Fusion':<11} {'NDT_visited':<12} {'FUSED_visited':<13}")
print(hdr)
print("-"*110)

for r in results:
    wp = r['wp']
    wp_pos = f"({wp['x']:.1f},{wp['y']:.1f})"
    print(f"{wp['name']:<6} {wp['id']:<5} {wp_pos:<16} "
          f"{r['min_ndt_dist']:<9.2f} {r['min_fused_dist']:<10.2f} {r['min_odom_dist']:<10.2f} "
          f"{r['avg_ndt']:<9.3f} {r['fusion_state']:<11} "
          f"{'YES' if r['ndt_visited'] else 'NO':<12} "
          f"{'YES' if r['fused_visited'] else 'NO':<13}")

# ═══════════════════════════════════════════════════════════════
# 结论
# ═══════════════════════════════════════════════════════════════
print("\n" + "="*110)
print("结论")
print("="*110)

# 计算融合改善
improvements = [r['improvement'] for r in results if r['improvement'] > 0]
avg_improvement = sum(improvements)/len(improvements) if improvements else 0

print(f"""
1. 定位精度:
   - NDT-only: {ndt_visited_count}/{len(waypoints)} 路点成功 ({(ndt_visited_count/len(waypoints)*100):.0f}%)
   - +融合节点: {fused_visited_count}/{len(waypoints)} 路点成功 ({(fused_visited_count/len(waypoints)*100):.0f}%)
   - 纯里程计: {odom_visited_count}/{len(waypoints)} 路点成功 ({(odom_visited_count/len(waypoints)*100):.0f}%)

2. 融合节点效果:
   - 挽救路点数: {fusion_saved_count} 个 (NDT 漂移但融合保持正确)
   - DEGRADED 触发次数: {degraded_count} 次
   - DEGRADED 且挽救: {degraded_and_saved} 次
   - 平均改善距离: {avg_improvement:.1f}m
   - NDT 跳变 (>5m): {ndt_jumps} 次 → 融合后: {fused_jumps} 次

3. 里程计可靠性:
   - Odom 轨迹平滑度: {'✓ 高' if odom_len < fused_len*1.5 else '⚠ 有漂移'}
   - Odom 作为 backup 可行性: {'✓ 可行' if fusion_saved_count >= degraded_and_saved else '⚠ 需结合 NDT'}
""")

# 建议
if fusion_saved_count > 0:
    print("4. 建议: 融合节点有效，建议部署。")
    if degraded_and_saved > 0:
        print(f"   在 {degraded_and_saved} 个路点附近，NDT 漂移但融合节点保持了正确位姿。")
else:
    print("4. 建议: 此 bag 中 NDT 漂移后融合节点未能完全挽救，建议进一步调参。")

if odom_saved_count > fusion_saved_count:
    print("5. 注意: 纯里程计在这条路径上表现优于融合节点。")
    print("   可以考虑在 DEGRADED 时完全信任里程计（冻结 map->odom 不变）。")
elif odom_len < ndt_len * 1.2:
    print("5. Odom 轨迹与 NDT 轨迹一致，里程计短期精度可靠。")

print(f"\n总耗时: {time.time()-t0:.0f}s")
