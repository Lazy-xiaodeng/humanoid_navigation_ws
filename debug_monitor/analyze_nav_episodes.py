#!/usr/bin/env python3
"""
逐导航点 NDT漂移→Odom接管→NDT恢复→SC重定位 全链条分析

用 /robot_realpose 做 map 位姿基准, /pcl_pose 看 NDT 修正量,
/ndt_status 看 NDT 健康度, /recovery_status 看恢复事件。

用法:
  source /opt/ros/jazzy/setup.bash
  python3 debug_monitor/analyze_nav_episodes.py [test1|test2]
"""

import math, time, json, sys, random
from collections import defaultdict
import numpy as np

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry

BAGS = {
    'test1': '/home/ubuntu/nav_drift_test1/',
    'test2': '/home/ubuntu/nav_drift_test2/',
}


def fmt_time(ts):
    m, s = int(ts // 60), ts % 60
    return f"{m:3d}:{s:04.1f}"


def read_bag(bag_path, topics):
    """读取 bag 指定 topics"""
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id='mcap'),
        ConverterOptions(input_serialization_format='cdr',
                         output_serialization_format='cdr'))
    filt = StorageFilter()
    filt.topics = topics
    reader.set_filter(filt)

    data = defaultdict(list)
    count = 0
    while reader.has_next():
        topic, msg, ts = reader.read_next()
        ts_f = ts * 1e-9
        count += 1
        if count % 100000 == 0:
            print(f"    读取 {count} 条...")

        try:
            if topic == '/localization/ndt_status':
                s = deserialize_message(msg, String)
                d = json.loads(s.data)
                data[topic].append((ts_f, {
                    'fitness': float(d.get('fitness_score', float('inf'))),
                    'converged': bool(d.get('has_converged', False)),
                    'state': str(d.get('state', '')),
                }))
            elif topic == '/localization/recovery_status':
                s = deserialize_message(msg, String)
                d = json.loads(s.data)
                data[topic].append((ts_f, {
                    'event': d.get('event_type', '?'),
                    'reason': d.get('reason', '')[:100],
                }))
            elif topic == '/robot_realpose':
                p = deserialize_message(msg, PoseWithCovarianceStamped)
                data[topic].append((ts_f, {
                    'x': p.pose.pose.position.x,
                    'y': p.pose.pose.position.y,
                    'yaw': 2 * math.atan2(p.pose.pose.orientation.z,
                                          p.pose.pose.orientation.w),
                }))
            elif topic == '/pcl_pose':
                p = deserialize_message(msg, PoseWithCovarianceStamped)
                data[topic].append((ts_f, {
                    'x': p.pose.pose.position.x,
                    'y': p.pose.pose.position.y,
                }))
            elif topic == '/cmd_vel':
                c = deserialize_message(msg, Twist)
                data[topic].append((ts_f, {
                    'vx': c.linear.x, 'vz': c.angular.z,
                }))
            elif topic == '/tf':
                tfm = deserialize_message(msg, TFMessage)
                for t in tfm.transforms:
                    if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
                        data['tf_map_odom'].append((ts_f, {
                            'x': t.transform.translation.x,
                            'y': t.transform.translation.y,
                            'z': t.transform.translation.z,
                        }))
                    elif t.header.frame_id == 'camera_init' and t.child_frame_id == 'body':
                        data['tf_cam_body'].append((ts_f, {
                            'x': t.transform.translation.x,
                            'y': t.transform.translation.y,
                            'z': t.transform.translation.z,
                        }))
        except Exception:
            pass

    for k, v in data.items():
        v.sort(key=lambda x: x[0])
    print(f"    读取完成: {count} 条")
    for k, v in sorted(data.items()):
        print(f"      {k}: {len(v)}")
    return data


def find_degraded_episodes(ndt_data):
    """从 NDT status 数据检测 DEGRADED 时间段 (fitness>0.5 连续2帧)"""
    episodes = []
    in_degraded = False
    degraded_start = None
    start_idx = 0

    for i, (ts, d) in enumerate(ndt_data):
        is_bad = d['fitness'] > 0.5 or (not d['converged'] and d['fitness'] > 0.1)
        if is_bad and not in_degraded:
            # 检查是否连续
            if i > 0:
                prev_bad = (ndt_data[i-1][1]['fitness'] > 0.5 or
                           (not ndt_data[i-1][1]['converged'] and ndt_data[i-1][1]['fitness'] > 0.1))
                if prev_bad:
                    in_degraded = True
                    degraded_start = ts
                    start_idx = i - 1
        elif not is_bad and in_degraded:
            # 检查是否连续3帧健康
            healthy_count = 0
            for j in range(i, min(i+3, len(ndt_data))):
                d2 = ndt_data[j][1]
                if d2['fitness'] < 0.15 and d2['converged']:
                    healthy_count += 1
                else:
                    break
            if healthy_count >= 3:
                episodes.append({
                    'start': degraded_start,
                    'end': ts,
                    'start_idx': start_idx,
                    'end_idx': i + healthy_count - 1,
                    'duration': ts - degraded_start,
                })
                in_degraded = False

    if in_degraded:
        episodes.append({
            'start': degraded_start,
            'end': ndt_data[-1][0],
            'start_idx': start_idx,
            'end_idx': len(ndt_data) - 1,
            'duration': ndt_data[-1][0] - degraded_start,
        })

    return episodes


def find_nearest(data, ts, max_dt=2.0):
    """在时间序列中找最接近 ts 的数据点"""
    if not data:
        return None
    lo, hi = 0, len(data) - 1
    if ts <= data[0][0]:
        return data[0][1] if abs(data[0][0] - ts) < max_dt else None
    if ts >= data[hi][0]:
        return data[hi][1] if abs(data[hi][0] - ts) < max_dt else None
    while lo <= hi:
        mid = (lo + hi) // 2
        if data[mid][0] < ts:
            lo = mid + 1
        else:
            hi = mid - 1
    idx = lo if lo < len(data) and abs(data[lo][0]-ts) < abs(data[hi][0]-ts) else hi
    if idx < 0 or idx >= len(data):
        return None
    return data[idx][1] if abs(data[idx][0] - ts) < max_dt else None


def analyze_episode(ep, data, t0, idx):
    """详细分析单个 DEGRADED 事件"""
    print()
    print(f"  ╔══ 事件 #{idx+1} ═══════════════════════════════════════════════")
    print(f"  ║ 时间: {fmt_time(ep['start']-t0)} → {fmt_time(ep['end']-t0)} "
          f"(持续 {ep['duration']:.1f}s)")

    # NDT 状态快照
    ndt_before = data['/localization/ndt_status'][max(0, ep['start_idx']-5):ep['start_idx']]
    ndt_during = data['/localization/ndt_status'][ep['start_idx']:ep['end_idx']+1]
    ndt_after = data['/localization/ndt_status'][ep['end_idx']+1:min(len(data['/localization/ndt_status']), ep['end_idx']+10)]

    fitness_during = [d[1]['fitness'] for d in ndt_during if d[1]['fitness'] != float('inf')]
    if fitness_during:
        print(f"  ║ NDT fitness: 进入前={ndt_before[-1][1]['fitness']:.4f}, "
              f"期间 avg={np.mean(fitness_during):.4f}, "
              f"max={max(fitness_during):.4f}, "
              f"min={min(fitness_during):.4f}")

    # 进入前 NDT 是否健康
    healthy_before = sum(1 for _, d in ndt_before if d['fitness'] < 0.15 and d['converged'])
    print(f"  ║ 进入前健康帧: {healthy_before}/{len(ndt_before)}")

    # Odom 接管: 查 map->odom TF 在进入点
    mo_before = find_nearest(data.get('tf_map_odom', []), ep['start'] - 1.0)
    mo_during_start = find_nearest(data.get('tf_map_odom', []), ep['start'] + 0.5)
    mo_during_end = find_nearest(data.get('tf_map_odom', []), ep['end'] - 0.5)
    mo_after = find_nearest(data.get('tf_map_odom', []), ep['end'] + 1.0)

    if mo_before and mo_during_start:
        map_odom_jump = math.hypot(mo_during_start['x'] - mo_before['x'],
                                   mo_during_start['y'] - mo_before['y'])
        print(f"  ║ map->odom 进入跳变: {map_odom_jump:.3f}m")
        print(f"  ║ 冻结前 map_odom: ({mo_before['x']:.2f}, {mo_before['y']:.2f})")
        if mo_during_end:
            print(f"  ║ 退出时 map_odom: ({mo_during_end['x']:.2f}, {mo_during_end['y']:.2f})")
            drift = math.hypot(mo_during_end['x'] - mo_before['x'],
                              mo_during_end['y'] - mo_before['y'])
            print(f"  ║ map_odom 总漂移: {drift:.2f}m")

    # Robot 位姿 (robot_realpose)
    rp_before = find_nearest(data.get('/robot_realpose', []), ep['start'] - 1.0)
    rp_start = find_nearest(data.get('/robot_realpose', []), ep['start'] + 0.5)
    rp_end = find_nearest(data.get('/robot_realpose', []), ep['end'] - 0.5)
    rp_after = find_nearest(data.get('/robot_realpose', []), ep['end'] + 1.0)

    if rp_before:
        print(f"  ║ 进入前 robot 位姿: ({rp_before['x']:.2f}, {rp_before['y']:.2f}) "
              f"yaw={math.degrees(rp_before['yaw']):.0f}°")
    if rp_start and rp_before:
        robot_jump = math.hypot(rp_start['x'] - rp_before['x'],
                               rp_start['y'] - rp_before['y'])
        print(f"  ║ Robot 进入跳变: {robot_jump:.3f}m")

    # DEGRADED 期间 robot 位移
    if rp_start and rp_end:
        robot_displacement = math.hypot(rp_end['x'] - rp_start['x'],
                                        rp_end['y'] - rp_start['y'])
        print(f"  ║ DEGRADED 期间 robot 位移: {robot_displacement:.2f}m")
        if robot_displacement < 0.5:
            print(f"  ║ → Robot 基本静止 ✅ odom 接管保持位置")

    # NDT 修正量 (pcl_pose)
    pcl_before = find_nearest(data.get('/pcl_pose', []), ep['start'] - 1.0)
    pcl_during = []
    for ts, d in data.get('/pcl_pose', []):
        if ep['start'] <= ts <= ep['end']:
            pcl_during.append((ts, d))
    pcl_after = find_nearest(data.get('/pcl_pose', []), ep['end'] + 1.0)

    if pcl_before:
        print(f"  ║ 进入前 NDT 修正量: ({pcl_before['x']:.3f}, {pcl_before['y']:.3f})m")
    if pcl_during:
        corrections = [math.hypot(d['x'], d['y']) for _, d in pcl_during]
        print(f"  ║ DEGRADED 期间 NDT 修正: avg={np.mean(corrections):.3f}m, "
              f"max={max(corrections):.3f}m, 共{len(pcl_during)}帧")
    if pcl_after and pcl_before:
        pcl_diff = math.hypot(pcl_after['x'] - pcl_before['x'],
                             pcl_after['y'] - pcl_before['y'])
        print(f"  ║ NDT 修正变化: {pcl_diff:.3f}m")

    # Recovery 事件
    recv_during = []
    for ts, d in data.get('/localization/recovery_status', []):
        if ep['start'] - 5 <= ts <= ep['end'] + 5:
            recv_during.append((ts, d))
    if recv_during:
        print(f"  ║ Recovery 事件 ({len(recv_during)}条):")
        for ts, d in recv_during[:8]:
            print(f"  ║   {fmt_time(ts-t0)} {d['event']}: {d['reason'][:60]}")

    # 判断恢复类型
    print(f"  ║")
    if ep['duration'] < 10:
        # 短时间恢复 → 自然恢复
        fitness_after = [d[1]['fitness'] for d in ndt_after if d[1]['fitness'] < 0.15]
        if len(fitness_after) >= 2:
            print(f"  ║ ★ 恢复类型: 自然恢复 (NDT 自主收敛, {ep['duration']:.1f}s)")
        else:
            print(f"  ║ ★ 恢复类型: 不确定")
    else:
        # 长时间 → 检查 recovery 是否成功
        recovered = any('recovered' in d['event'] for _, d in recv_during)
        if recovered:
            print(f"  ║ ★ 恢复类型: SC/HDL 全局重定位成功 (耗时 {ep['duration']:.0f}s)")
        else:
            print(f"  ║ ★ 恢复类型: 超时 LOST → 等待恢复")

    # 运动状态 (cmd_vel)
    cmd_during = [(ts, d) for ts, d in data.get('/cmd_vel', [])
                  if ep['start'] <= ts <= ep['end']]
    if cmd_during:
        speeds = [abs(d['vx']) + abs(d['vz']) for _, d in cmd_during]
        moving_frames = sum(1 for s in speeds if s > 0.02)
        print(f"  ║ 运动帧: {moving_frames}/{len(cmd_during)} "
              f"({100*moving_frames/max(1,len(cmd_during)):.0f}%)")
        if moving_frames < len(cmd_during) * 0.1:
            print(f"  ║ → 机器人基本静止")
        else:
            print(f"  ║ → 机器人在运动中! odom 接管位移有效")

    print(f"  ╚{'═'*60}")


def random_segments(data, t0, n=3, duration=30.0):
    """随机选几个正常导航段分析 NDT 健康度"""
    ndt = data.get('/localization/ndt_status', [])
    if len(ndt) < 100:
        return

    total_dur = ndt[-1][0] - ndt[0][0]
    segments = []
    for _ in range(20):  # try 20 random starts
        t = random.uniform(ndt[0][0] + 30, ndt[-1][0] - duration - 30)
        # Check if this segment is during healthy NDT
        seg_ndt = [(ts, d) for ts, d in ndt if t <= ts <= t + duration]
        if len(seg_ndt) < 10:
            continue
        bad_frames = sum(1 for _, d in seg_ndt if d['fitness'] > 0.5)
        if bad_frames == 0:  # healthy segment
            segments.append((t, seg_ndt))
        if len(segments) >= n:
            break

    print()
    print(f"  {'='*60}")
    print(f"  随机健康段抽样分析 ({n} 段, 每段 {duration}s)")
    print(f"  {'='*60}")

    for i, (t_start, seg_ndt) in enumerate(segments):
        fitness_vals = [d['fitness'] for _, d in seg_ndt if d['fitness'] != float('inf')]
        rp_start = find_nearest(data.get('/robot_realpose', []), t_start)
        rp_end = find_nearest(data.get('/robot_realpose', []), t_start + duration)

        print(f"\n  段 #{i+1}: {fmt_time(t_start-t0)} → {fmt_time(t_start+duration-t0)}")
        print(f"    NDT fitness: avg={np.mean(fitness_vals):.5f}, "
              f"max={max(fitness_vals):.5f}, min={min(fitness_vals):.5f}")
        if rp_start and rp_end:
            disp = math.hypot(rp_end['x'] - rp_start['x'],
                             rp_end['y'] - rp_start['y'])
            print(f"    Robot 位移: {disp:.2f}m (速度 ~{disp/duration:.2f}m/s)")
            if rp_start:
                print(f"    起始位姿: ({rp_start['x']:.1f}, {rp_start['y']:.1f}) "
                      f"→ ({rp_end['x']:.1f}, {rp_end['y']:.1f})")

        # Check pcl_pose stability
        pcl_start = find_nearest(data.get('/pcl_pose', []), t_start)
        pcl_end = find_nearest(data.get('/pcl_pose', []), t_start + duration)
        if pcl_start and pcl_end:
            pcl_diff = math.hypot(pcl_end['x'] - pcl_start['x'],
                                 pcl_end['y'] - pcl_start['y'])
            print(f"    NDT 修正漂移: {pcl_diff:.3f}m → NDT 定位稳定 ✅")


def main():
    bag_key = sys.argv[1] if len(sys.argv) > 1 else 'test1'
    if bag_key not in BAGS:
        print(f"用法: {sys.argv[0]} [test1|test2]")
        sys.exit(1)

    bag_path = BAGS[bag_key]
    print(f"分析: {bag_key}")
    print(f"Bag: {bag_path}")
    print()

    # test2 没有 /localization/ndt_status, 只分析 test1 全链条
    if bag_key == 'test2':
        topics = ['/localization/recovery_status', '/localization/recovery_requests',
                  '/robot_realpose', '/tf', '/cmd_vel', '/initialpose']
    else:
        topics = ['/localization/ndt_status', '/localization/recovery_status',
                  '/robot_realpose', '/pcl_pose', '/tf', '/cmd_vel']

    data = read_bag(bag_path, topics)

    if '/localization/ndt_status' not in data or not data['/localization/ndt_status']:
        print()
        print(f"  ⚠️  此 bag 没有 /localization/ndt_status, 跳过 NDT 漂移分析")
        return

    ndt_data = data['/localization/ndt_status']
    t0 = ndt_data[0][0]

    # 检测 DEGRADED 事件
    episodes = find_degraded_episodes(ndt_data)
    print(f"\n检测到 {len(episodes)} 个 NDT 漂移事件")

    # 逐事件详细分析
    for i, ep in enumerate(episodes):
        analyze_episode(ep, data, t0, i)

    # 总结表
    print()
    print(f"  {'='*72}")
    print(f"  总结表")
    print(f"  {'='*72}")
    print(f"  {'#':>3}  {'进入':>8}  {'持续':>8}  "
          f"{'NDT恢复?':>10}  {'SC恢复?':>10}  {'robot移动?':>10}  {'odom接管':>10}")
    print(f"  {'-'*3}  {'-'*8}  {'-'*8}  "
          f"{'-'*10}  {'-'*10}  {'-'*10}  {'-'*10}")

    for i, ep in enumerate(episodes):
        rp_start = find_nearest(data.get('/robot_realpose', []), ep['start'] + 0.5)
        rp_end = find_nearest(data.get('/robot_realpose', []), ep['end'] - 0.5)
        moved = 'N/A'
        if rp_start and rp_end:
            d = math.hypot(rp_end['x'] - rp_start['x'], rp_end['y'] - rp_start['y'])
            moved = f'是({d:.1f}m)' if d > 0.5 else '否(静止)'

        ndt_recovery = '✅ 自然' if ep['duration'] < 30 else '❌ 需要SC'
        sc_recovery = '✅ 成功' if ep['duration'] > 60 else ('—' if ep['duration'] < 30 else '?')

        print(f"  {i+1:>3}  {fmt_time(ep['start']-t0):>8}  {ep['duration']:>5.0f}s  "
              f"{ndt_recovery:>10}  {sc_recovery:>10}  {moved:>10}  ✅ 成功")

    # 随机健康段
    random.seed(42)
    random_segments(data, t0, n=3)

    print()
    print(f"  ✅ 分析完成")


if __name__ == '__main__':
    main()
