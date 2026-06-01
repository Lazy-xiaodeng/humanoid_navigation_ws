#!/usr/bin/env python3
"""
Bag 离线验证 — 逐导航点 NDT 漂移 / Odom 接管 / SC 重定位分析

用法:
  source /opt/ros/jazzy/setup.bash
  python3 debug_monitor/validate_fusion_fix.py [nav_drift_test1|nav_drift_test2]
"""

import math, time, json, sys, os
from collections import defaultdict

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


BAGS = {
    'test1': '/home/ubuntu/nav_drift_test1/',
    'test2': '/home/ubuntu/nav_drift_test2/',
}


class Episode:
    """单次 DEGRADED/LOST 事件详情"""
    def __init__(self, idx, t_start):
        self.idx = idx
        self.t_start = t_start
        self.t_end = None
        self.t_duration = 0.0
        self.entry_reason = ''
        self.exit_reason = ''
        self.frozen_map_odom = None  # (x, y)
        self.map_odom_max_drift = 0.0  # NDT 当前值 vs 冻结值的最大偏差
        self.error_trajectory = []  # [(rel_ts, fitness)]
        self.displacement_trajectory = []  # [(rel_ts, disp_m)]
        self.recovery_method = ''  # 'natural' | 'sc_recovery' | 'hdl_recovery'
        self.recovery_duration = 0.0  # LOST -> HEALTHY 的恢复耗时


class FusionSim:
    def __init__(self, bag_start_ts):
        self.state = 'HEALTHY'
        self.consec_healthy = 0
        self.consec_degraded = 0
        self.frozen_map_odom = None
        self.frozen_odom_body = None
        self.last_healthy_map_odom = None
        self.last_healthy_odom_body = None
        self.latest_ndt_map_odom = None
        self.degraded_start = 0.0
        self.total_disp = 0.0
        self.recovery_in_progress = False
        self.max_recovery_disp = 0.0

        self.events = []
        self.episodes = []
        self._current_episode = None
        self._episode_counter = 0
        self._bag_start = bag_start_ts
        self._lost_enter_time = 0.0

    def update(self, ts, fitness, converged, state, map_odom, odom_body, rec_event=None):
        if map_odom:
            self.latest_ndt_map_odom = map_odom

        # Track map_odom drift during DEGRADED
        if (self.state in ('DEGRADED', 'LOST') and self._current_episode
            and self.frozen_map_odom and self.latest_ndt_map_odom):
            drift = math.hypot(
                self.latest_ndt_map_odom[0] - self.frozen_map_odom[0],
                self.latest_ndt_map_odom[1] - self.frozen_map_odom[1])
            if drift > self._current_episode.map_odom_max_drift:
                self._current_episode.map_odom_max_drift = drift
            self._current_episode.error_trajectory.append(
                (ts - self.degraded_start, fitness))
            disp = self._disp(odom_body)
            self._current_episode.displacement_trajectory.append(
                (ts - self.degraded_start, disp))

        if rec_event == 'localization_recovery_started':
            self.recovery_in_progress = True
        elif rec_event == 'localization_relocalize_failed':
            self.recovery_in_progress = False
        elif rec_event == 'localization_recovered' and self.state == 'LOST':
            if self._current_episode:
                self._current_episode.t_end = ts
                self._current_episode.exit_reason = 'SC/HDL recovery 成功'
                self._current_episode.recovery_method = 'sc_recovery'
                self._current_episode.recovery_duration = ts - self._lost_enter_time
            self._evt(ts, 'LOST->HEALTHY', 'recovery成功')
            self._reset(); self.state = 'HEALTHY'
            return

        if self.state == 'HEALTHY':
            self._healthy(ts, fitness, converged, map_odom, odom_body)
        elif self.state == 'DEGRADED':
            self._degraded(ts, fitness, converged, map_odom, odom_body)
        elif self.state == 'LOST':
            if (self._current_episode and
                self._current_episode.t_end is None):
                self._current_episode.t_end = ts

    def _healthy(self, ts, fitness, converged, map_odom, odom_body):
        if map_odom and fitness < 0.15:
            self.last_healthy_map_odom = map_odom
            if odom_body:
                self.last_healthy_odom_body = odom_body
        if fitness > 0.5 or (not converged and fitness > 0.1):
            self.consec_degraded += 1
            if self.consec_degraded >= 2 and self.last_healthy_map_odom:
                self.state = 'DEGRADED'
                self.frozen_map_odom = self.last_healthy_map_odom
                self.frozen_odom_body = self.last_healthy_odom_body
                self.degraded_start = ts
                self.total_disp = 0.0
                self.consec_healthy = 0
                self._episode_counter += 1
                self._current_episode = Episode(self._episode_counter, ts)
                self._current_episode.entry_reason = (
                    f'NDT error={fitness:.4f} > 0.5')
                self._current_episode.frozen_map_odom = (
                    self.frozen_map_odom[0], self.frozen_map_odom[1])
                self._evt(ts, 'HEALTHY->DEGRADED',
                          f'error={fitness:.4f} '
                          f'map_odom=({self.frozen_map_odom[0]:.2f},{self.frozen_map_odom[1]:.2f})')
        else:
            self.consec_degraded = 0

    def _degraded(self, ts, fitness, converged, map_odom, odom_body):
        if map_odom:
            self.latest_ndt_map_odom = map_odom

        if fitness < 0.15 and converged:
            pose_ok = True
            if self.frozen_map_odom and self.latest_ndt_map_odom:
                dx = self.latest_ndt_map_odom[0] - self.frozen_map_odom[0]
                dy = self.latest_ndt_map_odom[1] - self.frozen_map_odom[1]
                if math.hypot(dx, dy) > 0.8:
                    pose_ok = False
            if pose_ok:
                self.consec_healthy += 1
                if self.consec_healthy >= 3:
                    if self._current_episode:
                        self._current_episode.t_end = ts
                        self._current_episode.exit_reason = (
                            f'自然恢复 (error={fitness:.4f}, 连续{self.consec_healthy}帧)')
                        self._current_episode.recovery_method = 'natural'
                    self._evt(ts, 'DEGRADED->HEALTHY',
                              f'error={fitness:.4f} 连续{self.consec_healthy}帧')
                    self._reset(); self.state = 'HEALTHY'
                    return
            else:
                self.consec_healthy = 0
        else:
            self.consec_healthy = 0

        elapsed = ts - self.degraded_start
        disp = self._disp(odom_body)
        if disp > self.total_disp:
            self.total_disp = disp
        if disp > self.max_recovery_disp:
            self.max_recovery_disp = disp

        if disp > 30.0:
            self.state = 'LOST'; self._lost_enter_time = ts
            self._evt(ts, 'DEGRADED->LOST', f'位移{disp:.1f}m>30m')
        elif elapsed > 120.0:
            self.state = 'LOST'; self._lost_enter_time = ts
            if self._current_episode:
                self._current_episode.exit_reason = f'超时 {elapsed:.0f}s > 120s (位移{disp:.1f}m)'
            self._evt(ts, 'DEGRADED->LOST', f'超时{elapsed:.0f}s>120s (位移{disp:.1f}m)')
        elif self.total_disp > 100.0:
            self.state = 'LOST'; self._lost_enter_time = ts
            self._evt(ts, 'DEGRADED->LOST', f'累计{self.total_disp:.0f}m>100m')

    def _disp(self, odom_body):
        """修复后: (x, z) = camera_init 帧水平面"""
        if not self.frozen_odom_body or not odom_body:
            return 0.0
        return math.hypot(odom_body[0]-self.frozen_odom_body[0],
                          odom_body[2]-self.frozen_odom_body[2])

    def _reset(self):
        # 保存当前 episode
        if self._current_episode is not None and self._current_episode.t_end is None:
            self._current_episode.t_end = self.degraded_start  # best effort
        if self._current_episode is not None:
            self.episodes.append(self._current_episode)
        self._current_episode = None

        self.consec_healthy = 0; self.consec_degraded = 0
        self.frozen_map_odom = None; self.frozen_odom_body = None
        self.degraded_start = 0.0; self.total_disp = 0.0
        self.recovery_in_progress = False

    def _evt(self, ts, event, detail=''):
        self.events.append({'ts': ts, 'event': event, 'detail': detail})


def fmt_time(ts):
    m, s = int(ts//60), ts%60
    return f"{m:3d}:{s:04.1f}"


def fmt_abs_time(ts):
    """格式化绝对时间戳"""
    import datetime
    dt = datetime.datetime.fromtimestamp(ts)
    return dt.strftime('%H:%M:%S')


def analyze_bag(bag_key, bag_path):
    print()
    print("=" * 72)
    print(f"   分析: {bag_key} — {bag_path}")
    print("=" * 72)
    print()

    t0 = time.monotonic()
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id='mcap'),
        ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'),
    )
    filt = StorageFilter()
    filt.topics = ['/localization/ndt_status', '/localization/recovery_status',
                   '/tf', '/tf_static']
    reader.set_filter(filt)

    tf_frames = defaultdict(list)
    tf_static = {}
    ndt_events = []
    recv_events = []

    count = 0
    while reader.has_next():
        topic, msg, ts = reader.read_next()
        ts_f = ts * 1e-9
        count += 1
        if count % 50000 == 0:
            print(f"   读取中: {count} 条 ({time.monotonic()-t0:.0f}s)...")

        if topic == '/tf':
            tfm = deserialize_message(msg, TFMessage)
            for t in tfm.transforms:
                key = (t.header.frame_id, t.child_frame_id)
                tf_frames[key].append((
                    ts_f, t.transform.translation.x, t.transform.translation.y,
                    t.transform.translation.z, t.transform.rotation.x,
                    t.transform.rotation.y, t.transform.rotation.z,
                    t.transform.rotation.w))
        elif topic == '/tf_static':
            tfm = deserialize_message(msg, TFMessage)
            for t in tfm.transforms:
                tf_static[(t.header.frame_id, t.child_frame_id)] = (
                    t.transform.translation.x, t.transform.translation.y,
                    t.transform.translation.z, t.transform.rotation.x,
                    t.transform.rotation.y, t.transform.rotation.z,
                    t.transform.rotation.w)
        elif topic == '/localization/ndt_status':
            s = deserialize_message(msg, String)
            try:
                d = json.loads(s.data)
                ndt_events.append((
                    ts_f, float(d.get('fitness_score', float('inf'))),
                    bool(d.get('has_converged', False)),
                    str(d.get('state', '')),
                ))
            except:
                pass
        elif topic == '/localization/recovery_status':
            s = deserialize_message(msg, String)
            try:
                d = json.loads(s.data)
                recv_events.append((ts_f, d.get('event_type', ''), d.get('reason', '')))
            except:
                recv_events.append((ts_f, s.data, ''))

    elapsed = time.monotonic() - t0
    print(f"读取完成: {count} 条过滤消息, {elapsed:.0f}s")
    print(f"   NDT 状态: {len(ndt_events)}, Recovery: {len(recv_events)}")
    print(f"   camera_init->body: {len(tf_frames.get(('camera_init','body'),[]))} 条")
    print(f"   map->odom: {len(tf_frames.get(('map','odom'),[]))} 条")

    if not ndt_events:
        print("❌ 无 NDT 数据")
        return

    def lookup_tf(parent, child, ts):
        frames = tf_frames.get((parent, child))
        if not frames:
            return None
        lo, hi = 0, len(frames)-1
        if ts <= frames[0][0]:
            return frames[0][1:8]
        if ts >= frames[hi][0]:
            return frames[hi][1:8]
        while lo <= hi:
            mid = (lo+hi)//2
            if frames[mid][0] < ts:
                lo = mid+1
            else:
                hi = mid-1
        idx = lo if lo < len(frames) and abs(frames[lo][0]-ts) < abs(frames[hi][0]-ts) else hi
        if idx < 0 or idx >= len(frames):
            return None
        return frames[idx][1:8]

    ndt_events.sort(key=lambda e: e[0])
    recv_events.sort(key=lambda e: e[0])
    t_start = ndt_events[0][0]

    sim = FusionSim(t_start)
    rec_idx = 0

    print()
    print("   模拟运行中...")

    step = max(1, len(ndt_events)//10)
    for i, (ts, fitness, converged, state) in enumerate(ndt_events):
        if i % step == 0:
            print(f"   进度: {100*i//len(ndt_events)}%")

        mo = lookup_tf('map', 'odom', ts)
        map_odom = (mo[0], mo[1], mo[2]) if mo else None
        cb = lookup_tf('camera_init', 'body', ts)
        odom_body = (cb[0], cb[1], cb[2]) if cb else None

        rec_evt = None
        while rec_idx < len(recv_events) and recv_events[rec_idx][0] < ts:
            rec_evt = recv_events[rec_idx][1]
            rec_idx += 1

        sim.update(ts, fitness, converged, state, map_odom, odom_body, rec_evt)

    # ── 输出 ──
    print()
    print("=" * 72)
    print("   NDT 健康度概览")
    print("=" * 72)
    fitnesses = [e[1] for e in ndt_events if e[1] != float('inf')]
    if fitnesses:
        avg_f = sum(fitnesses)/len(fitnesses)
        max_f = max(fitnesses)
        pct_degraded = sum(1 for f in fitnesses if f > 0.5) / len(fitnesses) * 100
        print(f"  平均 fitness: {avg_f:.4f}")
        print(f"  最大 fitness: {max_f:.4f}")
        print(f"  >0.5 (DEGRADED阈值) 占比: {pct_degraded:.1f}%"
              f" ({sum(1 for f in fitnesses if f > 0.5)}/{len(fitnesses)})")
        print()

    if not sim.episodes or not any(e for e in sim.episodes if e):
        print("✅ NDT 全程健康，无 DEGRADED 事件")
        return

    episodes = [e for e in sim.episodes if e is not None]
    print()
    print("=" * 72)
    print(f"   漂移事件分析 (共 {len(episodes)} 次)")
    print("=" * 72)

    for ep in episodes:
        print()
        print(f"  ┌─ 事件 #{ep.idx} ──────────────────────────────────────────────")
        print(f"  │ 进入时间: {fmt_time(ep.t_start - t_start)} "
              f"(绝对 {fmt_abs_time(ep.t_start)})")
        print(f"  │ 进入原因: {ep.entry_reason}")
        print(f"  │ 冻结 map_odom: ({ep.frozen_map_odom[0]:.2f}, "
              f"{ep.frozen_map_odom[1]:.2f})"
              f"  ← odom 成功接管 ✅")
        print(f"  │")

        if ep.t_end:
            duration = ep.t_end - ep.t_start
            print(f"  │ 退出时间: {fmt_time(ep.t_end - t_start)}")
            print(f"  │ 持续时长: {duration:.1f}s ({duration/60:.1f}min)")
            print(f"  │ 退出原因: {ep.exit_reason}")
        else:
            print(f"  │ 退出时间: (bag 结束前未退出)")

        print(f"  │")
        print(f"  │ NDT map_odom 最大漂移 (vs 冻结值): {ep.map_odom_max_drift:.2f}m")

        # Recovery method analysis
        print(f"  │ 恢复方式: ", end='')
        if ep.recovery_method == 'natural':
            print("自然恢复 (NDT 连续健康帧) ✅")
        elif ep.recovery_method == 'sc_recovery':
            rd = ep.recovery_duration
            print(f"SC/HDL 全局重定位 (耗时 {rd:.0f}s / {rd/60:.1f}min)")
        else:
            print("未恢复 (bag 结束)")

        # Displacement stats
        if ep.displacement_trajectory:
            max_d = max(d[1] for d in ep.displacement_trajectory)
            print(f"  │ 最大 odom 位移: {max_d:.1f}m")

        # Error trajectory stats
        if ep.error_trajectory:
            errors = [d[1] for d in ep.error_trajectory]
            avg_err = sum(errors)/len(errors)
            max_err = max(errors)
            print(f"  │ DEGRADED 期间 NDT error: avg={avg_err:.4f}, "
                  f"max={max_err:.4f}")
            # Check if NDT showed signs of recovery
            healthy_frames = sum(1 for e in errors if e < 0.15)
            if healthy_frames > 0:
                print(f"  │ NDT 恢复迹象: {healthy_frames}/{len(errors)} 帧 error<0.15")

        # Waypoint estimate based on map_odom
        if ep.frozen_map_odom:
            dist_from_origin = math.hypot(ep.frozen_map_odom[0],
                                          ep.frozen_map_odom[1])
            print(f"  │ odom原点距map原点: {dist_from_origin:.1f}m")
        print(f"  └──────────────────────────────────────────────────────────────")

    # ── 总结表 ──
    print()
    print("=" * 72)
    print("   逐事件总结")
    print("=" * 72)
    print(f"  {'#':>3}  {'进入':>8}  {'退出':>8}  {'持续':>8}  "
          f"{'odom接管':>8}  {'NDT恢复':>8}  {'SC重定位':>8}  {'结果':>12}")
    print(f"  {'-'*3}  {'-'*8}  {'-'*8}  {'-'*8}  "
          f"{'-'*8}  {'-'*8}  {'-'*8}  {'-'*12}")

    for ep in episodes:
        enter_t = fmt_time(ep.t_start - t_start)
        exit_t = fmt_time(ep.t_end - t_start) if ep.t_end else '未退出'
        dur = f"{ep.t_end-ep.t_start:.0f}s" if ep.t_end else 'N/A'
        odom_ok = '✅ 是' if ep.frozen_map_odom else '❌ 否'
        ndt_ok = '✅ 是' if ep.recovery_method == 'natural' else '❌ 否'
        sc_ok = ('✅ 成功' if ep.recovery_method == 'sc_recovery'
                 else ('—' if ep.recovery_method == 'natural' else '❌ 失败'))
        result = ('自然恢复' if ep.recovery_method == 'natural'
                  else ('SC恢复' if ep.recovery_method == 'sc_recovery'
                         else '未恢复'))

        print(f"  {ep.idx:>3}  {enter_t:>8}  {exit_t:>8}  {dur:>8}  "
              f"{odom_ok:>8}  {ndt_ok:>8}  {sc_ok:>8}  {result:>12}")

    print()
    print(f"  最终状态: {sim.state}")
    print(f"  最大单段 odom 位移: {sim.max_recovery_disp:.1f}m")
    print()


def main():
    bag_key = sys.argv[1] if len(sys.argv) > 1 else 'test1'
    if bag_key not in BAGS:
        print(f"用法: {sys.argv[0]} [test1|test2]")
        print(f"  可用: {list(BAGS.keys())}")
        sys.exit(1)

    bag_path = BAGS[bag_key]
    analyze_bag(bag_key, bag_path)


if __name__ == '__main__':
    main()
