#!/usr/bin/env python3
"""
NDT-Fusion-SC Navigation Monitor — 实时终端仪表盘
=================================================
监控项:
  - NDT 定位状态 (score, fitness, reason, correction, pose_jump)
  - Fusion 融合状态 (HEALTHY/DEGRADED/LOST, frozen map→odom, 退化原因)
  - SC 重定位状态 (触发次数, 结果, GICP fitness)
  - 导航状态 (当前路点, 距离, 进度, 定位恢复)
  - 机器人位姿 (真实位姿, map→odom, NDT pcl_pose)
  - 实时告警 + 事件日志
  - JSONL 日志输出 (导航结束后可回放分析)

使用: python3 ndt_fusion_monitor.py [--log-dir <dir>]
"""

import json
import math
import os
import signal
import sys
import time
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64, String

from rich.layout import Layout
from rich.panel import Panel
from rich.live import Live

# ── Constants ────────────────────────────────────────────────────
NDT_SCORE_THRESHOLD = 0.3         # NDT score 阈值 (launch config)
FUSION_CORR_THRESHOLD = 0.5      # fusion 判据0b 平移阈值 (m)
FUSION_PCL_JUMP_THRESHOLD = 0.5  # fusion 判据0c /pcl_pose 跳变阈值 (m)
NDT_JUMP_THRESHOLD = 0.8         # NDT 内部 pose_jump 平移阈值 (m)
EVENT_LOG_MAX = 200
RENDER_HZ = 4

TOPIC_NDT_STATUS = '/localization/ndt_status'
TOPIC_FUSION_STATUS = '/localization/fusion_status'
TOPIC_RECOVERY_STATUS = '/localization/recovery_status'
TOPIC_RECOVERY_REQUESTS = '/localization/recovery_requests'
TOPIC_NAV_STATUS = '/navigation/status'
TOPIC_PCL_POSE = '/pcl_pose'
TOPIC_ROBOT_REALPOSE = '/robot_realpose'
TOPIC_SC_CANDIDATES = '/scancontext_global_localization/candidates'
TOPIC_SC_BEST_POSE = '/scancontext_global_localization/best_pose'
TOPIC_ODOM_DISPLACEMENT = '/localization/fusion_odom_displacement'

# ── Helpers ───────────────────────────────────────────────────────

def quat_to_yaw(qx, qy, qz, qw):
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)

def rad2deg(r):
    return r * 180.0 / math.pi

def state_style(state: str) -> str:
    s = (state or '').upper()
    if s in ('HEALTHY', 'ACCEPTED', 'OK', 'EXECUTING'):
        return 'green'
    if s in ('DEGRADED', 'TRANSITIONING', 'PAUSED', 'POSE_JUMP_CANDIDATE', 'CONFIRMING'):
        return 'yellow'
    if s in ('LOST', 'REJECTED', 'CONFIRMED_POSE_JUMP', 'RECOVERABLE_FAILED', 'NAVIGATION_FAILED'):
        return 'red'
    if s in ('INITIALIZING', 'IDLE'):
        return 'dim cyan'
    return 'white'

def state_dot(state: str) -> str:
    s = (state or '').upper()
    if s in ('HEALTHY', 'ACCEPTED', 'OK', 'EXECUTING'):
        return '[green]●[/]'
    if s in ('DEGRADED', 'TRANSITIONING', 'POSE_JUMP_CANDIDATE', 'CONFIRMING'):
        return '[yellow]◐[/]'
    if s in ('LOST', 'REJECTED', 'CONFIRMED_POSE_JUMP', 'RECOVERABLE_FAILED', 'NAVIGATION_FAILED'):
        return '[red]◌[/]'
    if s in ('INITIALIZING',):
        return '[dim cyan]◑[/]'
    return '[white]○[/]'

def fmt_3f(v):
    return f'{v:.3f}' if v is not None else 'N/A'

def fmt_2f(v):
    return f'{v:.2f}' if v is not None else 'N/A'

def ts_now():
    return datetime.now().strftime('%H:%M:%S')

def ts_full():
    return datetime.now().strftime('%Y-%m-%d %H:%M:%S')

# ── JSONL Log Writer ──────────────────────────────────────────────

class LogWriter:
    def __init__(self, log_dir: str):
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path = os.path.join(log_dir, f'ndt_fusion_monitor_{ts}.jsonl')
        self._file = open(self.path, 'w', encoding='utf-8')
        self._count = 0

    def write(self, record: dict):
        record['_seq'] = self._count
        self._count += 1
        self._file.write(json.dumps(record, ensure_ascii=False) + '\n')
        self._file.flush()

    def close(self):
        self._file.close()

# ── Monitor Node ──────────────────────────────────────────────────

class MonitorNode(Node):
    def __init__(self, log_writer: LogWriter):
        super().__init__('ndt_fusion_monitor')
        self.log = log_writer

        # ── Latest state stores ──
        self.ndt = {}             # NDT status JSON
        self.fusion = {}          # Fusion status JSON
        self.nav = {}             # Navigation status JSON
        self.recovery = {}        # Latest recovery_status event
        self.recovery_request = {}  # Latest recovery_request
        self.sc_candidates = {}   # Latest SC candidates
        self.sc_best = None       # Latest SC best_pose
        self.robot_pose = None    # /robot_realpose
        self.pcl_pose = None      # /pcl_pose (NDT map->odom raw)
        self.odom_disp = 0.0      # Odom displacement

        # ── Derived tracking ──
        self.prev_fusion_state = ''
        self.prev_ndt_reason = ''
        self.prev_pcl = None           # for /pcl_pose frame-to-frame
        self._prev_loc_rec = False
        self.sc_trigger_count = 0
        self.last_sc_trigger_ts = ''
        self.last_sc_result = ''
        self.fusion_degrade_reason = ''  # 记录上次进入 DEGRADED 的原因
        self.fusion_degrade_ndt_status = {}
        self.alerts = []               # [(ts, msg, level), ...]
        self.events = deque(maxlen=EVENT_LOG_MAX)

        # 定期日志限流计数器
        self._pose_log_seq = 0

        # ── Subscribers ──
        self._make_sub(TOPIC_NDT_STATUS, self._on_ndt)
        self._make_sub(TOPIC_FUSION_STATUS, self._on_fusion)
        self._make_sub(TOPIC_NAV_STATUS, self._on_nav)
        self._make_sub(TOPIC_RECOVERY_STATUS, self._on_recovery_status)
        self._make_sub(TOPIC_RECOVERY_REQUESTS, self._on_recovery_request)
        self._make_sub(TOPIC_SC_CANDIDATES, self._on_sc_candidates)
        self._make_sub(TOPIC_SC_BEST_POSE, self._on_sc_best_pose)
        self._make_sub(TOPIC_ROBOT_REALPOSE, self._on_robot_pose, PoseWithCovarianceStamped)
        self._make_sub(TOPIC_PCL_POSE, self._on_pcl_pose, PoseWithCovarianceStamped)
        self._make_sub(TOPIC_ODOM_DISPLACEMENT, self._on_odom_disp, Float64)

        self.get_logger().info('NDT-Fusion-SC Monitor 已启动，等待数据...')
        self._add_event('MONITOR', 'Monitor started, waiting for topics...', 'dim cyan')

    def _make_sub(self, topic, cb, msg_type=None):
        if msg_type is None:
            msg_type = String
        self.create_subscription(msg_type, topic, cb, 10)

    def _add_event(self, src: str, msg: str, color: str = 'white'):
        self.events.append((ts_now(), src, msg, color))
        self.log.write({'ts': ts_full(), 'type': 'event', 'source': src, 'message': msg})

    def _add_alert(self, msg: str, level: str = 'warn'):
        self.alerts.append((ts_full(), msg, level))
        if len(self.alerts) > 50:
            self.alerts = self.alerts[-30:]

    # ── Callbacks ─────────────────────────────────────────────────

    def _on_ndt(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        prev_reason = self.ndt.get('reason', '')
        prev_state = self.ndt.get('state', '')
        self.ndt = data

        new_reason = data.get('reason', '')
        new_state = data.get('state', '')
        corr = data.get('correction_translation', 0)

        # 只在有意义的变化时写日志 (避免高频全量)
        significant = (
            new_reason != prev_reason or
            new_state != prev_state or
            corr > FUSION_CORR_THRESHOLD or
            'jump' in str(new_reason).lower()
        )
        if significant:
            self.log.write({'ts': ts_full(), 'type': 'ndt_status', **data})

        # 检测 NDT reason 变化
        if new_reason != prev_reason and prev_reason:
            color = 'red' if 'jump' in str(new_reason).lower() else 'yellow'
            self._add_event('NDT', f'reason: {prev_reason} → {new_reason}', color)
            if 'jump' in str(new_reason).lower():
                self._add_alert(f'NDT pose_jump: {new_reason} (corr={fmt_2f(data.get("correction_translation"))}m)', 'crit')

        # 检测 score 超阈值
        score = data.get('fitness_score', 0)
        if score > NDT_SCORE_THRESHOLD:
            self._add_alert(f'NDT fitness_score {score:.4f} > {NDT_SCORE_THRESHOLD}', 'warn')

        # /pcl_pose frame-to-frame jump detection (判据0c 复现)
        corr = data.get('correction_translation', 0)
        if corr > FUSION_CORR_THRESHOLD:
            self._add_event('NDT', f'correction {corr:.3f}m > fusion threshold {FUSION_CORR_THRESHOLD}m', 'yellow')

    def _on_fusion(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        prev_state = self.fusion.get('state', '')
        self.fusion = data

        new_state = data.get('state', '')
        # 只在状态变化时写日志 (避免 30Hz 全量日志)
        if new_state != prev_state:
            self.log.write({'ts': ts_full(), 'type': 'fusion_status', **data})

        if new_state != prev_state and prev_state:
            color = state_style(new_state)
            # 记录 DEGRADED 原因
            if new_state == 'DEGRADED':
                reason_parts = []
                ndt_reason = self.ndt.get('reason', '')
                ndt_corr = self.ndt.get('correction_translation', 0)
                if ndt_reason and ndt_reason != 'ok':
                    reason_parts.append(f'NDT reason={ndt_reason}')
                if ndt_corr > FUSION_CORR_THRESHOLD:
                    reason_parts.append(f'correction={ndt_corr:.3f}m > {FUSION_CORR_THRESHOLD}m')
                if self.ndt.get('fitness_score', 0) > NDT_SCORE_THRESHOLD:
                    reason_parts.append(f'score={self.ndt.get("fitness_score", 0):.4f} > {NDT_SCORE_THRESHOLD}')
                self.fusion_degrade_reason = '; '.join(reason_parts) or 'ndt_error elevated'
                self.fusion_degrade_ndt_status = dict(self.ndt)
                frozen = data.get('frozen_map_odom', 'N/A')
                self._add_event('FUSION',
                    f'{prev_state} → {new_state} | reason: {self.fusion_degrade_reason} | frozen map→odom=({frozen})',
                    'red' if new_state == 'LOST' else 'yellow')
                self._add_alert(f'Fusion {new_state}: {self.fusion_degrade_reason}',
                    'crit' if new_state == 'LOST' else 'warn')
            elif new_state == 'HEALTHY':
                self.fusion_degrade_reason = ''
                self.fusion_degrade_ndt_status = {}
                self._add_event('FUSION', f'{prev_state} → {new_state} (recovered)', 'green')
                self._add_alert('Fusion recovered to HEALTHY', 'info')
            elif new_state == 'LOST':
                self._add_event('FUSION', f'{prev_state} → {new_state} — waiting for SC recovery', 'red')
                self._add_alert('Fusion LOST: SC recovery should be triggered', 'crit')
            else:
                self._add_event('FUSION', f'{prev_state} → {new_state}', color)

    def _on_nav(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        self.nav = data
        # 只记录有 event_type 的更新 (避免每秒都写相同的 status summary)
        if data.get('event_type'):
            self.log.write({'ts': ts_full(), 'type': 'navigation_status', **data})

        loc_rec = data.get('localization_recovery_active', False)
        if loc_rec and not self._prev_loc_rec:
            reason = data.get('localization_recovery_reason', 'unknown')
            self._add_event('NAV', f'Localization recovery ACTIVE: {reason}', 'magenta')
        self._prev_loc_rec = loc_rec

    def _on_recovery_status(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        self.recovery = data
        evt = data.get('event_type', '')
        self.log.write({'ts': ts_full(), 'type': 'recovery_status', **data})

        if 'started' in evt:
            self._add_event('RECOVERY', f'Started: {evt} (reason={data.get("reason", "")})', 'magenta')
        elif 'completed' in evt or 'recovered' in evt:
            self.last_sc_result = 'success'
            self._add_event('RECOVERY', f'Completed: {evt}', 'green')
        elif 'failed' in evt:
            self.last_sc_result = 'failed'
            self._add_event('RECOVERY', f'Failed: {evt} (reason={data.get("reason", "")})', 'red')
        elif 'published' in evt:
            self._add_event('RECOVERY', f'Initialpose published: {evt}', 'cyan')

    def _on_recovery_request(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        self.recovery_request = data
        self.sc_trigger_count += 1
        self.last_sc_trigger_ts = ts_full()
        reason = data.get('reason', 'unknown')
        self.log.write({'ts': ts_full(), 'type': 'recovery_request', **data})
        self._add_event('RECOVERY', f'SC TRIGGER #{self.sc_trigger_count}: reason={reason}', 'bright_magenta')
        self._add_alert(f'SC recovery triggered (#{self.sc_trigger_count}): {reason}', 'crit')

    def _on_sc_candidates(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        self.sc_candidates = data
        self.log.write({'ts': ts_full(), 'type': 'sc_candidates', **data})
        accepted = data.get('accepted', False)
        reason = data.get('failure_reason', '')
        fitness = data.get('gicp_fitness', None)
        if accepted:
            self._add_event('SC', f'Candidate ACCEPTED | GICP fitness={fmt_3f(fitness)}', 'green')
        else:
            self._add_event('SC', f'Candidate REJECTED: {reason} | GICP fitness={fmt_3f(fitness)}', 'red')

    def _on_sc_best_pose(self, msg: PoseWithCovarianceStamped):
        self.sc_best = msg
        p = msg.pose.pose.position
        yaw = quat_to_yaw(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                          msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.log.write({'ts': ts_full(), 'type': 'sc_best_pose',
                        'x': p.x, 'y': p.y, 'z': p.z, 'yaw_deg': rad2deg(yaw)})

    def _on_robot_pose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.robot_pose = {'x': p.x, 'y': p.y, 'z': p.z,
                           'yaw_rad': quat_to_yaw(o.x, o.y, o.z, o.w)}
        # 每 5s 记录一次 (约每 150 帧 @30Hz)
        self._pose_log_seq += 1
        if self._pose_log_seq % 150 == 0:
            self.log.write({'ts': ts_full(), 'type': 'robot_pose_snapshot',
                           'x': p.x, 'y': p.y, 'z': p.z,
                           'yaw_deg': rad2deg(self.robot_pose['yaw_rad'])})

    def _on_pcl_pose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        cur = {'x': p.x, 'y': p.y, 'z': p.z, 'qx': o.x, 'qy': o.y, 'qz': o.z, 'qw': o.w,
               'yaw_rad': quat_to_yaw(o.x, o.y, o.z, o.w)}
        # frame-to-frame jump detection
        if self.prev_pcl is not None:
            jump = math.hypot(cur['x'] - self.prev_pcl['x'], cur['y'] - self.prev_pcl['y'])
            if jump > FUSION_PCL_JUMP_THRESHOLD:
                self._add_event('PCL', f'/pcl_pose frame jump: {jump:.3f}m > {FUSION_PCL_JUMP_THRESHOLD}m', 'yellow')
        self.prev_pcl = cur
        self.pcl_pose = cur
        # 每 5s 记录一次
        if self._pose_log_seq % 150 == 0:
            self.log.write({'ts': ts_full(), 'type': 'pcl_pose_snapshot',
                           'x': p.x, 'y': p.y, 'z': p.z,
                           'yaw_deg': rad2deg(cur['yaw_rad'])})

    def _on_odom_disp(self, msg: Float64):
        self.odom_disp = msg.data

# ── Dashboard Renderer ────────────────────────────────────────────

class Dashboard:
    def __init__(self, node: MonitorNode):
        self.node = node
        self.layout = self._make_layout()
        self.start_time = time.monotonic()

    def _make_layout(self) -> Layout:
        root = Layout()
        root.split(
            Layout(name='header', size=3),
            Layout(name='body', ratio=1),
            Layout(name='footer', size=10),
        )
        root['body'].split_row(
            Layout(name='left', ratio=3),
            Layout(name='right', ratio=2),
        )
        root['left'].split(
            Layout(name='ndt', ratio=3),
            Layout(name='fusion', ratio=3),
            Layout(name='nav_panel', ratio=2),
        )
        root['right'].split(
            Layout(name='pose', ratio=3),
            Layout(name='recovery', ratio=3),
            Layout(name='alerts', ratio=2),
        )
        return root

    # ── Panel builders ────────────────────────────────────────────

    def _panel_ndt(self) -> Panel:
        d = self.node.ndt
        if not d:
            return Panel('[dim]等待 /localization/ndt_status ...[/]', title='🔬 NDT 定位状态')

        score = d.get('fitness_score', 0)
        score_color = 'green' if score <= NDT_SCORE_THRESHOLD else ('yellow' if score < 0.5 else 'red')
        reason = d.get('reason', '?')
        corr_t = d.get('correction_translation', 0)
        corr_y = d.get('correction_yaw', 0)
        corr_color = 'green' if corr_t < FUSION_CORR_THRESHOLD else ('yellow' if corr_t < NDT_JUMP_THRESHOLD else 'red')
        state = d.get('state', '?')

        lines = [
            f' Score:       [{score_color}]{score:.4f}[/]  (threshold={NDT_SCORE_THRESHOLD})',
            f' Fitness:     {fmt_3f(d.get("fitness_score"))}  (inlier_ratio={fmt_3f(d.get("has_converged", None))})',
            f' Reason:      [{state_style(reason)}]{reason}[/]',
            f' Correction:  [{corr_color}]{fmt_3f(corr_t)}m[/] / {rad2deg(corr_y):.1f}°  '
            f'[dim](fusion>{FUSION_CORR_THRESHOLD}m / NDT>{NDT_JUMP_THRESHOLD}m)[/]',
            f' State:       {state_dot(state)} {state}',
            f' Converged:   {"✓" if d.get("has_converged") else "✗"}  '
            f'JumpCand: {d.get("pose_jump_candidate_count", 0)}  '
            f'RejectFrames: {d.get("consecutive_rejected_frames", 0)}',
            f' Reacquiring: {"[yellow]YES[/]" if d.get("initialpose_reacquiring") else "no"}',
            f' Filtered pts:{d.get("filtered_points", "?")}',
        ]
        return Panel('\n'.join(lines), title='🔬 NDT 定位状态', border_style=state_style(state))

    def _panel_fusion(self) -> Panel:
        d = self.node.fusion
        if not d:
            return Panel('[dim]等待 /localization/fusion_status ...[/]', title='🛡️ Fusion 融合')

        state = d.get('state', '?')
        style = state_style(state)
        lines = [
            f' State:       {state_dot(state)} [{style}]{state}[/]',
            f' NDT Error:   {fmt_3f(d.get("ndt_error"))}  Inlier: {fmt_3f(d.get("ndt_inlier"))}',
        ]

        if state == 'DEGRADED':
            elapsed = d.get('degraded_elapsed_sec', 0)
            frozen = d.get('frozen_map_odom', 'N/A')
            lines += [
                f' Degraded:    [yellow]{elapsed:.1f}s[/]  '
                f'(锁定期30s | 总超时180s)',
                f' Frozen M→O:  [yellow bold]{frozen}[/]',
                f' Degrade为何: [yellow]{self.node.fusion_degrade_reason or "unknown"}[/]',
                f' 锁定期:      [yellow]{"锁定期内" if elapsed < 30 else "验证期"}[/] '
                f'{"🔒 拒绝NDT恢复" if elapsed < 30 else "🔍 严格验证中"}',
                f' Odom Displ:  {self.node.odom_disp:.2f}m / 30m (连续) / 100m (累计)',
            ]
        elif state == 'TRANSITIONING':
            prog = d.get('transition_progress', 0)
            lines += [
                f' Transition:  [bright_yellow]进度 {prog*100:.0f}%[/]',
            ]
        elif state == 'LOST':
            elapsed = d.get('degraded_elapsed_sec', 0)
            frozen = d.get('frozen_map_odom', 'N/A')
            lines += [
                f' Lost 时长:   [red]{elapsed:.1f}s[/]',
                f' Frozen M→O:  [red bold]{frozen}[/]',
                f' Degrade为何: [red]{self.node.fusion_degrade_reason or "unknown"}[/]',
                f' Odom Displ:  {self.node.odom_disp:.2f}m',
                f' [red blink]等待 SC 重定位恢复...[/]',
            ]
        else:
            lines.append(f' Odom Displ:  {self.node.odom_disp:.2f}m / 100m')

        return Panel('\n'.join(lines), title='🛡️ Fusion 融合', border_style=style)

    def _panel_nav(self) -> Panel:
        d = self.node.nav
        if not d:
            return Panel('[dim]等待 /navigation/status ...[/]', title='🧭 导航状态')

        state = d.get('current_state', d.get('detailed_state', '?'))
        wp_idx = d.get('current_waypoint_index', 0)
        wp_total = d.get('total_waypoints', 0)
        goal = d.get('current_goal', {})
        wp_name = goal.get('waypoint_name', '-') if goal else '-'
        dist = d.get('distance_to_goal', None)
        progress = d.get('progress_percentage', 0)
        dur = d.get('navigation_duration', 0)
        loc_rec = d.get('localization_recovery_active', False)
        loc_reason = d.get('localization_recovery_reason', '')

        lines = [
            f' State:       {state_dot(state)} {state}',
            f' Waypoint:    {wp_idx}/{wp_total} → [cyan]{wp_name}[/]',
            f' Distance:    {fmt_2f(dist)}m' if dist is not None else ' Distance:    N/A',
            f' Progress:    {progress:.1f}%  [dim]({dur:.0f}s)[/]',
        ]

        if loc_rec:
            lines.append(f' LocRecovery: [magenta]ACTIVE[/] reason={loc_reason}')
        else:
            lines.append(f' LocRecovery: 非活跃')

        recov = d.get('recovery_active', False)
        blocked = d.get('obstacle_blocked', False)
        if recov or blocked:
            flags = []
            if recov: flags.append('[yellow]RECOVERING[/]')
            if blocked: flags.append('[red]BLOCKED[/]')
            lines.append(f' Flags:       {", ".join(flags)}')

        return Panel('\n'.join(lines), title='🧭 导航状态', border_style=state_style(state))

    def _panel_pose(self) -> Panel:
        robot = self.node.robot_pose
        pcl = self.node.pcl_pose
        frozen_str = self.node.fusion.get('frozen_map_odom', '')
        lines = []

        if robot:
            yaw_d = rad2deg(robot['yaw_rad'])
            lines.append(f' 真实位姿:   ({fmt_3f(robot["x"])}, {fmt_3f(robot["y"])}, {yaw_d:.1f}°)')
        else:
            lines.append(' 真实位姿:   [dim]等待 /robot_realpose ...[/]')

        if pcl:
            yaw_d = rad2deg(pcl['yaw_rad'])
            lines.append(f' NDT PclPose:({fmt_3f(pcl["x"])}, {fmt_3f(pcl["y"])}, {yaw_d:.1f}°)')
        else:
            lines.append(' NDT PclPose:[dim]等待 /pcl_pose ...[/]')

        if frozen_str:
            lines.append(f' Frozen M→O: [yellow bold]{frozen_str}[/]')
            # 计算当前位置偏离冻结点的距离
            if robot:
                try:
                    fx, fy = frozen_str.split(',')
                    fx, fy = float(fx), float(fy)
                    dev = math.hypot(robot['x'] - fx, robot['y'] - fy)
                    dev_color = 'green' if dev < 0.5 else ('yellow' if dev < 2.0 else 'red')
                    lines.append(f' 偏离冻结:   [{dev_color}]{dev:.3f}m[/] ' +
                                 f'[dim](安全<0.5m / 警告<2.0m)[/]')
                except Exception:
                    pass
        else:
            lines.append(' Frozen M→O:  —')

        if pcl and robot:
            diff = math.hypot(pcl['x'] - robot['x'], pcl['y'] - robot['y'])
            diff_color = 'green' if diff < 0.3 else ('yellow' if diff < 1.0 else 'red')
            lines.append(f' NDT-真实偏差:[{diff_color}]{diff:.3f}m[/]')

        return Panel('\n'.join(lines), title='📍 机器人位姿')

    def _panel_recovery(self) -> Panel:
        sc = self.node.sc_candidates
        req = self.node.recovery_request
        rec = self.node.recovery
        fusion_state = self.node.fusion.get('state', '')

        # 检测 recovery 引擎类型
        has_sc = bool(sc or self.node.sc_best)
        engine = 'SC' if has_sc else 'HDL'

        lines = [
            f' 引擎:        [bold]{engine}[/] ({"SC在线" if has_sc else "HDL bootstrap"})',
            f' Recovery触发: [bold]{self.node.sc_trigger_count}[/]次',
            f' 最后触发:    {self.node.last_sc_trigger_ts or "N/A"}',
        ]

        # Recovery 请求详情
        if req:
            lines += [
                f' 请求原因:    [yellow]{req.get("reason", "?")}[/]',
                f' 搜索半径:    {req.get("search_radius_m", "?")}m',
            ]
        else:
            lines.append(' 请求:        无活跃请求')

        # SC 候选详情
        if sc:
            lines += [
                f' [cyan]SC Mode:[/]     {sc.get("mode", "?")}',
                f' [cyan]SC Accepted:[/] {"[green]YES[/]" if sc.get("accepted") else "[red]NO[/]"}',
                f' [cyan]SC GICP:[/]     {fmt_3f(sc.get("gicp_fitness"))}',
                f' [cyan]SC Failure:[/]  {sc.get("failure_reason") or "—"}',
            ]

        # Recovery 状态事件
        if rec:
            evt = rec.get('event_type', '')
            reason = rec.get('reason', '')
            mode = rec.get('relocalization_mode', '')
            attempts = rec.get('relocalize_attempts', '')
            recovery_cnt = rec.get('recovery_count', '')

            evt_color = 'green' if 'recovered' in evt or 'completed' in evt else \
                       ('red' if 'failed' in evt else 'cyan')
            lines.append(f' 恢复事件:    [{evt_color}]{evt}[/]')
            if reason:
                lines.append(f' 事件原因:    {reason}')
            if mode:
                lines.append(f' 重定位模式:  {mode}')
            if attempts:
                lines.append(f' 尝试次数:    {attempts}')
        else:
            lines.append(' 恢复事件:    [dim]等待 /recovery_status ...[/]')

        # SC best_pose
        if self.node.sc_best:
            bp = self.node.sc_best.pose.pose.position
            lines.append(f' [cyan]SC Best:[/]    ({bp.x:.3f}, {bp.y:.3f})')

        return Panel('\n'.join(lines), title='🔄 Recovery 重定位')

    def _panel_alerts(self) -> Panel:
        alerts = self.node.alerts[-8:]  # 最近 8 条
        if not alerts:
            return Panel('[dim green]无活跃告警[/]', title='⚠️ 告警')

        lines = []
        for ts, msg, level in reversed(alerts):
            c = 'red' if level == 'crit' else 'yellow'
            short_ts = ts[-15:] if len(ts) > 16 else ts
            lines.append(f' [{c}]{short_ts}[/] {msg}')

        return Panel('\n'.join(lines), title='⚠️ 告警', border_style='yellow')

    def _panel_events(self) -> Panel:
        events = list(self.node.events)[-15:]
        if not events:
            return Panel('[dim]等待事件...[/]', title='📜 事件日志')

        lines = []
        for ts, src, msg, color in events:
            src_tag = f'[{color}]{src:<8}[/]'
            lines.append(f' {ts} │ {src_tag} {msg[:110]}')

        return Panel('\n'.join(lines), title=f'📜 事件日志 (最近 15/{EVENT_LOG_MAX})')

    def _header(self) -> Panel:
        fusion = self.node.fusion
        ndt = self.node.ndt
        nav = self.node.nav

        f_state = fusion.get('state', 'INITIALIZING') if fusion else 'WAITING'
        n_reason = ndt.get('reason', '?') if ndt else '?'
        n_corr = ndt.get('correction_translation', 0) if ndt else 0
        nav_state = nav.get('current_state', '?') if nav else '?'
        wp_idx = nav.get('current_waypoint_index', 0) if nav else 0
        wp_total = nav.get('total_waypoints', 0) if nav else 0
        goal = nav.get('current_goal', {}) if nav else {}
        wp_name = goal.get('waypoint_name', '-') if goal else '-'
        dist = nav.get('distance_to_goal', None) if nav else None
        frozen = fusion.get('frozen_map_odom', '') if fusion else ''

        f_dot = state_dot(f_state)
        parts = [
            f'{f_dot} [{state_style(f_state)}]{f_state}[/]',
            f'NDT: [bold]{n_reason}[/] ({fmt_2f(n_corr)}m)',
            f'Nav: {nav_state}({wp_idx}/{wp_total} {wp_name})',
        ]
        if dist is not None:
            parts.append(f'{dist:.2f}m→')

        runtime = time.monotonic() - self.start_time
        title = f'🧭 NDT-Fusion-SC Monitor    {ts_full()}    [dim]运行 {runtime:.0f}s[/]'

        body = ' │ '.join(parts)
        if frozen:
            body += f'\n[bold yellow]⚠ Frozen map→odom: {frozen}[/]'

        return Panel(body, title=title)

    def render(self):
        self.layout['header'].update(self._header())
        self.layout['ndt'].update(self._panel_ndt())
        self.layout['fusion'].update(self._panel_fusion())
        self.layout['nav_panel'].update(self._panel_nav())
        self.layout['pose'].update(self._panel_pose())
        self.layout['recovery'].update(self._panel_recovery())
        self.layout['alerts'].update(self._panel_alerts())
        self.layout['footer'].update(self._panel_events())

# ── Main ──────────────────────────────────────────────────────────

def main(args=None):
    log_dir = None
    argv = sys.argv[1:]
    i = 0
    while i < len(argv):
        if argv[i] == '--log-dir' and i + 1 < len(argv):
            log_dir = argv[i + 1]
            i += 1
        i += 1

    if not log_dir:
        ws = os.environ.get('HUMANOID_WS', os.path.expanduser('~/humanoid_ws'))
        log_dir = os.path.join(ws, 'debug_logs')

    log_writer = LogWriter(log_dir)
    print(f'📝 日志输出: {log_writer.path}')

    rclpy.init(args=args)
    node = MonitorNode(log_writer)
    dashboard = Dashboard(node)

    running = True
    def _shutdown(sig=None, frame=None):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    exec_start = time.monotonic()
    frame_count = 0

    try:
        with Live(dashboard.layout, refresh_per_second=RENDER_HZ, screen=True) as live:
            while running and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
                frame_count += 1
                if frame_count % 2 == 0:  # 减少渲染频率
                    dashboard.render()
                    live.update(dashboard.layout)

    except KeyboardInterrupt:
        pass
    finally:
        runtime = time.monotonic() - exec_start
        node._add_event('MONITOR', f'Shutting down after {runtime:.0f}s. Log: {log_writer.path}', 'dim cyan')
        node.get_logger().info(f'📝 日志已保存到: {log_writer.path}')
        log_writer.close()
        node.destroy_node()
        rclpy.shutdown()
        print(f'\n📝 日志文件: {log_writer.path}')
        print(f'   共 {log_writer._count} 条记录')
        print('   使用以下命令分析:')
        print(f'   cat {log_writer.path} | jq .')

if __name__ == '__main__':
    main()
