#!/usr/bin/env python3
"""
ndt_drift_diagnostic.py — NDT 漂移 vs 里程计漂移 分离诊断

原理：
  - NDT 漂移: correction_translation 持续指向同方向 + mean_corr_dist 升高
    表现为 NDT 把机器人"拉"到错误位置，修正量持续且方向一致
  - 里程计漂移: correction_translation 小且方向随机, 但全局位置缓慢偏移
    特征为 NDT 不做大修正 (信任里程计)，位置误差逐渐累积

用法：
  ros2 run humanoid_navigation2 ndt_drift_diagnostic [--log-dir /path/to/logs]

诊断信号：
  ┌─────────────────────┬──────────────────────┬──────────────────────┐
  │ 信号                │ NDT 漂移             │ Odom 漂移            │
  ├─────────────────────┼──────────────────────┼──────────────────────┤
  │ correction 方向一致性│ 高 (>60%)           │ 低 (<40%)            │
  │ mean_corr_dist      │ 上升 (>1.0m)        │ 正常 (<0.5m)         │
  │ correction 幅度     │ 持续 >0.1m          │ 持续 <0.05m          │
  │ fusion 状态         │ 频繁 DEGRADED/LOST  │ 长期 HEALTHY         │
  │ 位置误差             │ 可能回正(闭环校正)   │ 单向累积不回正       │
  └─────────────────────┴──────────────────────┴──────────────────────┘
"""

import json
import math
import os
import sys
import time
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from geometry_msgs.msg import PoseWithCovarianceStamped


WINDOW = 50          # 滑动窗口帧数
MIN_CORR = 0.03      # 最小修正量 (m), 低于此值不计入方向统计
DIR_CONSISTENCY_WARN = 0.5   # 方向一致性警告阈值
DIR_CONSISTENCY_CRIT = 0.65  # 方向一致性严重阈值
MEAN_CORR_DIST_WARN = 1.0    # 平均关联距离警告阈值 (m)
MEAN_CORR_DIST_CRIT = 1.5    # 平均关联距离严重阈值 (m)
LARGE_CORR_RATIO_WARN = 0.3  # 大修正帧比例警告阈值
FUSION_DEGRADE_RATIO_WARN = 0.3  # fusion DEGRADED/LOST 时间比例警告

STATUS_INTERVAL = 5.0  # 诊断输出间隔 (秒)
LOG_FLUSH_INTERVAL = 10.0


def ts_now():
    return time.time()


def ts_full():
    return datetime.now().isoformat()


def angle_diff_deg(a_deg, b_deg):
    d = (a_deg - b_deg + 180) % 360 - 180
    return abs(d)


def vec_to_angle(x, y):
    return math.degrees(math.atan2(y, x)) % 360


class NdtDriftDiagnostic(Node):
    def __init__(self, log_dir=None):
        super().__init__('ndt_drift_diagnostic')

        # 滑动窗口
        self.corr_history = deque(maxlen=WINDOW)  # [(ts, corr_m, corr_deg, mean_corr_dist, reason, fusion_state)]
        self.fusion_state_history = deque(maxlen=WINDOW)

        # 累积统计
        self.total_frames = 0
        self.fusion_degraded_frames = 0
        self.fusion_lost_frames = 0
        self.session_start = ts_now()

        # 最新数据
        self.latest_ndt = {}
        self.latest_fusion = {}

        # 实时位姿跟踪
        self.robot_pose = None  # (x, y, yaw)
        self.robot_pose_initial = None
        self.pcl_pose = None

        # 日志
        self.log_dir = log_dir or os.path.join(os.path.expanduser('~'), 'humanoid_ws', 'logs')
        os.makedirs(self.log_dir, exist_ok=True)
        log_name = f"ndt_drift_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
        self.log_path = os.path.join(self.log_dir, log_name)
        self.log_file = open(self.log_path, 'w')

        # 订阅
        self.create_subscription(String, '/localization/ndt_status', self._on_ndt, 10)
        self.create_subscription(String, '/localization/fusion_status', self._on_fusion, 10)
        self.create_subscription(Float64, '/odom_displacement', self._on_odom_disp, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/robot_realpose', self._on_robot_pose, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/pcl_pose', self._on_pcl_pose, 10)

        # 定时器
        self.create_timer(STATUS_INTERVAL, self._print_status)
        self.create_timer(LOG_FLUSH_INTERVAL, self._flush_log)

        self.get_logger().info(f'NDT Drift Diagnostic 已启动, 日志: {self.log_path}')

        self._write_log({'ts': ts_full(), 'type': 'session_start', 'log_path': self.log_path})

    # ── Callbacks ────────────────────────────────────────────────

    def _on_ndt(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        self.latest_ndt = data

        corr_m = data.get('correction_translation', 0.0)
        corr_yaw = data.get('correction_yaw', 0.0)
        mean_corr_dist = data.get('mean_corr_dist', -1.0)
        reason = data.get('reason', '?')
        state = data.get('state', '?')
        fusion_state = data.get('fusion_state', '?')

        # 用 yaw 修正量近似方向 (NDT 修正的主要是水平方向)
        corr_deg = math.degrees(corr_yaw) % 360 if abs(corr_yaw) > 0.001 else 0.0

        self.corr_history.append({
            'ts': ts_now(),
            'corr_m': corr_m,
            'corr_yaw_deg': corr_deg,
            'mean_corr_dist': mean_corr_dist,
            'reason': reason,
            'state': state,
        })
        self.total_frames += 1

    def _on_fusion(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        prev_state = self.latest_fusion.get('state', '')
        self.latest_fusion = data

        new_state = data.get('state', '?')
        self.fusion_state_history.append({'ts': ts_now(), 'state': new_state})

        if new_state != prev_state and prev_state:
            self._write_log({'ts': ts_full(), 'type': 'fusion_state_change',
                             'from': prev_state, 'to': new_state})

        if new_state == 'DEGRADED':
            self.fusion_degraded_frames += 1
        elif new_state == 'LOST':
            self.fusion_lost_frames += 1

    def _on_odom_disp(self, msg: Float64):
        pass  # 保留以备将来扩展

    def _on_robot_pose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.degrees(2 * math.atan2(q.z, q.w))  # approximation
        self.robot_pose = (p.x, p.y, yaw)
        if self.robot_pose_initial is None:
            self.robot_pose_initial = self.robot_pose

    def _on_pcl_pose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        self.pcl_pose = (p.x, p.y, p.z)

    # ── Analysis ──────────────────────────────────────────────────

    def _analyze(self):
        if len(self.corr_history) < 10:
            return None

        recents = list(self.corr_history)
        n = len(recents)

        # 1. 方向一致性: 显著修正 (>MIN_CORR) 中, 指向主方向 ±30° 的比例
        sig_corrs = [r for r in recents if r['corr_m'] > MIN_CORR]
        if len(sig_corrs) >= 5:
            angles = [r['corr_yaw_deg'] for r in sig_corrs]
            # 用圆形均值求主导方向
            sin_sum = sum(math.sin(math.radians(a)) for a in angles)
            cos_sum = sum(math.cos(math.radians(a)) for a in angles)
            dominant_deg = math.degrees(math.atan2(sin_sum, cos_sum)) % 360

            consistent = sum(1 for a in angles if angle_diff_deg(a, dominant_deg) < 30)
            dir_consistency = consistent / len(angles)
        else:
            dir_consistency = 0.0
            dominant_deg = 0.0

        # 2. mean_corr_dist 均值和趋势
        mcds = [r['mean_corr_dist'] for r in recents if r['mean_corr_dist'] >= 0]
        avg_mcd = sum(mcds) / len(mcds) if mcds else -1.0
        # 趋势: 后一半 vs 前一半
        if len(mcds) >= 10:
            half = len(mcds) // 2
            mcd_trend = (sum(mcds[-half:]) / half) - (sum(mcds[:half]) / half)
        else:
            mcd_trend = 0.0

        # 3. 大修正帧比例
        large_corr_ratio = sum(1 for r in recents if r['corr_m'] > 0.1) / n

        # 4. Fusion 状态异常比例
        if self.fusion_state_history:
            degraded_ratio = sum(1 for r in self.fusion_state_history
                                 if r['state'] in ('DEGRADED', 'LOST')) / len(self.fusion_state_history)
        else:
            degraded_ratio = 0.0

        # 5. 最近平均修正量
        avg_corr = sum(r['corr_m'] for r in recents) / n

        # 6. 位姿漂移 (robot_realpose 位移)
        pose_drift_m = 0.0
        if self.robot_pose and self.robot_pose_initial:
            dx = self.robot_pose[0] - self.robot_pose_initial[0]
            dy = self.robot_pose[1] - self.robot_pose_initial[1]
            pose_drift_m = math.hypot(dx, dy)

        # 7. 诊断结论
        # NDT 漂移: 方向一致 + 大关联距离 + 频繁大修正
        ndt_drift_score = 0.0
        ndt_drift_signals = []
        if dir_consistency > DIR_CONSISTENCY_CRIT:
            ndt_drift_score += 3
            ndt_drift_signals.append(f'方向一致性{dir_consistency*100:.0f}% > {DIR_CONSISTENCY_CRIT*100:.0f}%')
        elif dir_consistency > DIR_CONSISTENCY_WARN:
            ndt_drift_score += 1.5
            ndt_drift_signals.append(f'方向一致性{dir_consistency*100:.0f}% > {DIR_CONSISTENCY_WARN*100:.0f}%')
        if avg_mcd > MEAN_CORR_DIST_CRIT:
            ndt_drift_score += 3
            ndt_drift_signals.append(f'mean_corr_dist={avg_mcd:.2f}m > {MEAN_CORR_DIST_CRIT}m')
        elif avg_mcd > MEAN_CORR_DIST_WARN:
            ndt_drift_score += 1.5
            ndt_drift_signals.append(f'mean_corr_dist={avg_mcd:.2f}m > {MEAN_CORR_DIST_WARN}m')
        if large_corr_ratio > LARGE_CORR_RATIO_WARN:
            ndt_drift_score += 1
            ndt_drift_signals.append(f'大修正比例{large_corr_ratio*100:.0f}% > {LARGE_CORR_RATIO_WARN*100:.0f}%')

        # Odom 漂移: 方向随机 + 低关联距离 + 小修正 BUT 位置漂移大
        odom_drift_score = 0.0
        odom_drift_signals = []
        if dir_consistency < 0.3 and pose_drift_m > 0.5:
            odom_drift_score += 2
            odom_drift_signals.append(f'方向分散({dir_consistency*100:.0f}%) + 位姿漂移{pose_drift_m:.2f}m')
        if avg_corr < 0.03 and degraded_ratio > FUSION_DEGRADE_RATIO_WARN:
            odom_drift_score += 1
            odom_drift_signals.append(f'小NDT修正但频繁degrade')

        # 综合诊断
        if ndt_drift_score >= 4:
            diagnosis = 'NDT_DRIFT'
        elif ndt_drift_score >= 2:
            diagnosis = 'NDT_DRIFT_SUSPECT'
        elif odom_drift_score >= 2:
            diagnosis = 'ODOM_DRIFT_SUSPECT'
        else:
            diagnosis = 'NORMAL'

        return {
            'diagnosis': diagnosis,
            'ndt_drift_score': round(ndt_drift_score, 1),
            'odom_drift_score': round(odom_drift_score, 1),
            'dir_consistency': round(dir_consistency, 3),
            'dominant_deg': round(dominant_deg, 1),
            'sig_corr_count': len(sig_corrs),
            'avg_mcd': round(avg_mcd, 3),
            'mcd_trend': round(mcd_trend, 3),
            'large_corr_ratio': round(large_corr_ratio, 3),
            'avg_corr': round(avg_corr, 4),
            'degraded_ratio': round(degraded_ratio, 3),
            'pose_drift_m': round(pose_drift_m, 3),
            'ndt_signals': ndt_drift_signals,
            'odom_signals': odom_drift_signals,
            'window_n': n,
        }

    # ── Output ─────────────────────────────────────────────────────

    def _print_status(self):
        r = self._analyze()
        if r is None:
            self.get_logger().info('等待数据... (至少需要10帧)')
            return

        diag = r['diagnosis']
        if diag == 'NDT_DRIFT':
            icon = '🔴'
        elif diag == 'NDT_DRIFT_SUSPECT':
            icon = '🟡'
        elif diag == 'ODOM_DRIFT_SUSPECT':
            icon = '🟠'
        else:
            icon = '🟢'

        ndt = self.latest_ndt
        fusion = self.latest_fusion

        lines = [
            f'',
            f'{icon} [{diag}] NDT漂移={r["ndt_drift_score"]} Odom漂移={r["odom_drift_score"]} '
            f'(窗口={r["window_n"]}帧)',
            f'  方向一致性: {r["dir_consistency"]*100:.1f}% '
            f'(主导={r["dominant_deg"]:.0f}°, 有效帧={r["sig_corr_count"]})',
            f'  mean_corr_dist: {r["avg_mcd"]:.3f}m ',
            f'  趋势={r["mcd_trend"]:+.3f}m',
            f'  大修正比例: {r["large_corr_ratio"]*100:.0f}%  '
            f'平均修正: {r["avg_corr"]*100:.1f}cm',
            f'  Fusion异常: {r["degraded_ratio"]*100:.0f}%  '
            f'位姿漂移: {r["pose_drift_m"]:.2f}m',
        ]

        if r['ndt_signals']:
            lines.append(f'  NDT漂移信号:')
            for s in r['ndt_signals']:
                lines.append(f'    → {s}')
        if r['odom_signals']:
            lines.append(f'  Odom漂移信号:')
            for s in r['odom_signals']:
                lines.append(f'    → {s}')

        # 实时快照
        if ndt:
            lines.append(f'  实时: score={ndt.get("fitness_score","?"):.4f} '
                         f'corr={ndt.get("correction_translation",0):.3f}m '
                         f'reason={ndt.get("reason","?")}')
        if fusion:
            lines.append(f'        fusion={fusion.get("state","?")} '
                         f'err={fusion.get("ndt_error","?"):.3f}')

        self.get_logger().info('\n'.join(lines))

        # JSONL 摘要
        if diag != 'NORMAL':
            self._write_log({'ts': ts_full(), 'type': 'diagnosis', **r})

    def _write_log(self, entry):
        try:
            self.log_file.write(json.dumps(entry, ensure_ascii=False) + '\n')
        except Exception:
            pass

    def _flush_log(self):
        try:
            self.log_file.flush()
        except Exception:
            pass

    def destroy_node(self):
        self._write_log({'ts': ts_full(), 'type': 'session_end'})
        self.log_file.close()
        super().destroy_node()


def main():
    rclpy.init(args=sys.argv)

    log_dir = None
    for i, a in enumerate(sys.argv):
        if a == '--log-dir' and i + 1 < len(sys.argv):
            log_dir = sys.argv[i + 1]

    node = NdtDriftDiagnostic(log_dir=log_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
