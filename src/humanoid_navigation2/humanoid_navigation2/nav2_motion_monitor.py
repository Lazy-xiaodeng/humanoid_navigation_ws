#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Nav2 Motion Monitor

功能：
1. 实时显示全局规划路径 /plan
2. 实时显示机器人实际轨迹，默认使用 TF: map -> base_footprint
3. 实时显示目标点 /goal_pose
4. 实时显示 RPP 输出速度 /cmd_vel
5. 实时显示实际速度 /odom
6. 实时显示路径长度、目标误差、重规划次数等指标
7. 关闭窗口时自动保存 CSV 和 JSON 数据

适用场景：
- 检查 SmacPlannerHybrid + DUBIN + RPP 是否导致绕圈
- 检查目标点是否漂移
- 检查 /plan 是否频繁变化
- 检查 RPP 输出速度和实际速度是否一致
"""

import os
import sys
import csv
import json
import math
import time
import argparse
import threading
from datetime import datetime
from collections import deque

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from std_msgs.msg import Header

import tf2_ros
from tf2_ros import TransformException

from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

def yaw_from_quaternion(q):
    """
    四元数转 yaw。
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def dist2d(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)

def compute_path_length(points):
    """
    points: [(x, y), ...]
    """
    if len(points) < 2:
        return 0.0

    length = 0.0
    for i in range(1, len(points)):
        length += dist2d(points[i - 1][0], points[i - 1][1],
                         points[i][0], points[i][1])
    return length

class NavMonitorNode(Node):
    def __init__(self, args):
        super().__init__('nav2_motion_monitor')

        self.args = args
        self.lock = threading.Lock()

        # ------------------------------
        # 数据缓存
        # ------------------------------
        self.start_time_wall = time.time()
        self.latest_plan = []
        self.latest_plan_stamp = None
        self.plan_update_count = 0
        self.plan_update_times = deque(maxlen=100)

        self.latest_goal = None
        self.latest_goal_stamp = None

        self.latest_robot_pose = None  # (x, y, yaw)
        self.latest_robot_pose_stamp = None

        self.latest_cmd_vel = None     # (vx, wz)
        self.latest_cmd_vel_stamp = None

        self.latest_odom_vel = None    # (vx, wz)
        self.latest_odom_vel_stamp = None

        # 历史数据，用于绘图和保存
        self.robot_path = []
        self.time_series = []

        # 最新指标
        self.metrics = {
            'plan_length': 0.0,
            'plan_end_to_goal_dist': float('nan'),
            'robot_to_goal_dist': float('nan'),
            'plan_update_rate': 0.0,
            'cmd_vx': 0.0,
            'cmd_wz': 0.0,
            'odom_vx': 0.0,
            'odom_wz': 0.0,
        }

        # ------------------------------
        # TF
        # ------------------------------
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ------------------------------
        # 订阅话题
        # ------------------------------
        self.create_subscription(Path, args.plan_topic, self.plan_cb, 10)
        self.create_subscription(PoseStamped, args.goal_topic, self.goal_cb, 10)
        self.create_subscription(Odometry, args.odom_topic, self.odom_cb, 20)

        if args.cmd_vel_type == 'twist':
            self.create_subscription(Twist, args.cmd_vel_topic, self.cmd_vel_cb, 20)
            self.get_logger().info(f"订阅 RPP 输出速度 Twist: {args.cmd_vel_topic}")
        elif args.cmd_vel_type == 'twist_stamped':
            self.create_subscription(TwistStamped, args.cmd_vel_topic, self.cmd_vel_stamped_cb, 20)
            self.get_logger().info(f"订阅 RPP 输出速度 TwistStamped: {args.cmd_vel_topic}")
        else:
            raise ValueError("cmd_vel_type 必须是 twist 或 twist_stamped")

        # 定时采样 TF 和记录数据
        self.timer = self.create_timer(1.0 / args.sample_rate, self.sample_timer_cb)

        self.get_logger().info("Nav2 Motion Monitor Node started.")
        self.get_logger().info(f"plan_topic      : {args.plan_topic}")
        self.get_logger().info(f"goal_topic      : {args.goal_topic}")
        self.get_logger().info(f"odom_topic      : {args.odom_topic}")
        self.get_logger().info(f"cmd_vel_topic   : {args.cmd_vel_topic}")
        self.get_logger().info(f"global_frame    : {args.global_frame}")
        self.get_logger().info(f"base_frame      : {args.base_frame}")

    def now_sec(self):
        return time.time() - self.start_time_wall

    def plan_cb(self, msg: Path):
        points = []
        for ps in msg.poses:
            points.append((ps.pose.position.x, ps.pose.position.y))

        with self.lock:
            self.latest_plan = points
            self.latest_plan_stamp = self.now_sec()
            self.plan_update_count += 1
            self.plan_update_times.append(self.latest_plan_stamp)

            self.metrics['plan_length'] = compute_path_length(points)

            # 计算 plan 更新频率
            if len(self.plan_update_times) >= 2:
                dt = self.plan_update_times[-1] - self.plan_update_times[0]
                if dt > 1e-6:
                    self.metrics['plan_update_rate'] = (len(self.plan_update_times) - 1) / dt

            # 路径终点到目标点距离
            if points and self.latest_goal is not None:
                end_x, end_y = points[-1]
                gx, gy = self.latest_goal
                self.metrics['plan_end_to_goal_dist'] = dist2d(end_x, end_y, gx, gy)

    def goal_cb(self, msg: PoseStamped):
        gx = msg.pose.position.x
        gy = msg.pose.position.y

        with self.lock:
            self.latest_goal = (gx, gy)
            self.latest_goal_stamp = self.now_sec()

            if self.latest_plan:
                end_x, end_y = self.latest_plan[-1]
                self.metrics['plan_end_to_goal_dist'] = dist2d(end_x, end_y, gx, gy)

            if self.latest_robot_pose is not None:
                rx, ry, _ = self.latest_robot_pose
                self.metrics['robot_to_goal_dist'] = dist2d(rx, ry, gx, gy)

    def odom_cb(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z

        with self.lock:
            self.latest_odom_vel = (vx, wz)
            self.latest_odom_vel_stamp = self.now_sec()
            self.metrics['odom_vx'] = vx
            self.metrics['odom_wz'] = wz

    def cmd_vel_cb(self, msg: Twist):
        vx = msg.linear.x
        wz = msg.angular.z

        with self.lock:
            self.latest_cmd_vel = (vx, wz)
            self.latest_cmd_vel_stamp = self.now_sec()
            self.metrics['cmd_vx'] = vx
            self.metrics['cmd_wz'] = wz

    def cmd_vel_stamped_cb(self, msg: TwistStamped):
        vx = msg.twist.linear.x
        wz = msg.twist.angular.z

        with self.lock:
            self.latest_cmd_vel = (vx, wz)
            self.latest_cmd_vel_stamp = self.now_sec()
            self.metrics['cmd_vx'] = vx
            self.metrics['cmd_wz'] = wz

    def sample_timer_cb(self):
        """
        周期性从 TF 读取机器人在 global_frame 下的位置，
        并记录当前数据。
        """
        t = self.now_sec()

        # 读取 TF: global_frame -> base_frame
        robot_pose = None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.args.global_frame,
                self.args.base_frame,
                rclpy.time.Time()
            )

            x = tf.transform.translation.x
            y = tf.transform.translation.y
            yaw = yaw_from_quaternion(tf.transform.rotation)
            robot_pose = (x, y, yaw)

        except TransformException:
            # TF 暂时没有，不打印太频繁
            robot_pose = None

        with self.lock:
            if robot_pose is not None:
                self.latest_robot_pose = robot_pose
                self.latest_robot_pose_stamp = t
                self.robot_path.append((t, robot_pose[0], robot_pose[1], robot_pose[2]))

                if self.latest_goal is not None:
                    gx, gy = self.latest_goal
                    self.metrics['robot_to_goal_dist'] = dist2d(robot_pose[0], robot_pose[1], gx, gy)

            cmd_vx, cmd_wz = self.latest_cmd_vel if self.latest_cmd_vel is not None else (0.0, 0.0)
            odom_vx, odom_wz = self.latest_odom_vel if self.latest_odom_vel is not None else (0.0, 0.0)

            rx = robot_pose[0] if robot_pose is not None else float('nan')
            ry = robot_pose[1] if robot_pose is not None else float('nan')
            ryaw = robot_pose[2] if robot_pose is not None else float('nan')

            gx = self.latest_goal[0] if self.latest_goal is not None else float('nan')
            gy = self.latest_goal[1] if self.latest_goal is not None else float('nan')

            plan_start_x = float('nan')
            plan_start_y = float('nan')
            plan_end_x = float('nan')
            plan_end_y = float('nan')

            if self.latest_plan:
                plan_start_x, plan_start_y = self.latest_plan[0]
                plan_end_x, plan_end_y = self.latest_plan[-1]

            row = {
                't': t,
                'robot_x': rx,
                'robot_y': ry,
                'robot_yaw': ryaw,
                'goal_x': gx,
                'goal_y': gy,
                'cmd_vx': cmd_vx,
                'cmd_wz': cmd_wz,
                'odom_vx': odom_vx,
                'odom_wz': odom_wz,
                'plan_start_x': plan_start_x,
                'plan_start_y': plan_start_y,
                'plan_end_x': plan_end_x,
                'plan_end_y': plan_end_y,
                'plan_length': self.metrics['plan_length'],
                'plan_end_to_goal_dist': self.metrics['plan_end_to_goal_dist'],
                'robot_to_goal_dist': self.metrics['robot_to_goal_dist'],
                'plan_update_count': self.plan_update_count,
                'plan_update_rate': self.metrics['plan_update_rate'],
            }

            self.time_series.append(row)

    def get_snapshot(self):
        """
        给 GUI 线程读取数据。
        """
        with self.lock:
            return {
                'latest_plan': list(self.latest_plan),
                'robot_path': list(self.robot_path),
                'latest_goal': self.latest_goal,
                'latest_robot_pose': self.latest_robot_pose,
                'metrics': dict(self.metrics),
                'plan_update_count': self.plan_update_count,
                'time_series': list(self.time_series),
            }

    def save_data(self, output_dir):
        os.makedirs(output_dir, exist_ok=True)

        with self.lock:
            rows = list(self.time_series)
            plan = list(self.latest_plan)
            robot_path = list(self.robot_path)
            metrics = dict(self.metrics)
            goal = self.latest_goal
            plan_update_count = self.plan_update_count

        # 保存 CSV
        csv_path = os.path.join(output_dir, 'nav_monitor_data.csv')
        if rows:
            keys = list(rows[0].keys())
            with open(csv_path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=keys)
                writer.writeheader()
                writer.writerows(rows)

        # 保存最后一条路径
        plan_path = os.path.join(output_dir, 'latest_plan.csv')
        with open(plan_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['index', 'x', 'y'])
            for i, p in enumerate(plan):
                writer.writerow([i, p[0], p[1]])

        # 保存机器人轨迹
        robot_path_file = os.path.join(output_dir, 'robot_path.csv')
        with open(robot_path_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['t', 'x', 'y', 'yaw'])
            for item in robot_path:
                writer.writerow(item)

        # 保存摘要
        summary = {
            'saved_time': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'duration_sec': rows[-1]['t'] if rows else 0.0,
            'goal': {
                'x': goal[0],
                'y': goal[1],
            } if goal is not None else None,
            'final_metrics': metrics,
            'plan_update_count': plan_update_count,
            'data_files': {
                'time_series_csv': 'nav_monitor_data.csv',
                'latest_plan_csv': 'latest_plan.csv',
                'robot_path_csv': 'robot_path.csv',
            }
        }

        json_path = os.path.join(output_dir, 'summary.json')
        with open(json_path, 'w') as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)

        return output_dir

class MonitorWindow(QtWidgets.QMainWindow):
    def __init__(self, node: NavMonitorNode, args):
        super().__init__()

        self.node = node
        self.args = args

        self.setWindowTitle("Nav2 Motion Monitor - SmacPlannerHybrid + DUBIN + RPP")
        self.resize(1500, 900)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        layout = QtWidgets.QVBoxLayout()
        central.setLayout(layout)

        # 顶部状态文字
        self.status_label = QtWidgets.QLabel()
        self.status_label.setStyleSheet("font-family: Monospace; font-size: 13px;")
        layout.addWidget(self.status_label)

        # 图形布局
        self.graphics = pg.GraphicsLayoutWidget()
        layout.addWidget(self.graphics)

        pg.setConfigOptions(antialias=True)

        # ------------------------------
        # XY 轨迹图
        # ------------------------------
        self.xy_plot = self.graphics.addPlot(row=0, col=0, rowspan=2, colspan=2)
        self.xy_plot.setTitle("XY 轨迹：蓝色=实际轨迹，绿色=当前 /plan，红色=目标点")
        self.xy_plot.setLabel('bottom', f'X [{args.global_frame}] / m')
        self.xy_plot.setLabel('left', f'Y [{args.global_frame}] / m')
        self.xy_plot.showGrid(x=True, y=True)
        self.xy_plot.setAspectLocked(True)

        self.plan_curve = self.xy_plot.plot([], [], pen=pg.mkPen('g', width=2), name='plan')
        self.robot_curve = self.xy_plot.plot([], [], pen=pg.mkPen('b', width=2), name='robot_path')
        self.goal_scatter = pg.ScatterPlotItem(size=14, brush=pg.mkBrush(255, 0, 0))
        self.robot_scatter = pg.ScatterPlotItem(size=12, brush=pg.mkBrush(0, 150, 255))
        self.xy_plot.addItem(self.goal_scatter)
        self.xy_plot.addItem(self.robot_scatter)

        # ------------------------------
        # 线速度图
        # ------------------------------
        self.vx_plot = self.graphics.addPlot(row=0, col=2)
        self.vx_plot.setTitle("线速度 vx：黄色=RPP cmd，青色=odom 实际")
        self.vx_plot.setLabel('bottom', 't / s')
        self.vx_plot.setLabel('left', 'vx / m/s')
        self.vx_plot.showGrid(x=True, y=True)
        self.cmd_vx_curve = self.vx_plot.plot([], [], pen=pg.mkPen('y', width=2))
        self.odom_vx_curve = self.vx_plot.plot([], [], pen=pg.mkPen('c', width=2))

        # ------------------------------
        # 角速度图
        # ------------------------------
        self.wz_plot = self.graphics.addPlot(row=1, col=2)
        self.wz_plot.setTitle("角速度 wz：黄色=RPP cmd，青色=odom 实际")
        self.wz_plot.setLabel('bottom', 't / s')
        self.wz_plot.setLabel('left', 'wz / rad/s')
        self.wz_plot.showGrid(x=True, y=True)
        self.cmd_wz_curve = self.wz_plot.plot([], [], pen=pg.mkPen('y', width=2))
        self.odom_wz_curve = self.wz_plot.plot([], [], pen=pg.mkPen('c', width=2))

        # ------------------------------
        # 路径指标图
        # ------------------------------
        self.metric_plot = self.graphics.addPlot(row=2, col=0, colspan=3)
        self.metric_plot.setTitle("规划指标：品红=plan_length，红色=plan_end_to_goal_dist，绿色=robot_to_goal_dist")
        self.metric_plot.setLabel('bottom', 't / s')
        self.metric_plot.setLabel('left', 'm')
        self.metric_plot.showGrid(x=True, y=True)

        self.plan_length_curve = self.metric_plot.plot([], [], pen=pg.mkPen('m', width=2))
        self.plan_end_goal_curve = self.metric_plot.plot([], [], pen=pg.mkPen('r', width=2))
        self.robot_goal_curve = self.metric_plot.plot([], [], pen=pg.mkPen('g', width=2))

        # 定时刷新 GUI
        self.update_timer = QtCore.QTimer()
        self.update_timer.timeout.connect(self.update_gui)
        self.update_timer.start(int(1000.0 / args.gui_rate))

    def update_gui(self):
        snap = self.node.get_snapshot()

        plan = snap['latest_plan']
        robot_path = snap['robot_path']
        goal = snap['latest_goal']
        robot_pose = snap['latest_robot_pose']
        metrics = snap['metrics']
        rows = snap['time_series']

        # XY plot
        if plan:
            px = [p[0] for p in plan]
            py = [p[1] for p in plan]
            self.plan_curve.setData(px, py)

        if robot_path:
            rx = [p[1] for p in robot_path]
            ry = [p[2] for p in robot_path]
            self.robot_curve.setData(rx, ry)

        if goal is not None:
            self.goal_scatter.setData([goal[0]], [goal[1]])
        else:
            self.goal_scatter.setData([], [])

        if robot_pose is not None:
            self.robot_scatter.setData([robot_pose[0]], [robot_pose[1]])
        else:
            self.robot_scatter.setData([], [])

        # Time series
        if rows:
            t = [r['t'] for r in rows]

            cmd_vx = [r['cmd_vx'] for r in rows]
            odom_vx = [r['odom_vx'] for r in rows]
            cmd_wz = [r['cmd_wz'] for r in rows]
            odom_wz = [r['odom_wz'] for r in rows]

            plan_length = [r['plan_length'] for r in rows]
            plan_end_to_goal_dist = [r['plan_end_to_goal_dist'] for r in rows]
            robot_to_goal_dist = [r['robot_to_goal_dist'] for r in rows]

            self.cmd_vx_curve.setData(t, cmd_vx)
            self.odom_vx_curve.setData(t, odom_vx)

            self.cmd_wz_curve.setData(t, cmd_wz)
            self.odom_wz_curve.setData(t, odom_wz)

            self.plan_length_curve.setData(t, plan_length)
            self.plan_end_goal_curve.setData(t, plan_end_to_goal_dist)
            self.robot_goal_curve.setData(t, robot_to_goal_dist)

        # 顶部状态文字
        goal_text = "None"
        if goal is not None:
            goal_text = f"({goal[0]:.3f}, {goal[1]:.3f})"

        robot_text = "None"
        if robot_pose is not None:
            robot_text = f"({robot_pose[0]:.3f}, {robot_pose[1]:.3f}, yaw={robot_pose[2]:.3f})"

        status = (
            f"Robot pose [{self.args.global_frame}]: {robot_text}    "
            f"Goal: {goal_text}\n"
            f"cmd_vx={metrics['cmd_vx']:.3f} m/s, cmd_wz={metrics['cmd_wz']:.3f} rad/s    "
            f"odom_vx={metrics['odom_vx']:.3f} m/s, odom_wz={metrics['odom_wz']:.3f} rad/s\n"
            f"plan_length={metrics['plan_length']:.3f} m    "
            f"plan_end_to_goal_dist={metrics['plan_end_to_goal_dist']:.3f} m    "
            f"robot_to_goal_dist={metrics['robot_to_goal_dist']:.3f} m    "
            f"plan_update_count={snap['plan_update_count']}    "
            f"plan_update_rate={metrics['plan_update_rate']:.2f} Hz"
        )

        self.status_label.setText(status)

    def closeEvent(self, event):
        """
        关闭窗口时自动保存。
        """
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_dir = os.path.expanduser(self.args.output_dir)
        output_dir = os.path.join(output_dir, f'nav_monitor_{timestamp}')

        print("\n正在保存导航监控数据...")
        saved_dir = self.node.save_data(output_dir)
        print(f"数据已保存到: {saved_dir}")

        event.accept()

def parse_args():
    parser = argparse.ArgumentParser(description='Nav2 Motion Monitor GUI')

    parser.add_argument('--plan-topic', default='/plan',
                        help='全局路径话题，默认 /plan')

    parser.add_argument('--goal-topic', default='/goal_pose',
                        help='目标点话题，默认 /goal_pose')

    parser.add_argument('--odom-topic', default='/odom',
                        help='里程计话题，默认 /odom')

    parser.add_argument('--cmd-vel-topic', default='/cmd_vel',
                        help='RPP 输出速度话题，默认 /cmd_vel')

    parser.add_argument('--cmd-vel-type', default='twist',
                        choices=['twist', 'twist_stamped'],
                        help='cmd_vel 消息类型：twist 或 twist_stamped，默认 twist')

    parser.add_argument('--global-frame', default='map',
                        help='全局坐标系，默认 map。如果你用 map_ground，这里改成 map_ground')

    parser.add_argument('--base-frame', default='base_footprint',
                        help='机器人基座坐标系，默认 base_footprint')

    parser.add_argument('--sample-rate', type=float, default=20.0,
                        help='数据采样频率 Hz，默认 20')

    parser.add_argument('--gui-rate', type=float, default=10.0,
                        help='界面刷新频率 Hz，默认 10')

    parser.add_argument('--output-dir', default='~/nav2_monitor_logs',
                        help='数据保存目录，默认 ~/nav2_monitor_logs')

    return parser.parse_args()

def main():
    args = parse_args()

    rclpy.init()

    node = NavMonitorNode(args)

    # ROS spin 放到后台线程
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = QtWidgets.QApplication(sys.argv)
    win = MonitorWindow(node, args)
    win.show()

    ret = app.exec_()

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

    sys.exit(ret)

if __name__ == '__main__':
    main()