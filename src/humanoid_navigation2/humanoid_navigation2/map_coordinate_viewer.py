#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
地图坐标查看器 v2 - 实时显示机器人位姿和地图点击坐标

改进:
- 添加坐标变换，确保显示的是 map 坐标系下的坐标
- 增加距离显示，方便判断点击位置的远近
- 添加调试信息，帮助排查坐标异常问题

使用方法:
1. 启动导航系统
2. 运行此节点：ros2 run humanoid_navigation2 map_coordinate_viewer
3. 在 RViz 中选择 "Publish Point" 工具点击地图查看坐标
4. 机器人位姿会实时显示在 GUI 窗口中
"""

import sys
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import json
from datetime import datetime
from pathlib import Path
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


def quaternion_to_euler(x, y, z, w):
    """四元数转欧拉角"""
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


def calculate_distance(x1, y1, x2, y2):
    """计算两点间的平面距离"""
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


class CoordinateViewerGUI:
    """坐标查看器 GUI 界面"""

    def __init__(self, root, callbacks=None):
        self.root = root
        self.root.title("🤖 机器人位姿 & 地图坐标查看器 v2")
        self.root.geometry("580x600")
        self.root.attributes('-topmost', False)

        self.callbacks = callbacks or {}
        self.saved_points = []
        self.load_saved_points()

        # 机器人当前位置（用于计算距离）
        self.robot_x = 0.0
        self.robot_y = 0.0

        self.setup_ui()

    def setup_ui(self):
        """设置 UI 界面"""
        # 主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # 标题
        title_label = ttk.Label(
            main_frame,
            text="🤖 机器人位姿 & 地图坐标查看器 v2",
            font=('Helvetica', 14, 'bold')
        )
        title_label.grid(row=0, column=0, columnspan=2, pady=(0, 10))

        # ==================== 机器人位姿区域 ====================
        pose_frame = ttk.LabelFrame(main_frame, text="🤖 机器人实时位姿", padding="10")
        pose_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)

        # 位置
        ttk.Label(pose_frame, text="位置:", font=('Helvetica', 10, 'bold')).grid(
            row=0, column=0, columnspan=2, sticky=tk.W, padx=5, pady=(0, 5))

        ttk.Label(pose_frame, text="X:").grid(row=1, column=0, sticky=tk.W, padx=5)
        self.robot_x_var = tk.StringVar(value="0.000")
        self.robot_x_entry = ttk.Entry(pose_frame, textvariable=self.robot_x_var,
                                        width=15, font=('Courier', 11))
        self.robot_x_entry.grid(row=1, column=1, padx=5, pady=2)

        ttk.Label(pose_frame, text="Y:").grid(row=2, column=0, sticky=tk.W, padx=5)
        self.robot_y_var = tk.StringVar(value="0.000")
        self.robot_y_entry = ttk.Entry(pose_frame, textvariable=self.robot_y_var,
                                        width=15, font=('Courier', 11))
        self.robot_y_entry.grid(row=2, column=1, padx=5, pady=2)

        ttk.Label(pose_frame, text="Z:").grid(row=3, column=0, sticky=tk.W, padx=5)
        self.robot_z_var = tk.StringVar(value="0.000")
        self.robot_z_entry = ttk.Entry(pose_frame, textvariable=self.robot_z_var,
                                        width=15, font=('Courier', 11))
        self.robot_z_entry.grid(row=3, column=1, padx=5, pady=2)

        # 姿态（欧拉角）
        ttk.Label(pose_frame, text="姿态 (欧拉角):", font=('Helvetica', 10, 'bold')).grid(
            row=0, column=2, columnspan=2, sticky=tk.W, padx=10, pady=(0, 5))

        ttk.Label(pose_frame, text="Roll:").grid(row=1, column=2, sticky=tk.W, padx=10)
        self.robot_roll_var = tk.StringVar(value="0.0°")
        ttk.Label(pose_frame, textvariable=self.robot_roll_var,
                  font=('Courier', 11)).grid(row=1, column=3, sticky=tk.W, padx=5)

        ttk.Label(pose_frame, text="Pitch:").grid(row=2, column=2, sticky=tk.W, padx=10)
        self.robot_pitch_var = tk.StringVar(value="0.0°")
        ttk.Label(pose_frame, textvariable=self.robot_pitch_var,
                  font=('Courier', 11)).grid(row=2, column=3, sticky=tk.W, padx=5)

        ttk.Label(pose_frame, text="Yaw:").grid(row=3, column=2, sticky=tk.W, padx=10)
        self.robot_yaw_var = tk.StringVar(value="0.0°")
        ttk.Label(pose_frame, textvariable=self.robot_yaw_var,
                  font=('Courier', 11)).grid(row=3, column=3, sticky=tk.W, padx=5)

        # 位姿时间戳和坐标系
        ttk.Label(pose_frame, text="坐标系:").grid(row=4, column=0, sticky=tk.W, padx=5)
        self.robot_frame_var = tk.StringVar(value="map")
        ttk.Label(pose_frame, textvariable=self.robot_frame_var,
                  font=('Courier', 11)).grid(row=4, column=1, sticky=tk.W, padx=5)

        ttk.Label(pose_frame, text="更新时间:").grid(row=4, column=2, sticky=tk.W, padx=10)
        self.robot_time_var = tk.StringVar(value="-")
        ttk.Label(pose_frame, textvariable=self.robot_time_var,
                  font=('Courier', 9)).grid(row=4, column=3, sticky=tk.W, padx=5)

        # 复制机器人位姿按钮
        self.copy_pose_btn = ttk.Button(
            pose_frame,
            text="📋 复制位姿",
            command=self.copy_robot_pose_to_clipboard,
            width=12
        )
        self.copy_pose_btn.grid(row=5, column=0, columnspan=2, padx=5, pady=5)

        # 保存机器人位姿按钮
        self.save_pose_btn = ttk.Button(
            pose_frame,
            text="💾 保存位姿",
            command=self.save_robot_pose,
            width=12
        )
        self.save_pose_btn.grid(row=5, column=2, columnspan=2, padx=5, pady=5)

        # ==================== 地图点击坐标区域 ====================
        click_frame = ttk.LabelFrame(main_frame, text="📍 地图点击坐标", padding="10")
        click_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=5)

        # X 坐标
        ttk.Label(click_frame, text="X:").grid(row=0, column=0, sticky=tk.W, padx=5)
        self.x_var = tk.StringVar(value="0.000")
        self.x_entry = ttk.Entry(click_frame, textvariable=self.x_var,
                                  width=15, font=('Courier', 11))
        self.x_entry.grid(row=0, column=1, padx=5, pady=2)

        # Y 坐标
        ttk.Label(click_frame, text="Y:").grid(row=1, column=0, sticky=tk.W, padx=5)
        self.y_var = tk.StringVar(value="0.000")
        self.y_entry = ttk.Entry(click_frame, textvariable=self.y_var,
                                  width=15, font=('Courier', 11))
        self.y_entry.grid(row=1, column=1, padx=5, pady=2)

        # Z 坐标
        ttk.Label(click_frame, text="Z:").grid(row=2, column=0, sticky=tk.W, padx=5)
        self.z_var = tk.StringVar(value="0.000")
        self.z_entry = ttk.Entry(click_frame, textvariable=self.z_var,
                                  width=15, font=('Courier', 11))
        self.z_entry.grid(row=2, column=1, padx=5, pady=2)

        # 距离显示
        ttk.Label(click_frame, text="距离:").grid(row=3, column=0, sticky=tk.W, padx=5)
        self.distance_var = tk.StringVar(value="0.00 m")
        self.distance_label = ttk.Label(click_frame, textvariable=self.distance_var,
                                         font=('Courier', 11), foreground='blue')
        self.distance_label.grid(row=3, column=1, sticky=tk.W, padx=5)

        # 帧信息
        ttk.Label(click_frame, text="坐标系:").grid(row=4, column=0, sticky=tk.W, padx=5)
        self.frame_var = tk.StringVar(value="map")
        ttk.Label(click_frame, textvariable=self.frame_var,
                  font=('Courier', 11)).grid(row=4, column=1, sticky=tk.W, padx=5)

        # 时间戳
        ttk.Label(click_frame, text="时间:").grid(row=4, column=2, sticky=tk.W, padx=10)
        self.time_var = tk.StringVar(value="-")
        ttk.Label(click_frame, textvariable=self.time_var,
                  font=('Courier', 9)).grid(row=4, column=3, sticky=tk.W, padx=10)

        # 按钮区域
        btn_frame = ttk.Frame(click_frame)
        btn_frame.grid(row=3, column=2, rowspan=2, padx=10)

        # 复制坐标按钮
        self.copy_btn = ttk.Button(
            btn_frame,
            text="📋 复制",
            command=self.copy_to_clipboard,
            width=10
        )
        self.copy_btn.grid(row=0, column=0, pady=2)

        # 保存坐标按钮
        self.save_btn = ttk.Button(
            btn_frame,
            text="💾 保存",
            command=self.save_current_point,
            width=10
        )
        self.save_btn.grid(row=1, column=0, pady=2)

        # ==================== 历史坐标列表 ====================
        history_frame = ttk.LabelFrame(main_frame, text="📜 已保存的坐标点", padding="10")
        history_frame.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E, tk.N, tk.S), pady=5)

        # 创建 Treeview 显示历史坐标
        columns = ('name', 'x', 'y', 'z', 'type')
        self.history_tree = ttk.Treeview(history_frame, columns=columns, show='headings', height=6)

        self.history_tree.heading('name', text='名称')
        self.history_tree.heading('x', text='X')
        self.history_tree.heading('y', text='Y')
        self.history_tree.heading('z', text='Z')
        self.history_tree.heading('type', text='类型')

        self.history_tree.column('name', width=100)
        self.history_tree.column('x', width=60)
        self.history_tree.column('y', width=60)
        self.history_tree.column('z', width=60)
        self.history_tree.column('type', width=70)

        # 滚动条
        scrollbar = ttk.Scrollbar(history_frame, orient=tk.VERTICAL,
                                   command=self.history_tree.yview)
        self.history_tree.configure(yscrollcommand=scrollbar.set)

        self.history_tree.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))

        # 双击加载坐标
        self.history_tree.bind('<Double-1>', self.on_history_double_click)

        # 操作按钮行
        op_frame = ttk.Frame(main_frame)
        op_frame.grid(row=4, column=0, columnspan=2, pady=5)

        # 清空按钮
        self.clear_btn = ttk.Button(
            op_frame,
            text="🗑️ 清空历史",
            command=self.clear_history,
            width=15
        )
        self.clear_btn.grid(row=0, column=0, padx=5)

        # 打开保存目录按钮
        self.open_dir_btn = ttk.Button(
            op_frame,
            text="📂 打开保存目录",
            command=self.open_save_directory,
            width=15
        )
        self.open_dir_btn.grid(row=0, column=1, padx=5)

        # 状态栏
        self.status_var = tk.StringVar(
            value="就绪 - 请在 RViz 中使用 Publish Point 工具点击地图，或查看机器人实时位姿")
        status_bar = ttk.Label(main_frame, textvariable=self.status_var,
                                relief=tk.SUNKEN, anchor=tk.W)
        status_bar.grid(row=5, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(10, 0))

        # 配置行列权重
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(3, weight=1)

        self.update_history_display()

    def set_robot_position(self, x, y):
        """设置机器人位置（用于计算距离）"""
        self.robot_x = x
        self.robot_y = y

    def update_robot_pose(self, x, y, z, roll, pitch, yaw, frame_id="map", timestamp=None):
        """更新机器人位姿显示"""
        self.robot_x_var.set(f"{x:.3f}")
        self.robot_y_var.set(f"{y:.3f}")
        self.robot_z_var.set(f"{z:.3f}")
        self.robot_frame_var.set(frame_id)

        # 更新机器人位置（用于距离计算）
        self.set_robot_position(x, y)

        # 转换为角度
        self.robot_roll_var.set(f"{math.degrees(roll):.1f}°")
        self.robot_pitch_var.set(f"{math.degrees(pitch):.1f}°")
        self.robot_yaw_var.set(f"{math.degrees(yaw):.1f}°")

        if timestamp:
            self.robot_time_var.set(datetime.fromtimestamp(timestamp).strftime('%H:%M:%S.%f')[:-3])

    def update_coordinate(self, point: PointStamped, robot_x=0.0, robot_y=0.0):
        """更新坐标显示"""
        self.x_var.set(f"{point.point.x:.3f}")
        self.y_var.set(f"{point.point.y:.3f}")
        self.z_var.set(f"{point.point.z:.3f}")
        self.frame_var.set(point.header.frame_id)

        # 计算距离
        distance = calculate_distance(robot_x, robot_y, point.point.x, point.point.y)
        self.distance_var.set(f"{distance:.2f} m")

        # 转换时间戳为可读格式
        ts = point.header.stamp.sec + point.header.stamp.nanosec / 1e9
        self.time_var.set(datetime.fromtimestamp(ts).strftime('%H:%M:%S.%f')[:-3])

        self.status_var.set(f"✅ 收到坐标点 @ {datetime.now().strftime('%H:%M:%S')} "
                           f"(距离：{distance:.2f}m)")

        # 高亮闪烁效果
        self.root.after(0, lambda: self.flash_effect())

    def flash_effect(self):
        """闪烁效果"""
        original_bg = self.root.cget('bg')
        self.root.configure(bg='#90EE90')
        self.root.after(200, lambda: self.root.configure(bg=original_bg))

    def copy_to_clipboard(self):
        """复制坐标到剪贴板"""
        coord_str = f"({self.x_var.get()}, {self.y_var.get()}, {self.z_var.get()})"
        self.root.clipboard_clear()
        self.root.clipboard_append(coord_str)
        self.status_var.set(f"📋 已复制：{coord_str}")

    def copy_robot_pose_to_clipboard(self):
        """复制机器人位姿到剪贴板"""
        pose_str = (f"位置：({self.robot_x_var.get()}, {self.robot_y_var.get()}, "
                    f"{self.robot_z_var.get()})\n"
                    f"姿态：R={self.robot_roll_var.get()}, "
                    f"P={self.robot_pitch_var.get()}, "
                    f"Y={self.robot_yaw_var.get()}")
        self.root.clipboard_clear()
        self.root.clipboard_append(pose_str)
        self.status_var.set(f"📋 已复制机器人位姿")

    def save_current_point(self):
        """保存当前坐标"""
        dialog = PointSaveDialog(self.root)
        if dialog.result:
            point_data = {
                'name': dialog.result,
                'x': float(self.x_var.get()),
                'y': float(self.y_var.get()),
                'z': float(self.z_var.get()),
                'frame_id': self.frame_var.get(),
                'timestamp': datetime.now().isoformat(),
                'type': 'clicked'
            }
            self.saved_points.append(point_data)
            self.save_saved_points()
            self.update_history_display()
            self.status_var.set(f"💾 已保存：{dialog.result}")

    def save_robot_pose(self):
        """保存机器人当前位姿"""
        dialog = PointSaveDialog(self.root, prefix="机器人位姿")
        if dialog.result:
            point_data = {
                'name': dialog.result,
                'x': float(self.robot_x_var.get()),
                'y': float(self.robot_y_var.get()),
                'z': float(self.robot_z_var.get()),
                'roll': math.radians(float(self.robot_roll_var.get().replace('°', ''))),
                'pitch': math.radians(float(self.robot_pitch_var.get().replace('°', ''))),
                'yaw': math.radians(float(self.robot_yaw_var.get().replace('°', ''))),
                'frame_id': self.robot_frame_var.get(),
                'timestamp': datetime.now().isoformat(),
                'type': 'robot_pose'
            }
            self.saved_points.append(point_data)
            self.save_saved_points()
            self.update_history_display()
            self.status_var.set(f"💾 已保存机器人位姿：{dialog.result}")

    def clear_history(self):
        """清空历史记录"""
        if messagebox.askyesno("确认", "确定要清空所有保存的坐标点吗？"):
            self.saved_points = []
            self.save_saved_points()
            self.update_history_display()
            self.status_var.set("🗑️ 已清空历史记录")

    def update_history_display(self):
        """更新历史坐标显示"""
        for item in self.history_tree.get_children():
            self.history_tree.delete(item)

        for point in self.saved_points:
            point_type = point.get('type', 'clicked')
            type_display = "位姿" if point_type == 'robot_pose' else "点击"
            self.history_tree.insert('', tk.END, values=(
                point['name'],
                f"{point['x']:.3f}",
                f"{point['y']:.3f}",
                f"{point['z']:.3f}",
                type_display
            ))

    def on_history_double_click(self, event):
        """双击历史坐标"""
        selection = self.history_tree.selection()
        if selection:
            item = self.history_tree.item(selection[0])
            values = item['values']
            self.x_var.set(values[1])
            self.y_var.set(values[2])
            self.z_var.set(values[3])
            self.status_var.set(f"📌 已加载：{values[0]}")

    def save_saved_points(self):
        """保存坐标点到文件"""
        save_path = Path.home() / '.humanoid_navigation' / 'saved_points.json'
        save_path.parent.mkdir(parents=True, exist_ok=True)
        with open(save_path, 'w', encoding='utf-8') as f:
            json.dump(self.saved_points, f, ensure_ascii=False, indent=2)

    def load_saved_points(self):
        """从文件加载坐标点"""
        save_path = Path.home() / '.humanoid_navigation' / 'saved_points.json'
        if save_path.exists():
            try:
                with open(save_path, 'r', encoding='utf-8') as f:
                    self.saved_points = json.load(f)
            except Exception as e:
                print(f"加载保存的坐标点失败：{e}")
                self.saved_points = []

    def open_save_directory(self):
        """打开保存目录"""
        save_path = Path.home() / '.humanoid_navigation'
        save_path.mkdir(parents=True, exist_ok=True)
        # 使用 xdg-open 打开文件管理器
        import subprocess
        try:
            subprocess.run(['xdg-open', str(save_path)], check=False)
        except Exception:
            self.status_var.set(f"📂 保存目录：{save_path}")


class PointSaveDialog:
    """保存坐标点的对话框"""

    def __init__(self, parent, prefix="坐标点"):
        self.result = None

        self.dialog = tk.Toplevel(parent)
        self.dialog.title("保存坐标点")
        self.dialog.geometry("320x140")
        self.dialog.transient(parent)
        self.dialog.grab_set()

        ttk.Label(self.dialog, text=f"{prefix}名称:").pack(pady=10)

        self.name_var = tk.StringVar()
        name_entry = ttk.Entry(self.dialog, textvariable=self.name_var, width=30)
        name_entry.pack(pady=5)
        name_entry.focus()

        # 预设名称建议
        if prefix == "机器人位姿":
            suggestions = ["展台点", "导航目标点", "工作站", "充电点", "等待点"]
        else:
            suggestions = ["展台点", "导航目标点", "工作站", "充电点", "等待点"]

        suggestion_frame = ttk.Frame(self.dialog)
        suggestion_frame.pack(pady=5)
        for sug in suggestions:
            btn = ttk.Button(
                suggestion_frame,
                text=sug,
                width=7,
                command=lambda s=sug: self.set_name(s)
            )
            btn.pack(side=tk.LEFT, padx=2)

        btn_frame = ttk.Frame(self.dialog)
        btn_frame.pack(pady=10)

        ttk.Button(
            btn_frame,
            text="保存",
            command=self.on_save
        ).pack(side=tk.LEFT, padx=10)

        ttk.Button(
            btn_frame,
            text="取消",
            command=self.dialog.destroy
        ).pack(side=tk.LEFT, padx=10)

        # 绑定回车键
        self.dialog.bind('<Return>', lambda e: self.on_save())

        self.dialog.wait_window()

    def set_name(self, name):
        self.name_var.set(name)

    def on_save(self):
        name = self.name_var.get().strip()
        if name:
            self.result = name
            self.dialog.destroy()


class CoordinateViewerNode(Node):
    """ROS 2 节点 - 订阅位姿和 clicked_point 话题"""

    def __init__(self, gui):
        super().__init__('map_coordinate_viewer')
        self.gui = gui

        # TF2 buffer 和 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 订阅 RViz 的 PublishPoint 话题
        self.clicked_subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.point_callback,
            10
        )

        # 订阅机器人位姿话题 (/pose 或 /odom)
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',
            self.pose_callback,
            10
        )

        # 备用：订阅里程计话题
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.last_pose_time = 0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.get_logger().info('🤖 地图坐标查看器 v2 已启动，显示机器人位姿和点击坐标...')
        self.get_logger().info('💡 提示：坐标会自动转换到 map 坐标系，并显示到机器人的距离')

    def point_callback(self, msg: PointStamped):
        """坐标点回调"""
        # 尝试转换到 map 坐标系
        map_point = None
        try:
            # 获取从 msg.header.frame_id 到 map 的变换
            trans = self.tf_buffer.lookup_transform(
                'map', msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # 转换坐标
            map_point = PointStamped()
            map_point.header.frame_id = 'map'
            map_point.header.stamp = self.get_clock().now().to_msg()
            map_point.point.x = msg.point.x + trans.transform.translation.x
            map_point.point.y = msg.point.y + trans.transform.translation.y
            map_point.point.z = msg.point.z + trans.transform.translation.z
            
            self.get_logger().info(
                f'📍 收到坐标点 (转换后): x={map_point.point.x:.3f}, '
                f'y={map_point.point.y:.3f}, z={map_point.point.z:.3f} [@ map]'
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            # TF 变换失败，使用原始坐标
            self.get_logger().warn(f'⚠️  TF 变换失败，使用原始坐标：{e}')
            map_point = msg
        
        # 更新 GUI（使用机器人当前位置计算距离）
        self.gui.update_coordinate(map_point, self.robot_x, self.robot_y)

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """机器人位姿回调（来自 /pose 话题）"""
        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z

        # 更新机器人位置
        self.robot_x = x
        self.robot_y = y

        # 四元数转欧拉角
        roll, pitch, yaw = quaternion_to_euler(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )

        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.gui.update_robot_pose(x, y, z, roll, pitch, yaw,
                                    msg.header.frame_id, timestamp)

    def odom_callback(self, msg: Odometry):
        """里程计回调（来自 /odom 话题）"""
        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z

        # 更新机器人位置
        self.robot_x = x
        self.robot_y = y

        # 四元数转欧拉角
        roll, pitch, yaw = quaternion_to_euler(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )

        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.gui.update_robot_pose(x, y, z, roll, pitch, yaw,
                                    msg.header.frame_id, timestamp)


def main():
    # 检查 tkinter 是否可用
    try:
        root = tk.Tk()
        root.withdraw()
        root.destroy()
    except tk.TclError:
        print("⚠️  tkinter 不可用，将以纯终端模式运行")
        # 纯终端模式
        rclpy.init()
        node = rclpy.create_node('map_coordinate_viewer_terminal')
        node.create_subscription(
            PointStamped,
            '/clicked_point',
            lambda msg: node.get_logger().info(
                f'📍 坐标：x={msg.point.x:.3f}, y={msg.point.y:.3f}, z={msg.point.z:.3f} '
                f'[@ {msg.header.frame_id}]'
            ),
            10
        )
        node.get_logger().info('📍 地图坐标查看器已启动 (终端模式)')
        rclpy.spin(node)
        return

    # GUI 模式
    rclpy.init()

    root = tk.Tk()
    gui = CoordinateViewerGUI(root)

    node = CoordinateViewerNode(gui)

    # 在单独的线程中运行 ROS 2
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,))
    ros_thread.start()

    # 处理窗口关闭
    def on_closing():
        node.destroy_node()
        rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)

    print("🤖 机器人位姿 & 地图坐标查看器 v2 已启动")
    print("💡 使用方法：")
    print("   1. 机器人位姿会实时显示（订阅 /pose 或 /odom 话题）")
    print("   2. 在 RViz 中选择 'Publish Point' 工具点击地图可查看坐标")
    print("   3. 坐标会自动转换到 map 坐标系，并显示到机器人的距离")
    print("   4. 可以保存常用坐标点以便快速复用")

    root.mainloop()

    ros_thread.join()


if __name__ == '__main__':
    main()
