#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
机器人运动学死区测试工具 v2.0
==============================================================================

功能：
  1. 线速度死区测试（自动识别最小有效速度）
  2. 角速度死区测试（自动识别最小有效角速度）
  3. 转弯半径测试（测量实际转弯半径并与理论值对比）
  4. 急停功能（全局最高优先级，立即停止所有测试）
  5. 按钮自锁（测试中禁用开始按钮，启用停止按钮）
  6. 实时轨迹绘制（显示机器人运动轨迹）
  7. 测试结果保存（自动保存为 CSV 文件）
  8. 配置建议生成（根据测试结果自动生成 Nav2 配置建议）

依赖：
  sudo apt install ros-humble-desktop python3-pyside6
  pip3 install matplotlib numpy

使用方法：
  python3 velocity_tester_gui.py

作者：AI Assistant
日期：2025
==============================================================================
"""

import sys
import time
import math
import threading
from datetime import datetime
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit, QTextEdit, QGroupBox, QSpinBox,
    QDoubleSpinBox, QCheckBox, QTabWidget, QTableWidget, QTableWidgetItem,
    QHeaderView, QMessageBox, QFileDialog
)
from PySide6.QtCore import Qt, QThread, Signal, QTimer
from PySide6.QtGui import QFont, QColor
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np

# ==============================================================================
# ROS2 节点：负责发布速度指令和接收里程计数据
# ==============================================================================
class VelocityPublisherNode(Node):
    """
    ROS2 节点，负责：
    1. 发布 /cmd_vel 速度指令
    2. 订阅 /odom 里程计数据
    3. 记录机器人轨迹
    """
    
    def __init__(self):
        super().__init__('velocity_tester_node')
        
         # 创建速度指令发布器
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # TF 监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 存储机器人位置轨迹
        self.trajectory = deque(maxlen=1000)
        
        # 当前位置和姿态
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # 起始位置
        self.start_x = None
        self.start_y = None
        
        # 定时器：定期查询 TF
        self.create_timer(0.1, self.update_pose_from_tf)
        
        self.get_logger().info('速度测试节点已启动（使用TF）')
    
    def update_pose_from_tf(self):
        """
        从 TF 树获取 base_footprint 在 odom 下的位姿
        """
        try:
            # 查询 base_footprint 在 odom 坐标系下的变换
            transform = self.tf_buffer.lookup_transform(
                'odom',
                'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # 提取位置（标准 ROS 坐标系）
            self.current_x = transform.transform.translation.x
            self.current_y = transform.transform.translation.y
            
            # 提取姿态（四元数转欧拉角）
            q = transform.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
            
            # 记录轨迹
            self.trajectory.append((self.current_x, self.current_y))
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            # TF 查询失败时不打印（避免刷屏）
            pass
    
    def publish_velocity(self, linear_x, angular_z):
        """
        发布速度指令
        
        参数：
            linear_x: 线速度 (m/s)
            angular_z: 角速度 (rad/s)
        """
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_publisher.publish(msg)
    
    def stop_robot(self):
        """
        立即停止机器人
        连续发送多次零速度指令，确保机器人完全停止
        """
        for _ in range(10):
            self.publish_velocity(0.0, 0.0)
            time.sleep(0.05)
    
    def reset_trajectory(self):
        """
        重置轨迹记录
        """
        self.trajectory.clear()
        self.start_x = self.current_x
        self.start_y = self.current_y
    
    def get_trajectory(self):
        """
        获取当前轨迹
        
        返回：
            list: [(x1, y1), (x2, y2), ...]
        """
        return list(self.trajectory)
    
    def get_displacement(self):
        """
        计算从起始点到当前位置的位移
        
        返回：
            float: 位移距离 (m)
        """
        if self.start_x is None:
            return 0.0
        
        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        return math.sqrt(dx**2 + dy**2)

# ==============================================================================
# 测试工作线程：在后台执行测试任务
# ==============================================================================
class TestWorkerThread(QThread):
    """
    测试工作线程
    在后台执行测试任务，避免阻塞 UI 主线程
    """
    
    # 定义信号，用于与 UI 主线程通信
    log_signal = Signal(str)              # 发送日志消息
    progress_signal = Signal(int, int)    # 发送进度 (当前, 总数)
    result_signal = Signal(dict)          # 发送测试结果
    finished_signal = Signal()            # 测试完成信号
    
    def __init__(self, ros_node, test_type, params):
        """
        初始化测试线程
        
        参数：
            ros_node: ROS2 节点实例
            test_type: 测试类型 ('linear', 'angular', 'turning')
            params: 测试参数字典
        """
        super().__init__()
        self.ros_node = ros_node
        self.test_type = test_type
        self.params = params
        
        # 控制标志
        self.is_running = True
        self.is_stopped = False
        self.emergency_stop = False
    
    def stop(self):
        """
        停止测试
        """
        self.is_stopped = True
        self.is_running = False
    
    def emergency_stop_now(self):
        """
        紧急停止
        """
        self.emergency_stop = True
        self.is_stopped = True
        self.is_running = False
    

    def should_stop(self):
        """
        判断当前测试是否应该停止
        """
        return (not self.is_running) or self.is_stopped or self.emergency_stop

    def interruptible_sleep(self, total_time, step=0.05):
        """
        可中断 sleep，避免普通 time.sleep() 导致停止按钮响应慢
        """
        start_time = time.time()
        while time.time() - start_time < total_time:
           if self.should_stop():
              return False
           time.sleep(step)
        return True

    def run(self):
        """
        线程主函数
        根据测试类型调用相应的测试方法
        """
        try:
            if self.test_type == 'linear':
                self.test_linear_velocity()
            elif self.test_type == 'angular':
                self.test_angular_velocity()
            elif self.test_type == 'turning':
                self.test_turning_radius()
        except Exception as e:
            self.log_signal.emit(f'❌ 测试出错: {str(e)}')
        finally:
            # 确保机器人停止
            self.ros_node.stop_robot()

            if self.emergency_stop:
                self.log_signal.emit('🛑 测试线程已因急停退出')
            elif self.is_stopped:
                self.log_signal.emit('⚠️  测试线程已被用户停止')
            else:
                self.log_signal.emit('测试线程正常结束')

            self.finished_signal.emit()
    
    
    def test_linear_velocity(self):
        """
        线速度死区测试
        """
        velocities = self.params['velocities']
        duration = self.params['duration']
        threshold = self.params['threshold']

        self.log_signal.emit('=' * 60)
        self.log_signal.emit('开始线速度死区测试')
        self.log_signal.emit('=' * 60)

        results = []
        min_valid_velocity = None

        for i, vel in enumerate(velocities):
            if self.should_stop():
                self.log_signal.emit('⚠️  测试已停止')
                self.ros_node.stop_robot()
                return

            self.progress_signal.emit(i + 1, len(velocities))
            self.log_signal.emit(f'\n测试线速度: {vel:.3f} m/s')

            self.ros_node.reset_trajectory()

            start_time = time.time()
            while time.time() - start_time < duration:
                if self.should_stop():
                    self.log_signal.emit('⚠️  线速度测试被用户停止')
                    self.ros_node.stop_robot()
                    return

                self.ros_node.publish_velocity(vel, 0.0)

                if not self.interruptible_sleep(0.1):
                   self.log_signal.emit('⚠️  线速度测试被用户停止')
                   self.ros_node.stop_robot()
                   return

            self.ros_node.stop_robot()

            if not self.interruptible_sleep(0.5):
                self.log_signal.emit('⚠️  线速度测试被用户停止')
                self.ros_node.stop_robot()
                return

            if self.should_stop():
                self.log_signal.emit('⚠️  线速度测试被用户停止')
                self.ros_node.stop_robot()
                return

            displacement = self.ros_node.get_displacement()
            is_valid = displacement >= threshold

            if is_valid and min_valid_velocity is None:
                min_valid_velocity = vel

            result = {
               'velocity': vel,
               'displacement': displacement,
               'is_valid': is_valid
            }
            results.append(result)

            status = '✓ 有效' if is_valid else '✗ 无效'
            self.log_signal.emit(f'  位移: {displacement:.3f} m  {status}')

            if not self.interruptible_sleep(1.0):
                self.log_signal.emit('⚠️  线速度测试被用户停止')
                self.ros_node.stop_robot()
                return

        if self.should_stop():
            self.log_signal.emit('⚠️  测试已停止')
            self.ros_node.stop_robot()
            return

        self.log_signal.emit('\n' + '=' * 60)
        if min_valid_velocity is not None:
            self.log_signal.emit(f'✓ 最小有效线速度: {min_valid_velocity:.3f} m/s')
        else:
            self.log_signal.emit('✗ 未找到有效线速度')
        self.log_signal.emit('=' * 60)

        self.result_signal.emit({
           'type': 'linear',
           'results': results,
           'min_valid': min_valid_velocity
        })
    
    def test_angular_velocity(self):
        """
        角速度死区测试
        """
        velocities = self.params['velocities']
        duration = self.params['duration']
        threshold = self.params['threshold']

        self.log_signal.emit('=' * 60)
        self.log_signal.emit('开始角速度死区测试')
        self.log_signal.emit('=' * 60)

        results = []
        min_valid_velocity = None
        has_displacement = False

        for i, vel in enumerate(velocities):
            if self.should_stop():
                self.log_signal.emit('⚠️  测试已停止')
                self.ros_node.stop_robot()
                return

            self.progress_signal.emit(i + 1, len(velocities))
            self.log_signal.emit(f'\n测试角速度: {vel:.3f} rad/s')

            self.ros_node.reset_trajectory()
            start_yaw = self.ros_node.current_yaw

            start_time = time.time()
            while time.time() - start_time < duration:
                if self.should_stop():
                    self.log_signal.emit('⚠️  角速度测试被用户停止')
                    self.ros_node.stop_robot()
                    return

                self.ros_node.publish_velocity(0.0, vel)

                if not self.interruptible_sleep(0.1):
                    self.log_signal.emit('⚠️  角速度测试被用户停止')
                    self.ros_node.stop_robot()
                    return

            self.ros_node.stop_robot()

            if not self.interruptible_sleep(0.5):
                self.log_signal.emit('⚠️  角速度测试被用户停止')
                self.ros_node.stop_robot()
                return

            if self.should_stop():
                self.log_signal.emit('⚠️  角速度测试被用户停止')
                self.ros_node.stop_robot()
                return

            end_yaw = self.ros_node.current_yaw
            yaw_change = abs(end_yaw - start_yaw)

            if yaw_change > math.pi:
                yaw_change = 2 * math.pi - yaw_change

            displacement = self.ros_node.get_displacement()
            is_valid = yaw_change >= threshold
            has_disp = displacement > 0.03

            if is_valid and min_valid_velocity is None:
                min_valid_velocity = vel

            if has_disp:
                has_displacement = True

            result = {
                 'velocity': vel,
                 'yaw_change': yaw_change,
                 'displacement': displacement,
                 'is_valid': is_valid,
                 'has_displacement': has_disp
            }
            results.append(result)

            status = '✓ 有效' if is_valid else '✗ 无效'
            disp_status = f'(位移: {displacement:.3f} m)' if has_disp else ''
            self.log_signal.emit(f'  角度变化: {math.degrees(yaw_change):.1f}°  {status} {disp_status}')
 
            if not self.interruptible_sleep(1.0):
               self.log_signal.emit('⚠️  角速度测试被用户停止')
               self.ros_node.stop_robot()
               return

        if self.should_stop():
            self.log_signal.emit('⚠️  测试已停止')
            self.ros_node.stop_robot()
            return

        self.log_signal.emit('\n' + '=' * 60)
        if min_valid_velocity is not None:
            self.log_signal.emit(f'✓ 最小有效角速度: {min_valid_velocity:.3f} rad/s')
            if has_displacement:
                self.log_signal.emit('⚠️  警告: 旋转时有明显位移，不是理想原地旋转')
        else:
            self.log_signal.emit('✗ 未找到有效角速度')
        self.log_signal.emit('=' * 60)

        self.result_signal.emit({
            'type': 'angular',
            'results': results,
            'min_valid': min_valid_velocity,
            'has_displacement': has_displacement
        })
        
        
    def test_turning_radius(self):
        """
        转弯半径测试
        """
        test_cases = self.params['test_cases']
        duration = self.params['duration']

        self.log_signal.emit('=' * 60)
        self.log_signal.emit('开始转弯半径测试')
        self.log_signal.emit('=' * 60)

        results = []

        for i, (linear_vel, angular_vel) in enumerate(test_cases):
            if self.should_stop():
                self.log_signal.emit('⚠️  测试已停止')
                self.ros_node.stop_robot()
                return

            self.progress_signal.emit(i + 1, len(test_cases))
            self.log_signal.emit(f'\n测试组合: v={linear_vel:.3f} m/s, ω={angular_vel:.3f} rad/s')

            if abs(angular_vel) > 0.001:
                theoretical_radius = abs(linear_vel / angular_vel)
            else:
                theoretical_radius = float('inf')

            self.log_signal.emit(f'  理论转弯半径: {theoretical_radius:.3f} m')

            self.ros_node.reset_trajectory()

            start_time = time.time()
            while time.time() - start_time < duration:
                if self.should_stop():
                    self.log_signal.emit('⚠️  转弯半径测试被用户停止')
                    self.ros_node.stop_robot()
                    return

                self.ros_node.publish_velocity(linear_vel, angular_vel)

                if not self.interruptible_sleep(0.1):
                    self.log_signal.emit('⚠️  转弯半径测试被用户停止')
                    self.ros_node.stop_robot()
                    return

            self.ros_node.stop_robot()

            if not self.interruptible_sleep(0.5):
                self.log_signal.emit('⚠️  转弯半径测试被用户停止')
                self.ros_node.stop_robot()
                return

            if self.should_stop():
                self.log_signal.emit('⚠️  转弯半径测试被用户停止')
                self.ros_node.stop_robot()
                return

            trajectory = self.ros_node.get_trajectory()

            if len(trajectory) > 10:
                actual_radius = self.calculate_turning_radius(trajectory)
            else:
                actual_radius = 0.0

            if theoretical_radius != float('inf') and actual_radius > 0:
                error = abs(actual_radius - theoretical_radius) / theoretical_radius * 100
            else:
                error = 0.0

            result = {
                'linear_vel': linear_vel,
                'angular_vel': angular_vel,
                'theoretical_radius': theoretical_radius,
                'actual_radius': actual_radius,
                'error': error,
                'trajectory': trajectory
            }
            results.append(result)

            if actual_radius > 0:
                self.log_signal.emit(f'  实际转弯半径: {actual_radius:.3f} m')
                self.log_signal.emit(f'  误差: {error:.1f}%')
            else:
                self.log_signal.emit('  ⚠️  轨迹点不足，无法计算')

            if not self.interruptible_sleep(1.0):
                self.log_signal.emit('⚠️  转弯半径测试被用户停止')
                self.ros_node.stop_robot()
                return

        if self.should_stop():
            self.log_signal.emit('⚠️  测试已停止')
            self.ros_node.stop_robot()
            return

        self.log_signal.emit('\n' + '=' * 60)
        self.log_signal.emit('转弯半径测试完成')
        self.log_signal.emit('=' * 60)

        self.result_signal.emit({
            'type': 'turning',
            'results': results
        })
        
    def calculate_turning_radius(self, trajectory):
        """
        从轨迹点计算转弯半径
        使用最小二乘法拟合圆
        
        参数：
            trajectory: [(x1, y1), (x2, y2), ...]
        
        返回：
            float: 转弯半径 (m)
        """
        if len(trajectory) < 3:
            return 0.0
        
        # 转换为 numpy 数组
        points = np.array(trajectory)
        x = points[:, 0]
        y = points[:, 1]
        
        # 使用最小二乘法拟合圆
        # 圆的方程: (x - cx)^2 + (y - cy)^2 = r^2
        # 展开: x^2 + y^2 - 2*cx*x - 2*cy*y + cx^2 + cy^2 - r^2 = 0
        # 令 A = -2*cx, B = -2*cy, C = cx^2 + cy^2 - r^2
        # 则: x^2 + y^2 + A*x + B*y + C = 0
        
        # 构建矩阵
        n = len(x)
        X = np.column_stack([x, y, np.ones(n)])
        Y = -(x**2 + y**2)
        
        # 求解 [A, B, C]
        try:
            params = np.linalg.lstsq(X, Y, rcond=None)[0]
            A, B, C = params
            
            # 计算圆心和半径
            cx = -A / 2
            cy = -B / 2
            radius = math.sqrt(cx**2 + cy**2 - C)
            
            return radius
        except:
            return 0.0

# ==============================================================================
# 轨迹绘图画布
# ==============================================================================
class TrajectoryCanvas(FigureCanvas):
    """
    轨迹绘图画布
    使用 matplotlib 绘制机器人运动轨迹
    """
    
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        # 创建图形
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = self.fig.add_subplot(111)
        
        super().__init__(self.fig)
        self.setParent(parent)
        
        # 初始化绘图
        self.axes.set_xlabel('X (m)')
        self.axes.set_ylabel('Y (m)')
        self.axes.set_title('机器人运动轨迹')
        self.axes.grid(True)
        self.axes.axis('equal')
    
    def plot_trajectory(self, trajectory, title='机器人运动轨迹'):
        """
        绘制轨迹
        
        参数：
            trajectory: [(x1, y1), (x2, y2), ...]
            title: 图表标题
        """
        self.axes.clear()
        
        if len(trajectory) > 0:
            points = np.array(trajectory)
            x = points[:, 0]
            y = points[:, 1]
            
            # 绘制轨迹
            self.axes.plot(x, y, 'b-', linewidth=2, label='轨迹')
            
            # 标记起点和终点
            self.axes.plot(x[0], y[0], 'go', markersize=10, label='起点')
            self.axes.plot(x[-1], y[-1], 'ro', markersize=10, label='终点')
            
            self.axes.legend()
        
        self.axes.set_xlabel('X (m)')
        self.axes.set_ylabel('Y (m)')
        self.axes.set_title(title)
        self.axes.grid(True)
        self.axes.axis('equal')
        
        self.draw()
    
    def plot_turning_radius(self, trajectory, theoretical_radius, actual_radius):
        """
        绘制转弯半径测试结果
        
        参数：
            trajectory: [(x1, y1), (x2, y2), ...]
            theoretical_radius: 理论转弯半径
            actual_radius: 实际转弯半径
        """
        self.axes.clear()
        
        if len(trajectory) > 0:
            points = np.array(trajectory)
            x = points[:, 0]
            y = points[:, 1]
            
            # 绘制轨迹
            self.axes.plot(x, y, 'b-', linewidth=2, label='实际轨迹')
            
            # 标记起点和终点
            self.axes.plot(x[0], y[0], 'go', markersize=10, label='起点')
            self.axes.plot(x[-1], y[-1], 'ro', markersize=10, label='终点')
            
            # 绘制理论圆
            if theoretical_radius != float('inf') and actual_radius > 0:
                # 计算圆心（简化：假设从起点开始）
                cx = x[0]
                cy = y[0] - theoretical_radius
                
                theta = np.linspace(0, 2 * np.pi, 100)
                circle_x = cx + theoretical_radius * np.cos(theta)
                circle_y = cy + theoretical_radius * np.sin(theta)
                
                self.axes.plot(circle_x, circle_y, 'g--', linewidth=1, 
                             label=f'理论圆 (R={theoretical_radius:.2f}m)')
            
            self.axes.legend()
        
        self.axes.set_xlabel('X (m)')
        self.axes.set_ylabel('Y (m)')
        self.axes.set_title(f'转弯半径测试 (实际: {actual_radius:.2f}m)')
        self.axes.grid(True)
        self.axes.axis('equal')
        
        self.draw()

# ==============================================================================
# 主窗口
# ==============================================================================
class MainWindow(QMainWindow):
    """
    主窗口
    包含所有测试功能的 UI 界面
    """
    
    def __init__(self):
        super().__init__()
        
        # 初始化 ROS2
        rclpy.init()
        self.ros_node = VelocityPublisherNode()
        
        # 启动 ROS2 spin 线程
        self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_thread.start()
        
        # 测试线程
        self.test_thread = None

        # 初始化 test_type
        self.test_type = None
        
        # 测试结果
        self.linear_results = None
        self.angular_results = None
        self.turning_results = None
        
        # 初始化 UI
        self.init_ui()
        
        # 定时器：定期更新轨迹显示
        self.trajectory_timer = QTimer()
        self.trajectory_timer.timeout.connect(self.update_trajectory_display)
        self.trajectory_timer.start(500)  # 每 500ms 更新一次
    
    def spin_ros(self):
        """
        ROS2 spin 线程
        在后台持续处理 ROS2 消息
        """
        while rclpy.ok():
            rclpy.spin_once(self.ros_node, timeout_sec=0.1)
    
    def init_ui(self):
        """
        初始化用户界面
        """
        self.setWindowTitle('机器人运动学死区测试工具 v2.0')
        self.setGeometry(100, 100, 1400, 900)
        
        # 创建中心部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 主布局
        main_layout = QVBoxLayout()
        central_widget.setLayout(main_layout)
        
        # 顶部：急停按钮和状态显示
        top_layout = self.create_top_panel()
        main_layout.addLayout(top_layout)
        
        # 中间：选项卡（线速度、角速度、转弯半径）
        self.tab_widget = QTabWidget()
        main_layout.addWidget(self.tab_widget)
        
        # 创建各个测试选项卡
        self.create_linear_velocity_tab()
        self.create_angular_velocity_tab()
        self.create_turning_radius_tab()
        self.create_results_tab()

        # 选项卡切换事件绑定
        self.tab_widget.currentChanged.connect(self.on_tab_changed)
        
        # 底部：日志显示
        log_group = QGroupBox('测试日志')
        log_layout = QVBoxLayout()
        log_group.setLayout(log_layout)
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(200)
        log_layout.addWidget(self.log_text)
        
        main_layout.addWidget(log_group)
        
        # 底部按钮
        bottom_layout = QHBoxLayout()
        
        self.save_button = QPushButton('💾 保存测试结果')
        self.save_button.clicked.connect(self.save_results)
        self.save_button.setEnabled(False)
        bottom_layout.addWidget(self.save_button)
        
        self.config_button = QPushButton('⚙️  生成配置建议')
        self.config_button.clicked.connect(self.generate_config)
        self.config_button.setEnabled(False)
        bottom_layout.addWidget(self.config_button)
        
        bottom_layout.addStretch()
        
        self.about_button = QPushButton('ℹ️  关于')
        self.about_button.clicked.connect(self.show_about)
        bottom_layout.addWidget(self.about_button)
        
        main_layout.addLayout(bottom_layout)
    
    def create_top_panel(self):
        """
        创建顶部面板（急停按钮和状态显示）
        """
        layout = QHBoxLayout()
        
        # 急停按钮
        self.emergency_stop_button = QPushButton('🛑 紧急停止')
        self.emergency_stop_button.setStyleSheet(
            'QPushButton { background-color: #ff4444; color: white; '
            'font-size: 18px; font-weight: bold; padding: 15px; }'
            'QPushButton:hover { background-color: #cc0000; }'
        )
        self.emergency_stop_button.clicked.connect(self.emergency_stop)
        layout.addWidget(self.emergency_stop_button)
        
        # 状态显示
        status_group = QGroupBox('系统状态')
        status_layout = QVBoxLayout()
        status_group.setLayout(status_layout)
        
        self.ros_status_label = QLabel('ROS2: ✓ 已连接')
        self.ros_status_label.setStyleSheet('color: green; font-weight: bold;')
        status_layout.addWidget(self.ros_status_label)

        self.test_status_label = QLabel('测试状态：空闲')
        status_layout.addWidget(self.test_status_label)

        layout.addWidget(status_group)
        layout.addStretch()
        return layout
    
    def create_linear_velocity_tab(self):
        """
        创建线速度测试选项卡的UI
        """
        tab = QWidget()
        layout = QVBoxLayout()
        tab.setLayout(layout)

        input_layout = QHBoxLayout()
        input_label = QLabel('线速度列表 (m/s，用逗号分隔):')
        self.linear_velocity_input = QLineEdit('0.20,0.15,0.12,0.10,0.08')
        input_layout.addWidget(input_label)
        input_layout.addWidget(self.linear_velocity_input)
        layout.addLayout(input_layout)

        duration_layout = QHBoxLayout()
        duration_label = QLabel('测试每速度持续时间 (秒):')
        self.linear_duration_input = QSpinBox()
        self.linear_duration_input.setMinimum(1)
        self.linear_duration_input.setMaximum(30)
        self.linear_duration_input.setValue(3)
        duration_layout.addWidget(duration_label)
        duration_layout.addWidget(self.linear_duration_input)
        layout.addLayout(duration_layout)

        threshold_layout = QHBoxLayout()
        threshold_label = QLabel('判定有效位移阈值 (米):')
        self.linear_threshold_input = QDoubleSpinBox()
        self.linear_threshold_input.setDecimals(3)
        self.linear_threshold_input.setMinimum(0.01)
        self.linear_threshold_input.setMaximum(0.5)
        self.linear_threshold_input.setValue(0.03)
        threshold_layout.addWidget(threshold_label)
        threshold_layout.addWidget(self.linear_threshold_input)
        layout.addLayout(threshold_layout)

        # 按钮区域
        button_layout = QHBoxLayout()
        self.linear_start_button = QPushButton('开始测试')
        self.linear_stop_button = QPushButton('停止测试')
        self.linear_stop_button.setEnabled(False)
        button_layout.addWidget(self.linear_start_button)
        button_layout.addWidget(self.linear_stop_button)
        layout.addLayout(button_layout)

        # 状态显示
        self.linear_progress_label = QLabel('未开始')
        layout.addWidget(self.linear_progress_label)

        # 按钮信号绑定
        self.linear_start_button.clicked.connect(self.start_linear_test)
        self.linear_stop_button.clicked.connect(self.stop_test)

        self.tab_widget.addTab(tab, '线速度测试')

    def create_angular_velocity_tab(self):
        """
        创建角速度测试选项卡的UI
        """
        tab = QWidget()
        layout = QVBoxLayout()
        tab.setLayout(layout)

        input_layout = QHBoxLayout()
        input_label = QLabel('角速度列表 (rad/s，用逗号分隔):')
        self.angular_velocity_input = QLineEdit('0.30,0.25,0.20,0.15,0.10')
        input_layout.addWidget(input_label)
        input_layout.addWidget(self.angular_velocity_input)
        layout.addLayout(input_layout)

        duration_layout = QHBoxLayout()
        duration_label = QLabel('测试每速度持续时间 (秒):')
        self.angular_duration_input = QSpinBox()
        self.angular_duration_input.setMinimum(1)
        self.angular_duration_input.setMaximum(30)
        self.angular_duration_input.setValue(3)
        duration_layout.addWidget(duration_label)
        duration_layout.addWidget(self.angular_duration_input)
        layout.addLayout(duration_layout)

        threshold_layout = QHBoxLayout()
        threshold_label = QLabel('判定有效旋转角度阈值 (度):')
        self.angular_threshold_input = QDoubleSpinBox()
        self.angular_threshold_input.setDecimals(1)
        self.angular_threshold_input.setMinimum(1.0)
        self.angular_threshold_input.setMaximum(180.0)
        self.angular_threshold_input.setValue(15.0)
        threshold_layout.addWidget(threshold_label)
        threshold_layout.addWidget(self.angular_threshold_input)
        layout.addLayout(threshold_layout)

        # 按钮区域
        button_layout = QHBoxLayout()
        self.angular_start_button = QPushButton('开始测试')
        self.angular_stop_button = QPushButton('停止测试')
        self.angular_stop_button.setEnabled(False)
        button_layout.addWidget(self.angular_start_button)
        button_layout.addWidget(self.angular_stop_button)
        layout.addLayout(button_layout)

        self.angular_progress_label = QLabel('未开始')
        layout.addWidget(self.angular_progress_label)

        # 按钮信号绑定
        self.angular_start_button.clicked.connect(self.start_angular_test)
        self.angular_stop_button.clicked.connect(self.stop_test)

        self.tab_widget.addTab(tab, '角速度测试')

    def create_turning_radius_tab(self):
        """
        创建转弯半径测试选项卡UI
        """
        tab = QWidget()
        layout = QVBoxLayout()
        tab.setLayout(layout)

        input_vel_layout = QHBoxLayout()
        linear_label = QLabel('线速度列表 (m/s，用逗号分隔):')
        self.turn_linear_input = QLineEdit('0.25,0.20,0.15')
        input_vel_layout.addWidget(linear_label)
        input_vel_layout.addWidget(self.turn_linear_input)
        layout.addLayout(input_vel_layout)

        input_ang_layout = QHBoxLayout()
        angular_label = QLabel('角速度列表 (rad/s，用逗号分隔):')
        self.turn_angular_input = QLineEdit('0.15,0.30,0.60')
        input_ang_layout.addWidget(angular_label)
        input_ang_layout.addWidget(self.turn_angular_input)
        layout.addLayout(input_ang_layout)

        duration_layout = QHBoxLayout()
        duration_label = QLabel('测试每组合持续时间(秒):')
        self.turn_duration_input = QSpinBox()
        self.turn_duration_input.setMinimum(1)
        self.turn_duration_input.setMaximum(30)
        self.turn_duration_input.setValue(5)
        duration_layout.addWidget(duration_label)
        duration_layout.addWidget(self.turn_duration_input)
        layout.addLayout(duration_layout)

        button_layout = QHBoxLayout()
        self.turn_start_button = QPushButton('开始测试')
        self.turn_stop_button = QPushButton('停止测试')
        self.turn_stop_button.setEnabled(False)
        button_layout.addWidget(self.turn_start_button)
        button_layout.addWidget(self.turn_stop_button)
        layout.addLayout(button_layout)

        self.turn_progress_label = QLabel('未开始')
        layout.addWidget(self.turn_progress_label)

        # 轨迹绘图画布
        self.turn_figure_canvas = TrajectoryCanvas(self, width=5, height=4, dpi=100)
        layout.addWidget(self.turn_figure_canvas)

        # 按钮信号绑定
        self.turn_start_button.clicked.connect(self.start_turn_test)
        self.turn_stop_button.clicked.connect(self.stop_test)

        self.tab_widget.addTab(tab, '转弯半径测试')

    def create_results_tab(self):
        """
        创建测试结果展示选项卡
        """
        tab = QWidget()
        layout = QVBoxLayout()
        tab.setLayout(layout)

        self.results_text = QTextEdit()
        self.results_text.setReadOnly(True)
        layout.addWidget(self.results_text)

        self.tab_widget.addTab(tab, '测试结果')

    def start_linear_test(self):
        """
        启动线速度测试
        """
        # 从输入框解析速度列表
        try:
            velocities = [float(v.strip()) for v in self.linear_velocity_input.text().split(',') if v.strip()]
        except Exception:
            QMessageBox.warning(self, '输入错误', '请检查线速度列表格式，只能是逗号分隔的数字')
            return

        duration = self.linear_duration_input.value()
        threshold = self.linear_threshold_input.value()

        params = {
            'velocities': velocities,
            'duration': duration,
            'threshold': threshold
        }
        self.start_test('linear', params)

    def start_angular_test(self):
        """
        启动角速度测试
        """
        try:
            velocities = [float(v.strip()) for v in self.angular_velocity_input.text().split(',') if v.strip()]
        except Exception:
            QMessageBox.warning(self, '输入错误', '请检查角速度列表格式，只能是逗号分隔的数字')
            return

        duration = self.angular_duration_input.value()
        threshold_deg = self.angular_threshold_input.value()
        threshold_rad = math.radians(threshold_deg)

        params = {
            'velocities': velocities,
            'duration': duration,
            'threshold': threshold_rad
        }
        self.start_test('angular', params)

    def start_turn_test(self):
        """
        启动转弯半径测试
        """
        try:
            linear_vels = [float(v.strip()) for v in self.turn_linear_input.text().split(',') if v.strip()]
            angular_vels = [float(v.strip()) for v in self.turn_angular_input.text().split(',') if v.strip()]
        except Exception:
            QMessageBox.warning(self, '输入错误', '请检查线速度和角速度列表格式，只能是逗号分隔的数字')
            return

        duration = self.turn_duration_input.value()

        # 自动生成所有组合
        test_cases = []
        for lv in linear_vels:
            for av in angular_vels:
                test_cases.append((lv, av))

        params = {
            'test_cases': test_cases,
            'duration': duration
        }
        self.start_test('turning', params)

    def start_test(self, test_type, params):
        """
          启动测试线程并设置按钮状态
        """
        if self.test_thread is not None and self.test_thread.isRunning():
            QMessageBox.warning(self, '测试进行中', '已有测试在运行，请先停止当前测试')
            return
 
        self.test_type = test_type

        # 禁用所有开始按钮，启用对应停止按钮，禁用其他停止按钮
        self.set_buttons_enabled(False, True, test_type)

        # 清空日志和状态
        self.log_text.clear()
        self.update_test_status('测试中...')
        self.update_progress(0, 1)
        self.plot_clear()

        # 创建测试线程
        self.test_thread = TestWorkerThread(self.ros_node, test_type, params)
        self.test_thread.log_signal.connect(self.append_log)
        self.test_thread.progress_signal.connect(self.update_progress)
        self.test_thread.result_signal.connect(self.handle_test_results)
        self.test_thread.finished_signal.connect(self.test_finished)

        self.test_thread.start()

    def stop_test(self):
        """
        停止当前测试线程
        """
        if self.test_thread is not None and self.test_thread.isRunning():
            # 第一优先级：先通知测试线程停止
           self.test_thread.stop()

           # 第二优先级：立即发布零速度
           self.ros_node.stop_robot()

           # 第三优先级：再写日志，日志失败也不会影响停车
           self.append_log('用户请求停止测试，已发送停止指令...')
           self.update_test_status('正在停止...')

           # 等待线程结束，最多 3 秒，避免界面永久卡死
           self.test_thread.wait(3000)

           # 再次确保机器人停止
           self.ros_node.stop_robot()

        else:
           self.ros_node.stop_robot()
           self.set_buttons_enabled(True, False)
           self.update_test_status('空闲')

    def emergency_stop(self):
        """
        紧急停止所有测试并立即停车
        """
        # 第一优先级：立即让线程进入急停状态
        if self.test_thread is not None and self.test_thread.isRunning():
            self.test_thread.emergency_stop_now()

        # 第二优先级：立即发布零速度
        self.ros_node.stop_robot()

        # 第三优先级：再写日志
        self.append_log('🛑 紧急停止触发！已停止测试并发布零速度')

        # 等待线程退出，最多 3 秒
        if self.test_thread is not None and self.test_thread.isRunning():
            self.test_thread.wait(3000)

        # 再次确保停车
        self.ros_node.stop_robot()

        # 重置按钮状态
        self.set_buttons_enabled(True, False)
        self.update_test_status('急停状态，机器人已停止')

    def set_buttons_enabled(self, start_enabled, stop_enabled, current_test=None):
        """
        设置所有测试按钮状态
        start_enabled: 是否启用所有开始按钮
        stop_enabled: 是否启用相应停止按钮
        current_test: 当前进行测试的类型，限制其他开始按钮
        """
        # 线速度按钮
        self.linear_start_button.setEnabled(start_enabled if current_test in (None, 'linear') else False)
        self.linear_stop_button.setEnabled(stop_enabled if current_test == 'linear' else False)
        # 角速度按钮
        self.angular_start_button.setEnabled(start_enabled if current_test in (None, 'angular') else False)
        self.angular_stop_button.setEnabled(stop_enabled if current_test == 'angular' else False)
        # 转弯半径按钮
        self.turn_start_button.setEnabled(start_enabled if current_test in (None, 'turning') else False)
        self.turn_stop_button.setEnabled(stop_enabled if current_test == 'turning' else False)


    def on_tab_changed(self, index):
        """
        选项卡切换事件，保证无测试运行时按钮恢复正常
        """
        if self.test_thread is None or not self.test_thread.isRunning():
            self.set_buttons_enabled(True, False)
            self.update_test_status('空闲')

    def append_log(self, message):
        try:
           timestr = datetime.now().strftime('[%H:%M:%S]')
           self.log_text.append(f'{timestr} {message}')
           self.log_text.ensureCursorVisible()
        except Exception as e:
           # 日志函数绝对不能影响主流程，尤其不能影响停止测试
           print(f'[append_log error] {e}')

    def update_test_status(self, status):
        """
        更新测试状态标签
        """
        self.test_status_label.setText(f'测试状态：{status}')

    def update_progress(self, current, total):
        """
        更新各测试进度显示
        """
        text = f'测试进度：{current} / {total}'
        if self.test_type == 'linear':
            self.linear_progress_label.setText(text)
        elif self.test_type == 'angular':
            self.angular_progress_label.setText(text)
        elif self.test_type == 'turning':
            self.turn_progress_label.setText(text)

    def handle_test_results(self, result):
        """
        处理测试结果信号，更新UI显示
        """
        if result['type'] == 'linear':
            self.linear_results = result
            self.append_log('线速度测试完成')
            self.results_text.append('=== 线速度测试结果 ===')
            for r in result['results']:
                status = '有效' if r['is_valid'] else '无效'
                self.results_text.append(
                    f"速度: {r['velocity']:.3f} m/s, 位移: {r['displacement']:.3f} m, 结果: {status}")
            if result['min_valid'] is not None:
                self.results_text.append(f"最小有效线速度: {result['min_valid']:.3f} m/s\n")
            else:
                self.results_text.append('未找到有效线速度\n')

        elif result['type'] == 'angular':
            self.angular_results = result
            self.append_log('角速度测试完成')
            self.results_text.append('=== 角速度测试结果 ===')
            for r in result['results']:
                status = '有效' if r['is_valid'] else '无效'
                disp = '有位移' if r['has_displacement'] else '无位移'
                self.results_text.append(
                    f"速度: {r['velocity']:.3f} rad/s, 角度变化: {math.degrees(r['yaw_change']):.1f}°, "
                    f"位移: {r['displacement']:.3f} m, {status}, {disp}")
            if result['min_valid'] is not None:
                self.results_text.append(f"最小有效角速度: {result['min_valid']:.3f} rad/s\n")
                if result['has_displacement']:
                    self.results_text.append("警告：旋转时有明显位移，非理想原地旋转\n")
            else:
                self.results_text.append('未找到有效角速度\n')

        elif result['type'] == 'turning':
            self.turning_results = result
            self.append_log('转弯半径测试完成')
            self.results_text.append('=== 转弯半径测试结果 ===')
            for r in result['results']:
                self.results_text.append(
                    f"线速度: {r['linear_vel']:.3f} m/s, 角速度: {r['angular_vel']:.3f} rad/s, "
                    f"理论半径: {r['theoretical_radius']:.3f} m, 实际半径: {r['actual_radius']:.3f} m, "
                    f"误差: {r['error']:.1f}%")
                # 绘制最后一个轨迹
                self.plot_trajectory(r['trajectory'], r['theoretical_radius'], r['actual_radius'])

        self.save_button.setEnabled(True)
        self.config_button.setEnabled(True)

    def test_finished(self):
        """
        测试线程结束回调，恢复按钮状态
        """
        was_emergency = False
        was_stopped = False

        if self.test_thread is not None:
            was_emergency = self.test_thread.emergency_stop
            was_stopped = self.test_thread.is_stopped

        if was_emergency:
            self.append_log('🛑 急停处理完成')
            self.update_test_status('急停状态，机器人已停止')
        elif was_stopped:
            self.append_log('测试已被用户停止')
            self.update_test_status('空闲')
        else:
            self.append_log('测试结束')
            self.update_test_status('空闲')

        self.ros_node.stop_robot()

        # 重新启用所有开始按钮，禁用所有停止按钮
        self.set_buttons_enabled(True, False)
        self.update_progress(0, 0)

        # 清理测试线程引用
        self.test_thread = None

    def update_trajectory_display(self):
        """
        定时器回调，实时更新轨迹显示（只在转弯测试选项卡显示轨迹）
        """
        if self.test_type == 'turning' and self.turn_figure_canvas is not None:
            trajectory = self.ros_node.get_trajectory()
            self.turn_figure_canvas.plot_trajectory(trajectory)

    def plot_clear(self):
        """
        清空轨迹绘图
        """
        if self.turn_figure_canvas is not None:
            self.turn_figure_canvas.axes.clear()
            self.turn_figure_canvas.draw()

    def plot_trajectory(self, trajectory, theoretical_radius, actual_radius):
        """
        绘制转弯半径测试轨迹和圆形辅助图形
        """
        if self.turn_figure_canvas is not None:
            self.turn_figure_canvas.plot_turning_radius(trajectory, theoretical_radius, actual_radius)

    def save_results(self):
        """
        保存测试结果到 CSV 文件
        """
        options = QFileDialog.Options()
        filename, _ = QFileDialog.getSaveFileName(
            self, "保存测试结果", "", "CSV 文件 (*.csv);;所有文件 (*)", options=options)
        if filename:
            try:
                with open(filename, 'w', encoding='utf-8') as f:
                    f.write('测试类型,参数1,参数2,理论半径,实际半径,误差,位移,有效性,备注\n')
                    # 保存线速度测试结果
                    if self.linear_results:
                        for r in self.linear_results['results']:
                            line = f"线速度,{r['velocity']:.3f},,," \
                                   f",{r['displacement']:.3f},,{r['is_valid']},\n"
                            f.write(line)
                    # 保存角速度测试结果
                    if self.angular_results:
                        for r in self.angular_results['results']:
                            line = f"角速度,,{r['velocity']:.3f},,,{r['displacement']:.3f},{r['is_valid']}," \
                                   f"{'有位移' if r['has_displacement'] else '无位移'}\n"
                            f.write(line)
                    # 保存转弯半径测试结果
                    if self.turning_results:
                        for r in self.turning_results['results']:
                            line = f"转弯半径,{r['linear_vel']:.3f},{r['angular_vel']:.3f}," \
                                   f"{r['theoretical_radius']:.3f},{r['actual_radius']:.3f},{r['error']:.1f},,,\n"
                            f.write(line)
                QMessageBox.information(self, '保存成功', f'测试结果保存到: {filename}')
            except Exception as e:
                QMessageBox.critical(self, '保存失败', f'保存文件出错: {str(e)}')

    def generate_config(self):
        """
        根据测试结果生成 Nav2 配置建议，并显示在日志区
        """
        self.append_log('\n=== Nav2 配置建议 ===')
        # 线速度相关建议
        if self.linear_results and self.linear_results['min_valid']:
            min_lin = self.linear_results['min_valid']
            min_x_vel_thr = max(0.05, min_lin - 0.02)
            desired_vel = min(0.30, min_lin * 2)
            self.append_log(f'# 线速度阈值和期望速度')
            self.append_log(f'min_x_velocity_threshold: {min_x_vel_thr:.2f}')
            self.append_log(f'min_approach_linear_velocity: {min_lin:.2f}')
            self.append_log(f'desired_linear_vel: {desired_vel:.2f}')
        
        # 角速度相关建议
        if self.angular_results and self.angular_results['min_valid']:
            min_ang = self.angular_results['min_valid']
            min_theta_thr = max(0.05, min_ang - 0.02)
            self.append_log(f'\n# 角速度阈值')
            self.append_log(f'min_theta_velocity_threshold: {min_theta_thr:.2f}')
            self.append_log(f'rotate_to_heading_angular_vel: {min_ang:.2f}')
            
            # 判断是否支持原地旋转
            if self.angular_results['has_displacement']:
                self.append_log(f'use_rotate_to_heading: false  # 不支持原地旋转')
            else:
                self.append_log(f'use_rotate_to_heading: true   # 支持原地旋转')
        
        # 转弯半径相关建议
        if self.turning_results:
            # 找出实际转弯半径的最小值
            min_radius = float('inf')
            for r in self.turning_results['results']:
                if r['actual_radius'] > 0:
                    min_radius = min(min_radius, r['actual_radius'])
            
            if min_radius != float('inf'):
                # 建议规划器的最小转弯半径设置为实际测量值的 80%
                suggested_radius = min_radius * 0.8
                self.append_log(f'\n# 转弯半径配置')
                self.append_log(f'minimum_turning_radius: {suggested_radius:.2f}  # 实际最小: {min_radius:.2f}m')
                
                # 前瞻距离建议
                lookahead = max(0.3, suggested_radius * 1.5)
                self.append_log(f'lookahead_dist: {lookahead:.2f}')
                self.append_log(f'min_lookahead_dist: {lookahead * 0.75:.2f}')
                self.append_log(f'max_lookahead_dist: {lookahead * 1.5:.2f}')
        
        # 曲率调速建议
        self.append_log(f'\n# 曲率调速配置')
        self.append_log(f'use_regulated_linear_velocity_scaling: true')
        if self.turning_results:
            if min_radius != float('inf'):
                scaling_radius = min_radius * 1.5
                self.append_log(f'regulated_linear_scaling_min_radius: {scaling_radius:.2f}')
        if self.linear_results and self.linear_results['min_valid']:
            self.append_log(f'regulated_linear_scaling_min_speed: {self.linear_results["min_valid"]:.2f}')
        
        self.append_log('=== 配置建议生成完成 ===\n')
        
        # 同时显示在结果选项卡
        self.results_text.append('\n=== Nav2 配置建议 ===')
        if self.linear_results and self.linear_results['min_valid']:
            min_lin = self.linear_results['min_valid']
            min_x_vel_thr = max(0.05, min_lin - 0.02)
            desired_vel = min(0.30, min_lin * 2)
            self.results_text.append(f'min_x_velocity_threshold: {min_x_vel_thr:.2f}')
            self.results_text.append(f'min_approach_linear_velocity: {min_lin:.2f}')
            self.results_text.append(f'desired_linear_vel: {desired_vel:.2f}')
        
        if self.angular_results and self.angular_results['min_valid']:
            min_ang = self.angular_results['min_valid']
            min_theta_thr = max(0.05, min_ang - 0.02)
            self.results_text.append(f'min_theta_velocity_threshold: {min_theta_thr:.2f}')
            self.results_text.append(f'rotate_to_heading_angular_vel: {min_ang:.2f}')
            if self.angular_results['has_displacement']:
                self.results_text.append(f'use_rotate_to_heading: false')
            else:
                self.results_text.append(f'use_rotate_to_heading: true')
        
        if self.turning_results:
            min_radius = float('inf')
            for r in self.turning_results['results']:
                if r['actual_radius'] > 0:
                    min_radius = min(min_radius, r['actual_radius'])
            
            if min_radius != float('inf'):
                suggested_radius = min_radius * 0.8
                self.results_text.append(f'minimum_turning_radius: {suggested_radius:.2f}')
                lookahead = max(0.3, suggested_radius * 1.5)
                self.results_text.append(f'lookahead_dist: {lookahead:.2f}')
        
        self.results_text.append('')

    def show_about(self):
        """
        显示关于对话框
        """
        about_text = """
        <h2>机器人运动学死区测试工具 v2.0</h2>
        <p><b>功能：</b></p>
        <ul>
            <li>线速度死区测试</li>
            <li>角速度死区测试</li>
            <li>转弯半径测试</li>
            <li>实时轨迹绘制</li>
            <li>自动生成 Nav2 配置建议</li>
        </ul>
        <p><b>使用说明：</b></p>
        <ol>
            <li>确保 ROS2 节点正常运行</li>
            <li>在各选项卡输入测试参数</li>
            <li>点击"开始测试"按钮</li>
            <li>观察机器人运动并查看测试结果</li>
            <li>紧急情况下点击"紧急停止"按钮</li>
            <li>测试完成后可保存结果和生成配置建议</li>
        </ol>
        <p><b>注意事项：</b></p>
        <ul>
            <li>测试前确保机器人周围有足够空间</li>
            <li>随时准备使用紧急停止按钮</li>
            <li>建议在安全环境下进行测试</li>
        </ul>
        <p><b>作者：</b>AI Assistant</p>
        <p><b>日期：</b>2025</p>
        """
        QMessageBox.about(self, '关于', about_text)

    def closeEvent(self, event):
        """
        窗口关闭事件处理
        确保安全关闭所有线程和 ROS2 节点
        """
        # 停止测试线程
        if self.test_thread is not None and self.test_thread.isRunning():
            self.test_thread.stop()
            self.test_thread.wait()
        
        # 停止机器人
        self.ros_node.stop_robot()
        
        # 停止定时器
        self.trajectory_timer.stop()
        
        # 关闭 ROS2
        self.ros_node.destroy_node()
        rclpy.shutdown()
        
        event.accept()

# ==============================================================================
# 主函数
# ==============================================================================
def main():
    """
    主函数
    创建 Qt 应用并启动主窗口
    """
    app = QApplication(sys.argv)
    
    # 设置应用样式
    app.setStyle('Fusion')
    
    # 设置全局字体
    font = QFont('Microsoft YaHei', 10)
    app.setFont(font)
    
    # 创建主窗口
    window = MainWindow()
    window.show()
    
    # 运行应用
    sys.exit(app.exec())

if __name__ == '__main__':
    main()