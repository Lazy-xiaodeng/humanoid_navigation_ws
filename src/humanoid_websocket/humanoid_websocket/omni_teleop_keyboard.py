#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select
import threading

class OmniTeleopKeyboard(Node):
    def __init__(self):
        super().__init__('omni_teleop_keyboard')
        
        # 创建速度发布者
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 基础速度配置 (m/s 和 rad/s)
        self.speed_x = 0.3    # 前后
        self.speed_y = 0.2    # 左右
        self.speed_z = 0.4    # 旋转
        self.speed_step = 0.05 # 步长

        # 存储当前激活的运动指令
        self.active_commands = set()
        
        # 保存终端原始设置，用于程序退出时恢复
        self.settings = termios.tcgetattr(sys.stdin)
        
        # 启动高频发送定时器 (50Hz)
        self.timer = self.create_timer(0.02, self.publish_velocity)
        
        self.print_usage()
        
        # 启动按键读取循环线程 (非全局监听，仅限本窗口)
        self.running = True
        self.thread = threading.Thread(target=self.run_keyboard_handler)
        self.thread.daemon = True
        self.thread.start()

    def print_usage(self):
        usage = """
        ========================================
        🤖 人形机器人全向移动控制器 (窗口聚焦模式)
        ========================================
        移动控制 (支持多键组合，如同时按W+A):
               W (前进)
        A (左移)   S (后退)   D (右移)
        
        Q (左转)              E (右转)
        
        速度调节:
        U / J : 增加 / 减小 整体速度
        
        其它:
        Space : 紧急刹车归零
        Ctrl+C: 退出程序 (请在本窗口操作)
        ========================================
        💡 提示：按键仅在当前终端窗口“被选中/聚焦”时生效。
        """
        print(usage)
        self.print_current_speed()

    def print_current_speed(self):
        sys.stdout.write(f"\r当前速度设定: 前后 [ {self.speed_x:.2f} ] | 左右 [ {self.speed_y:.2f} ] | 旋转 [ {self.speed_z:.2f} ]   ")
        sys.stdout.flush()

    def getKey(self):
        """读取单个按键字符，非阻塞模式"""
        tty.setraw(sys.stdin.fileno())
        # 设定 select 等待时间极短，确保护持 50Hz 的灵敏度
        rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_keyboard_handler(self):
        """按键处理逻辑：由于标准 Linux 终端不区分 Press/Release，
        我们采用快速轮询方式，若 0.1s内未收到重复按键则视为松开"""
        last_key_time = {}
        timeout = 0.15 # 判定松开的阈值(秒)

        while self.running:
            key = self.getKey()
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            if key != '':
                char = key.lower()
                
                # 速度调节逻辑
                if char == 'u':
                    self.speed_x = min(1.0, self.speed_x + self.speed_step)
                    self.speed_y = min(1.0, self.speed_y + self.speed_step)
                    self.speed_z = min(1.0, self.speed_z + self.speed_step)
                    self.print_current_speed()
                elif char == 'j':
                    self.speed_x = max(0.05, self.speed_x - self.speed_step)
                    self.speed_y = max(0.05, self.speed_y - self.speed_step)
                    self.speed_z = max(0.05, self.speed_z - self.speed_step)
                    self.print_current_speed()
                
                # 记录按键活跃状态
                if char in ['w', 'a', 's', 'd', 'q', 'e', ' ']:
                    cmd = 'space' if char == ' ' else char
                    self.active_commands.add(cmd)
                    last_key_time[cmd] = current_time
                
                # 退出指令
                if key == '\x03': # Ctrl+C
                    self.running = False
                    break

            # 自动清理过期的按键（实现“松开即停”）
            expired = []
            for cmd, l_time in last_key_time.items():
                if current_time - l_time > timeout:
                    expired.append(cmd)
            
            for cmd in expired:
                if cmd in self.active_commands:
                    self.active_commands.remove(cmd)
                del last_key_time[cmd]

    def publish_velocity(self):
        """定时发布速度指令 (50Hz)"""
        twist = Twist()
        
        if 'space' in self.active_commands:
            self.cmd_vel_pub.publish(twist)
            return

        # 组合运动逻辑
        if 'w' in self.active_commands: twist.linear.x += self.speed_x
        if 's' in self.active_commands: twist.linear.x -= self.speed_x
        if 'a' in self.active_commands: twist.linear.y += self.speed_y
        if 'd' in self.active_commands: twist.linear.y -= self.speed_y
        if 'q' in self.active_commands: twist.angular.z += self.speed_z
        if 'e' in self.active_commands: twist.angular.z -= self.speed_z

        # 安全幅值限制
        twist.linear.x = max(-1.0, min(float(twist.linear.x), 1.0))
        twist.linear.y = max(-1.0, min(float(twist.linear.y), 1.0))
        twist.angular.z = max(-1.0, min(float(twist.angular.z), 1.0))

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = OmniTeleopKeyboard()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 恢复终端设置
        node.running = False
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        
        # 发送停止指令
        stop_msg = Twist()
        node.cmd_vel_pub.publish(stop_msg)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()