# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import serial
import yaml
import os
from std_msgs.msg import String
import time
import glob  

class FacialDriver(Node):
    def __init__(self):
        super().__init__('facial_driver_node')

        # 1. 参数配置
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('config_path', 'src/humanoid_locomotion/config/facial_gestures.yaml') 
        
        port = self.get_parameter('port').value
        config_path = self.get_parameter('config_path').value
        
        # 2. 串口初始化
        try:
            self.ser = serial.Serial(port, 115200, timeout=0.1)
            self.get_logger().info(f"✅ 已连接到仿生头模块: {port}")
            time.sleep(2.0)
        except Exception as e:
            self.get_logger().error(f"❌ 头部串口连接失败: {e}")
            self.ser = None  # 修复：必须初始化 self.ser，否则后续访问会报 AttributeError


        # 3. 加载 YAML 动作库
        self.gestures = {}
        self.load_config(config_path)

        # 4. 订阅控制信号
        self.subscription = self.create_subscription(
            String,
            '/robot/facial_raw_cmd',
            self.callback,
            10
        )

        # 5. 中断控制变量
        self.is_interrupted = False
        self.current_gesture = "idle"

        # 6. 初始化：执行 idle 状态
        self.execute_gesture("idle")

    def load_config(self, path):
        """解析 YAML 配置文件"""
        try:
            if not os.path.isabs(path):
                path = os.path.join(os.getcwd(), path)

            if not os.path.exists(path):
                self.get_logger().error(f"❌ 配置文件不存在: {path}")
                return

            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                self.gestures = data.get('facial_gestures', {})
                self.get_logger().info(f"📚 成功加载表情库，共 {len(self.gestures)} 个动作")
        except Exception as e:
            self.get_logger().error(f"❌ 配置文件解析失败: {e}")

    def callback(self, msg):
        """处理收到的动作指令（支持中断）"""
        action_name = msg.data.strip()
        if not action_name:
            self.get_logger().warning("⚠️ 收到空指令，忽略")
            return

        self.get_logger().info(f"🛑 收到新指令，中断当前动作: {action_name}")
        self.is_interrupted = True
        time.sleep(0.5)  

        # 清空串口缓冲区，避免指令堆积
        if self.ser and self.ser.is_open:
            self.ser.flushInput()
            self.ser.flushOutput()
            self.get_logger().info("🧹 清空串口缓冲区，避免指令残留")

        self.is_interrupted = False
        self.current_gesture = action_name
        self.execute_gesture(action_name)

    def execute_gesture(self, name):
        """执行动作序列"""

        # 对所有复合/普通动作，先终止眼部循环；只有 eye_stop自己不需要前置停止
        if name in self.gestures and name not in ["eye_stop", "mouth_speak_stop"]:

            self.get_logger().info("🔍 先终止眼部循环和嘴部动作循环...")

            # 发送2次eye_stop指令，避免丢包
            if "eye_stop" in self.gestures:
                for _ in range(2):
                    for cmd in self.gestures["eye_stop"]:
                        self.send_raw(cmd)
                        time.sleep(0.1) #已修改原0.3
                self.get_logger().info("✅ 强制终止眼部循环（发送1次终止指令）")

            # 嘴部终止逻辑同步优化
            if "mouth_speak_stop" in self.gestures:
                for _ in range(1): #已修改原2
                    for cmd in self.gestures["mouth_speak_stop"]:
                        self.send_raw(cmd)
                        time.sleep(0.1) #已修改原0.3
                self.get_logger().info("✅ 强制终止嘴部循环（发送1次终止指令）")

            # 眼部强制复位（眼球居中+正常睁眼）
            eye_reset_cmds = ['$DGL:1!\n'] #已删除'$DGB:1!\n'
            for cmd in eye_reset_cmds:
                self.send_raw(cmd)
                time.sleep(0.1) #已修改原0.2
            self.get_logger().info("🔄 眼部强制复位（眼球居中+正常睁眼）")

        if name not in self.gestures:
            self.get_logger().warning(f"⚠️ 表情库中无 {name}，尝试作为原始指令发送")
            self.send_raw(name)
            return

        commands = self.gestures[name]
        self.get_logger().info(f"🎬 执行表情: {name}")

        if self.ser and self.ser.is_open:
            self.ser.flushInput()
            self.ser.flushOutput()

        for cmd in commands:
            if self.is_interrupted:
                self.get_logger().info(f"⚠️ 动作被中断，停止发送 {name} 的剩余指令")
                break

            # 解析自定义延迟标记（比如 delay:0.2）
            if cmd.startswith('delay:'):
                try:
                    delay_time = float(cmd.split(':')[1])
                    time.sleep(delay_time)
                    self.get_logger().info(f"⏳ 延迟 {delay_time} 秒（渐变过渡）")
                except:
                    self.get_logger().warning(f"⚠️ 无效延迟指令: {cmd}")
                continue

            # 原有逻辑：处理普通指令
            if name == "idle" and '!normal_start' in cmd:
                time.sleep(0.5)
                self.send_raw(cmd)
                time.sleep(0.8)
            else:
                self.send_raw(cmd)

                if name == "stop":
                    time.sleep(0.6)
                elif '!silent_start' in cmd or '!normal_start' in cmd:
                    time.sleep(0.8)
                else:
                    time.sleep(0.1)

    def send_raw(self, cmd_str):
        """发送原始指令（自动补全\r\n，兼容硬件）"""
        if not cmd_str or cmd_str.strip() == "":
            self.get_logger().warning("⚠️ 空指令，忽略发送")
            return

        if not (self.ser and self.ser.is_open):
            self.get_logger().error("❌ 串口未连接，无法发送指令")
            return

        try:
            send_cmd = cmd_str.strip() + "\r\n"
            self.ser.write(send_cmd.encode('utf-8'))
            self.get_logger().info(f"Serial Out: {send_cmd.strip()}")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ 串口发送失败: {e}")
        except Exception as e:
            self.get_logger().error(f"❌ 指令发送异常: {e}")

    def on_shutdown(self):
        """优雅关闭"""
        self.get_logger().info("🛑 开始优雅关闭...")
        if self.ser and self.ser.is_open:
            try:
                self.execute_gesture("sleeping")
                time.sleep(1.0)
                self.ser.close()
                self.get_logger().info("🔌 串口已安全关闭")
            except Exception as e:
                self.get_logger().error(f"❌ 关闭串口失败: {e}")
        else:
            self.get_logger().info("🔌 串口未连接，无需关闭")


def main(args=None):
    rclpy.init(args=args)
    node = FacialDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("⚠️ 收到键盘中断信号（Ctrl+C）")
    except Exception as e:
        node.get_logger().error(f"❌ 节点运行异常: {e}")
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()
        node.get_logger().info("✅ 节点已正常退出")


if __name__ == '__main__':
    main()