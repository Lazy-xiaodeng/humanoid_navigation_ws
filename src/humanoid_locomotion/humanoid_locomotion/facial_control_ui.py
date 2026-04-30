# -*- coding: utf-8 -*-
import sys
import rclpy
from std_msgs.msg import String
from PyQt5.QtWidgets import (    QApplication, QMainWindow, QWidget, QPushButton,
    QVBoxLayout, QHBoxLayout, QGroupBox, QStatusBar
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QPalette, QColor

class RosSenderThread(QThread):
    status_signal = pyqtSignal(str)
    def __init__(self, action_name):
        super().__init__()
        self.action_name = action_name

    def run(self):
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            node = rclpy.create_node('facial_ui_sender')
            from std_msgs.msg import String
            publisher = node.create_publisher(
                String,
                '/robot/facial_raw_cmd',
                10
            )
            msg = String()
            msg.data = self.action_name
            publisher.publish(msg)
            self.status_signal.emit(f"指令发送成功：{self.action_name}")
        except Exception as e:
            self.status_signal.emit(f"指令发送失败：{str(e)}")
        finally:
            if 'node' in locals():
                node.destroy_node()

class FacialControlUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.current_thread = None

    # # 在 FacialControlUI 类里添加或修改 closeEvent 方法
    # def closeEvent(self, event):
    #     """窗口关闭时自动发送 sleep 指令，让机器进入睡眠状态"""
    #     try:
    #         # 1. 发送睡眠指令
    #         if rclpy.ok():
    #             # 临时创建发布者发送 sleep 指令
    #             node = rclpy.create_node('ui_shutdown_sender')
    #             publisher = node.create_publisher(String, '/robot/facial_raw_cmd', 10)
    #             msg = String()
    #             msg.data = "sleeping"
    #             publisher.publish(msg)
    #             node.get_logger().info("✅ 窗口关闭，已发送睡眠指令")
    #             # 短暂等待，确保指令发送到驱动
    #             time.sleep(0.5)
    #             node.destroy_node()
    #     except Exception as e:
    #         print(f"⚠️ 关闭时发送睡眠指令失败: {e}")
    #     finally:
    #         # 2. 优雅关闭 ROS 节点
    #         RosSenderThread.shutdown_ros_node()
    #         self.status_timer.stop()
    #         event.accept()

    def init_ui(self):
        self.setWindowTitle("Robots Facial Expression Control System")
        self.setGeometry(100, 100, 850, 550)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(25)
        main_layout.setContentsMargins(40, 40, 40, 30)

        basic_group = QGroupBox("基础状态")
        basic_group.setStyleSheet("QGroupBox { font-size: 40px; font-weight: bold; margin-top: 8px; }")
        basic_layout = QHBoxLayout(basic_group)
        self.btn_idle = self.create_button("待机 (idle)", self.send_action, "idle")
        self.btn_sleeping = self.create_button("睡眠 (sleeping)", self.send_action, "sleeping")
        self.btn_stop = self.create_button("停止动作 (stop)", self.send_action, "stop", is_danger=True)
        basic_layout.addWidget(self.btn_idle)
        basic_layout.addWidget(self.btn_sleeping)
        basic_layout.addWidget(self.btn_stop)

        emotion_group = QGroupBox("情绪表情")
        emotion_group.setStyleSheet("QGroupBox { font-size: 40px; font-weight: bold; margin-top: 8px; }")
        emotion_layout = QHBoxLayout(emotion_group)
        self.btn_surprised = self.create_button("惊讶 (surprised)", self.send_action, "surprised")
        self.btn_excited = self.create_button("兴奋 (excited)", self.send_action, "excited")
        self.btn_sad = self.create_button("伤心 (sad)", self.send_action, "sad")
        emotion_layout.addWidget(self.btn_surprised)
        emotion_layout.addWidget(self.btn_excited)
        emotion_layout.addWidget(self.btn_sad)

        think_group = QGroupBox("思考表情")
        think_group.setStyleSheet("QGroupBox { font-size: 40px; font-weight: bold; margin-top: 8px; }")
        think_layout = QHBoxLayout(think_group)
        self.btn_thinking1 = self.create_button("思考1 (thinking1)", self.send_action, "thinking1")
        self.btn_thinking2 = self.create_button("思考2 (thinking2)", self.send_action, "thinking2")
        think_layout.addWidget(self.btn_thinking1)
        think_layout.addWidget(self.btn_thinking2)

        interact_group = QGroupBox("交互动作")
        interact_group.setStyleSheet("QGroupBox { font-size: 40px; font-weight: bold; margin-top: 8px; }")
        interact_layout = QHBoxLayout(interact_group)
        self.btn_talk = self.create_button("说话 (talk)", self.send_action, "talk")
        self.btn_eyeball_lr = self.create_button("眼球左右转 (eyeball_lr)", self.send_action, "eyeball_lr")
        interact_layout.addWidget(self.btn_talk)
        interact_layout.addWidget(self.btn_eyeball_lr)

        main_layout.addWidget(basic_group)
        main_layout.addWidget(emotion_group)
        main_layout.addWidget(think_group)
        main_layout.addWidget(interact_group)

        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("硬件连接状态：已连接O | 当前执行动作：无")
        self.status_bar.setStyleSheet("QStatusBar { font-size: 26px; padding: 5px; }")

        self.setStyleSheet("""
            QMainWindow {
                background-color: #f8f9fa;
            }
            QPushButton {
                border-radius: 8px;
                font-size: 38px;
            }
        """)

    def create_button(self, text, callback, action_name, is_danger=False):
        btn = QPushButton(text)
        btn.setMinimumSize(160, 120)
        if is_danger:
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #dc3545;
                    color: white;
                    border: none;
                }
                QPushButton:hover {
                    background-color: #c82333;
                }
                QPushButton:pressed {
                    background-color: #a71d2a;
                }
            """)
        else:
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #007bff;
                    color: white;
                    border: none;
                }
                QPushButton:hover {
                    background-color: #0056b3;
                }
                QPushButton:pressed {
                    background-color: #004085;
                }
            """)
        btn.clicked.connect(lambda: callback(action_name))
        return btn

    def send_action(self, action_name):
        self.status_bar.showMessage(f"硬件连接状态：已连接 | 当前执行动作：{action_name}")
        self.current_thread = RosSenderThread(action_name)
        self.current_thread.status_signal.connect(self.update_status)
        self.current_thread.finished.connect(lambda: self.set_all_buttons_enabled(True))
        self.current_thread.start()

    def update_status(self, msg):
        base_status = self.status_bar.currentMessage().split("|")[0]
        self.status_bar.showMessage(f"{base_status} | {msg}")

    def set_all_buttons_enabled(self, enabled):
        self.btn_idle.setEnabled(enabled)
        self.btn_sleeping.setEnabled(enabled)
        self.btn_stop.setEnabled(enabled)
        self.btn_surprised.setEnabled(enabled)
        self.btn_excited.setEnabled(enabled)
        self.btn_sad.setEnabled(enabled)
        self.btn_thinking1.setEnabled(enabled)
        self.btn_thinking2.setEnabled(enabled)
        self.btn_talk.setEnabled(enabled)
        self.btn_eyeball_lr.setEnabled(enabled)

def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(248, 249, 250))
    app.setPalette(palette)
    window = FacialControlUI()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
