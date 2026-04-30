#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import yaml
import queue
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GestureControlNode(Node):
    """
    ROS2 后端节点：
    1. 发布 /app/robot_control 控制指令
    2. 订阅 /robot_status_raw 获取机器人状态
    """

    def __init__(self, log_queue):
        super().__init__('gesture_control_gui_plus_node')

        self.log_queue = log_queue

        self.robot_control_pub = self.create_publisher(
            String,
            '/app/robot_control',
            10
        )

        self.robot_status_sub = self.create_subscription(
            String,
            '/robot_status_raw',
            self.robot_status_callback,
            10
        )

        self.current_robot_state = "Unknown"
        self.last_status_time = 0.0
        self.has_robot_status = False

        self.log("✅ ROS2 GUI 节点已启动")
        self.log("✅ 发布器已创建: /app/robot_control")
        self.log("✅ 状态订阅已创建: /robot_status_raw")

    def log(self, text):
        try:
            self.get_logger().info(str(text))
        except Exception:
            pass
        self.log_queue.put(str(text))

    def robot_status_callback(self, msg: String):
        try:
            import time
            data = json.loads(msg.data)

            # 你的 websocket_client.py 发布的 /robot_status_raw 大致结构：
            # {
            #   "values": {
            #       "robot_status": "Walk"
            #   },
            #   ...
            # }
            values = data.get("values", {})
            state = values.get("robot_status", "Unknown")

            self.current_robot_state = str(state)
            self.last_status_time = time.time()
            self.has_robot_status = True

        except Exception as e:
            self.log(f"⚠️ 解析机器人状态失败: {e}")

    def send_gesture_command(self, gesture_id: str):
        """
        发送动作执行命令。
        这个格式与你 websocket_client.py 的 robot_control_callback() 匹配：
        {
            "command_type": "execute_gesture",
            "parameters": {
                "gesture_id": "nod"
            }
        }
        """
        try:
            cmd = {
                "command_type": "execute_gesture",
                "parameters": {
                    "gesture_id": gesture_id
                }
            }

            msg = String()
            msg.data = json.dumps(cmd, ensure_ascii=False)
            self.robot_control_pub.publish(msg)

            self.log(f"📤 已发送动作指令: {gesture_id}")

        except Exception as e:
            self.log(f"❌ 发送动作失败: {e}")

    def send_force_walk_command(self):
        """
        预留：发送回 Walk 模式指令。
        注意：你的 websocket_client.py 当前需要额外支持 force_walk_mode 才会生效。
        """
        try:
            cmd = {
                "command_type": "force_walk_mode",
                "parameters": {}
            }

            msg = String()
            msg.data = json.dumps(cmd, ensure_ascii=False)
            self.robot_control_pub.publish(msg)

            self.log("📤 已发送 force_walk_mode 指令")

        except Exception as e:
            self.log(f"❌ 发送 force_walk_mode 失败: {e}")

class GestureControlGUIPlus:
    def __init__(self, root, ros_node, yaml_path):
        self.root = root
        self.ros_node = ros_node
        self.yaml_path = yaml_path
        self.log_queue = ros_node.log_queue

        self.all_gestures = []
        self.filtered_gestures = []
        self.gesture_buttons = []

        self.is_busy = False
        self.busy_seconds = tk.IntVar(value=5)
        self.confirm_before_execute = tk.BooleanVar(value=True)

        self.search_var = tk.StringVar()
        self.type_filter_var = tk.StringVar(value="全部")

        self.selected_gesture = None

        self.root.title("机器人动作库控制面板 Plus")
        self.root.geometry("1250x760")
        self.root.minsize(1050, 650)

        self._setup_style()
        self._build_ui()

        self.load_gestures()

        self._poll_log_queue()
        self._update_status_bar()

    # ==================== UI 构建 ====================

    def _setup_style(self):
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except Exception:
            pass

        style.configure("Title.TLabel", font=("Arial", 20, "bold"))
        style.configure("SubTitle.TLabel", font=("Arial", 11))
        style.configure("Status.TLabel", font=("Arial", 10))
        style.configure("Small.TLabel", font=("Arial", 9))
        style.configure("Action.TButton", font=("Arial", 10))

    def _build_ui(self):
        # 顶部区域
        top_frame = ttk.Frame(self.root, padding=(12, 10))
        top_frame.pack(fill=tk.X)

        title = ttk.Label(
            top_frame,
            text="机器人动作库控制面板 Plus",
            style="Title.TLabel"
        )
        title.pack(side=tk.LEFT)

        self.status_light = tk.Canvas(top_frame, width=18, height=18, highlightthickness=0)
        self.status_light.pack(side=tk.RIGHT, padx=(8, 0))
        self.status_light_id = self.status_light.create_oval(3, 3, 15, 15, fill="gray")

        self.status_label = ttk.Label(
            top_frame,
            text="状态：初始化中",
            style="Status.TLabel"
        )
        self.status_label.pack(side=tk.RIGHT, padx=10)

        # 主体布局
        body = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        body.pack(fill=tk.BOTH, expand=True)

        left_panel = ttk.Frame(body)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 8))

        right_panel = ttk.Frame(body)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=False)
        right_panel.configure(width=390)

        # 左上：工具栏
        control_frame = ttk.LabelFrame(left_panel, text="控制区", padding=10)
        control_frame.pack(fill=tk.X, pady=(0, 8))

        ttk.Label(control_frame, text="搜索:").pack(side=tk.LEFT)
        search_entry = ttk.Entry(control_frame, textvariable=self.search_var, width=28)
        search_entry.pack(side=tk.LEFT, padx=(5, 12))
        search_entry.bind("<KeyRelease>", lambda event: self.apply_filter())

        ttk.Label(control_frame, text="分类:").pack(side=tk.LEFT)
        self.type_combo = ttk.Combobox(
            control_frame,
            textvariable=self.type_filter_var,
            width=16,
            state="readonly"
        )
        self.type_combo.pack(side=tk.LEFT, padx=(5, 12))
        self.type_combo.bind("<<ComboboxSelected>>", lambda event: self.apply_filter())

        ttk.Button(
            control_frame,
            text="刷新动作库",
            command=self.load_gestures
        ).pack(side=tk.LEFT, padx=4)

        ttk.Button(
            control_frame,
            text="清空搜索",
            command=self.clear_search
        ).pack(side=tk.LEFT, padx=4)

        ttk.Button(
            control_frame,
            text="回到行走模式",
            command=self.on_force_walk
        ).pack(side=tk.LEFT, padx=4)

        # 左中：选项
        options_frame = ttk.Frame(left_panel)
        options_frame.pack(fill=tk.X, pady=(0, 8))

        ttk.Checkbutton(
            options_frame,
            text="执行前确认",
            variable=self.confirm_before_execute
        ).pack(side=tk.LEFT, padx=(0, 15))

        ttk.Label(options_frame, text="执行后锁定按钮秒数:").pack(side=tk.LEFT)
        busy_spin = ttk.Spinbox(
            options_frame,
            from_=1,
            to=60,
            textvariable=self.busy_seconds,
            width=5
        )
        busy_spin.pack(side=tk.LEFT, padx=5)

        self.count_label = ttk.Label(
            options_frame,
            text="动作数量: 0",
            style="Small.TLabel"
        )
        self.count_label.pack(side=tk.RIGHT)

        # 左下：动作按钮滚动区
        actions_frame = ttk.LabelFrame(left_panel, text="动作按钮区", padding=10)
        actions_frame.pack(fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(actions_frame, highlightthickness=0)
        self.scrollbar = ttk.Scrollbar(actions_frame, orient="vertical", command=self.canvas.yview)
        self.button_container = ttk.Frame(self.canvas)

        self.button_container.bind(
            "<Configure>",
            lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        )

        self.canvas_window = self.canvas.create_window(
            (0, 0),
            window=self.button_container,
            anchor="nw"
        )

        self.canvas.configure(yscrollcommand=self.scrollbar.set)

        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.canvas.bind("<Configure>", self._on_canvas_configure)

        # 鼠标滚轮
        self.canvas.bind_all("<MouseWheel>", self._on_mousewheel)

        # 右侧：详情区
        detail_frame = ttk.LabelFrame(right_panel, text="动作详情", padding=10)
        detail_frame.pack(fill=tk.X, pady=(0, 8))

        self.detail_name_label = ttk.Label(
            detail_frame,
            text="未选择动作",
            font=("Arial", 14, "bold")
        )
        self.detail_name_label.pack(anchor="w", pady=(0, 5))

        self.detail_id_label = ttk.Label(detail_frame, text="ID: -")
        self.detail_id_label.pack(anchor="w")

        self.detail_type_label = ttk.Label(detail_frame, text="Type: -")
        self.detail_type_label.pack(anchor="w")

        self.detail_desc_label = ttk.Label(
            detail_frame,
            text="Description: -",
            wraplength=350,
            justify=tk.LEFT
        )
        self.detail_desc_label.pack(anchor="w", pady=(5, 0))

        ttk.Button(
            detail_frame,
            text="执行当前选中动作",
            command=self.execute_selected_gesture
        ).pack(fill=tk.X, pady=(10, 0))

        # 右侧：日志区
        log_frame = ttk.LabelFrame(right_panel, text="日志输出", padding=10)
        log_frame.pack(fill=tk.BOTH, expand=True)

        self.log_text = tk.Text(
            log_frame,
            wrap=tk.WORD,
            font=("Consolas", 10),
            height=20
        )
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        log_scroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        log_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.configure(yscrollcommand=log_scroll.set)

        log_btn_frame = ttk.Frame(right_panel)
        log_btn_frame.pack(fill=tk.X, pady=(6, 0))

        ttk.Button(log_btn_frame, text="清空日志", command=self.clear_log).pack(side=tk.LEFT)

        # 底部提示
        bottom = ttk.Frame(self.root, padding=(10, 4))
        bottom.pack(fill=tk.X)

        self.bottom_label = ttk.Label(
            bottom,
            text=f"动作库文件: {self.yaml_path}",
            foreground="gray"
        )
        self.bottom_label.pack(side=tk.LEFT)

    def _on_canvas_configure(self, event):
        self.canvas.itemconfig(self.canvas_window, width=event.width)

    def _on_mousewheel(self, event):
        try:
            self.canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
        except Exception:
            pass

    # ==================== 动作库加载与过滤 ====================

    def load_gestures(self):
        try:
            if not os.path.exists(self.yaml_path):
                raise FileNotFoundError(f"找不到动作库文件: {self.yaml_path}")

            with open(self.yaml_path, "r", encoding="utf-8") as f:
                config = yaml.safe_load(f)

            actions = config.get("actions", {})
            gestures = []

            for action_id, info in actions.items():
                if not isinstance(info, dict):
                    continue

                gestures.append({
                    "id": str(action_id),
                    "name": str(info.get("name", action_id)),
                    "type": str(info.get("type", "unknown")),
                    "description": str(info.get("description", "")),
                    "raw": info
                })

            gestures.sort(key=lambda x: x["id"])

            self.all_gestures = gestures

            # 更新分类下拉框
            types = sorted(list(set(g["type"] for g in gestures)))
            self.type_combo["values"] = ["全部"] + types
            if self.type_filter_var.get() not in self.type_combo["values"]:
                self.type_filter_var.set("全部")

            self.apply_filter()

            self.append_log(f"✅ 动作库加载成功，共 {len(self.all_gestures)} 个动作")
            self.append_log(f"📄 YAML: {self.yaml_path}")

        except Exception as e:
            messagebox.showerror("动作库加载失败", str(e))
            self.append_log(f"❌ 动作库加载失败: {e}")

    def apply_filter(self):
        keyword = self.search_var.get().strip().lower()
        type_filter = self.type_filter_var.get()

        result = []

        for g in self.all_gestures:
            if type_filter != "全部" and g["type"] != type_filter:
                continue

            haystack = " ".join([
                g["id"],
                g["name"],
                g["type"],
                g["description"]
            ]).lower()

            if keyword and keyword not in haystack:
                continue

            result.append(g)

        self.filtered_gestures = result
        self.render_gesture_buttons()

    def clear_search(self):
        self.search_var.set("")
        self.type_filter_var.set("全部")
        self.apply_filter()

    # ==================== 动作按钮渲染 ====================

    def render_gesture_buttons(self):
        for w in self.button_container.winfo_children():
            w.destroy()

        self.gesture_buttons.clear()

        count = len(self.filtered_gestures)
        self.count_label.config(
            text=f"动作数量: {count} / {len(self.all_gestures)}"
        )

        if count == 0:
            ttk.Label(
                self.button_container,
                text="没有匹配的动作",
                font=("Arial", 13)
            ).grid(row=0, column=0, padx=20, pady=20)
            return

        # 根据窗口宽度使用 3 列，比较稳
        columns = 3

        for idx, gesture in enumerate(self.filtered_gestures):
            row = idx // columns
            col = idx % columns

            frame = tk.Frame(
                self.button_container,
                bd=1,
                relief=tk.RIDGE,
                bg="#FFFFFF"
            )
            frame.grid(row=row, column=col, padx=8, pady=8, sticky="nsew")

            name = gesture["name"]
            gid = gesture["id"]

            btn_text = f"{name}\n{gid}"

            btn = tk.Button(
                frame,
                text=btn_text,
                width=24,
                height=4,
                bg="#EAF4FF",
                activebackground="#BFDFFF",
                fg="#000000",
                command=lambda g=gesture: self.on_click_gesture(g)
            )
            btn.pack(fill=tk.BOTH, expand=True)

            desc_label = tk.Label(
                frame,
                text=gesture["description"],
                bg="#FFFFFF",
                fg="#555555",
                wraplength=190,
                justify=tk.LEFT,
                font=("Arial", 9)
            )
            desc_label.pack(fill=tk.X, padx=4, pady=(2, 4))

            # 鼠标经过时，在右侧显示详情
            btn.bind("<Enter>", lambda event, g=gesture: self.show_gesture_detail(g))
            desc_label.bind("<Enter>", lambda event, g=gesture: self.show_gesture_detail(g))
            frame.bind("<Enter>", lambda event, g=gesture: self.show_gesture_detail(g))

            self.gesture_buttons.append(btn)

        for c in range(columns):
            self.button_container.grid_columnconfigure(c, weight=1)

        self.update_buttons_state()

    # ==================== 动作执行 ====================

    def on_click_gesture(self, gesture):
        if self.is_busy:
            self.append_log("⚠️ 当前处于执行锁定中，请稍后再试")
            return

        self.show_gesture_detail(gesture)

        gesture_id = gesture["id"]
        gesture_name = gesture["name"]

        if self.confirm_before_execute.get():
            ok = messagebox.askyesno(
                "确认执行动作",
                f"是否执行动作？\n\n名称: {gesture_name}\nID: {gesture_id}"
            )
            if not ok:
                self.append_log(f"已取消执行: {gesture_name} ({gesture_id})")
                return

        self.ros_node.send_gesture_command(gesture_id)
        self.append_log(f"🎬 执行动作: {gesture_name} ({gesture_id})")

        self.enter_busy_state()

    def execute_selected_gesture(self):
        if not self.selected_gesture:
            messagebox.showinfo("提示", "请先选择一个动作")
            return
        self.on_click_gesture(self.selected_gesture)

    def enter_busy_state(self):
        self.is_busy = True
        self.update_buttons_state()

        seconds = max(1, int(self.busy_seconds.get()))
        self.append_log(f"⏳ 按钮锁定 {seconds} 秒，防止重复触发")

        self.root.after(seconds * 1000, self.exit_busy_state)

    def exit_busy_state(self):
        self.is_busy = False
        self.update_buttons_state()
        self.append_log("✅ 按钮锁定解除，可以继续执行动作")

    def update_buttons_state(self):
        state = tk.DISABLED if self.is_busy else tk.NORMAL
        for btn in self.gesture_buttons:
            try:
                btn.config(state=state)
                if self.is_busy:
                    btn.config(bg="#DDDDDD")
                else:
                    btn.config(bg="#EAF4FF")
            except Exception:
                pass

    def on_force_walk(self):
        if self.confirm_before_execute.get():
            ok = messagebox.askyesno(
                "确认",
                "是否发送回到行走模式指令？\n\n注意：需要 websocket_client.py 支持 force_walk_mode 才会生效。"
            )
            if not ok:
                return

        self.ros_node.send_force_walk_command()
        self.append_log("📤 已请求回到行走模式")

    # ==================== 详情与日志 ====================

    def show_gesture_detail(self, gesture):
        self.selected_gesture = gesture

        self.detail_name_label.config(text=gesture["name"])
        self.detail_id_label.config(text=f"ID: {gesture['id']}")
        self.detail_type_label.config(text=f"Type: {gesture['type']}")
        self.detail_desc_label.config(
            text=f"Description: {gesture['description'] or '-'}"
        )

    def append_log(self, text):
        self.log_text.insert(tk.END, str(text) + "\n")
        self.log_text.see(tk.END)

    def clear_log(self):
        self.log_text.delete("1.0", tk.END)

    def _poll_log_queue(self):
        try:
            while True:
                msg = self.log_queue.get_nowait()
                self.append_log(msg)
        except queue.Empty:
            pass

        self.root.after(200, self._poll_log_queue)

    def _update_status_bar(self):
        import time

        state = self.ros_node.current_robot_state
        has_status = self.ros_node.has_robot_status

        if has_status:
            dt = time.time() - self.ros_node.last_status_time
            if dt < 3.0:
                light_color = "#28A745"  # green
                comm = "在线"
            elif dt < 10.0:
                light_color = "#FFC107"  # yellow
                comm = "状态延迟"
            else:
                light_color = "#DC3545"  # red
                comm = "状态超时"
        else:
            light_color = "gray"
            comm = "未收到状态"

        self.status_light.itemconfig(self.status_light_id, fill=light_color)
        self.status_label.config(
            text=f"机器人状态: {state} | 通信: {comm} | 执行锁: {'ON' if self.is_busy else 'OFF'}"
        )

        self.root.after(500, self._update_status_bar)

def ros_spin_thread(node):
    try:
        rclpy.spin(node)
    except Exception as e:
        node.log(f"❌ ROS spin 异常: {e}")

def main():
    import argparse

    # 自动推断默认 yaml 路径
    try:
        pkg_share = get_package_share_directory('humanoid_locomotion')
        default_yaml = os.path.join(pkg_share, 'config', 'gestures.yaml')
    except Exception:
        # 如果还没 install，就退回源码路径
        default_yaml = 'src/humanoid_locomotion/config/gestures.yaml'

    parser = argparse.ArgumentParser(description="机器人动作库控制 GUI Plus")
    parser.add_argument(
        "--yaml",
        default=default_yaml,
        help=f"gestures.yaml 路径，默认: {default_yaml}"
    )
    args = parser.parse_args()

    rclpy.init()

    log_queue = queue.Queue()
    ros_node = GestureControlNode(log_queue)

    thread = threading.Thread(
        target=ros_spin_thread,
        args=(ros_node,),
        daemon=True
    )
    thread.start()

    root = tk.Tk()
    app = GestureControlGUIPlus(root, ros_node, args.yaml)

    def on_close():
        try:
            ros_node.log("正在关闭 GUI...")
        except Exception:
            pass

        try:
            ros_node.destroy_node()
        except Exception:
            pass

        try:
            rclpy.shutdown()
        except Exception:
            pass

        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()

if __name__ == "__main__":
    main()