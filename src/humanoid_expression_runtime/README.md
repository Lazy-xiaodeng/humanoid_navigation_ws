# humanoid_expression_runtime

`humanoid_expression_runtime` 是机器人表情运行层 C++ 功能包，用于替换旧的 Python `facial_driver` 常驻节点。它保持原有输入话题和表情 YAML 格式不变，方便在 Todesk 工作区内灰度验证，并保留 Python 版本随时回退。

## 包作用

- 订阅 `/robot/facial_raw_cmd`，接收 APP 网关或调试工具发布的表情动作名。
- 读取本包 `config/facial_gestures.yaml` 中的 `facial_gestures` 动作表。
- 将表情动作序列转换为串口指令，发送到仿生头控制板。
- 支持 `delay:x` 延时指令、动作前停止眼部/嘴部循环、眼部复位、串口断线重连和退出睡眠动作。

## 上下游链路

上游：

- `humanoid_app_gateway_runtime/app_gateway_node`：将 APP `facial_control` 命令转换为纯动作名字符串。
- `facial_control_ui`：手动调试工具，可直接发布表情动作名。

输入话题：

- `/robot/facial_raw_cmd`，消息类型 `std_msgs/msg/String`，内容示例：`talk`、`idle`、`sleeping`。

下游：

- 仿生头串口控制板，默认设备 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`。
- 如果 by-id 不存在，会按 `fallback_ports` 继续尝试 `/dev/ttyUSB0`、`/dev/ttyUSB1`，避免 USB 序号偶发漂移导致表情不可用。

依赖配置：

- `config/facial_gestures.yaml`：表情动作库，`facial_driver_cpp` 会直接读取它。
- `config/gestures.yaml`：机器人上半身动作库副本，供后续动作库资源归拢或 APP 展示链路迁移使用。
- `config/gestures_app.yaml`：APP 展示用上半身动作库副本，供后续资源归拢使用。

## 文件说明

- `src/facial_driver.cpp`：C++ 表情串口驱动主节点，负责参数加载、YAML 解析、ROS 订阅、串口连接和动作执行。
- `config/expression_runtime.yaml`：表情运行层参数文件，每个参数都有中文说明。
- `config/facial_gestures.yaml`：表情动作库。
- `config/gestures.yaml`：上半身动作库副本。
- `config/gestures_app.yaml`：APP 上半身动作展示库副本。
- `launch/expression_runtime.launch.py`：单独启动 C++ 表情运行层的 launch 文件。
- `CMakeLists.txt`：C++ 编译和安装规则。
- `package.xml`：ROS2 包依赖声明。

## 使用方式

单独启动：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 launch humanoid_expression_runtime expression_runtime.launch.py
```

发送测试表情：

```bash
ros2 topic pub --once /robot/facial_raw_cmd std_msgs/msg/String "{data: 'talk'}"
```

## 参数说明

主要参数位于 `config/expression_runtime.yaml`：

- `config_path`：表情 YAML 路径，留空时自动查找本包 `config/facial_gestures.yaml`。
- `command_topic`：表情命令输入话题，默认 `/robot/facial_raw_cmd`。
- `port`：仿生头串口设备路径。
- `fallback_ports`：串口兜底候选列表，`port` 打不开时按顺序尝试。
- `baudrate`：串口波特率，默认 `115200`。
- `startup_gesture`：节点启动后自动执行的表情，默认 `idle`。
- `shutdown_gesture`：节点退出时尝试执行的表情，默认 `sleeping`。
- `pre_stop_eye_enable` / `pre_stop_mouth_enable`：执行普通表情前是否先停止眼部和嘴部循环。
- `eye_reset_commands`：前置停止后的眼部复位指令。

## 灰度建议

1. 先单独启动本包，使用 `ros2 topic pub` 发送 `idle/talk/sleeping` 测试串口动作。
2. 确认动作序列、延时和串口日志与旧 Python 节点一致。
3. 再把 `humanoid_bringup` 中的 `locomotion.launch.py` 灰度切到 `facial_driver_cpp`。
4. 实机验证通过前不要删除 Python `facial_driver`。
