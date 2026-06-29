# humanoid_broadcast_runtime

`humanoid_broadcast_runtime` 是机器人播报运行层 C++ 功能包，用于提供本机文字播报、播报音量设置和播报服务健康检查。它保持原有 ROS service 名称、请求字段、响应字段和主要环境变量语义不变，方便与 APP 网关、路线任务链路和调试脚本继续对接。

## 包作用

- 提供 `/xiaorui_broadcast/play` 服务，接收播报文本并执行播放。
- 提供 `/xiaorui_broadcast/set_volume` 服务，供 APP 设置本机播报音量。
- 提供 `/xiaorui_broadcast/health` 服务，返回当前音频设备、后端和默认音量。
- 自动选择 PipeWire/Pulse sink 或 ALSA 设备。
- 支持 `dry_run`、`command`、`beep` 三种播放模式。

## 上下游链路

上游：

- `humanoid_app_gateway_runtime/app_gateway_node`：处理 APP 的 `set_broadcast_volume` 命令，并调用 `/xiaorui_broadcast/set_volume`。
- 调试脚本 `xiaorui-call-ros-broadcast.sh`：调用 `/xiaorui_broadcast/play` 进行手动播报测试。
- 其他业务节点也可以直接调用 `/xiaorui_broadcast/play`。

输入服务：

- `/xiaorui_broadcast/play`，类型 `humanoid_interfaces/srv/PlayBroadcast`。
- `/xiaorui_broadcast/set_volume`，类型 `humanoid_interfaces/srv/SetBroadcastVolume`。
- `/xiaorui_broadcast/health`，类型 `humanoid_interfaces/srv/GetBroadcastHealth`。

下游：

- PipeWire/Pulse：通过 `pactl` 查询 sink 和设置音量。
- ALSA：通过 `aplay -l` 查询设备，通过 `amixer` 设置音量，通过 `aplay` 播放测试音。
- 外部播放器：`command` 模式可调用 `edge-tts`、`ffplay` 或自定义 shell 命令。

## 文件说明

- `src/broadcast_service_node.cpp`：C++ 播报服务主节点，负责参数加载、音频设备选择、音量设置、播放执行和 ROS service 回调。
- `src/call_broadcast_service.cpp`：C++ 播报服务命令行客户端，用于通过环境变量手动调用 `/xiaorui_broadcast/play`。
- `config/broadcast_runtime.yaml`：播报运行层参数文件，每个参数都有中文说明。
- `launch/broadcast_runtime.launch.py`：单独启动 C++ 播报服务的 launch 文件。
- `CMakeLists.txt`：C++ 编译和安装规则。
- `package.xml`：ROS2 包依赖声明。

## 使用方式

单独启动 C++ 播报服务：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 launch humanoid_broadcast_runtime broadcast_runtime.launch.py
```

使用 dry-run 测试播放服务：

```bash
ros2 service call /xiaorui_broadcast/play humanoid_interfaces/srv/PlayBroadcast \
  "{text: '测试播报', broadcast_id: 'test_001', waypoint_id: 'wp_001', volume_percent: 72, use_request_volume: true}"
```

使用 C++ 命令行客户端测试：

```bash
XIAORUI_BROADCAST_TEXT="测试播报" \
XIAORUI_BROADCAST_ID="manual_test" \
XIAORUI_BROADCAST_WAYPOINT_ID="wp_test" \
XIAORUI_BROADCAST_VOLUME=72 \
ros2 run humanoid_broadcast_runtime call_broadcast_service_cpp
```

设置音量：

```bash
ros2 service call /xiaorui_broadcast/set_volume humanoid_interfaces/srv/SetBroadcastVolume \
  "{volume_percent: 60}"
```

查询健康状态：

```bash
ros2 service call /xiaorui_broadcast/health humanoid_interfaces/srv/GetBroadcastHealth "{}"
```

## 参数说明

主要参数位于 `config/broadcast_runtime.yaml`：

- `audio_backend`：音频后端选择策略，默认 `auto`。
- `audio_sink`：PipeWire/Pulse 指定 sink，默认 `auto`。
- `alsa_device`：ALSA 指定设备，默认 `auto`。
- `player_mode`：播放模式，默认 `dry_run`。
- `player_command`：`command` 模式下的外部命令模板。
- `dry_run_sec`：dry-run 模拟播放耗时。
- `default_volume_percent`：默认播报音量。
- `reselect_each_request`：是否每次请求都重新选择音频设备。
- `log_command_error`：外部命令失败时是否打印错误摘要。

## 兼容环境变量

为兼容旧播报服务和启动脚本，C++ 节点会读取以下环境变量作为参数默认值：

- `XIAORUI_AUDIO_BACKEND`
- `XIAORUI_AUDIO_SINK`
- `XIAORUI_ALSA_DEVICE`
- `XIAORUI_BROADCAST_PLAYER`
- `XIAORUI_BROADCAST_PLAYER_COMMAND`
- `XIAORUI_BROADCAST_DRY_RUN_SEC`
- `XIAORUI_BROADCAST_DEFAULT_VOLUME`
- `XIAORUI_AUDIO_RESELECT_EACH_REQUEST`

## 灰度建议

1. 先用 `dry_run` 模式启动 C++ 服务，验证三个 service 的字段和返回值。
2. 再用 `command` 模式接入原 `xiaorui-edge-tts-player.sh`，验证真实播报。
3. 确认 APP 音量设置、服务健康状态和播放失败返回语义正常。
4. 实机验证通过前，不删除旧 Python `humanoid_broadcast` 包。
