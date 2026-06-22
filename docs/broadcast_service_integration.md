# 小瑞播报服务接入说明

本方案只写入 ToDesk 工作区 `/home/ubuntu/software/Todesk/Files/humanoid_ws`，未修改 `/home/ubuntu/humanoid_ws`。

## 架构

1. Gateway 收到 ROS `broadcast_requested`。
2. Gateway 调用外部播报执行器脚本。
3. 脚本调用 ROS 服务 `/xiaorui_broadcast/play`。
4. ROS 播报服务自动识别扬声器并执行播放。
5. 播放成功后脚本退出 `0`，Gateway 发送 `broadcast_finished`。

## ROS 服务

- `/xiaorui_broadcast/play`：播放播报文本。
- `/xiaorui_broadcast/set_volume`：设置 ROS 侧播报音量。
- `/xiaorui_broadcast/health`：查看自动识别到的扬声器和当前音量。

## 自动扬声器识别

优先级：

1. `XIAORUI_AUDIO_SINK` 或 `XIAORUI_ALSA_DEVICE` 显式指定。
2. PipeWire/PulseAudio 默认输出。
3. USB Audio / analog stereo。
4. 其它 ALSA 输出。
5. HDMI 最后考虑。

## 播放模式

默认：

```bash
XIAORUI_BROADCAST_PLAYER=dry_run
```

该模式只做流程联调，不真实出声。

测试蜂鸣：

```bash
XIAORUI_BROADCAST_PLAYER=beep
```

外部命令：

```bash
XIAORUI_BROADCAST_PLAYER=command
XIAORUI_BROADCAST_PLAYER_COMMAND='your_tts_or_player "{{text}}" "{{volumePercent}}"'
```

推荐使用包装脚本读取环境变量，避免中文播报文本中的引号或标点破坏 shell 模板：

```bash
XIAORUI_BROADCAST_PLAYER=command
XIAORUI_BROADCAST_PLAYER_COMMAND=/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_broadcast/scripts/xiaorui-edge-tts-player.sh
XIAORUI_EDGE_TTS_VOICE=zh-CN-XiaoxiaoNeural
```

外部命令可读取环境变量：

- `XIAORUI_BROADCAST_TEXT`
- `XIAORUI_BROADCAST_VOLUME`
- `XIAORUI_BROADCAST_ID`
- `XIAORUI_BROADCAST_WAYPOINT_ID`
- `XIAORUI_AUDIO_SELECTED_DEVICE`
- `XIAORUI_AUDIO_BACKEND_SELECTED`

## 启动

```bash
cd /home/ubuntu/software/Todesk/Files/humanoid_ws
colcon build --packages-select humanoid_interfaces humanoid_broadcast humanoid_websocket --symlink-install
source install/setup.bash
./start_broadcast_service.sh
```

## Gateway 播报执行器配置

Gateway 可配置：

```bash
BROADCAST_PLAYBACK_COMMAND=/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_broadcast/scripts/xiaorui-call-ros-broadcast.sh
BROADCAST_PLAYBACK_ARGS_JSON=[]
BROADCAST_PLAYBACK_TIMEOUT_MS=120000
```

## 健康检查

```bash
ros2 service call /xiaorui_broadcast/health humanoid_interfaces/srv/GetBroadcastHealth "{}"
```

## 音量控制

App 设置页选择“后端控制”时，Gateway 每次播放会把 `broadcastVolume` 传给 `/xiaorui_broadcast/play`。

App 设置页选择“ROS 控制”时，Gateway 会向 ROS WebSocket 发送 `set_broadcast_volume`。`humanoid_websocket` 已拦截该命令并调用 `/xiaorui_broadcast/set_volume`，然后发布 `navigation_command_result` 回执。
