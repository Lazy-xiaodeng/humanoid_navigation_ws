# 路线任务调试台使用说明

这个调试台用于在 APP 侧改造完成前，临时模拟 APP 控制链路，并集中观察 ROS 推送状态、地图、点位、机器人位姿、速度、电量、导航事件和本地调试日志。

## 启动方式

在 Todesk 工作区启动，不要在 `/home/ubuntu/humanoid_ws` 里启动：

```bash
cd /home/ubuntu/software/Todesk/Files/humanoid_ws
source install/setup.bash
python3 debug_monitor/route_task_dashboard_server.py
```

浏览器打开：

```text
http://127.0.0.1:18080
```

如果浏览器不在机器人本机，把 `127.0.0.1` 换成机器人 IP。

## 使用顺序

1. 先用一键启动脚本拉起导航、WebSocket server、data integration 等 ROS 节点。
2. 启动本调试台服务。
3. 页面里确认 WebSocket 地址，默认是 `ws://127.0.0.1:8765`。
4. 点击“连接”，页面会自动订阅 `robot_pose`、`robot_speed`、`robot_status`、`system_status`、`navigation_status`、`waypoints_data`、`system_exception`。
5. 点击“开始录制”开始 bag，点击“停止录制”安全结束 bag。
6. 用“开始路线 / 暂停 / 继续 / 终止 / 跳到选中点 / 播报完成”模拟 APP 控制。

## Bag 录制

bag 默认输出目录：

```text
/home/ubuntu/software/Todesk/Files/humanoid_ws/debug_bags/route_task_dashboard/
```

默认录制 topic 包含：

```text
/tf
/tf_static
/map
/odom
/robot_realpose
/cmd_vel
/navigation/status
/navigation/requests
/navigation/acknowledgments
/navigation/waypoints_data
/app/navigation_command
/app/waypoint_command
/localization/prior_map_odom_bridge_status
/prior_localization/confidence
/robot_status_processed
/system/exceptions
```

页面里可以按现场问题临时增删 topic。开始录制时会调用 `ros2 bag record -o ...`，停止录制时会先发送 `SIGINT`，尽量保证 bag 索引完整。

## 日志

调试台会把页面侧收到/发出的 WebSocket 消息、bag 开停事件写入 JSONL：

```text
/home/ubuntu/software/Todesk/Files/humanoid_ws/debug_logs/route_task_dashboard/
```

如果现场发现异常，建议同时保留：

- 对应时间段的 bag 目录；
- 对应时间段的 `dashboard_*.jsonl`；
- 终端启动日志；
- 问题发生的大概时间、当前点位、目标点位、是否刚点击过暂停/继续/跳步/播报完成。

## 控制命令口径

页面发出的导航控制均走：

```json
{
  "protocol_version": "2.0",
  "message_type": "command",
  "data_type": "navigation_control",
  "source": "route_task_dashboard",
  "destination": "ros",
  "data": {
    "command_type": "start_route_task"
  }
}
```

具体命令包括：

- `start_route_task`：开始路线任务，默认使用 `route_waypoint_ids` + `waypoints_revision`，由 ROS 从当前点位库补全点位详情。
- `pause_route_task`：手动暂停当前路线任务。
- `resume_route_task`：继续当前路线任务，也用于人工清障/人工脱困后继续。
- `stop_route_task`：终止当前路线任务并清理上下文。
- `jump_to_waypoint`：跳到选中的任务点，辅助点由 ROS 根据区间自动吸收。
- `broadcast_finished`：模拟 APP 播报完成回执，ROS 校验当前等待播报上下文后继续后续路线。

## 注意事项

- 调试台只是测试工具，不参与 ROS 控制闭环，不会替代正式 APP。
- 跳步下拉框只展示任务点，避免误跳到辅助点。
- 如果页面显示 WebSocket 已连接但按钮无响应，优先看日志里是否返回 `navigation_command_result` 或错误消息。
- 如果 bag 开始失败，确认启动调试台的终端已经 `source install/setup.bash`，并且能直接执行 `ros2 bag record`。
