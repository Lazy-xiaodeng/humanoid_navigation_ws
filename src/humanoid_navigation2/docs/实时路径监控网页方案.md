# 实时路径监控网页使用说明

本文档说明如何在电脑或手机浏览器中打开导航监控页面，实时查看：

- 当前 `/plan` 规划路径
- `/robot_realpose` 实际行走轨迹
- 机器人当前位置到规划路径的偏差
- 剩余路径、距离目标点、路径进度

## 1. 需要启动的节点

先进入工作空间并加载环境：

```bash
cd /home/ubuntu/humanoid_ws
source install/setup.bash
```

启动 WebSocket server：

```bash
ros2 launch humanoid_websocket websocket_server.launch.py
```

启动实时路径监控节点：

```bash
ros2 launch humanoid_navigation2 navigation_path_monitor.launch.py
```

该节点会订阅：

```text
/plan
/robot_realpose
```

并通过现有 WebSocket 数据链路推送：

```text
/integration/push_messages
data_type = navigation_path_monitor
```

## 2. 在机器人电脑本机打开

直接用浏览器打开：

```text
/home/ubuntu/humanoid_ws/debug_monitor/navigation_app_simulator.html
```

页面中的 WebSocket 地址填写：

```text
ws://127.0.0.1:8765
```

点击：

```text
连接 -> 发送订阅
```

订阅框中应包含：

```text
navigation_path_monitor
```

当前网页默认已经包含该订阅项。

## 3. 在手机或外部设备打开

手机需要和机器人电脑在同一个网络下。

在机器人电脑上启动一个静态网页服务：

```bash
cd /home/ubuntu/humanoid_ws/debug_monitor
python3 -m http.server 8080
```

查看机器人电脑 IP：

```bash
ip addr
```

假设机器人电脑 IP 是：

```text
10.192.1.150
```

手机浏览器打开：

```text
http://10.192.1.150:8080/navigation_app_simulator.html
```

网页中的 WebSocket 地址填写：

```text
ws://10.192.1.150:8765
```

然后点击：

```text
连接 -> 发送订阅
```

## 4. 页面显示含义

实时路径监控区域中：

```text
绿色：当前 /plan 规划路径
蓝色：机器人实际轨迹 /robot_realpose
红色虚线：当前位置到规划路径的最近偏差
灰色虚线：起点到目标点直线
```

右侧指标：

```text
路径偏差：机器人当前位置到 /plan 最近点的距离
剩余路径：沿 /plan 到终点的剩余距离
路径进度：沿 /plan 的完成比例
轨迹长度：本次规划后机器人实际走过的轨迹长度
```

偏差等级：

```text
ok   ：偏差 < 0.35 m
warn ：偏差 >= 0.35 m
bad  ：偏差 >= 0.70 m
```

阈值在 `navigation_path_monitor.py` 中可通过参数调整：

```text
deviation_warn_m
deviation_bad_m
```

## 5. 常见问题排查

如果网页无法连接 WebSocket：

```bash
ros2 node list | grep websocket
```

确认 WebSocket server 已启动。

如果页面能连接但没有路径图：

```bash
ros2 topic hz /plan
ros2 topic hz /robot_realpose
ros2 topic hz /integration/push_messages
```

确认三个话题有数据。

如果手机打不开网页：

- 确认手机和机器人电脑在同一个网络
- 确认 URL 使用的是机器人电脑 IP，不是 `127.0.0.1`
- 确认机器人电脑上 `python3 -m http.server 8080` 正在运行
- 确认网络或防火墙没有拦截 `8080` 和 `8765`

如果手机能打开网页但 WebSocket 连接失败：

- WebSocket 地址必须是 `ws://机器人电脑IP:8765`
- 不要在手机上使用 `ws://127.0.0.1:8765`
- 确认 WebSocket server 监听地址是 `0.0.0.0`

## 6. 推荐启动顺序

```bash
cd /home/ubuntu/humanoid_ws
source install/setup.bash
ros2 launch humanoid_websocket websocket_server.launch.py
```

新开一个终端：

```bash
cd /home/ubuntu/humanoid_ws
source install/setup.bash
ros2 launch humanoid_navigation2 navigation_path_monitor.launch.py
```

新开一个终端：

```bash
cd /home/ubuntu/humanoid_ws/debug_monitor
python3 -m http.server 8080
```

手机访问：

```text
http://机器人电脑IP:8080/navigation_app_simulator.html
```
