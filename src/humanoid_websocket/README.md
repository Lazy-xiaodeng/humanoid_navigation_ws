# humanoid_websocket

## 这个包是做什么的

`humanoid_websocket` 是系统当前 WebSocket 入口包，负责把 APP / 导航页、数据整合、机器人本体 WebSocket 三条链路组织起来。

当前包仍保留原有 Python 节点作为默认运行链路，同时在 launch 层提供 C++ runtime 灰度切换入口：

- APP 网关与数据整合 C++ 实现在 `humanoid_app_gateway_runtime`。
- 机器人本体网关 C++ 实现在 `humanoid_robot_gateway_runtime`。
- 默认所有 C++ 灰度开关都是关闭或安全关闭，不会改变原运行链路。

## 默认启动链路

默认启动时使用原有 Python 节点：

- `websocket_server_node`：连接 APP / 导航页。
- `data_integration_node_recoverable`：整合机器人、导航、地图、点位和动作数据。
- `websocket_client_node`：连接机器人本体 WebSocket。

```bash
ros2 launch humanoid_websocket websocket_server.launch.py
```

## C++ 灰度开关

APP 侧 C++ 灰度：

```bash
ros2 launch humanoid_websocket websocket_server.launch.py \
  use_cpp_app_gateway:=true \
  cpp_app_websocket_server_enable:=true \
  cpp_data_integration_enable:=true
```

只验证 C++ 数据整合，不抢占 APP WebSocket 端口：

```bash
ros2 launch humanoid_websocket websocket_server.launch.py \
  use_cpp_app_gateway:=true \
  cpp_app_websocket_server_enable:=false \
  cpp_data_integration_enable:=true
```

机器人侧 C++ 灰度，但不连接真实机器人：

```bash
ros2 launch humanoid_websocket websocket_server.launch.py \
  use_cpp_robot_gateway:=true \
  cpp_robot_ws_enable:=false \
  cpp_robot_walk_velocity_send_enable:=false \
  cpp_robot_motion_execution_enable:=false
```

机器人侧 C++ 灰度，连接真实机器人但禁止真实速度和动作：

```bash
ros2 launch humanoid_websocket websocket_server.launch.py \
  use_cpp_robot_gateway:=true \
  cpp_robot_ws_enable:=true \
  cpp_robot_walk_velocity_send_enable:=false \
  cpp_robot_motion_execution_enable:=false
```

## 控制层总入口

如果通过控制层总入口启动，可以使用环境变量控制灰度开关：

```bash
USE_CPP_APP_GATEWAY=true \
CPP_APP_WEBSOCKET_SERVER_ENABLE=false \
CPP_DATA_INTEGRATION_ENABLE=true \
USE_CPP_ROBOT_GATEWAY=true \
CPP_ROBOT_WS_ENABLE=false \
CPP_ROBOT_WALK_VELOCITY_SEND_ENABLE=false \
CPP_ROBOT_MOTION_EXECUTION_ENABLE=false \
./start_ros_control_plane.sh
```

## 状态检查

切换前后建议运行：

```bash
source install/setup.bash
tools/check_ws_runtime_status.sh
```

这个脚本会检查：

- 当前 Python / C++ ws 节点数量。
- 是否出现 Python 和 C++ 节点并存导致重复发布。
- APP WebSocket 端口 `8765` 是否被占用。
- 关键 topic 的 publisher/subscriber 数量。
- ws 相关进程的 CPU 和 RSS。

## 空载 Smoke

真实灰度前建议先运行 C++ 空载 smoke：

```bash
tools/run_ws_cpp_gray_smoke.sh
```

它会使用独立 `ROS_DOMAIN_ID=229` 拉起 C++ APP 网关、C++ 数据整合、C++ 机器人本体网关，并强制：

- `cpp_app_websocket_server_enable=false`，不抢占 APP 端口。
- `cpp_data_integration_enable=true`，验证数据整合订阅发布链路能启动。
- `cpp_robot_ws_enable=false`，不连接真实机器人。
- `cpp_robot_walk_velocity_send_enable=false`，不发送真实速度。
- `cpp_robot_gesture_sync_enable=false`，不自动写入动作库 YAML；需要验证动作库同步时再单独打开。
- `cpp_robot_motion_execution_enable=false`，不执行真实动作。

如果需要验证统一 ws 入口下的 C++ APP WebSocket 真实收发，但不抢占 8765：

```bash
tools/run_ws_cpp_app_gateway_smoke.py
```

它会自动选择本机空闲端口，启动 C++ APP 网关和 C++ 数据整合，连接 WebSocket 后验证：

- `connection_ack` 能返回。
- APP request 能转发到 `/websocket/data_requests`。
- `/integration/data_responses` 能回发到 WebSocket 客户端。

## 回退方式

如果 C++ 灰度验证异常，把开关改回 false 即可回退默认 Python 链路：

```bash
USE_CPP_APP_GATEWAY=false \
USE_CPP_ROBOT_GATEWAY=false \
./start_ros_control_plane.sh
```

涉及真实机器人运动时，必须保持以下默认值，除非已经单独验证安全：

- `CPP_ROBOT_WALK_VELOCITY_SEND_ENABLE=false`
- `CPP_ROBOT_MOTION_EXECUTION_ENABLE=false`
- `CPP_ROBOT_MOTION_ALLOW_ENTER_MENU=false`
- `CPP_ROBOT_MOTION_ALLOW_RETURN_WALK=false`
