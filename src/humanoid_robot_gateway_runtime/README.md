# humanoid_robot_gateway_runtime

## 这个包是做什么的

`humanoid_robot_gateway_runtime` 是面向机器人本体的 C++ 网关运行层包。

它负责承接当前机器人本体 WebSocket 网关职责：连接机器人本体 WebSocket，
把 ROS 中的速度、机器人控制、动作执行请求转换成机器人底层协议，并把机器人状态、动作结果、摇杆数据等发布回 ROS。

它不负责 APP WebSocket 服务，也不负责 APP 数据整合推送。这些由 `humanoid_app_gateway_runtime` 负责。

## 当前状态

当前采用“安全验证、按开关接入”的方式推进：

- `robot_gateway_node`：已接入机器人本体 WebSocket 客户端生命周期、状态/摇杆发布、`/cmd_vel` 输入、`/app/robot_control` 输入、动作结果发布和低风险消息分发；默认 `robot_ws_enable=false`、`walk_velocity_send_enable=false`、`motion_execution_enable=false`，不会连接或控制真实机器人。
- `robot_protocol`：已实现机器人本体 `accid/title/timestamp/guid/data` 协议组包、基础消息解析和身份字段清洗逻辑，并通过 协议一致性验证。
- `response_waiter`：已实现 guid 响应缓存、命中、读取、删除和超时清理逻辑，并通过 协议一致性验证。
- `robot_status_parser`：已实现 `notify_robot_info` 到 `/robot_status_raw` 的字段解析、单位转换、通信质量估算，以及 `notify_joy_data` 到 Joy 数组的解析逻辑，并通过 协议一致性验证。
- `action_result_builder`：已实现 `/robot/action_result` 动作完成事件组包、动作耗时计算、命令时间戳透传和动作通知数据透传逻辑，并通过 协议一致性验证。
- `walk_velocity_controller`：已实现 `/cmd_vel` 缓存、速度限幅、Walk 状态门控、动作执行门控和零速度停车退出的纯逻辑，并通过 协议一致性验证；真实 WebSocket 发送仍默认关闭。
- `gesture_sync`：已接入连接后动作库同步链路，可从机器人拉取动作列表、更新 `gestures.yaml`、重新加载动作时长并发布 `/system/gesture_list_updated`；默认 `gesture_sync_enable=false`，不会自动写文件。
- `motion_controller`：已实现 APP 动作命令解析、忙碌拒绝、动作预期时长到超时时间的计算逻辑，并通过 协议一致性验证；真实 Menu/Walk 切换和动作下发仍默认关闭。
- `robot_ws_client`：已实现机器人 WebSocket URL 解析、真实连接/重连线程、命令发送、guid 响应等待、原始消息 title 分发、身份字段刷新和行走速度错误分类逻辑，并通过离线 协议一致性验证；默认不连接真实机器人。
- 真实动作执行、真实速度发送和 Walk/Menu 自动切换仍由独立 YAML 开关控制；默认关闭时已经通过 smoke 验证会安全拒绝动作执行，不会误控机器人。
- 默认保持安全接入模式，不连接机器人本体 WebSocket、不发送 `cmd_vel`、不切换 Walk/Menu、不执行动作。

## 规划中的主要输入输出接口

保持当前线上 ROS 接口稳定：

- 订阅 `/cmd_vel`：接收 Nav2/controller 输出速度，转换为机器人行走速度命令。
- 订阅 `/app/robot_control`：接收 APP/网关发来的机器人控制命令。
- 发布 `/robot_status_raw`：发布机器人原始状态，供数据整合和导航门控使用。
- 发布 `/joy_raw`：发布机器人本体摇杆/遥控输入。
- 发布 `/robot/action_result`：发布动作执行结果。
- 发布 `/system/gesture_list_updated`：通知数据整合节点动作库已更新。

## 使用方式

当前用于独立编译、模拟验证和单独启动验证：

```bash
ros2 launch humanoid_robot_gateway_runtime robot_gateway_runtime.launch.py
```

安全单独启动，不连接真实机器人：

```bash
ros2 launch humanoid_robot_gateway_runtime robot_gateway_runtime.launch.py \
  robot_ws_enable:=false \
  walk_velocity_send_enable:=false \
  motion_execution_enable:=false
```

通过统一 ws 入口分阶段接管机器人本体连接，但仍禁止真实速度和动作：

```bash
ros2 launch humanoid_websocket websocket_server.launch.py \
  use_cpp_app_gateway:=false \
  use_cpp_robot_gateway:=true \
  cpp_robot_ws_enable:=true \
  cpp_robot_walk_velocity_send_enable:=false \
  cpp_robot_motion_execution_enable:=false \
  cpp_robot_gesture_sync_enable:=false
```

通过统一 ws 入口只验证 C++ 节点启动，不连接真实机器人：

```bash
ros2 launch humanoid_websocket websocket_server.launch.py \
  use_cpp_app_gateway:=false \
  use_cpp_robot_gateway:=true \
  cpp_robot_ws_enable:=false \
  cpp_robot_walk_velocity_send_enable:=false \
  cpp_robot_motion_execution_enable:=false
```

通过控制层总入口分阶段接管机器人本体连接：

```bash
ros2 launch humanoid_bringup robot_control_plane.launch.py \
  use_cpp_robot_gateway:=true \
  cpp_robot_ws_enable:=true \
  cpp_robot_walk_velocity_send_enable:=false \
  cpp_robot_motion_execution_enable:=false \
  cpp_robot_gesture_sync_enable:=false
```

涉及真实运动的功能必须逐项打开：先只打开 `robot_ws_enable` 看状态和 `/joy_raw`，再评估 `gesture_sync_enable` 是否能安全同步动作库，再评估 `walk_velocity_send_enable`，最后才评估 `motion_execution_enable` 与 Walk/Menu 自动切换。

动作库同步单独打开示例：

```bash
ros2 launch humanoid_bringup robot_control_plane.launch.py \
  use_cpp_robot_gateway:=true \
  cpp_robot_ws_enable:=true \
  cpp_robot_gesture_sync_enable:=true \
  cpp_robot_walk_velocity_send_enable:=false \
  cpp_robot_motion_execution_enable:=false
```

`gesture_sync_enable=true` 会在机器人 WebSocket 连接稳定后请求 `request_get_atomic_motion_list`，默认写入 `humanoid_locomotion/config/gestures.yaml`，然后通知数据整合节点热重载。若需要写到临时文件，可在 `robot_gateway.yaml` 中设置 `gestures_yaml_path`。

切换前后建议运行状态检查：

```bash
source install/setup.bash
tools/check_ws_runtime_status.sh
```

真实接入前建议先运行空载 smoke：

```bash
tools/run_ws_cpp_gray_smoke.sh
```

## 参数配置

参数文件在：

```text
config/robot_gateway.yaml
```

每个参数旁边都有中文注释。现场调试优先修改 YAML，不要把临时经验值写死在源码里。

## 文件功能清单

### 顶层文件

- `CMakeLists.txt`：定义 C++ 编译目标、依赖和安装规则；上游是 colcon/ament 构建系统，下游是安装后的 `robot_gateway_node`。
- `package.xml`：声明 ROS 包元信息和依赖；上游是 colcon/rosdep，下游是 ROS 包发现和 launch 查找。
- `README.md`：说明本包职责、接口、文件结构和维护原则；上游是开发维护者，下游是后续实现和调试人员。

### 配置与启动

- `config/robot_gateway.yaml`：配置机器人本体 WebSocket、速度发送、动作执行、状态发布和动作库同步；上游是 launch 参数加载，下游是 `robot_gateway_node`。
- `launch/robot_gateway_runtime.launch.py`：单独启动机器人本体网关 C++ 运行节点；上游是开发者/bringup 运行开关，下游是 `robot_gateway_node`。

### 机器人本体网关节点相关

- `include/.../robot_gateway_types.hpp`：集中定义机器人本体网关配置结构；上游是 YAML 参数，下游是节点外壳和所有内部模块。
- `include/.../robot_gateway_config.hpp` / `src/robot_gateway_config.cpp`：声明和读取 ROS 参数；上游是 `robot_gateway.yaml`，下游是 `robot_gateway_node`。
- `src/robot_gateway_node.cpp`：机器人本体网关 ROS Node 外壳；上游是 `/cmd_vel`、`/app/robot_control` 和机器人本体 WebSocket，下游是 `/robot_status_raw`、`/joy_raw`、`/robot/action_result`、`/system/gesture_list_updated`。
- `include/.../robot_ws_client.hpp` / `src/robot_ws_client.cpp`：机器人本体 WebSocket 连接、重连、发送、guid 响应等待和接收消息分发边界；上游是机器人本体 WS 服务，下游是状态解析、响应等待器、动作和速度控制模块。
- `src/robot_ws_client_probe.cpp`：机器人 WebSocket 客户端开发期验证工具；上游是 协议一致性验证脚本定义的 URL 和机器人原始消息，下游是 stdout 输出的连接参数、身份状态和消息分发结果。
- `include/.../robot_protocol.hpp` / `src/robot_protocol.cpp`：机器人本体协议组包和解析边界；上游是 motion/walk/gesture 模块的命令请求，下游是 `robot_ws_client`。
- `src/robot_protocol_probe.cpp`：机器人底层协议开发期验证工具；上游是 协议一致性验证脚本定义的 command 和机器人返回消息，下游是 stdout 输出的协议处理结果。
- `include/.../response_waiter.hpp` / `src/response_waiter.cpp`：请求 guid 与响应等待关系管理；上游是 `robot_ws_client` 收到的响应，下游是同步命令、动作执行和动作库同步。
- `src/response_waiter_probe.cpp`：响应等待缓存开发期验证工具；上游是 协议一致性验证脚本定义的 guid 注册/完成/清理场景，下游是 stdout 输出的响应等待状态。
- `include/.../robot_status_parser.hpp` / `src/robot_status_parser.cpp`：解析机器人状态、身份、joy 数据；上游是机器人本体 notify 消息，下游是 `/robot_status_raw`、`/joy_raw`。
- `src/robot_status_parser_probe.cpp`：机器人状态解析开发期验证工具；上游是 协议一致性验证脚本定义的 notify_robot_info 和 notify_joy_data，下游是 stdout 输出的状态/joy 解析结果。
- `include/.../walk_velocity_controller.hpp` / `src/walk_velocity_controller.cpp`：管理 `/cmd_vel` 缓存、定频发送决策、Walk 状态门控、动作执行门控和零速度停车退出；上游是 Nav2/controller，下游是机器人本体行走速度命令。
- `src/walk_velocity_controller_probe.cpp`：行走速度控制开发期验证工具；上游是 协议一致性验证脚本定义的速度输入和机器人状态，下游是 stdout 输出的速度发送/拦截决策。
- `include/.../motion_controller.hpp` / `src/motion_controller.cpp`：管理 APP 动作命令解析、动作入口门控、动作超时计算；上游是 `/app/robot_control`，下游是机器人本体动作命令和 `/robot/action_result`。
- `src/motion_controller_probe.cpp`：动作控制入口开发期验证工具；上游是 协议一致性验证脚本定义的 APP 控制命令和动作时长，下游是 stdout 输出的解析结果、拒绝结果和超时计算。
- `include/.../gesture_sync.hpp` / `src/gesture_sync.cpp`：解析机器人动作库响应、生成 gestures.yaml 内容、生成热重载通知；上游是机器人本体动作库接口，下游是 `/system/gesture_list_updated` 和数据整合节点。
- `src/gesture_sync_probe.cpp`：动作库同步开发期验证工具；上游是 协议一致性验证脚本定义的动作库响应和旧 YAML，下游是 stdout 输出的新 YAML 与 reload 消息。
- `include/.../action_result_builder.hpp` / `src/action_result_builder.cpp`：统一构建动作执行结果 JSON；上游是 motion_controller，下游是 `/robot/action_result`。
- `src/action_result_builder_probe.cpp`：动作结果组包开发期验证工具；上游是 协议一致性验证脚本定义的动作完成场景，下游是 stdout 输出的动作结果 JSON。

## 维护原则

- 机器人本体 WebSocket、速度发送、动作执行和 Walk/Menu 切换都属于高风险逻辑，必须按“先状态、再速度、再动作”的顺序逐项打开。
- 状态解析、摇杆发布、动作结果组包、`/cmd_vel` 输入和 APP 动作命令入口都必须先在默认安全关闭模式下验证。
- 打开 `walk_velocity_send_enable` 前，必须确认 `robot_ws_enable=true` 后机器人状态稳定进入 `WALK`，并保留超时停和零速度保护。
- 打开 `gesture_sync_enable` 前，建议先把 `gestures_yaml_path` 指向临时文件验证写入内容；确认无误后再写正式 `gestures.yaml`。
- 打开 `motion_execution_enable`、`motion_allow_enter_menu`、`motion_allow_return_walk` 前，必须先单独验证 Menu/Walk 切换和动作完成通知，避免导航中动作未结束就恢复行走。
- 任何本包输出的 `/robot_status_raw`、`/robot/action_result` 字段都必须和 线上协议对齐。
