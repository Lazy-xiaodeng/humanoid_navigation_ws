# humanoid_robot_gateway_runtime

## 这个包是做什么的

`humanoid_robot_gateway_runtime` 是面向机器人本体的 C++ 网关运行层包。

它负责承接当前机器人本体 WebSocket 网关职责：连接机器人本体 WebSocket，
把 ROS 中的速度、机器人控制、动作执行请求转换成机器人底层协议，并把机器人状态、动作结果、摇杆数据等发布回 ROS。

它不负责 APP WebSocket 服务，也不负责 APP 数据整合推送。这些由 `humanoid_app_gateway_runtime` 负责。

## 当前状态

当前采用“正式导航联调默认打开、离线验证可显式关闭”的方式推进：

- `robot_gateway_node`：已接入机器人本体 WebSocket 客户端生命周期、状态/摇杆发布、`/cmd_vel` 输入、`/app/robot_control` 输入、动作结果发布和低风险消息分发；正式导航默认连接机器人并允许速度/动作链路。
- `robot_protocol`：已实现机器人本体 `accid/title/timestamp/guid/data` 协议组包、基础消息解析和身份字段清洗逻辑，并通过 协议一致性验证。
- `response_waiter`：已实现 guid 响应缓存、命中、读取、删除和超时清理逻辑，并通过 协议一致性验证。
- `robot_status_parser`：已实现 `notify_robot_info` 到 `/robot_status_raw` 的字段解析、单位转换、通信质量估算，以及 `notify_joy_data` 到 Joy 数组的解析逻辑，并通过 协议一致性验证。
- `action_result_builder`：已实现 `/robot/action_result` 动作完成事件组包、动作耗时计算、命令时间戳透传和动作通知数据透传逻辑，并通过 协议一致性验证。
- `walk_velocity_controller`：已实现 `/cmd_vel` 缓存、速度限幅、Walk 状态门控、动作执行门控和零速度停车退出的纯逻辑，并通过 协议一致性验证；正式导航默认允许真实 WebSocket 速度发送。
- `gesture_sync`：已接入连接后动作库同步链路，可从机器人拉取动作列表、更新 `gestures.yaml`、重新加载动作时长并发布 `/system/gesture_list_updated`；正式导航默认开启。
- `motion_controller`：已实现 APP 动作命令解析、忙碌拒绝、动作预期时长到超时时间的计算逻辑，并通过 协议一致性验证；正式导航默认允许动作下发和 Walk/Menu 自动切换。
- `robot_ws_client`：已实现机器人 WebSocket URL 解析、真实连接/重连线程、命令发送、guid 响应等待、原始消息 title 分发、身份字段刷新和行走速度错误分类逻辑，并通过离线 协议一致性验证；正式导航默认连接真实机器人。
- 真实动作执行、真实速度发送、动作库同步和 Walk/Menu 自动切换仍保留独立 YAML/launch 开关，离线验证或排查风险时可显式关闭。

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

通过控制层总入口按正式 C++ 控制层启动：

```bash
ros2 launch humanoid_bringup robot_control_plane.launch.py
```

通过控制层总入口只验证 C++ 节点启动，不连接真实机器人：

```bash
ros2 launch humanoid_bringup robot_control_plane.launch.py \
  robot_ws_enable:=false \
  robot_walk_velocity_send_enable:=false \
  robot_motion_execution_enable:=false
```

如果需要保留机器人连接，但临时禁止真实速度/动作/动作库同步：

```bash
ros2 launch humanoid_bringup robot_control_plane.launch.py \
  robot_ws_enable:=true \
  robot_walk_velocity_send_enable:=false \
  robot_motion_execution_enable:=false \
  robot_gesture_sync_enable:=false \
  robot_motion_allow_enter_menu:=false \
  robot_motion_allow_return_walk:=false
```

动作库同步单独关闭示例：

```bash
ros2 launch humanoid_bringup robot_control_plane.launch.py \
  robot_ws_enable:=true \
  robot_gesture_sync_enable:=false
```

`gesture_sync_enable=true` 会在机器人 WebSocket 连接稳定后请求 `request_get_atomic_motion_list`，默认写入 `humanoid_expression_runtime/config/gestures.yaml`，然后通知数据整合节点热重载。若需要写到临时文件，可在 `robot_gateway.yaml` 中设置 `gestures_yaml_path`。

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
- `include/.../robot_protocol.hpp` / `src/robot_protocol.cpp`：机器人本体协议组包和解析边界；上游是 motion/walk/gesture 模块的命令请求，下游是 `robot_ws_client`。
- `include/.../response_waiter.hpp` / `src/response_waiter.cpp`：请求 guid 与响应等待关系管理；上游是 `robot_ws_client` 收到的响应，下游是同步命令、动作执行和动作库同步。
- `include/.../robot_status_parser.hpp` / `src/robot_status_parser.cpp`：解析机器人状态、身份、joy 数据；上游是机器人本体 notify 消息，下游是 `/robot_status_raw`、`/joy_raw`。
- `include/.../walk_velocity_controller.hpp` / `src/walk_velocity_controller.cpp`：管理 `/cmd_vel` 缓存、定频发送决策、Walk 状态门控、动作执行门控和零速度停车退出；上游是 Nav2/controller，下游是机器人本体行走速度命令。
- `include/.../motion_controller.hpp` / `src/motion_controller.cpp`：管理 APP 动作命令解析、动作入口门控、动作超时计算；上游是 `/app/robot_control`，下游是机器人本体动作命令和 `/robot/action_result`。
- `include/.../gesture_sync.hpp` / `src/gesture_sync.cpp`：解析机器人动作库响应、生成 gestures.yaml 内容、生成热重载通知；上游是机器人本体动作库接口，下游是 `/system/gesture_list_updated` 和数据整合节点。
- `include/.../action_result_builder.hpp` / `src/action_result_builder.cpp`：统一构建动作执行结果 JSON；上游是 motion_controller，下游是 `/robot/action_result`。

### 开发期探针

`test/probes/*_probe.cpp` 是迁移和协议一致性验证时使用的开发期小程序，默认不参与正式构建和安装。需要单独验证某个纯逻辑模块时，可临时执行：

```bash
colcon build --packages-select humanoid_robot_gateway_runtime --cmake-args -DBUILD_RUNTIME_PROBES=ON
```

## 维护原则

- 机器人本体 WebSocket、速度发送、动作执行和 Walk/Menu 切换都属于高风险逻辑；正式联调默认打开，但测试现场必须保留急停和人工接管。
- `walk_velocity_send_enable=true` 时，必须确认机器人状态稳定进入 `WALK`，并保留超时停和零速度保护。
- `gesture_sync_enable=true` 会写入 `gestures.yaml`；如果只想验证连接，可临时设为 false。
- `motion_execution_enable=true`、`motion_allow_enter_menu=true`、`motion_allow_return_walk=true` 会允许动作链路切换 Menu/Walk；导航过程中如发现动作与行走冲突，可优先临时关闭这三个开关。
- 任何本包输出的 `/robot_status_raw`、`/robot/action_result` 字段都必须和 线上协议对齐。
