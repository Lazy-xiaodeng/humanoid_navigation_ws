# humanoid_app_gateway_runtime

## 这个包是做什么的

`humanoid_app_gateway_runtime` 是面向 APP / 导航页的 C++ 网关运行层包。

它负责承接当前 APP WebSocket 接入和数据整合职责：APP 协议解析、业务命令转 ROS
话题、系统数据聚合、订阅推送和统一响应。

它不直接连接机器人本体 WebSocket，也不直接驱动底盘运动。机器人本体连接和运动命令下发由
`humanoid_robot_gateway_runtime` 负责。

## 当前状态

当前采用“安全验证、按开关接入”的方式推进：

- `app_gateway_node`：已接入 APP WebSocket server、客户端连接管理、APP 请求/订阅转发、integration 响应回发和业务命令路由；正式导航默认 `websocket_server_enable=true`。
- `data_integration_node`：已接入机器人状态、导航状态、导航 ack、地图状态、地图响应、点位数据、动作结果、路径、定位位姿、odom 速度、APP 请求和 APP 订阅的核心数据链路；正式导航默认 `data_integration_enable=true`。
- `robot_status_adapter`：已实现 `/robot_status_raw` 到 `system_status` 的纯转换逻辑，并通过 协议一致性验证。
- `protocol_builder`：已实现 response、push、subscription_response、error_response 的协议组包逻辑，并通过 协议一致性验证。
- `data_store`：已实现数据缓存、更新时间、TTL 新鲜度判断和过期清理逻辑，并通过 协议一致性验证。
- `subscription_manager`：已实现订阅、取消订阅、推送频率节流、客户端移除和订阅统计逻辑，并通过 协议一致性验证。
- `app_protocol`：已实现 APP request/subscription 基础校验逻辑，并通过 协议一致性验证。
- `path_metrics`：已实现路径长度、预计耗时、平滑度、复杂度、安全等级等纯计算逻辑，并通过 协议一致性验证。
- `pose_speed_adapter`：已实现定位质量、置信度、速度死区、yaw、FastLIO twist 速度转换和基础速度 payload 逻辑，并通过 协议一致性验证。
- `navigation_status_adapter`：已实现导航事件立即推送判断和导航 ack 到 `navigation_command_result` 的基础转换逻辑，并通过 协议一致性验证。
- `map_waypoint_adapter`：已实现地图响应、地图状态和点位全量数据的 `data/metadata` 整理逻辑，并通过 协议一致性验证。
- `client_registry`：已实现 APP 客户端会话、本地订阅缓存、断开清理和订阅者查询逻辑，并通过 协议一致性验证。
- `integration_forwarder`：已实现 integration response、push、subscription_response 的 APP 路由决策逻辑，并通过 协议一致性验证。
- `business_command_router`：已实现 APP command 到导航、地图、机器人控制、表情、初始位姿和播报音量服务的路由决策逻辑，并通过 协议一致性验证；`system_command` 按当前协议表保持不可用，会返回统一错误响应。
- 当前 APP 侧核心纯逻辑模块、低风险 ROS/WebSocket 接入、连接初始快照、动作/表情库加载与热重载、定位恢复异常 `system_exception` 推送已完成第一轮实现和 smoke 验证。
- 后续仍需要在真实 APP/导航页真实接入环境里核对更细的状态增强字段、客户端健康检查和长连接异常恢复表现。
- 默认保持安全接入模式，不启动真实 APP WebSocket 服务、不接管正式 integration 链路、不改变现有运行链路。

## 规划中的主要输入输出接口

保持当前线上 ROS 接口稳定：

- APP 网关订阅 `/integration/data_responses`、`/integration/push_messages`、`/integration/subscription_responses`。
- APP 网关发布 `/websocket/data_requests`、`/websocket/data_subscriptions`。
- APP 网关发布 `/app/navigation_command`、`/app/waypoint_command`、`/app/map_command`、`/app/robot_control`。
- `/app/system_command` 参数仍保留在配置中，但当前协议表标记为不可用，C++ 网关不会路由该命令。
- APP 网关发布 `/robot/facial_raw_cmd` 和 `/initialpose`。
- 数据整合订阅 `/robot_status_raw`、`/robot_realpose`、`/odom`、`/plan`、`/navigation/status`、`/navigation/acknowledgments`、`/map/status`、`/map/response`、`/navigation/waypoints_data`、`/robot/action_result`、`/localization/recovery_status`。
- 数据整合发布 `/integration/data_responses`、`/integration/push_messages`、`/integration/subscription_responses`。

## 使用方式

当前用于独立编译、模拟验证和单独启动验证：

```bash
ros2 launch humanoid_app_gateway_runtime app_gateway_runtime.launch.py
```

安全单独启动数据整合验证，不抢占 APP 端口：

```bash
ros2 launch humanoid_app_gateway_runtime app_gateway_runtime.launch.py \
  websocket_server_enable:=false \
  data_integration_enable:=true
```

通过控制层总入口只验证 APP 网关，不连接机器人本体：

```bash
ros2 launch humanoid_bringup robot_control_plane.launch.py \
  app_websocket_server_enable:=false \
  data_integration_enable:=true \
  robot_ws_enable:=false
```

通过控制层总入口按正式 C++ 控制层启动：

```bash
ros2 launch humanoid_bringup robot_control_plane.launch.py
```

如果要同时验证 APP 侧 C++ 和机器人侧 C++，但不连接真实机器人：

```bash
ros2 launch humanoid_bringup robot_control_plane.launch.py \
  app_websocket_server_enable:=false \
  data_integration_enable:=true \
  robot_ws_enable:=false \
  robot_walk_velocity_send_enable:=false \
  robot_motion_execution_enable:=false \
  robot_gesture_sync_enable:=false
```

正式接入前应先在真实 APP 页面做长连接、订阅、刷新页面初始回显、导航事件和异常弹窗核对；如果发现问题，优先通过 launch 参数临时关闭 `app_websocket_server_enable` 或 `data_integration_enable` 做定位。

切换前后建议运行状态检查：

```bash
source install/setup.bash
tools/check_ws_runtime_status.sh
```

真实接入前建议先运行空载 smoke：

```bash
tools/run_ws_cpp_gray_smoke.sh
```

验证 C++ APP WebSocket 真实收发但不抢占 8765：

```bash
tools/run_ws_cpp_app_gateway_smoke.py
```

## 参数配置

参数文件在：

```text
config/app_gateway.yaml
config/data_integration.yaml
```

每个参数旁边都保留中文注释。现场调试优先修改 YAML，不要把临时经验值写死在源码里。

## 文件功能清单

### 顶层文件

- `CMakeLists.txt`：定义 C++ 编译目标、依赖、安装规则；上游是 colcon/ament 构建系统，下游是安装后的 `app_gateway_node` 和 `data_integration_node`。
- `package.xml`：声明 ROS 包元信息和依赖；上游是 colcon/rosdep，下游是 ROS 包发现和 launch 查找。
- `README.md`：说明本包职责、接口、文件结构和维护原则；上游是开发维护者，下游是后续实现和调试人员。

### 配置与启动

- `config/app_gateway.yaml`：配置 APP 网关节点的 WebSocket、客户端管理、APP 指令 topic；上游是 launch 参数加载，下游是 `app_gateway_node`。
- `config/data_integration.yaml`：配置数据整合节点的数据缓存、TTL、输入输出 topic；上游是 launch 参数加载，下游是 `data_integration_node`。
- `launch/app_gateway_runtime.launch.py`：单独启动 APP 网关和数据整合 C++ 运行节点；上游是开发者/bringup 运行开关，下游是两个 C++ 节点。

### APP 网关节点相关

- `include/.../app_gateway_types.hpp`：集中定义 APP 网关和数据整合配置结构；上游是 YAML 参数，下游是各模块和节点外壳。
- `include/.../app_gateway_config.hpp` / `src/app_gateway_config.cpp`：声明和读取 ROS 参数；上游是 YAML，下游是 `app_gateway_node`、`data_integration_node`。
- `src/app_gateway_node.cpp`：APP 网关 ROS Node 外壳；上游是 APP WebSocket 客户端和 integration 话题，下游是 `/websocket/*`、`/app/*` 等 ROS topic。
- `include/.../app_protocol.hpp` / `src/app_protocol.cpp`：APP 协议解析和基础消息校验；上游是 APP WebSocket JSON，下游是命令路由和数据请求转发。
- `include/.../client_registry.hpp` / `src/client_registry.cpp`：APP 客户端连接和订阅关系管理；上游是 WebSocket 连接事件，下游是 integration 转发和订阅推送。
- `include/.../business_command_router.hpp` / `src/business_command_router.cpp`：APP 业务命令路由；上游是 APP business_command，下游是导航、地图、机器人控制、表情和系统命令 topic。
- `include/.../integration_forwarder.hpp` / `src/integration_forwarder.cpp`：integration 数据回推 APP；上游是 `/integration/data_responses`、`/integration/push_messages`、`/integration/subscription_responses`，下游是 APP WebSocket 客户端。

### 数据整合节点相关

- `src/data_integration_node.cpp`：数据整合 ROS Node 外壳；上游是导航、地图、定位、机器人状态和 APP 请求 topic，下游是 integration 系列 topic。
- `include/.../data_store.hpp` / `src/data_store.cpp`：统一数据缓存和 TTL 管理；上游是各 adapter 输出，下游是请求响应和订阅推送。
- `include/.../subscription_manager.hpp` / `src/subscription_manager.cpp`：APP 数据订阅关系与推送频率管理；上游是 `/websocket/data_subscriptions`，下游是 push message 生成。
- `include/.../protocol_builder.hpp` / `src/protocol_builder.cpp`：构建 response、push、error、subscription_response 协议消息；上游是 data store、adapter、APP 请求和订阅请求，下游是 integration publisher。
- `include/.../robot_status_adapter.hpp` / `src/robot_status_adapter.cpp`：转换 `/robot_status_raw` 为 `system_status`；上游是 `robot_gateway_node`，下游是 data store。
- `include/.../navigation_status_adapter.hpp` / `src/navigation_status_adapter.cpp`：转换导航状态、ack、异常事件；上游是 `humanoid_route_runtime` 和导航状态 topic，下游是 data store / push message。
- `include/.../map_waypoint_adapter.hpp` / `src/map_waypoint_adapter.cpp`：转换地图状态、地图响应和点位库；上游是地图/点位管理节点，下游是 data store / APP 初始快照。
- `include/.../pose_speed_adapter.hpp` / `src/pose_speed_adapter.cpp`：转换定位位姿、odom 和速度估算；上游是定位/odom 话题，下游是 `robot_pose`、`robot_speed`。
- `include/.../path_metrics.hpp` / `src/path_metrics.cpp`：路径长度、平滑度、复杂度等纯计算工具；上游是 `/plan`，下游是 navigation_path 响应增强字段。

### 开发期探针

`test/probes/*_probe.cpp` 是迁移和协议一致性验证时使用的开发期小程序，默认不参与正式构建和安装。需要单独验证某个纯逻辑模块时，可临时执行：

```bash
colcon build --packages-select humanoid_app_gateway_runtime --cmake-args -DBUILD_RUNTIME_PROBES=ON
```

## 维护原则

- 保持 APP 协议字段兼容，尤其是 `system_status`、`navigation_status`、`map_status`、`waypoints_data`。
- 业务命令和数据响应要先完成协议一致性验证，再接入真实 APP。
- WebSocket server、客户端连接表和订阅推送涉及异步线程，必须避免跨线程直接修改共享状态。
- 数据整合模块应保持“纯聚合、纯转换”，不直接控制机器人运动。
- 当前阶段通过 YAML/launch 开关控制是否接入正式链路；真实 APP 页面验证稳定后再考虑调整默认启动方式。
