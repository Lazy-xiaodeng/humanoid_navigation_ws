# humanoid_control_runtime

## 这个包是做什么的

`humanoid_control_runtime` 是机器人导航系统的常驻控制层运行包。

它位于 APP/WebSocket 接入层与导航执行层之间，主要负责“低频但关键”的业务状态维护：
点位库管理、路线任务命令转发、地图上下文维护、切图过程协调和状态回传。这个包不直接控制底盘，
也不直接执行 Nav2 行为树，而是把上层命令整理成稳定的 ROS 话题协议，交给下游路线运行层和导航定位层处理。

它承接两个常驻控制层节点：

- `dynamic_waypoints_manager_cpp`：负责 APP 点位管理命令、路线任务命令转发、点位 JSON 持久化、点位版本号维护。
- `map_context_manager_cpp`：负责地图列表/当前地图状态、切图命令、定位稳定状态监听、初始位姿回显。

这个包属于“控制层”，生命周期和导航定位层不同。它应该随 `robot_control_plane.launch.py` 常驻运行，切换地图时不应被停止。

## 当前状态

`dynamic_waypoints_manager_cpp` 已接入以下逻辑：

- 多地图点位 JSON 加载和保存。
- 旧 `dynamic_waypoints.json` 到多地图点位文件的兼容导入入口。
- 启动后 `/navigation/waypoints_data` 初始同步。
- `/app/waypoint_command` 的 set/update/delete/get/clear 处理。
- `/app/navigation_command` 的轻校验、ID 字符串归一化和 `/navigation/requests` 透传。

`map_context_manager_cpp` 已接入以下逻辑：

- 地图注册表加载、兜底创建和 `current_map_id` 持久化。
- `/app/map_command` 的 get_map_list/get_current_map/switch_map 处理。
- 导航执行中拒绝切图、地图禁用、2D/3D 地图文件缺失、切图脚本缺失等保护分支。
- 跨地图切换脚本调用、同地图定位 reset、初始位姿重复发布和定位稳定等待。
- `/map/response` 与 `/map/status` 协议封装。

当前版本已通过同输入一致性验证脚本验证核心语义，并已接入控制层分阶段启动链路。
仍可通过运行开关选择正式链路，只有显式打开 `use_cpp_control_runtime` / `USE_CPP_CONTROL_RUNTIME=true`
时才会启用 C++ 节点；切到默认启用前仍需要继续做 bag 回放、静态启动、实机导航、地图切换和异常恢复测试。

## 主要输入输出接口

`dynamic_waypoints_manager_cpp` 保持以下接口不变：

- 订阅 `/app/waypoint_command`：APP 下发点位新增、更新、删除、查询等命令。
- 订阅 `/app/navigation_command`：APP 下发路线任务开始、暂停、继续、停止、跳点等命令。
- 订阅 `/navigation/acknowledgments`：接收路线状态机的兼容确认消息。
- 发布 `/navigation/requests`：转发给路线运行层的导航/路线任务请求。
- 发布 `/navigation/waypoints_data`：向路线运行层同步当前点位库和 revision。

`map_context_manager_cpp` 保持以下接口不变：

- 订阅 `/app/map_command`：APP 查询地图、切换地图等命令。
- 订阅 `/navigation/status`：判断当前是否正在导航，导航中拒绝切图。
- 订阅 `/localization/prior_map_odom_bridge_status`：判断切图后的定位是否稳定。
- 发布 `/initialpose`：根据地图 registry 回显或重复发布初始位姿。
- 发布 `/map/response`：返回 APP 地图命令响应。
- 发布 `/map/status`：周期推送当前地图状态。

## 使用方式

仅用于开发验证时可以单独启动：

```bash
ros2 launch humanoid_control_runtime control_runtime.launch.py
```

完整系统回归完成前，建议通过独立 launch 或 `USE_CPP_CONTROL_RUNTIME=true` 做分阶段验证，默认启动仍可切换到既有链路。

## 当前验证

开发期可以使用 `.codex_tmp` 下的同输入一致性验证脚本验证外部协议语义。
这些脚本只用于开发回归，不属于运行时依赖。

`map_context_manager_cpp` 已覆盖地图列表、当前地图、未知命令、非法 JSON、未注册地图、禁用地图、2D 地图缺失、3D prior 地图缺失、导航中拒绝切图、脚本缺失、脚本异常退出、定位等待超时、failed 后同地图 reset 恢复、跨地图切换、切图中重复请求、初始位姿发布、JSON/纯文本定位稳定 ready、默认 registry 创建、损坏 registry fallback、`event_type` 导航状态忽略、`route_task.awaiting_broadcast` 拒绝切图、`paused` 拒绝切图和 registry 持久化。

## 参数配置

参数文件在：

```text
config/control_runtime.yaml
```

每个参数旁边都有中文注释。现场调试时优先通过本 YAML 修改参数，避免把临时测试值写死在源码里。

## 维护原则

- 保持话题名、JSON 字段、状态字段和错误码兼容。
- 写文件、切图、调用脚本这类有副作用的逻辑必须先做独立验证，再考虑把运行开关改为默认启用。
- 切图脚本调用必须谨慎，默认不要主动改已有启动链路。
- 点位库和地图注册表属于现场运行数据，调试时要确认路径指向当前工作区，避免误写其他工作区。
- APP、WebSocket、路线运行层依赖统一 JSON 外壳，新增字段可以兼容添加，已有字段不要随意改名或改变类型。
