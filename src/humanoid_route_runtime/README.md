# humanoid_route_runtime

## 这个包是做什么的

`humanoid_route_runtime` 是机器人导航系统的路线任务运行层包。

它负责把 APP/控制层发来的路线任务转换成可执行的运行态，并把导航过程中的状态、事件、失败原因、播报等待、暂停恢复等信息发布回系统。

它不替代 Nav2、定位、点云滤波或控制器，而是站在这些模块上方做任务编排。

## 当前状态

当前已经接入路线任务运行主链路：

- 节点可以独立启动，声明完整运行参数，创建 ROS 话题、action client 和 TF listener。
- 可以缓存点位库 revision、地图状态、定位状态、机器人底层状态、odom/costmap/ROI/BT 日志。
- 可以解析 `/navigation/requests`，支持 `start_route_task`、`pause_route_task`、`resume_route_task`、`stop_route_task`、`jump_to_waypoint`、`broadcast_finished`。
- 可以构建 task/transit 路线段，发布 `waypoint_passed`、`broadcast_requested`、`task_waypoint_completed`、`route_task_completed` 等事件。
- 可以通过 `NavigateThroughPoses` 执行 transit 段，通过 `NavigateToPose` 执行最终 task 对齐；该能力由 `route_task.nav2_execution_enable` 控制，便于分阶段验证。
- 可以处理 jump 重规划、goal rejected 重试、feedback 超时、暂停/恢复/停止、节点退出清理。
- 可以在机器人状态未就绪时缓存 start 请求，等待 Walk 和定位/地图就绪后自动启动。
- 可以根据 Nav2 失败、速度停滞、BT 日志、costmap 和 ROI 进入障碍等待，并在前方 clear 后自动恢复。

当前已接入导航层分阶段启动链路，并保留运行开关。仍可通过运行开关选择正式链路，只有显式打开
`use_cpp_route_runtime` / `USE_CPP_ROUTE_RUNTIME=true` 时才会启用 C++ 路线运行层。
切到默认启用前，仍建议继续用 bag、模拟 action server、实机路线对比验证事件字段和导航行为。

## 主要输入输出接口

保持以下 ROS 接口稳定：

- 订阅 `/navigation/requests`：接收控制层或 APP 桥接层发来的路线任务命令。
- 订阅 `/navigation/waypoints_data`：接收点位库和 revision。
- 订阅 `/odom`：读取 Fast-LIO/定位链路输出，用于速度、阻塞和状态估计。
- 订阅 `/local_costmap/costmap`：障碍暂停恢复时检查前方 costmap 是否清空。
- 订阅 `/front_obstacle/has_obstacle`：结合 ROI 判断障碍是否离开正前方。
- 订阅 `/robot_status_raw`：判断机器人是否处于可行走模式。
- 订阅 `/behavior_tree_log`：识别 Nav2 是否处于恢复、等待、后退等行为树状态。
- 订阅 `/localization/prior_map_odom_bridge_status`：判断定位健康状态。
- 订阅 `/map/status`：判断地图是否 ready，避免按旧地图启动路线。
- 发布 `/navigation/status`：发布路线任务事件和周期状态。
- 发布 `/navigation/acknowledgments`：兼容旧命令确认链路。
- 发布 `/goal_pose`：兼容可视化/调试目标点显示。
- 发布 `/cmd_vel`：停止/暂停/兜底时可发布零速度。
- Action client `navigate_to_pose`：最终 task 到点和 yaw 对齐。
- Action client `navigate_through_poses`：经过 transit 点的路线段执行。

## 使用方式

仅用于开发验证时可以单独启动：

```bash
ros2 launch humanoid_route_runtime route_runtime.launch.py
```

完整系统回归完成前，建议通过独立 launch 或 `USE_CPP_ROUTE_RUNTIME=true` 做分阶段验证，默认启动仍可切换到既有链路。

## 参数配置

参数文件在：

```text
config/route_runtime.yaml
```

每个参数旁边都有中文注释。现场调试时优先通过本 YAML 修改参数，避免把临时测试值写死在源码里。

## 维护原则

- 保持 `/navigation/status` JSON 事件字段完全兼容，否则导航页会出现按钮置灰、任务详情重复或状态错乱。
- 保持路线任务语义兼容：task/transit、jump、broadcast、pause/resume/stop、失败复盘都不能变。
- Nav2 action 发送、goal cancel、result callback 都属于高风险副作用逻辑，必须先用模拟脚本和 bag 验证。
- 先接入高频、确定、无副作用逻辑，再接入会驱动机器人运动的 action 回调和异常恢复逻辑。
- APP、WebSocket、路线运行层依赖统一 JSON 事件结构，新增字段可以兼容添加，已有字段不要随意改名或改变类型。
