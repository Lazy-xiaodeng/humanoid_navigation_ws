# 路线任务 ROS 环境接力验证说明

本文档用于把当前 Windows 本地 Codex 已完成的 route task 功能开发，交接给 ROS2 sourced 环境中的 Codex 继续执行运行级验证、构建验证和 APP 联调。

> 中文注释：当前本地环境没有可用的 `ros2` 和 `colcon`，因此本文档不是“已验收证明”，而是“接力验证手册”。ROS 环境 Codex 应以本文档、实施清单和代码修改记录为输入，继续完成最后一段真实运行验证。

---

## 1. 当前交接结论

当前功能代码侧已经完成首版 through 路线任务主链路开发，目标是拿到 ROS 环境后验证：

1. 首版 route task 以 `NavigateThroughPoses` 作为验收主路径。
2. `start_route_task / jump_to_waypoint / broadcast_finished` 三类 route task 命令能走统一业务 ack：`navigation_command_result`。
3. through feedback / pose fallback 只推进 transit，不直接完成 task。
4. `broadcast_requested -> broadcast_finished -> task_waypoint_completed` 播报闭环可运行。
5. 任意跳点能重建当前段，且 `passed_transit_waypoint_ids` 只作用于当前段。
6. 失败时发布 `navigation_failed(route_task=true)`，并带 `failure_code`、路线、目标、段信息。
7. APP 可通过 `event_id / timestamp / request_message_id / task_session_id / route_id` 做去重、排序、回溯和 UI 更新。

> 中文注释：这里说“代码侧完成”，只代表源码和本地静态护栏已经收口，不代表 ROS action server、Nav2 feedback 字段、launch 安装、APP 实际消费都已经通过。这些必须在 ROS 环境继续验证。

---

## 2. 需要携带/重点核对的文件

核心代码文件：

1. `src/humanoid_navigation/humanoid_navigation/navigation_state_manager.py`
2. `src/humanoid_navigation/humanoid_navigation/dynamic_waypoints_manager.py`
3. `src/humanoid_websocket/humanoid_websocket/websocket_server.py`
4. `src/humanoid_websocket/humanoid_websocket/data_integration_node_recoverable.py`
5. `src/humanoid_navigation/package.xml`
6. `src/humanoid_navigation/launch/navigation.launch.py`
7. `src/humanoid_navigation/launch/navigation_fusion.launch.py`
8. `src/humanoid_navigation/launch/navigation_fusion_sc.launch.py`
9. `src/humanoid_bringup/launch/robot_real.launch.py`

辅助校验和文档文件：

1. `src/humanoid_navigation2/scripts/validate_route_task_static.py`
2. `src/humanoid_navigation2/docs/路线任务改造函数级实施清单.md`
3. `src/humanoid_navigation2/docs/路线任务改造代码修改记录.md`
4. `src/humanoid_navigation2/docs/路线任务ROS环境接力验证说明.md`

> 中文注释：ROS 环境中如果只拷贝 Python 源码而漏掉 `package.xml` 或 launch 文件，可能出现源码能读、但安装依赖或默认入口仍不正确的问题。因此这里把 package 和 launch 也列入交接文件。

---

## 3. ROS 环境第一步：源码静态护栏

进入工作区根目录后先运行：

```bash
python src/humanoid_navigation2/scripts/validate_route_task_static.py
```

期望输出：

```text
路线任务静态校验通过：关键 through、协议、入口和事件不变量均存在。
```

这一步证明：

1. 首版 through 主路径仍在。
2. route task 协议字段、事件字段和 APP 消费说明仍在。
3. `navigation_command_result`、`broadcast_requested`、`waypoint_passed`、`jump_updated`、`task_waypoint_completed`、`route_task_completed`、`navigation_failed` 的关键不变量仍在。
4. `event_data.timestamp`、`event_id`、`request_message_id` 等追踪字段仍在。

这一步不能证明：

1. ROS2 action server 已经存在。
2. Nav2 runtime feedback 字段与源码假设一致。
3. `colcon build` 已经通过。
4. APP 已经正确消费 websocket 推送。

> 中文注释：静态护栏是“防止源码退化”的第一关，不是实机验收。它失败时应先修源码或清单，不建议直接进入 ROS 运行验证。

---

## 4. ROS 环境第二步：Python 语法和 XML/编码检查

运行完整语法检查：

```bash
python -m py_compile src/humanoid_navigation/humanoid_navigation/navigation_state_manager.py src/humanoid_navigation/humanoid_navigation/dynamic_waypoints_manager.py src/humanoid_websocket/humanoid_websocket/websocket_server.py src/humanoid_websocket/humanoid_websocket/data_integration_node_recoverable.py src/humanoid_navigation/launch/navigation.launch.py src/humanoid_navigation/launch/navigation_fusion.launch.py src/humanoid_navigation/launch/navigation_fusion_sc.launch.py src/humanoid_bringup/launch/robot_real.launch.py src/humanoid_navigation2/scripts/validate_route_task_static.py
```

运行文档编码和 `package.xml` 检查：

```bash
python -c "import xml.etree.ElementTree as ET; from pathlib import Path; ET.parse(r'src/humanoid_navigation/package.xml'); [Path(p).read_text(encoding='utf-8') for p in [r'src/humanoid_navigation2/docs/路线任务改造函数级实施清单.md', r'src/humanoid_navigation2/docs/路线任务改造代码修改记录.md', r'src/humanoid_navigation2/docs/路线任务ROS环境接力验证说明.md', r'src/humanoid_navigation2/scripts/validate_route_task_static.py', r'src/humanoid_navigation/humanoid_navigation/navigation_state_manager.py']]; print('XML and UTF-8 read ok')"
```

> 中文注释：Windows PowerShell 可能显示中文乱码，但只要 UTF-8 读取检查通过，通常说明文件本身没有损坏。ROS 环境中如果终端正常显示中文，可直接阅读这些文档。

---

## 5. ROS 环境第三步：Action Server 验证

source ROS2 和工作空间后运行：

```bash
ros2 action list | grep navigate_through_poses
ros2 action info /navigate_through_poses
```

必须确认：

1. action 列表中存在 `/navigate_through_poses`。
2. action 类型是 Nav2 的 `NavigateThroughPoses`。
3. action server 数量不为 0。
4. 当前命名空间和源码中的 action client 名称一致。

如果 action 名称不一致：

1. 不要退回逐点 `NavigateToPose`。
2. 先确认 Nav2 bringup 的 namespace。
3. 再调整 `ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')` 或 launch namespace。
4. 修改后同步更新实施清单和修改记录。

> 中文注释：这是首版 through 验收最关键的运行环境门槛。如果这里不通过，route task 不能算完成，也不应以“逐点先跑通”替代。

---

## 6. ROS 环境第四步：构建验证

在 ROS2 工作区根目录运行：

```bash
colcon build
```

建议如果全量构建太慢，可先尝试：

```bash
colcon build --packages-select humanoid_navigation humanoid_websocket humanoid_bringup
```

构建后重新 source：

```bash
source install/setup.bash
```

需要重点观察：

1. `nav2_msgs` 依赖是否可解析。
2. `action_msgs` 依赖是否可解析。
3. `humanoid_navigation` console script 是否安装成功。
4. launch 文件是否能找到 `navigation_state_manager` 新入口。

> 中文注释：当前本地 Windows 环境没有 `colcon`，所以这个步骤必须由 ROS 环境完成。构建失败时不要只改运行命令，应回到 package/launch/依赖声明定位根因。

---

## 7. ROS 环境第五步：节点和 Launch 入口验证

建议先验证节点入口：

```bash
ros2 run humanoid_navigation navigation_state_manager
```

再验证实际 launch：

```bash
ros2 launch humanoid_navigation navigation.launch.py
```

如现场使用融合或实机入口，再验证：

```bash
ros2 launch humanoid_navigation navigation_fusion.launch.py
ros2 launch humanoid_navigation navigation_fusion_sc.launch.py
ros2 launch humanoid_bringup robot_real.launch.py
```

需要确认：

1. 默认 launch 启动的是 `navigation_state_manager`，不是旧 recoverable 入口。
2. route task 参数已加载，例如 `route_task.nav2_feedback_timeout_sec`。
3. 节点启动后没有 import error、action type error、参数类型错误。

> 中文注释：`navigation_state_manager_recoverable.py` 首版不作为 route task 验收入口。若必须切回 recoverable，需要完整迁移 route task 逻辑，不应只改 launch 指向。

---

## 8. APP/协议联调主路径

建议先准备一条包含 task 和 transit 的路线，例如：

```text
3(task) -> 11(transit) -> 12(transit) -> 15(task) -> 18(task)
```

主路径期望事件顺序：

```text
APP start_route_task
-> websocket command_ack
-> navigation_command_result(status=success, command_type=start_route_task)
-> waypoint_passed(11)
-> waypoint_passed(12)
-> broadcast_requested(15) 或 task_waypoint_completed(15)
-> APP broadcast_finished
-> navigation_command_result(status=success, command_type=broadcast_finished)
-> task_waypoint_completed(15)
-> 下一段 through
-> route_task_completed
```

必须检查：

1. `navigation_command_result` 的 `request_message_id` 能对应 APP 外层 `message_id`。
2. 所有 route task 事件都有 `event_id / timestamp / task_session_id / route_id`。
3. transit 只发 `waypoint_passed`，不进入 `completed_task_ids`。
4. task 完成只由 through result 成功路径或播报完成路径推进。
5. `route_task_completed` 先带摘要发出，再清理运行态。

> 中文注释：websocket 的 `command_ack` 只表示收到包，不能当业务成功。APP 业务 UI 必须等 `navigation_command_result` 和后续 route task 事件。

---

## 9. Jump 联调路径

正向 jump：

```text
当前目标 15，APP jump_to_waypoint 18
```

反向 jump：

```text
当前或等待播报在 15，APP jump_to_waypoint 3
```

必须检查：

1. `jump_updated` 只在新 through 段发起成功后发布。
2. `execution_waypoint_ids` 是新段快照。
3. `skipped_task_ids` 去重，且不包含已完成 task。
4. `passed_transit_waypoint_ids` 是当前段级状态，新段重新置空。
5. `15 -> 3` 可以重新经过历史 transit，例如 `12 / 11`。
6. `already_current_target` 返回 `status=success` 且 `result_reason=already_current_target`，不取消旧 goal，不改状态。
7. 播报等待中 `interrupt_broadcast=false` 返回 `interrupt_broadcast_false_not_supported`。
8. 播报等待中 `interrupt_broadcast=true` 成功 jump，并退出旧播报等待 UI。

> 中文注释：jump 的核心不是“把目标点改掉”，而是从完整 route 数组重新计算当前进度锚点到新目标之间的 through 段。这个点是最容易在实机联调中暴露 bug 的地方。

---

## 10. 失败场景联调

建议至少覆盖：

1. 已有 route task 时再次 `start_route_task`：期望 `route_task_already_running`。
2. 普通导航运行时 `start_route_task`：期望 `navigation_busy`，且不取消旧导航。
3. route 中没有 task：期望 `missing_task_waypoints`。
4. waypoint role 非法：期望 `invalid_waypoint_role`。
5. jump 到 transit：期望 `target_waypoint_not_task`。
6. jump 到不存在点：期望 `invalid_target_waypoint`。
7. broadcast session 不匹配：期望 `invalid_task_session`。
8. broadcast 上下文不匹配：期望 `broadcast_context_mismatch`。
9. broadcast 显式空结果：期望 `unsupported_broadcast_result`。
10. Nav2 goal rejected / canceled / failed / feedback timeout：期望 `navigation_failed(route_task=true)`，并带对应 `failure_code`。

> 中文注释：route task 失败统一通过 `navigation_failed(route_task=true)` 做失败复盘，不再额外走旧 `/navigation/acknowledgments` 形成重复业务事件。命令同步校验错误才走 `navigation_command_result(error)`。

---

## 11. 发现问题后的回填规则

ROS 环境 Codex 修改任何代码后，必须同步更新：

1. `src/humanoid_navigation2/docs/路线任务改造代码修改记录.md`
2. 必要时更新 `src/humanoid_navigation2/docs/路线任务改造函数级实施清单.md`
3. 必要时更新 `src/humanoid_navigation2/scripts/validate_route_task_static.py`
4. 必要时更新本文档

修改记录要求：

1. 按时间戳顺序追加。
2. 写清楚“修改了什么、为什么改、验证命令、结果、下一步”。
3. 如果某个验证不能跑，必须写清楚原因，不能写成通过。
4. 如果发现 ROS 环境和本地假设不一致，要记录真实 action 名称、feedback 字段、错误日志。

> 中文注释：这份项目已经有较长的修改记录，后续继续接力时不要只在聊天里说明。必须把关键决策落到 Markdown，否则下一次接手又会重新摸索。

---

## 12. ROS 环境完成标准

当以下条件都满足时，ROS 环境接力验证可以认为完成：

1. 静态校验通过。
2. Python 语法检查通过。
3. UTF-8/XML 检查通过。
4. `colcon build` 通过。
5. `/navigate_through_poses` action server 验证通过。
6. 默认 launch 使用新 `navigation_state_manager` 启动通过。
7. APP 主路径联调通过。
8. APP 正向/反向 jump 联调通过。
9. 播报等待、播报完成、重复播报回执联调通过。
10. 主要失败场景返回预期 `error_code / failure_code`。
11. 修改记录已追加 ROS 环境验证结果。

> 中文注释：只有这些都满足，才建议把 route task 首版标记为可验收。若只完成源码静态检查，不能替代 ROS runtime 和 APP 联调。


---

## 13. 本轮 ROS 环境新增发现：`/navigate_through_poses` 未注册

本轮在 ROS2 Jazzy 环境中实际执行：

```bash
ros2 action list
```

当前输出只有：

```text
/compute_path_through_poses
/compute_path_to_pose
/follow_path
/navigate_to_pose
/spin
```

这说明当前运行中的 `bt_navigator` 没有注册 `/navigate_through_poses` action server。代码侧虽然已经接入 `NavigateThroughPoses`，但如果运行环境没有该 action，路线任务会在 through 段启动时失败。

已在 Todesk 目录下 6 套 `src/humanoid_navigation2/config/nav2_params*.yaml` 补齐：

```yaml
navigators: ["navigate_to_pose", "navigate_through_poses"]
navigate_to_pose:
  plugin: "nav2_bt_navigator::NavigateToPoseNavigator"
navigate_through_poses:
  plugin: "nav2_bt_navigator::NavigateThroughPosesNavigator"
```

验证要求更新为：修改后必须重启 Nav2/整机导航，再重新执行：

```bash
ros2 action list | grep navigate_through_poses
ros2 action info /navigate_through_poses
```

如果重启后仍没有 `/navigate_through_poses`，需要继续检查实际启动使用的 `nav2_params_file` 是否是 Todesk 目录下已修改的参数文件，或者 launch 是否被其它安装空间/旧 install 覆盖。


---

## 14. 路径隔离与 through action 临时验证补充

Todesk 实施目录已补充路径隔离要求：核心运行文件和安装产物不得再引用 `/home/ubuntu/humanoid_ws`。验证命令：

```bash
rg -n "/home/ubuntu/humanoid_ws" install/humanoid_navigation install/humanoid_navigation2 install/humanoid_bringup -S
```

期望无输出。

已完成一次不启动整机导航的临时 through action 验证：

1. source Todesk install。
2. 用 `/route_task_check` 命名空间临时启动 `bt_navigator`。
3. lifecycle configure 后可看到：

```text
/route_task_check/navigate_through_poses
/route_task_check/navigate_to_pose
```

4. `ros2 action type /route_task_check/navigate_through_poses` 输出：

```text
nav2_msgs/action/NavigateThroughPoses
```

这证明 Todesk 的 Nav2 参数已经能让 bt_navigator 注册 through action。完整可用性仍需在整机 Nav2 启动后确认非命名空间的 `/navigate_through_poses`，并继续做 APP 端路线任务联调。
