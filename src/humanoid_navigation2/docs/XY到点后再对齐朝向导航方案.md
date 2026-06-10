# XY 到点后再对齐朝向的 Nav2 分阶段导航方案

本文记录本次针对“大角度目标朝向导致绕圈/转圈”的修改内容，以及新的 Nav2 参数文件和行为树的控制逻辑。

## 目标

原来的路径规划和控制会同时追求目标点的 XY 位置和最终 yaw。目标朝向变化很大时，Hybrid/Dubins 类路径为了满足终点朝向约束，容易生成绕圈路径，或者在终点附近反复调整。

本次方案把“到达目标点”和“对齐最终朝向”拆成两个阶段：

1. 先让机器人朝向目标点 XY 方向。
2. 使用 2D 规划器和 RPP 控制器只跟踪路径到达目标 XY。
3. 到达 XY 容差后，不再用控制器继续修 yaw，而是由行为树调用 Spin action 对齐最终目标 yaw。
4. 最终旋转阶段允许机器人因为非理想原地旋转产生少量 XY 漂移，行为树不再重新检查 XY。

## 修改文件

新增文件：

- `src/humanoid_nav2_bt_nodes/package.xml`
- `src/humanoid_nav2_bt_nodes/CMakeLists.txt`
- `src/humanoid_nav2_bt_nodes/include/humanoid_nav2_bt_nodes/pose_angle_nodes.hpp`
- `src/humanoid_nav2_bt_nodes/src/pose_angle_nodes.cpp`
- `src/humanoid_navigation2/config/nav2_params_xy_yaw.yaml`
- `src/humanoid_navigation2/config/behavior_tree/navigate_xy_then_yaw.xml`
- `src/humanoid_navigation2/docs/XY到点后再对齐朝向导航方案.md`

修改文件：

- `src/humanoid_navigation2/launch/navigation2.launch.py`
  - 默认 Nav2 参数文件改为 `nav2_params_xy_yaw.yaml`。
  - 默认行为树文件改为 `navigate_xy_then_yaw.xml`。
  - 增加 `nav2_params_file` 和 `bt_xml_file` launch 参数，方便回退或切换配置。
- `src/humanoid_navigation2/package.xml`
  - 增加对 `nav2_behavior_tree` 和 `humanoid_nav2_bt_nodes` 的依赖。

## 新 Nav2 配置文件

文件：

```text
src/humanoid_navigation2/config/nav2_params_xy_yaw.yaml
```

这个文件保留了原导航栈中与地图、代价地图、传感器、定位框架相关的配置，只调整规划器、控制器、行为树和旋转行为相关参数。

### BT Navigator

核心变化：

- 默认行为树指向 `navigate_xy_then_yaw.xml`。
- 只额外加载自定义 BT 插件库 `humanoid_nav2_bt_nodes`。
- 不显式列出 Nav2 Jazzy 内置 BT 插件库，因为 Jazzy 会自动加载内置节点。显式重复加载会导致类似 `ID [ComputePathToPose] already registered` 的崩溃。

自定义插件库提供两个 BT 节点：

- `MakePoseTowardGoal`
- `SpinToPose`

### Planner Server

规划器从 Hybrid/Dubins 思路切换为 2D A*：

```yaml
GridBased:
  plugin: "nav2_smac_planner::SmacPlanner2D"
  tolerance: 0.6
  allow_unknown: false
  use_final_approach_orientation: false
```

含义：

- 规划器只负责在 2D costmap 上找一条到目标 XY 的路径。
- 最终目标 yaw 不再作为路径规划约束。
- `use_final_approach_orientation: false` 防止规划器为了最终朝向扭曲路径。
- 到点后的最终朝向交给行为树最后的 `SpinToPose(mode=goal_yaw)`。

### Controller Server

控制器仍使用 RPP：

```yaml
FollowPath:
  plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
  use_rotate_to_heading: false
  allow_reversing: false
```

关键变化：

- `use_rotate_to_heading: false`
  - 不再依赖 RPP 的原地转向逻辑。
  - 避免 RPP 输出低于机器人响应死区的小角速度。
- `allow_reversing: false`
  - 保持禁止倒车，符合后方无传感器的安全约束。
- RPP 只负责路径跟踪，不负责最终目标 yaw。

新增 XY-only goal checker：

```yaml
goal_checker_plugins: ["xy_goal_checker", "general_goal_checker"]

xy_goal_checker:
  plugin: "nav2_controller::PositionGoalChecker"
  stateful: True
  xy_goal_tolerance: 0.7
```

行为树中的 `FollowPath` 会指定：

```xml
goal_checker_id="xy_goal_checker"
```

因此路径跟踪阶段只判断 XY 是否到达，不检查最终 yaw。

### Behavior Server

`SpinToPose` 不直接发布 `/cmd_vel`，而是调用 `behavior_server` 的 `spin` action。因此旋转仍然走 Nav2 的行为服务器逻辑，包括旋转速度限制和碰撞预测。

关键参数：

```yaml
spin:
  plugin: "nav2_behaviors::Spin"
  simulate_ahead_time: 2.0
  max_rotational_vel: 0.6
  min_rotational_vel: 0.18
  rotational_acc_lim: 0.3
```

Nav2 Jazzy 的 `Spin` 行为发布的速度指令只设置 `twist.angular.z`，不设置线速度，因此 `linear.x` 和 `linear.y` 为 0。这里把 `max_rotational_vel` 设为 `0.6`，让大角度旋转尽快完成；`min_rotational_vel` 设为 `0.18`，高于机器人约 `0.15 rad/s` 的角速度响应死区，避免出现 RPP 原地转向时输出 `0.08 rad/s` 导致机器人不动的问题。

当前配置仍加载了 `backup` 作为最后恢复手段：

```yaml
behavior_plugins: ["spin", "wait", "backup"]
```

如果后续实机确认后退风险不可接受，可以再把行为树里的 `BackUp` 分支和 behavior server 的 `backup` 插件一起移除。

## 新行为树

文件：

```text
src/humanoid_navigation2/config/behavior_tree/navigate_xy_then_yaw.xml
```

主流程：

```text
NavigateToPose goal
  -> SpinToPose(mode=goal_position)
  -> MakePoseTowardGoal
  -> ComputePathToPose(planner_id=GridBased)
  -> FollowPath(goal_checker_id=xy_goal_checker)
  -> SpinToPose(mode=goal_yaw)
  -> Success
```

### 阶段 1：先转向目标点方向

```xml
<SpinToPose goal="{goal}"
            mode="goal_position"
            yaw_tolerance="0.25"
            position_tolerance="0.20"
            time_allowance="20.0"
            max_attempts="3"/>
```

逻辑：

- 读取当前机器人位姿和目标位姿。
- 计算机器人当前位置到目标 XY 的方向角 `atan2(goal_y - robot_y, goal_x - robot_x)`。
- 用最短角距离计算需要旋转的角度。
- 如果角度误差小于 `0.25 rad`，直接成功。
- 否则调用 `behavior_server/spin` 旋转。
- 如果机器人已经离目标 XY 小于 `position_tolerance`，则认为无需先朝向目标点。

### 阶段 2：生成只用于 XY 导航的 approach_goal

```xml
<MakePoseTowardGoal goal="{goal}"
                    output_goal="{approach_goal}"
                    min_heading_distance="0.20"/>
```

逻辑：

- 保留原始目标的 XY。
- 把目标朝向改成“从当前机器人位置指向目标 XY”的方向。
- 这个 `approach_goal` 只用于规划和跟踪到目标点。
- 如果距离目标点太近，就保持当前机器人 yaw，避免近距离角度计算抖动。

### 阶段 3：2D A* 规划并只判断 XY 到达

```xml
<ComputePathToPose goal="{approach_goal}"
                   path="{path}"
                   planner_id="GridBased"/>

<FollowPath path="{path}"
            controller_id="FollowPath"
            goal_checker_id="xy_goal_checker"/>
```

逻辑：

- `ComputePathToPose` 使用 `SmacPlanner2D` 在全局 costmap 上规划路径。
- `FollowPath` 使用 RPP 跟踪路径。
- `xy_goal_checker` 只检查 XY 容差，不检查最终 yaw。
- 这一步成功后，行为树认为“直线/路径到点”阶段完成。

### 阶段 4：到点后对齐最终目标 yaw

```xml
<SpinToPose goal="{goal}"
            mode="goal_yaw"
            yaw_tolerance="0.25"
            time_allowance="20.0"
            max_attempts="4"/>
```

逻辑：

- 读取原始目标里的最终 yaw。
- 读取当前机器人 yaw。
- 计算最短角距离。
- 通过 `behavior_server/spin` 旋转到目标朝向。
- 旋转过程中如果产生少量 XY 位移，行为树不再回头重新跑 XY goal checker。

### 重规划和恢复逻辑

行为树不是单次静态执行，还保留了 Nav2 的恢复能力：

- `RateController hz="1.0"`
  - 路径跟踪期间以 1 Hz 重新计算全局路径。
  - 动态障碍或 costmap 变化时，路径可以更新。
- `GoalUpdated`
  - 如果收到新目标，恢复分支会优先响应新目标，而不是继续旧恢复动作。
- `Wait wait_duration="8.0"`
  - 内层恢复用于等待动态障碍物离开。
- `BackUp -> Spin -> Wait`
  - 外层恢复用于卡住或贴障碍时先退一点，再旋转重新寻找可行方向。
  - 当前配置中 BackUp 是最后恢复手段；如果后续实机安全策略要求完全禁止后退，可以移除该分支。

## 自定义 BT 节点

包：

```text
src/humanoid_nav2_bt_nodes
```

### MakePoseTowardGoal

功能：

- 输入原始 `goal`。
- 查询当前机器人在全局坐标系中的位姿。
- 计算当前位置指向目标点的 yaw。
- 输出新的 `approach_goal`。

主要端口：

- `goal`
- `output_goal`
- `global_frame`
- `robot_base_frame`
- `min_heading_distance`
- `transform_tolerance`

### SpinToPose

功能：

- 动态计算当前 yaw 到目标 yaw 的误差。
- 根据模式决定目标角度：
  - `goal_position`：目标角度为“当前点指向目标 XY”的方向。
  - `goal_yaw`：目标角度为用户目标姿态中的最终 yaw。
- 调用 `nav2_msgs/action/Spin`，由 `behavior_server` 执行旋转。
- 每次 spin 完成后重新读取当前 TF，再判断是否已经进入 yaw 容差。

主要端口：

- `goal`
- `mode`
- `yaw_tolerance`
- `position_tolerance`
- `time_allowance`
- `max_attempts`
- `server_name`
- `error_code_id`

实现细节：

- 使用 `angles::shortest_angular_distance()` 计算最短旋转角。
- 使用 `tf2_ros::Buffer` 查询当前机器人位姿。
- 使用 `rclcpp_action::Client<nav2_msgs::action::Spin>` 调用行为服务器。
- CMake 中为插件库增加了 `BT_PLUGIN_EXPORT`，导出 `BT_RegisterNodesFromPlugin` 符号，否则 BT.CPP 无法动态加载该插件。

## 为什么这个方案能减少大角度绕圈

原问题的根本原因是“路径规划/控制”同时承担两个目标：

- 到达目标 XY。
- 满足目标最终 yaw。

当最终 yaw 和从 A 到 B 的行走方向差很多时，Hybrid/Dubins 规划器可能为了终点朝向生成大弧线或绕圈路径。

新方案把这两个目标拆开：

- 行走阶段只要求到达 XY。
- 最终 yaw 由到点后的独立旋转完成。

这样规划器不会再为了满足最终朝向而改变整体路径形状，RPP 也不会在路径跟踪过程中反复停下来修目标 yaw。

## 启动与验证结果

编译命令：

```bash
colcon build --packages-select humanoid_nav2_bt_nodes humanoid_navigation2 --allow-overriding humanoid_navigation2 --symlink-install
```

本次验证结果：

- 编译通过。
- `libhumanoid_nav2_bt_nodes.so` 已导出 `BT_RegisterNodesFromPlugin`。
- 一键启动命令可以启动到 Nav2 生命周期配置阶段：

```bash
ros2 launch humanoid_bringup robot_real.launch.py 2>&1 | tee debug_output.txt
```

日志已确认：

- `planner_server` 创建 `GridBased`，类型为 `nav2_smac_planner::SmacPlanner2D`。
- `controller_server` 创建 `xy_goal_checker`，类型为 `nav2_controller::PositionGoalChecker`。
- `bt_navigator` 创建 `navigate_to_pose` 和 `navigate_through_poses` 时没有再出现插件重复注册或符号缺失崩溃。

当前启动仍卡在 TF 链路：

```text
Timed out waiting for transform from base_footprint to map
Tf has two or more unconnected trees.
```

同时日志中有：

```text
ERRCODE_MSOPTIMEOUT
```

这表示当前实机环境下雷达/定位链路没有正常发布完整 `map -> ... -> base_footprint` TF。这个问题不属于本次行为树和 Nav2 分阶段控制逻辑本身，需要单独从雷达数据、Fast-LIO、hdl_localization 初始化和 map->odom 发布链路排查。

## 后续调参建议

优先调这些参数：

- 初始和最终旋转容差：
  - 行为树里的 `yaw_tolerance="0.25"`。
  - 如果最终朝向要求更准，可降到 `0.20`。
  - 如果终点旋转容易失败，可升到 `0.30 ~ 0.40`。
- XY 到点容差：
  - `xy_goal_checker.xy_goal_tolerance: 0.7`。
  - 如果终点精度不够，可逐步降到 `0.4`、`0.3`。
  - 不建议一开始设太小，否则人形机器人容易在终点附近来回修正。
- Spin 最小角速度：
  - `behavior_server.spin.min_rotational_vel: 0.18`。
  - 必须高于机器人角速度死区。
- RPP 巡航速度：
  - `desired_linear_vel: 0.40`。
  - 如果实机晃动明显，可降到 `0.30` 或 `0.25`。

如果后续实机测试发现“最终旋转导致 XY 偏移过大”，可以考虑在最终 yaw 后增加一个可选的 XY 复核分支。但这会重新引入终点附近反复修正的风险，默认不建议开启。
