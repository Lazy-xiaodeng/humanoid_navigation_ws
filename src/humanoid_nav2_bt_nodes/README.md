# humanoid_nav2_bt_nodes

## 这个包是做什么的

`humanoid_nav2_bt_nodes` 是 Nav2 自定义行为树插件包，用于扩展默认 Nav2 行为树能力。

它不是独立 ROS 节点，运行时通常不会在进程列表里看到单独进程。它会被 `bt_navigator` 根据 `nav2_params.yaml` 中的 `plugin_lib_names` 动态加载。

## 当前状态

- 当前默认 Nav2 参数已经加载 `humanoid_nav2_bt_nodes`。
- `navigate_xy_then_yaw.xml` 和 `navigate_reverse_xy_then_yaw.xml` 中使用了 `SpinToPose`。
- 这个包参与“先对准目标方向、走 XY、到点后再对齐最终 yaw”的分阶段导航逻辑。
- 删除或加载失败会导致 `bt_navigator` 无法识别自定义 BT 节点。

## 插件功能

- `MakePoseTowardGoal`：根据目标点生成朝向目标的中间 pose。
- `SpinToPose`：计算当前朝向和目标朝向差，调用 Nav2 `Spin` action 完成转向。

## 文件说明

- `src/pose_angle_nodes.cpp`：自定义 BT 节点实现和 BehaviorTree.CPP 注册入口。
- `include/humanoid_nav2_bt_nodes/pose_angle_nodes.hpp`：节点类定义、输入端口和内部状态。
- `CMakeLists.txt`：编译共享库并导出给 Nav2 行为树加载。
- `package.xml`：声明 BehaviorTree.CPP、Nav2 BT、Spin action、TF 等依赖。

## 上下游链路

上游：

- `bt_navigator` 读取 `nav2_params.yaml` 的 `plugin_lib_names`。
- 行为树 XML 中使用 `SpinToPose` 或 `MakePoseTowardGoal`。
- TF 系统提供机器人当前位姿。

下游：

- Nav2 `behavior_server` 的 `Spin` action。
- Nav2 行为树执行流程。

## 使用方式

正式导航中由 Nav2 自动加载，不需要单独启动。

确认参数中已配置：

```yaml
bt_navigator:
  ros__parameters:
    plugin_lib_names:
      - humanoid_nav2_bt_nodes
```

行为树 XML 示例：

```xml
<SpinToPose goal="{goal}" mode="goal_yaw"/>
```

## 维护注意事项

- 这是导航核心行为插件，不能作为“无进程包”直接删除。
- 修改 `SpinToPose` 会影响到点后对齐和路线任务最终朝向表现。
- 如果行为树 XML 中新增自定义节点，需要在本包中注册并确保 `plugin_lib_names` 加载该库。
