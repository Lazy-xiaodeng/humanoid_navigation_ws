# humanoid_route_runtime

`humanoid_route_runtime` 是 Test 工作区单地图导航状态管理器的 C++ 版本。它负责接收路点管理器发布到 `/navigation/requests` 的导航命令，执行 Nav2 `NavigateToPose` action，并向 APP 链路回传 `/navigation/status` 与 `/navigation/acknowledgments`。

## 文件说明

- `src/navigation_state_manager.cpp`：导航状态机主节点。包含启动门控、Nav2 action、暂停/恢复/停止、失败可恢复、障碍等待恢复、定位健康监听和状态 JSON 回传。
- `config/route_runtime.yaml`：节点参数，中文注释说明每个开关的作用和影响。
- `launch/route_runtime.launch.py`：单独启动 C++ 导航状态管理器，便于与 Python 版本做 A/B 对比。

## 上下游链路

- 上游输入：`/navigation/requests`、`/navigation/waypoints_data`、`/odom`、`/robot_status_raw`、`/localization/prior_map_odom_bridge_status`、`/local_costmap/costmap`、`/front_obstacle/has_obstacle`、`/behavior_tree_log`。
- 下游输出：`navigate_to_pose` action、`/cmd_vel` 零速度保护、`/navigation/status`、`/navigation/acknowledgments`、`/navigation/current_path`、`/localization/recovery_requests`。

## 使用方式

```bash
source install/setup.bash
ros2 launch humanoid_route_runtime route_runtime.launch.py
```

当前包保持 Test 工作区原有单地图协议，不包含多地图路线任务逻辑。默认一键启动暂不切换到本包，建议先完成模拟和实机对比后再替换入口。
