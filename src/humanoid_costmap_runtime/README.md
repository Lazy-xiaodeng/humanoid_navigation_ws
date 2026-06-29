# humanoid_costmap_runtime

`humanoid_costmap_runtime` 是机器人 Nav2 costmap 辅助运行层 C++ 功能包。当前承载 3D clearing cloud 发布节点，负责向 Nav2 点云层补充清障射线，帮助 costmap 清除过期障碍。

## 包作用

- 订阅点云输入。
- 按角度和距离生成 clearing 点云。
- 发布给 Nav2 3D 点云层，用于清理局部/全局 costmap 中的残留障碍。

## 文件说明

- `src/periodic_clearing_3d_publisher.cpp`：周期性 3D 清障点云发布节点。
- `config/costmap_runtime.yaml`：清障辅助参考参数，带中文注释。
- `launch/costmap_runtime.launch.py`：单独启动 costmap 辅助节点的 launch 文件。

## 上下游链路

上游：

- 点云滤波或雷达点云话题。

下游：

- Nav2 local/global costmap 的 PointCloud2 observation layer。

## 使用说明

正式导航通常由 `humanoid_navigation2` launch 统一启动；单独调试可运行：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 launch humanoid_costmap_runtime costmap_runtime.launch.py
```

## 后续扩展

如果以后恢复 `lidar_height_clearing_3d_publisher` 或 `protected_clearing_3d_publisher` 的 C++ 版本，建议都归入本包。
