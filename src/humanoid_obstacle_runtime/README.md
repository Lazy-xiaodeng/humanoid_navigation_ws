# humanoid_obstacle_runtime

`humanoid_obstacle_runtime` 是机器人障碍检测运行层 C++ 功能包。当前主要承载前方 ROI 障碍检测节点，后续如果增加碰撞检测、恢复校验、障碍状态融合等能力，也建议放在这里。

## 包作用

- 订阅点云，在机器人前方 ROI 窗口内检测障碍点。
- 发布 `/front_obstacle/has_obstacle`，供路线/恢复逻辑判断前方是否清空。
- 发布 `/front_obstacle/debug` 和 `/front_obstacle/roi_cloud`，用于调试检测质量。

## 文件说明

- `src/roi_obstacle_detector_cpp.cpp`：ROI 障碍检测主节点，负责点云坐标转换、地面补偿、ROI 过滤、体素降采样、聚类和稳定帧判定。
- `config/obstacle_runtime.yaml`：障碍检测参考参数，带中文注释；正式导航可继续复用原 ROI 参数文件。
- `launch/obstacle_runtime.launch.py`：单独启动 ROI 障碍检测节点的 launch 文件。

## 上下游链路

上游：

- 原始雷达点云或点云滤波结果。
- 可选 IMU，用于姿态 leveling。

下游：

- 路线运行层/恢复等待逻辑：读取 `/front_obstacle/has_obstacle`。
- 调试工具：查看 `/front_obstacle/debug` 与 `/front_obstacle/roi_cloud`。

## 使用说明

正式导航通常由 `humanoid_navigation2` launch 启动；单独调试可运行：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 launch humanoid_obstacle_runtime obstacle_runtime.launch.py
```

## 维护说明

ROI 障碍检测已经独立到本包。后续如果增加碰撞检测、恢复校验或障碍状态融合逻辑，建议继续放在本包内维护。
