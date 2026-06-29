# humanoid_navigation2

## 这个包是做什么的

`humanoid_navigation2` 是导航层编排和资源包，负责提供 Nav2 参数、行为树、地图、PCD 工具脚本、导航 launch 和若干调试工具。

它不是单一业务节点包，而是导航层的配置与编排中心。当前实际运行的高频业务逻辑已经逐步拆到 C++ runtime 包中，例如路线任务、点位管理、定位桥、ROI 障碍检测、点云滤波和 costmap 辅助。

## 当前状态

- 默认 RO/RoboSense 链路由 `navigation2_robosense_lidar.launch.py` 启动。
- OP/Open3D 链路由 `navigation2.launch.py` 启动。
- Nav2 行为树采用“XY 到点 + 最终 yaw 对齐”的分阶段逻辑。
- 旧的 Python 辅助节点大多已经被 C++ runtime 包替代，但部分脚本仍用于建图、调试、验证和离线工具。

## 主要文件说明

- `launch/navigation2_robosense_lidar.launch.py`：RO/RoboSense 定位导航链路启动文件。
- `launch/navigation2.launch.py`：OP/Open3D 先验地图定位导航链路启动文件。
- `launch/mapping_only.launch.py`：建图相关启动文件。
- `config/nav2_params.yaml`：Nav2 主参数，包含 costmap、planner、controller、bt_navigator 等配置。
- `config/behavior_tree/`：Nav2 行为树 XML。
- `maps/`：2D 栅格地图。
- `pcd/`：3D 先验点云地图。
- `scripts/`：地图生成、PCD 转换、静态检查、协议模拟、调试 bag 录制等工具脚本。
- `humanoid_navigation2/`：保留的 Python 工具节点和离线工具。

## 上下游链路

上游：

- `humanoid_bringup` 根据地图和定位模式 include 本包 launch。
- `start_navigation_stack.sh` 生成地图相关 runtime 配置并传入本包 launch。
- `fast_lio_robosense`、`rslidar_sdk`、点云滤波和定位 runtime 提供传感器/定位数据。

下游：

- Nav2 `planner_server`、`controller_server`、`bt_navigator`、`behavior_server`。
- `humanoid_route_runtime` 通过 Nav2 action 执行路线任务。
- APP 网关通过状态话题和数据整合展示导航进度。

## 使用方式

通常由完整启动脚本启动：

```bash
./start_navigation.sh
```

单独启动 RO 链路：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 launch humanoid_navigation2 navigation2_robosense_lidar.launch.py
```

单独启动 OP 链路：

```bash
ros2 launch humanoid_navigation2 navigation2.launch.py
```

检查 launch 参数：

```bash
ros2 launch humanoid_navigation2 navigation2_robosense_lidar.launch.py --show-args
```

## 维护注意事项

- 本包是导航配置中心，修改参数前要确认影响范围，尤其是 costmap、planner、controller 和行为树。
- 自定义 BT 节点由 `humanoid_nav2_bt_nodes` 提供，不能删掉对应插件库。
- OP 和 RO 两条定位链路保持分开，通过 bringup 的 `relocalization_engine` 切换。
- 如果新增 runtime 包，要同步更新 launch、`package.xml` 和根目录启动脚本。
