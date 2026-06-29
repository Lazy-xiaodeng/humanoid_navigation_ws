# humanoid_bringup

## 这个包是做什么的

`humanoid_bringup` 是整机启动编排包，负责把控制层、导航层、建图链路、机器人模型和各个运行时功能包串起来。

它本身不实现业务算法，也不直接处理点云、定位、路线任务或 WebSocket 协议；它的核心职责是通过 launch 文件统一启动这些功能包，并把地图、定位模式、配置文件、时间源等参数传递给下游节点。

## 当前状态

- 默认完整导航入口由工作区根目录的 `start_navigation.sh` 调用。
- 控制层入口主要启动 APP 网关、机器人本体网关、表情驱动、播报服务、点位管理和路线管理等常驻节点。
- 导航层入口根据 `relocalization_engine` 选择 RO/RoboSense 定位链路或 OP/Open3D 先验地图定位链路。
- 建图入口用于启动雷达、Fast-LIO、slam_toolbox、PCD 保存和 bag 记录流程。
- 当前包已经切到 C++ runtime 链路，默认不再启动旧 Python ws/导航状态管理器链路。

## 主要文件说明

- `launch/robot_real.launch.py`：整机总入口，面向真实机器人启动控制层和导航层。
- `launch/robot_control_plane.launch.py`：控制层常驻入口，启动 APP 网关、本体网关、表情、播报、点位和路线管理等。
- `launch/robot_navigation_stack.launch.py`：导航层入口，按地图和定位模式启动定位、Fast-LIO、Nav2、点云滤波和路线执行层。
- `launch/robot_mapping.launch.py`：建图入口，启动雷达、Fast-LIO、slam_toolbox 和建图辅助节点。
- `setup.py`：安装 launch 文件，供 `ros2 launch humanoid_bringup ...` 查找。

## 上下游链路

上游：

- `start_navigation.sh`、`start_navigation_stack.sh`、`start_mapping.sh` 等工作区脚本。
- APP 或运维人员传入的地图 ID、定位模式、配置路径和启动参数。

下游：

- `humanoid_app_gateway_runtime`、`humanoid_robot_gateway_runtime`、`humanoid_expression_runtime`、`humanoid_broadcast_runtime`。
- `humanoid_control_runtime`、`humanoid_route_runtime`。
- `fast_lio_robosense`、`humanoid_robosense_localization_runtime`、`humanoid_prior_localization_runtime`。
- `humanoid_navigation2`、`humanoid_point_cloud_filter`、`humanoid_obstacle_runtime`、`humanoid_costmap_runtime`。

## 使用方式

通常从工作区根目录使用一键脚本：

```bash
./start_navigation.sh
```

如果需要单独调试 bringup：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 launch humanoid_bringup robot_real.launch.py
```

只启动导航层：

```bash
ros2 launch humanoid_bringup robot_navigation_stack.launch.py relocalization_engine:=ro
```

切到 OP/Open3D 定位链路：

```bash
ros2 launch humanoid_bringup robot_navigation_stack.launch.py relocalization_engine:=op
```

## 维护注意事项

- 这个包是启动编排层，改动 launch 时要同步检查根目录启动脚本和下游 package 名。
- OP 和 RO 定位链路保持分开，不建议把两套定位前端合并到同一个功能包。
- 如果新增 C++ runtime 包，需要在对应 launch 和 `package.xml` 中显式声明依赖。
- 不建议在 bringup 里写业务逻辑；业务逻辑应放到对应 runtime 包中。
