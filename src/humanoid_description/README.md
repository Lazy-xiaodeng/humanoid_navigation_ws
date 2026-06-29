# humanoid_description

## 这个包是做什么的

`humanoid_description` 是机器人模型描述包，提供 URDF、SRDF、mesh、USD 和可视化 launch。

它不控制机器人运动，也不处理导航逻辑；它主要为 `robot_state_publisher`、RViz、Nav2 footprint/TF 调试和仿真工具提供机器人结构模型。

## 当前状态

- 当前导航启动链路会通过 `display.launch.py` 发布机器人模型和关节状态。
- URDF 模型用于生成机器人 TF 树中的固定结构部分。
- STL mesh 主要用于 RViz 可视化，不参与导航算法计算。
- USD 文件用于仿真或数字资产场景，不是当前导航运行的必要输入。

## 主要文件说明

- `launch/display.launch.py`：启动机器人模型显示相关节点。
- `urdf/HU_D04_01.urdf`：基础机器人 URDF。
- `urdf/HU_D04_01_with_gripper.urdf`：带夹爪版本 URDF。
- `urdf/HU_D04_01_with_hand.urdf`：带手部版本 URDF。
- `urdf/humanoid_robot.urdf.xacro`：可参数化的 xacro 模型入口。
- `urdf/HU_D04_01.srdf`：语义描述文件。
- `meshes/`：机器人各连杆 STL 几何模型。
- `usd/`：USD 格式模型和配置文件。
- `xml/HU_D04_01.xml`：机器人结构相关 XML 资产。

## 上下游链路

上游：

- bringup launch 或 RViz 调试命令。

下游：

- `robot_state_publisher`：读取 URDF 并发布机器人固定 TF。
- `joint_state_publisher`：发布静态或调试用关节状态。
- RViz：显示机器人模型。
- 导航调试工具：依赖 TF 树判断 `base_footprint`、雷达、机身等坐标关系。

## 使用方式

单独显示模型：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 launch humanoid_description display.launch.py
```

完整导航中通常由 `humanoid_bringup` 间接启动，不需要单独运行。

## 维护注意事项

- 修改 URDF 外参会影响 TF 链路，可能间接影响点云、定位和 Nav2 costmap。
- mesh 文件体积较大，仅影响显示，不建议为了导航性能随意删除。
- 如果修改 `base_footprint`、雷达、IMU 或 body 坐标关系，必须同步验证 Fast-LIO、点云滤波、定位桥和 costmap。
