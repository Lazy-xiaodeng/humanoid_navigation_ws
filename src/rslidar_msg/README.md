# rslidar_msg

## 这个包是做什么的

`rslidar_msg` 是 RoboSense 雷达驱动使用的自定义消息接口包。

它不启动节点，也不处理雷达数据；它只定义 `rslidar_sdk` 在 ROS2 环境中需要的雷达 packet 消息类型。

## 当前状态

- 当前作为 `rslidar_sdk` 的依赖包参与构建。
- 这是 RoboSense 官方驱动体系中的消息包，通常不需要修改。
- 修改消息字段会影响 `rslidar_sdk` 编译和运行。

## 上下游链路

上游：

- `rslidar_sdk` 构建系统和雷达驱动代码。

下游：

- RoboSense 驱动节点内部使用 packet 消息。
- 工作区构建时需要先生成本包消息接口。

## 使用方式

正常情况下不需要单独启动或调用。

如果单独构建：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rslidar_msg
```

## 维护注意事项

- 这是接口包，不要放业务节点。
- 不建议改消息字段，除非同步确认 `rslidar_sdk` 版本要求。
- 如果升级 RoboSense SDK，需要一起检查本包是否与新 SDK 匹配。
