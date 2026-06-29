# humanoid_decay_obstacle_layer

## 这个包是做什么的

`humanoid_decay_obstacle_layer` 是 Nav2 costmap 插件包，用于把点云中的动态障碍写入 costmap，并在一定时间后自动衰减清除。

它不是独立 ROS 节点，运行时不会在进程列表里看到单独的 `humanoid_decay_obstacle_layer` 进程。它会作为动态库被 Nav2 costmap 通过 pluginlib 加载。

## 当前状态

- 插件类名：`humanoid_decay_obstacle_layer::DecayObstacleLayer`。
- 插件描述文件：`decay_obstacle_layer.xml`。
- 当前 `nav2_params.yaml` 中保留了该插件配置，但默认 global costmap 的插件列表没有启用它。
- 这个包属于可选动态障碍层，目前不是默认资源消耗大头。

## 主要功能

- 订阅点云话题，例如 `/airy_points_filtered`。
- 根据距离、高度、命中次数等条件筛选障碍点。
- 将障碍点投影到 costmap cell 中。
- 使用 `decay_time` 控制障碍残留时间，超过时间后自动从 costmap 中清除。
- 适合处理临时出现的全局障碍，避免障碍永久残留。

## 文件说明

- `src/decay_obstacle_layer.cpp`：插件主实现，包含参数读取、点云订阅、障碍缓存、costmap 更新和衰减清理逻辑。
- `include/humanoid_decay_obstacle_layer/decay_obstacle_layer.hpp`：插件类定义和内部数据结构。
- `decay_obstacle_layer.xml`：pluginlib 插件注册文件。
- `package.xml`：声明 Nav2 costmap、pluginlib、TF 和点云消息依赖。

## 上下游链路

上游：

- 点云滤波节点，通常发布 `/airy_points_filtered`。
- TF 系统，提供点云坐标系到 costmap 全局坐标系的变换。

下游：

- Nav2 global costmap 或 local costmap。
- planner server 使用 costmap 结果进行路径规划。

## 使用方式

在 Nav2 参数中加入插件名：

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      plugins: ["static_layer", "decay_obstacle_layer", "inflation_layer"]
      decay_obstacle_layer:
        plugin: "humanoid_decay_obstacle_layer::DecayObstacleLayer"
        enabled: true
        topic: /airy_points_filtered
        decay_time: 3.0
```

## 维护注意事项

- 这是 costmap 插件，不要按普通节点方式启动。
- 启用前要评估动态障碍是否会影响全局规划稳定性。
- `decay_time` 太短会导致障碍过快消失，太长会导致残影影响规划。
- 如果当前静态地图和局部避障已经够用，可以继续保持关闭。
