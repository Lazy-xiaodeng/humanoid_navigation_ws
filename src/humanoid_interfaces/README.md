# humanoid_interfaces

## 这个包是做什么的

`humanoid_interfaces` 是本工程的自定义消息和服务接口包。

它不启动节点，也不包含业务逻辑；它负责定义多个功能包之间共享的 ROS 消息和服务类型，供 C++/Python 节点在编译和运行时引用。

## 当前状态

- 当前主要服务于播报、地形/步态相关接口。
- 作为接口包，它会被多个 runtime 包依赖。
- 修改 `.msg` 或 `.srv` 会触发依赖包重新编译。

## 接口说明

- `msg/GaitStatus.msg`：步态状态消息，用于描述机器人当前运动/步态状态。
- `msg/TerrainInfo.msg`：地形信息消息，用于传递地形分析或地面状态。
- `srv/PlayBroadcast.srv`：播报播放服务，供业务节点请求播放指定播报内容。
- `srv/SetBroadcastVolume.srv`：播报音量设置服务。
- `srv/GetBroadcastHealth.srv`：播报服务健康检查接口。

## 上下游链路

上游：

- 需要发布自定义消息或调用自定义服务的业务包。

下游：

- `humanoid_broadcast_runtime` 使用播报相关服务。
- 其他导航、地形、状态类包可复用消息定义。

## 使用方式

普通业务包在 `package.xml` 和 `CMakeLists.txt` 中声明依赖后即可使用：

```xml
<depend>humanoid_interfaces</depend>
```

C++ 中包含生成后的接口头文件：

```cpp
#include "humanoid_interfaces/srv/play_broadcast.hpp"
```

## 维护注意事项

- 接口字段一旦被下游使用，修改字段名或类型会造成 ABI/API 级破坏。
- 新增字段前要确认 APP、ws 网关、播报服务或其他调用方是否需要同步更新。
- 不建议在接口包中加入节点实现；接口包应保持轻量和稳定。
