# humanoid_control_runtime

该包是当前 Test 工作区单地图导航链路的 C++ 控制层运行包。

## 当前包含的节点

- `dynamic_waypoints_manager_cpp`：负责 APP 点位命令、单文件点位持久化、点位全集发布、导航命令转发。

## 当前不包含的功能

- 不包含多地图 `map_context_manager`。
- 不读取 `data/maps/map_registry.json`。
- 不改变当前默认 Python 启动链路，验证完成后再切换。

## 主要话题

- 订阅 `/app/waypoint_command`
- 订阅 `/app/navigation_command`
- 订阅 `/navigation/acknowledgments`
- 发布 `/navigation/requests`
- 发布 `/navigation/waypoints_data`
