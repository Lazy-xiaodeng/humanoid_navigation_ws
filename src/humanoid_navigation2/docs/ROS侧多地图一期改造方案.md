# ROS侧多地图一期改造方案

本文档基于当前 Todesk 工作区的新导航系统重构，适配已经完成的路线任务能力：

- `start_route_task` 整段路线任务；
- `route_waypoint_ids + waypoints_revision` 点位 ID 列表启动；
- `route_waypoints` 完整点位快照启动；
- 任意任务点跳步 `jump_to_waypoint`；
- task/transit 点位属性；
- task 点最终停靠对齐与播报等待；
- transit 辅助点无痕通过；
- 障碍物暂停、人工脱困后继续；
- 默认 RoboSense 定位链路，Open3D 作为可选链路。

旧版方案中基于 `start_multi_point_navigation`、APP 逐点下发、`navigation_state_manager_recoverable.py` 的内容已不再作为实施依据。

## 1. 当前系统现状

### 1.1 当前启动链路

当前一键启动入口为：

```bash
/home/ubuntu/software/Todesk/Files/humanoid_ws/start_navigation.sh
```

实际拉起：

```text
humanoid_bringup/launch/robot_real.launch.py
```

默认定位链路：

```text
relocalization_engine=ro
navigation2_robosense_lidar.launch.py
```

可选定位链路：

```text
relocalization_engine=op
navigation2.launch.py
```

因此多地图一期不能只改 Open3D 的 `prior_map_path`，必须优先适配当前默认 RoboSense 链路。

### 1.2 当前路线任务链路

APP 或调试台通过 WebSocket 下发：

```json
{
  "protocol_version": "2.0",
  "message_type": "command",
  "data_type": "navigation_control",
  "source": "app",
  "destination": "ros",
  "data": {
    "command_type": "start_route_task",
    "task_session_id": "tour_001",
    "route_id": "route_001",
    "route_waypoint_ids": ["1", "2", "3"],
    "waypoints_revision": "1781334566.203"
  }
}
```

WebSocket server 会把命令转发到 `/app/navigation_command`，由 `dynamic_waypoints_manager.py` 透传给 `navigation_state_manager.py`。

`navigation_state_manager.py` 会：

1. 校验 `task_session_id`、`route_id`；
2. 校验 `route_waypoint_ids` 和 `waypoints_revision`；
3. 从本地点位缓存补全完整 `route_waypoints`；
4. 归一化 `waypoint_role`、`need_broadcast`、`walk_direction` 等业务字段；
5. 构建任务点主序列和 transit 途经点段；
6. 使用 Nav2 `NavigateThroughPoses` / `NavigateToPose` 执行路线；
7. 到 task 点后对齐角度，并根据 `need_broadcast` 等待 APP 播报完成；
8. 支持 `pause_route_task`、`resume_route_task`、`stop_route_task`、`jump_to_waypoint`、`broadcast_finished`。

### 1.3 当前点位结构

当前点位存储文件仍是单文件：

```text
/home/ubuntu/software/Todesk/Files/humanoid_ws/data/dynamic_waypoints.json
```

当前点位已包含：

- `id`
- `name`
- `type`
- `position`
- `orientation`
- `frame_id`
- `properties.waypoint_role`
- `properties.walk_direction`
- `properties.need_broadcast`
- `properties.broadcast_id`
- `properties.stop_and_align`
- `properties.route_order`

多地图需要在此基础上新增 `map_id`，不能破坏上述路线任务字段。

考虑到当前 APP 逻辑是“切换地图后保存点位时，先发送清空点位命令，再重新 set_waypoint”，一期正式改造不建议继续使用单个全局 `dynamic_waypoints.json` 存放所有地图点位。否则 APP 在 `hall1` 上保存点位时，如果 ROS 仍按全局清空处理，就可能误删 `hall` 的点位。

因此一期推荐把点位按地图拆分为多个 JSON 文件：

```text
/home/ubuntu/software/Todesk/Files/humanoid_ws/data/waypoints/hall.json
/home/ubuntu/software/Todesk/Files/humanoid_ws/data/waypoints/hall1.json
/home/ubuntu/software/Todesk/Files/humanoid_ws/data/waypoints/lab.json
```

这样每张地图都可以继续使用 `1`、`2`、`3` 这类本地图内点位 ID，不要求 APP 或客户把点位 ID 改成全局唯一。

## 2. 一期目标

一期采用：

```text
多地图存储 + 单地图激活 + 路线任务开始前切图 + 切图后重新初始化定位
```

目标如下：

1. 支持注册多张地图。
2. 点位绑定 `map_id`。
3. ROS 按 `map_id` 将点位保存到 `data/waypoints/<map_id>.json`。
4. APP 设置点位时必须知道当前底图，并把点位保存到对应 `map_id`。
5. APP 可查询地图列表、当前地图、指定地图下的点位。
6. `start_route_task` 启动前，ROS 校验整条路线是否属于同一张地图。
7. 若目标地图不是当前激活地图，则在没有任务运行时先切图，再启动路线任务。
8. 切图后重置定位链路，注入目标地图预设初始位姿。
9. 定位稳定后再向 Nav2 发送导航 goal。
10. 切图失败、定位失败、路线跨地图等情况必须返回明确错误码。
11. 保持现有跳步、播报、辅助点、暂停恢复、障碍物暂停等功能不被破坏。

## 3. 一期明确不做

一期不做：

1. 导航执行过程中跨地图切换。
2. 一条路线任务内包含多张地图点位。
3. 跳步跨地图。
4. 多张地图同时在线。
5. 任意位置跨地图全局重定位。
6. 运行中无感热切 prior map。
7. 点位跨地图复制、合并、迁移。
8. 地图删除、权限管理、地图版本回滚。

## 4. 核心规则

### 4.1 单地图激活

任意时刻只有一个 `current_map_id`。

所有 Nav2 坐标系仍使用：

```text
map
map_ground
odom
base_footprint
```

`map` 的语义是“当前激活地图坐标系”。

### 4.2 路线任务地图一致性

一条 `start_route_task` 内所有 waypoint 必须属于同一个 `map_id`。

如果路线里存在多个 `map_id`：

```text
拒绝启动路线任务
error_code = route_waypoint_map_mismatch
```

### 4.3 跳步规则

`jump_to_waypoint` 只能在当前 active route 内跳转。

因为 active route 在启动时已经冻结为单地图路线，所以：

1. 正向跳、反向跳都可以。
2. 自动吸收当前 route 内的 transit 点。
3. 不允许跳到 active route 之外的点。
4. 不允许跳到其他地图点。
5. 不允许跳到 transit 点。

### 4.4 切图时机

允许切图：

- 当前没有 active route task；
- 当前没有 Nav2 goal 执行；
- 上一轮任务已经完成、终止或失败清理完成；
- 新 `start_route_task` 的目标地图与 `current_map_id` 不一致。

禁止切图：

- 正在导航；
- 暂停中；
- 障碍物暂停中；
- 人工脱困等待继续中；
- task 到达后等待播报中；
- final yaw align 中；
- 正在处理 jump。

禁止切图时返回：

```text
error_code = map_switch_rejected_route_task_active
```

### 4.5 切图后继续逻辑

切图只发生在 `start_route_task` 前。

切图成功后，仍执行原始 `start_route_task`：

- task/transit 语义不变；
- 播报协同不变；
- 跳步规则不变；
- 暂停/继续/终止不变；
- 障碍物暂停恢复不变。

### 4.6 点位存储隔离

多地图后，点位 ID 的唯一性范围从“全局唯一”调整为“同一地图内唯一”。

示例：

```text
hall  可以有 waypoint_id=1
hall1 也可以有 waypoint_id=1
```

ROS 内部查找点位时必须使用：

```text
map_id + waypoint_id
```

不能只用 `waypoint_id` 全局查找。

`clear_waypoints` 必须限定 `map_id`，默认只清空该地图点位文件；不允许不带 `map_id` 时执行全局清空。

## 5. 地图注册表

新增：

```text
src/humanoid_navigation2/config/map_registry.json
```

示例：

```json
{
  "default_map_id": "hall",
  "current_map_id": "hall",
  "maps": [
    {
      "map_id": "hall",
      "display_name": "展厅 hall",
      "enabled": true,
      "map_yaml": "/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_navigation2/maps/hall.yaml",
      "map_image": "/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_navigation2/maps/hall.pgm",
      "prior_map_pcd": "/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_navigation2/pcd/hall_open3d_grounded.pcd",
      "robosense_config_template": "/home/ubuntu/software/Todesk/Files/humanoid_ws/src/robosense_lidar_localization/config/robosense_lidar_localization.yaml",
      "robosense_runtime_config": "/home/ubuntu/software/Todesk/Files/humanoid_ws/data/runtime_maps/hall/robosense_lidar_localization.yaml",
      "initial_pose": {
        "x": 0.035185,
        "y": 0.087588,
        "z": 0.0,
        "yaw": -0.130823906
      },
      "localization_strategy": "preset_initial_pose"
    }
  ]
}
```

说明：

- `map_yaml` 用于 Nav2 `map_server`。
- `prior_map_pcd` 用于 RoboSense/Open3D prior localization。
- `robosense_runtime_config` 是根据目标地图动态生成的 RoboSense 配置，避免直接修改源配置文件。
- `initial_pose` 用于切图后定位初始化。

## 6. 点位模型改造

### 6.1 存储结构

一期推荐新增点位目录：

```text
data/waypoints/
```

每张地图一个文件：

```text
data/waypoints/<map_id>.json
```

示例：

```json
{
  "map_id": "hall1",
  "waypoints_revision": "1781334566.203",
  "waypoints": {
    "navigation_target": {
      "1": {
        "id": "1",
        "name": "hall1任务点1",
        "type": "navigation_target",
        "map_id": "hall1",
        "position": [1.0, 2.0, 0.0],
        "orientation": [0.0, 0.0, 0.0, 1.0],
        "frame_id": "map",
        "properties": {
          "waypoint_role": "task",
          "walk_direction": "forward",
          "need_broadcast": false,
          "broadcast_id": "",
          "stop_and_align": true,
          "route_order": 1
        }
      }
    }
  },
  "timestamp": 1781334566.203
}
```

说明：

1. `waypoints_revision` 按地图独立维护。
2. 修改 `hall1.json` 只刷新 `hall1` 的 revision，不影响 `hall.json`。
3. `start_route_task(map_id=hall1)` 只校验 `hall1` 的 revision。
4. `route_waypoint_ids=["1"]` 时，只在 `hall1.json` 中查找 `1`。

### 6.2 新增字段

每个点位新增顶层字段：

```json
{
  "id": "10",
  "name": "展厅任务点10",
  "type": "navigation_target",
  "map_id": "hall",
  "position": [1.0, 2.0, 0.0],
  "orientation": [0.0, 0.0, 0.0, 1.0],
  "frame_id": "map",
  "properties": {
    "waypoint_role": "task",
    "walk_direction": "forward",
    "need_broadcast": true,
    "broadcast_id": "broadcast_10",
    "stop_and_align": true,
    "route_order": 10
  }
}
```

### 6.3 兼容旧数据

旧版 `data/dynamic_waypoints.json` 兼容策略：

1. 启动时如果 `data/waypoints/` 不存在，但存在旧 `dynamic_waypoints.json`，则按 `default_map_id` 读取旧点位。
2. 读取旧点位时自动补 `map_id=default_map_id`。
3. 保存时写入 `data/waypoints/<default_map_id>.json`。
4. 建议新增迁移脚本，把旧文件显式迁移到新目录。
5. 迁移后旧文件只保留备份，不再作为主存储。

### 6.4 点位查询过滤

`get_waypoints` 支持 `map_id`：

```json
{
  "protocol_version": "2.0",
  "message_type": "command",
  "data_type": "waypoint_management",
  "source": "app",
  "destination": "ros",
  "data": {
    "command_type": "get_waypoints",
    "waypoint_type": "navigation_target",
    "map_id": "hall",
    "include_details": true
  }
}
```

如果未传 `map_id`：

- 第一版建议返回错误 `missing_map_id`，避免 APP 误以为拿到的是当前地图点位；
- 如需兼容旧 APP，可通过参数 `allow_legacy_global_waypoint_query=true` 临时返回全部点位；
- 新 APP 必须始终按地图过滤。

### 6.5 清空点位规则

APP 当前保存点位时会先清空再重新设置。多地图后该命令必须带 `map_id`：

```json
{
  "protocol_version": "2.0",
  "message_type": "command",
  "data_type": "waypoint_management",
  "source": "app",
  "destination": "ros",
  "data": {
    "command_type": "clear_waypoints",
    "map_id": "hall1",
    "waypoint_type": "navigation_target"
  }
}
```

规则：

1. 只清空 `data/waypoints/hall1.json` 中的对应点位类型。
2. 不影响 `hall.json`、`lab.json` 等其他地图点位。
3. 不带 `map_id` 默认拒绝。
4. 若确实需要清空所有地图，必须显式传 `clear_scope=all_maps`，并且建议只开放给运维工具，不开放给普通 APP 保存流程。

## 7. APP/ROS 协议

### 7.1 地图管理命令

新增 `data_type=map_management`。

#### 查询地图列表

```json
{
  "protocol_version": "2.0",
  "message_id": "map_list_001",
  "timestamp": 1781334566.203,
  "message_type": "command",
  "data_type": "map_management",
  "source": "app",
  "destination": "ros",
  "data": {
    "command_type": "get_map_list",
    "include_disabled": false
  }
}
```

返回：

```json
{
  "protocol_version": "2.0",
  "message_id": "map_response_001",
  "timestamp": 1781334566.303,
  "message_type": "response",
  "data_type": "map_response",
  "source": "map_context_manager",
  "destination": "app",
  "data": {
    "command_type": "get_map_list",
    "status": "success",
    "current_map_id": "hall",
    "default_map_id": "hall",
    "maps": [
      {
        "map_id": "hall",
        "display_name": "展厅 hall",
        "enabled": true
      }
    ]
  },
  "metadata": {
    "status": "success",
    "error_code": "",
    "error_message": "",
    "request_id": "map_list_001"
  }
}
```

#### 查询当前地图

```json
{
  "protocol_version": "2.0",
  "message_id": "current_map_001",
  "timestamp": 1781334566.203,
  "message_type": "command",
  "data_type": "map_management",
  "source": "app",
  "destination": "ros",
  "data": {
    "command_type": "get_current_map"
  }
}
```

#### 运维手动切图

一期可支持运维/调试使用，不建议 APP 在导航中随意调用：

```json
{
  "protocol_version": "2.0",
  "message_id": "switch_map_001",
  "timestamp": 1781334566.203,
  "message_type": "command",
  "data_type": "map_management",
  "source": "app",
  "destination": "ros",
  "data": {
    "command_type": "switch_map",
    "target_map_id": "hall1",
    "reason": "manual_debug_switch"
  }
}
```

### 7.2 start_route_task 增加地图字段

ID 列表模式：

```json
{
  "protocol_version": "2.0",
  "message_id": "start_route_001",
  "timestamp": 1781334566.203,
  "message_type": "command",
  "data_type": "navigation_control",
  "source": "app",
  "destination": "ros",
  "data": {
    "command_type": "start_route_task",
    "task_session_id": "tour_001",
    "route_id": "route_hall_001",
    "map_id": "hall",
    "route_waypoint_ids": ["1", "2", "3", "4"],
    "waypoints_revision": "1781334566.203"
  }
}
```

完整点位快照模式：

```json
{
  "protocol_version": "2.0",
  "message_id": "start_route_002",
  "timestamp": 1781334566.203,
  "message_type": "command",
  "data_type": "navigation_control",
  "source": "app",
  "destination": "ros",
  "data": {
    "command_type": "start_route_task",
    "task_session_id": "tour_002",
    "route_id": "route_hall_002",
    "map_id": "hall",
    "route_waypoints": [
      {
        "waypoint_id": "1",
        "waypoint_name": "任务点1",
        "map_id": "hall",
        "waypoint_role": "task",
        "frame_id": "map",
        "position": [0.0, 0.0, 0.0],
        "orientation": [0.0, 0.0, 0.0, 1.0],
        "need_broadcast": false,
        "broadcast_id": "",
        "broadcast_blocking": true,
        "stop_and_align": true,
        "walk_direction": "forward",
        "properties": {
          "waypoint_role": "task",
          "walk_direction": "forward"
        }
      }
    ]
  }
}
```

校验规则：

1. `data.map_id` 可选但建议必填。
2. 如果 `data.map_id` 缺失，ROS 从 route waypoint 中推导。
3. 如果 `data.map_id` 与点位 `map_id` 不一致，拒绝启动。
4. 如果 route 内存在多个 `map_id`，拒绝启动。
5. ID 列表模式下，ROS 按 `map_id + waypoint_id` 从 `data/waypoints/<map_id>.json` 补全点位。
6. `waypoints_revision` 校验目标地图自己的 revision，不再使用全局 revision。

## 8. ROS 节点职责

### 8.1 新增 map_context_manager

建议新增：

```text
src/humanoid_navigation2/humanoid_navigation2/map_context_manager.py
```

职责：

1. 加载 `map_registry.json`。
2. 维护 `current_map_id`。
3. 提供地图列表和当前地图查询。
4. 校验地图文件完整性。
5. 执行切图流程。
6. 发布地图状态。
7. 发布定位初始化状态。
8. 为 RoboSense 生成目标地图 runtime config。
9. 通过服务或 topic 向 `navigation_state_manager` 返回切图结果。

### 8.2 dynamic_waypoints_manager

需要修改：

1. `WaypointData` 增加 `map_id`。
2. 主存储从单个 `dynamic_waypoints.json` 改为 `data/waypoints/<map_id>.json`。
3. `set_waypoint/update_waypoint` 接收并保存 `map_id`。
4. `delete_waypoint/clear_waypoints` 只操作指定 `map_id` 的点位文件。
5. `clear_waypoints` 不带 `map_id` 默认拒绝，防止误删所有地图点位。
6. 旧点位加载时补默认 `map_id`，并迁移到默认地图文件。
7. `get_waypoints` 支持按 `map_id` 过滤。
8. `publish_waypoints_data` 推送中包含 `map_id`。
9. `waypoints_revision` 按地图独立维护。
10. 内存缓存建议改为 `waypoints_by_map[map_id][waypoint_type][waypoint_id]`。

不建议把切图逻辑放进该节点。

### 8.3 navigation_state_manager

需要修改：

1. `build_route_waypoints_from_ids()` 按 `map_id + waypoint_id` 补全点位。
2. `normalize_route_task_waypoints()` 归一化 `map_id`。
3. `handle_start_route_task()` 在创建 active route 前做地图一致性校验。
4. 如果目标地图不是 `current_map_id`，请求 `map_context_manager` 切图。
5. 切图成功后再继续原有 route task 启动逻辑。
6. 切图失败则发送 `navigation_command_result` error ack。
7. active route task 内保存 `map_id`。
8. `jump_to_waypoint` 校验目标点 `map_id` 必须等于 active route 的 `map_id`。
9. `pause/resume/stop/broadcast_finished` 继续沿用现有逻辑，不受地图字段影响。

### 8.4 websocket_server

需要修改：

1. 支持 `data_type=map_management`。
2. 转发地图管理命令到 `/app/map_command` 或 `/map/command`。
3. `navigation_control` 透传 `map_id`。
4. `waypoint_management` 透传 `map_id`、`clear_scope`。

### 8.5 data_integration_node_recoverable

需要修改：

1. 订阅地图状态 topic。
2. 将 `map_status` 推送给 APP。
3. 将 `localization_status` 推送给 APP。
4. 必要事件走立即推送白名单，例如：
   - `map_switch_started`
   - `map_switch_completed`
   - `map_switch_failed`
   - `localization_initializing`
   - `localization_ready`
   - `localization_failed`

## 9. 切图实现策略

### 9.1 推荐一期策略

一期推荐“受控重建地图/定位相关节点”，不追求运行时无感 reload。

原因：

1. 当前 RoboSense 定位配置通过 YAML 固化 `map_path/init_position/init_euler`。
2. `prior_map_odom_bridge` 当前没有 reset service。
3. Open3D/RoboSense 内部地图缓存不适合靠参数热改保证可靠。
4. 一期需求本身允许“任务前切图 + 新图初始化定位”。

### 9.2 切图步骤

`map_context_manager` 切图流程：

1. 发布 `map_switch_started`。
2. 校验当前没有 active route task。
3. 校验目标地图存在且 enabled。
4. 校验目标地图关键文件存在：
   - `map_yaml`
   - `map_image`
   - `prior_map_pcd`
5. 生成目标地图 RoboSense runtime config：
   - 替换 `map_path`
   - 替换 `init_position`
   - 替换 `init_euler`
6. 停止地图/定位相关节点或请求外层 launch 重建。
7. 重启/重载：
   - `map_server`
   - `lifecycle_manager_map`
   - `robosense_lidar_localization_node` 或 `prior_map_open3d_localization`
   - `prior_map_odom_bridge`
   - `rviz_initialpose_adapter`
   - `wait_for_localization_tf`
8. 等待 `/map` 重新发布。
9. 等待 `/localization/prior_map_odom_bridge_status` 出现 `ACCEPTED` 或连续 ready 状态。
10. 等待 `map_ground -> base_footprint` TF 连续可用。
11. 更新 `current_map_id`。
12. 发布 `map_switch_completed`。

### 9.3 定位 ready 判据

一期建议满足：

1. `map->odom` 已发布。
2. `map_ground->base_footprint` TF 可查询。
3. bridge status 最近 N 次不是 `WAITING no_accepted_map_to_odom`。
4. 额外 settle 2 到 3 秒。

失败返回：

```text
error_code = localization_not_ready_after_map_switch
```

## 10. 启动脚本与 launch 改造

### 10.1 start_navigation.sh

建议支持：

```bash
./start_navigation.sh --map hall
./start_navigation.sh --map hall1
./start_navigation.sh --relocalization-engine ro --map hall1
```

也可以先支持环境变量：

```bash
ACTIVE_MAP_ID=hall1 ./start_navigation.sh
```

### 10.2 robot_real.launch.py

需要把 `map_id` 参数传给：

- `navigation2_robosense_lidar.launch.py`
- `navigation2.launch.py`
- `navigation_fusion_sc.launch.py`

### 10.3 navigation2_robosense_lidar.launch.py

新增参数：

- `map_yaml_file`
- `robosense_config_file`
- `active_map_id`

当前硬编码：

```text
maps/hall.yaml
robosense_lidar_localization.yaml
```

需要改成 launch 参数。

### 10.4 navigation2.launch.py

Open3D 链路新增参数：

- `map_yaml_file`
- `prior_map_path`
- `active_map_id`
- `initialpose`

当前硬编码：

```text
maps/hall.yaml
pcd/hall_open3d_grounded.pcd
```

需要改成 launch 参数。

## 11. 状态推送

### 11.1 地图状态

```json
{
  "protocol_version": "2.0",
  "message_id": "map_status_001",
  "timestamp": 1781334566.203,
  "message_type": "push",
  "data_type": "map_status",
  "source": "map_context_manager",
  "destination": "app",
  "data": {
    "current_map_id": "hall",
    "target_map_id": "hall1",
    "switch_status": "switching_map",
    "event_type": "map_switch_started",
    "message": "正在切换地图 hall1"
  }
}
```

### 11.2 定位状态

```json
{
  "protocol_version": "2.0",
  "message_id": "localization_status_001",
  "timestamp": 1781334566.203,
  "message_type": "push",
  "data_type": "localization_status",
  "source": "map_context_manager",
  "destination": "app",
  "data": {
    "map_id": "hall1",
    "status": "waiting_localization",
    "event_type": "localization_initializing",
    "message": "已切换 hall1 地图，正在等待定位稳定"
  }
}
```

### 11.3 导航命令结果

如果切图失败，`start_route_task` 返回：

```json
{
  "protocol_version": "2.0",
  "message_id": "navigation_result_001",
  "timestamp": 1781334566.203,
  "message_type": "push",
  "data_type": "navigation_status",
  "source": "navigation_state_manager",
  "destination": "app",
  "data": {
    "event_type": "navigation_command_result",
    "command_type": "start_route_task",
    "status": "error",
    "task_session_id": "tour_001",
    "route_id": "route_hall1_001",
    "map_id": "hall1",
    "error_code": "localization_not_ready_after_map_switch",
    "message": "切换到 hall1 后定位未在超时时间内稳定，路线任务未启动"
  }
}
```

## 12. 错误码

建议新增：

| error_code | 含义 |
|---|---|
| `map_not_found` | 地图注册表中不存在目标地图 |
| `map_disabled` | 地图被禁用 |
| `map_file_missing` | 地图关键文件缺失 |
| `map_registry_invalid` | 地图注册表格式错误 |
| `map_switch_rejected_route_task_active` | 当前路线任务未结束，拒绝切图 |
| `map_switch_timeout` | 切图超时 |
| `map_switch_partial_failure` | 部分节点切换失败 |
| `localization_not_ready_after_map_switch` | 切图后定位未就绪 |
| `initial_pose_missing` | 目标地图缺少初始位姿 |
| `initial_pose_injection_failed` | 初始位姿注入失败 |
| `waypoint_map_mismatch` | 命令 map_id 与点位 map_id 不一致 |
| `route_waypoint_map_mismatch` | 一条路线包含多个 map_id |
| `active_route_map_mismatch` | 当前 active route 与命令地图不一致 |
| `jump_target_map_mismatch` | jump 目标点地图不一致 |

## 13. 与现有新功能的关系

### 13.1 跳步

多地图不会改变跳步语义。

只要 active route 是单地图路线：

- A 到 D 可以跳；
- H 到 C 可以反向跳；
- transit 按当前路线顺序吸收；
- 跳步结束后继续目标点后续任务点；
- 不能跳到非当前路线点；
- 不能跳到其他地图点。

### 13.2 辅助点

transit 点也必须有 `map_id`。

同一段 through 里的 transit 和最终 task 必须同地图。

### 13.3 播报

地图切换只发生在路线开始前，不影响：

- `broadcast_requested`
- APP 播报
- `broadcast_finished`
- 播报等待中跳步

### 13.4 障碍物暂停/人工脱困继续

暂停恢复不受 `map_id` 限制，但当前 active route 的 `map_id` 不能变化。

暂停中不允许切图。

### 13.5 调试台

当前调试台需要后续增加：

1. 地图列表选择；
2. 地图状态显示；
3. 按 `map_id` 过滤点位；
4. `start_route_task` 带 `map_id`；
5. bag 默认增加地图状态 topic。

## 14. 测试方案

### 14.1 单地图回归

1. 旧点位没有 `map_id`，启动后自动补 `hall`。
2. 旧 `dynamic_waypoints.json` 可迁移为 `data/waypoints/hall.json`。
3. APP/调试台只发 `route_waypoint_ids`，路线正常执行。
3. 辅助点无痕通过正常。
4. task 点对齐和播报正常。
5. 正向/反向跳步正常。
6. 暂停、继续、终止正常。
7. 障碍物暂停和人工继续正常。

### 14.2 多地图数据测试

1. `get_map_list` 返回 `hall/hall1`。
2. `get_current_map` 返回当前地图。
3. 设置 `hall` 点位只写入 `data/waypoints/hall.json`。
4. 设置 `hall1` 点位只写入 `data/waypoints/hall1.json`。
5. `clear_waypoints(map_id=hall1)` 只清空 `hall1.json`，不影响 `hall.json`。
6. 不带 `map_id` 的 `clear_waypoints` 被拒绝。
7. 查询点位时可按 `map_id` 过滤。
8. `waypoints_revision` 按地图独立变化。
9. `hall` 和 `hall1` 都可以存在 waypoint_id=`1`，互不覆盖。

### 14.3 路线任务地图测试

1. 全部点位 `map_id=hall`，当前地图 `hall`，直接启动。
2. 全部点位 `map_id=hall1`，当前地图 `hall`，先切图再启动。
3. 路线中混入 `hall` 和 `hall1`，拒绝启动。
4. 命令 `map_id=hall`，点位 `map_id=hall1`，拒绝启动。
5. active route 为 `hall` 时，jump 到 route 内 task 成功。
6. active route 为 `hall` 时，jump 到非 route 点或其他地图点失败。

### 14.4 切图异常测试

1. 目标地图不存在。
2. 目标地图 disabled。
3. `map_yaml` 缺失。
4. `prior_map_pcd` 缺失。
5. RoboSense runtime config 生成失败。
6. 定位超时。
7. 切图中收到第二条 `start_route_task`。
8. 暂停中请求切图。
9. 播报等待中请求切图。

## 15. 推荐实施顺序

### 阶段 1：数据和协议基础

1. 新增 `map_registry.json`。
2. 新增 `map_context_manager` 基础查询能力。
3. `websocket_server.py` 支持 `map_management`。
4. 新增 `data/waypoints/` 多地图点位目录。
5. `dynamic_waypoints_manager.py` 增加 `map_id`，并改为按地图文件读写。
6. `set/get/delete/clear_waypoints` 支持 `map_id`。
7. `clear_waypoints` 不带 `map_id` 默认拒绝。
8. `waypoints_revision` 按地图独立维护。
9. 新增旧 `dynamic_waypoints.json` 迁移脚本。
10. `data_integration_node_recoverable.py` 推送 `map_status/localization_status`。

这一阶段不做自动切图，只做数据闭环。

### 阶段 2：路线任务地图校验

1. `navigation_state_manager.py` 补全 route waypoint 的 `map_id`。
2. `start_route_task` 校验整条路线单地图。
3. active route 保存 `map_id`。
4. `jump_to_waypoint` 增加地图一致性保护。
5. 单地图回归和混地图拒绝测试。

这一阶段仍不做自动切图。

### 阶段 3：启动时指定地图

1. `start_navigation.sh` 支持 `--map` 或 `ACTIVE_MAP_ID`。
2. `robot_real.launch.py` 透传 `active_map_id`。
3. `navigation2_robosense_lidar.launch.py` 支持外部传入 `map_yaml_file/robosense_config_file`。
4. `navigation2.launch.py` 支持外部传入 `map_yaml_file/prior_map_path`。
5. 验证系统启动时直接加载指定地图。

这一阶段可以先满足“手动选择地图后启动系统”。

### 阶段 4：路线开始前自动切图

1. `navigation_state_manager.py` 在 `start_route_task` 前调用 `map_context_manager`。
2. `map_context_manager` 生成目标地图 runtime config。
3. 重建地图/定位相关节点。
4. 等待定位 ready。
5. 切图成功后启动原路线任务。
6. 切图失败返回 `navigation_command_result` error。

这是一期核心难点。

### 阶段 5：建图脚本入库

1. `start_mapping.sh` 支持地图名。
2. 默认拒绝重名覆盖。
3. 校验地图产物。
4. 生成 `<map_id>_open3d_grounded.pcd`。
5. 自动写入 `map_registry.json`。
6. 记录初始位姿。

## 16. 工期评估

按一名熟悉当前系统的人估算：

| 阶段 | 内容 | 预计 |
|---|---|---|
| 阶段 1 | 地图注册表、点位 `map_id`、地图查询协议 | 1.5 到 2 天 |
| 阶段 2 | 路线任务地图校验、jump 保护 | 1 到 1.5 天 |
| 阶段 3 | 启动时指定地图、launch 参数化 | 1 到 1.5 天 |
| 阶段 4 | 任务前自动切图、定位重置和 ready 等待 | 2 到 3 天 |
| 阶段 5 | 建图脚本自动注册 | 1 到 1.5 天 |
| 联调测试 | 实机/袋包/异常测试 | 2 到 3 天 |

总计建议排期：

```text
8 到 12 个工作日
```

如果第一版先不做自动切图，只做“启动时指定地图 + 路线任务地图校验”，可压缩到：

```text
3 到 5 个工作日
```

## 17. 风险评估

### 高风险

1. RoboSense 定位节点运行时切换地图后内部状态是否能稳定重置。
2. `prior_map_odom_bridge` 当前没有 reset service，切图后可能残留旧 map->odom。
3. 切图期间节点重启和 Nav2 lifecycle 状态协同复杂。
4. 新地图初始位姿不准会导致定位迟迟不 ready。

### 中风险

1. 点位旧数据迁移到多 JSON 文件。
2. APP 地图选择、点位过滤和按地图清空联调。
3. 调试台也要同步支持多地图，否则测试不方便。
4. `waypoints_revision` 按地图独立维护的一致性。
5. APP 保存点位时清空命令必须带 `map_id`，否则存在误删风险。

### 低风险

1. 地图列表查询。
2. 点位新增 `map_id`。
3. 路线任务启动前单地图校验。

## 18. 结论

多地图一期可以做，并且和当前跳步/路线任务功能兼容。

但实施方案必须基于当前新导航系统重构：

1. 导航入口以 `start_route_task` 为准。
2. 点位模型保留现有 task/transit/broadcast/walk_direction 字段，并新增 `map_id`。
3. 点位主存储改为 `data/waypoints/<map_id>.json`，每张地图独立 revision。
4. 一条路线任务必须单地图。
5. 跳步只在当前 active route 内执行，不跨地图。
6. 默认优先适配 RoboSense 定位链路。
7. 一期切图建议采用“任务前可控重建地图/定位相关节点”，不要追求无感热切。

推荐先落地：

```text
地图注册表 + 多 JSON 点位文件 + 点位 map_id + 路线任务地图校验 + 启动时指定地图
```

再推进：

```text
start_route_task 前自动切图 + 定位重置 + 定位 ready 后启动路线
```
