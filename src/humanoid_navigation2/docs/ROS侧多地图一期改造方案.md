# 多地图一期改造方案

## 1. 背景

当前系统默认只有一张地图在线使用，导航侧主要依赖固定命名的地图文件：

- 2D 栅格地图：`maps/hall.yaml`、`maps/hall.pgm`
- 3D prior map：`pcd/hall_open3d_grounded.pcd`
- 其他建图衍生产物：`hall.posegraph`、`hall.data`、`hall_sc.bin` 等

客户的新需求是：

1. 系统可以存多张地图。
2. app 在设置点位时先选择底图，再在该底图上设置点位。
3. 点位需要和底图绑定。
4. 真正开始导航前，如果当前激活地图不是目标地图，则先切图，再导航。
5. 不要求导航过程中动态切图。

本方案的一期前提已经明确：

1. 当需要从 `hall` 切到 `hall1` 时，机器人会被人工放到 `hall1` 建图原点附近。
2. 切图后的定位，不需要做复杂的跨地图全局重定位。
3. 切图后的定位策略按“新图开机初始化”处理：重置定位链路，给新图注入预设初始位姿，等待定位稳定后再导航。

这意味着一期不需要解决“导航中热切图”和“任意位置跨地图自动找回”的复杂问题，一期只做“任务前切图 + 新图初始化定位”。

## 2. 一期目标

一期目标如下：

1. 支持多地图文件管理和注册。
2. 每个点位绑定所属地图 `map_id`。
3. app 可以获取地图列表、选择地图、查询该地图下的点位。
4. app 发起导航时，ROS 在导航开始前自动校验并切换目标地图。
5. 切图后自动重置定位链路，并注入目标地图预设初始位姿。
6. 定位稳定后再开始导航；若切图或定位失败，则明确返回失败原因。
7. 建图脚本支持按地图名生成完整地图产物，并自动注册新地图。

## 3. 一期明确不做

一期不做以下内容：

1. 导航过程中切图。
2. 多张地图同时在线运行。
3. 自动跨图续航。
4. 任意位置切图后的全局重定位。
5. 点位跨地图复制、合并、迁移。
6. 地图版本管理、地图删除回收站、地图权限管理。
7. 无感热切 prior localization 的高级优化。

## 4. 现状评估

### 4.1 建图侧

现有建图脚本 [start_mapping.sh](/home/ubuntu/humanoid_ws/start_mapping.sh:4) 已支持通过第一个参数指定地图名：

```bash
MAP_NAME="${1:-hall}"
```

并且输出文件本身已经按 `MAP_NAME` 生成，例如：

- `maps/${MAP_NAME}.yaml`
- `maps/${MAP_NAME}.pgm`
- `maps/${MAP_NAME}.posegraph`
- `maps/${MAP_NAME}.data`
- `pcd/${MAP_NAME}.pcd`
- `pcd/${MAP_NAME}_localization_grounded.pcd`

这意味着建图脚本已经具备多地图命名基础，但还缺：

1. 产物命名规范与导航侧完全对齐。
2. 建图完成后自动注册地图。
3. 重名保护和覆盖策略。
4. 关键产物完整性校验。
5. 每张图的初始位姿信息登记。

### 4.2 点位侧

点位由 [dynamic_waypoints_manager.py](/home/ubuntu/humanoid_ws/src/humanoid_navigation/humanoid_navigation/dynamic_waypoints_manager.py:352) 管理，当前点位模型适合增加字段。点位数据会通过 [dynamic_waypoints_manager.py](/home/ubuntu/humanoid_ws/src/humanoid_navigation/humanoid_navigation/dynamic_waypoints_manager.py:716) 持久化保存。

当前点位没有地图归属字段，因此需要新增 `map_id`。

### 4.3 websocket 和 app 协议侧

[websocket_server.py](/home/ubuntu/humanoid_ws/src/humanoid_websocket/humanoid_websocket/websocket_server.py:680) 目前负责把 app 的点位和导航命令转发给 ROS 节点。它已经有稳定的 JSON 转发链路，因此适合扩展：

1. 地图列表查询接口。
2. 地图状态广播。
3. 点位命令里的 `map_id`。
4. 导航命令里的 `map_id`。
5. 定位初始化/定位就绪结果回传。

### 4.4 导航与定位侧

[navigation2.launch.py](/home/ubuntu/humanoid_ws/src/humanoid_navigation2/launch/navigation2.launch.py:61) 当前把 2D 地图固定为 `hall.yaml`；[navigation2.launch.py](/home/ubuntu/humanoid_ws/src/humanoid_navigation2/launch/navigation2.launch.py:43) 当前把 prior map 默认指向 `hall_open3d_grounded.pcd`。

因此一期的关键不在 Nav2 本身，而在于：

1. 把“当前地图”从硬编码常量提升为运行时上下文。
2. 在导航前引入地图一致性检查和切图流程。
3. 在切图后显式重置定位上下文，并按目标地图的预设初始位姿重新初始化定位。

## 5. 总体方案

一期采用“单地图激活、多地图存储、导航前切图、切图后初始化定位”方案。

### 5.1 核心原则

1. 系统中可以注册多张地图。
2. 任一时刻只允许一张地图处于激活状态。
3. 每个点位只能属于一张地图。
4. 导航命令只能在目标地图已经激活且定位已就绪后执行。
5. 地图切换只允许发生在导航开始前，或上一轮导航结束后下一轮导航开始前。
6. 切到新图时，默认机器人已经在该图原点附近，因此系统使用该图预设初始位姿重新初始化定位。

### 5.2 运行流程

1. app 获取地图列表。
2. app 选择底图 `hall1`。
3. app 在 `hall1` 上设置点位。
4. 点位保存时写入 `map_id=hall1`。
5. 用户把机器人放到 `hall1` 建图原点附近。
6. app 发起导航到某个点位。
7. ROS 检查目标点位 `map_id` 是否等于当前激活地图。
8. 若一致，直接导航。
9. 若不一致，先切图到 `hall1`。
10. ROS 重置定位链路。
11. ROS 自动给 `hall1` 注入预设初始位姿。
12. ROS 等待定位稳定。
13. 定位成功后开始导航。
14. 若切图失败或定位失败，则导航直接失败，不发 goal。

## 6. 地图数据模型

### 6.1 新增地图注册表

建议新增配置文件：

- `/home/ubuntu/humanoid_ws/src/humanoid_navigation2/config/map_registry.json`

示例：

```json
{
  "default_map_id": "hall",
  "maps": [
    {
      "map_id": "hall",
      "display_name": "Hall",
      "map_yaml": "/home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps/hall.yaml",
      "map_image": "/home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps/hall.pgm",
      "prior_map_pcd": "/home/ubuntu/humanoid_ws/src/humanoid_navigation2/pcd/hall_open3d_grounded.pcd",
      "localization_map_pcd": "/home/ubuntu/humanoid_ws/src/humanoid_navigation2/pcd/hall_localization_grounded.pcd",
      "posegraph_prefix": "/home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps/hall",
      "scan_context_db": "/home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps/hall_sc.bin",
      "initial_pose": {
        "x": 0.0,
        "y": 0.0,
        "yaw": 0.0
      },
      "initial_pose_tolerance": {
        "xy": 2.0,
        "yaw_deg": 30.0
      },
      "localization_strategy": "preset_initial_pose",
      "enabled": true
    },
    {
      "map_id": "hall1",
      "display_name": "Hall 1",
      "map_yaml": "/home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps/hall1.yaml",
      "map_image": "/home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps/hall1.pgm",
      "prior_map_pcd": "/home/ubuntu/humanoid_ws/src/humanoid_navigation2/pcd/hall1_open3d_grounded.pcd",
      "localization_map_pcd": "/home/ubuntu/humanoid_ws/src/humanoid_navigation2/pcd/hall1_localization_grounded.pcd",
      "posegraph_prefix": "/home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps/hall1",
      "scan_context_db": "/home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps/hall1_sc.bin",
      "initial_pose": {
        "x": 0.0,
        "y": 0.0,
        "yaw": 0.0
      },
      "initial_pose_tolerance": {
        "xy": 2.0,
        "yaw_deg": 30.0
      },
      "localization_strategy": "preset_initial_pose",
      "enabled": true
    }
  ]
}
```

### 6.2 注册表职责

地图注册表用于：

1. 给 app 返回地图列表。
2. 给切图节点查目标地图文件路径。
3. 约束地图文件命名和完整性。
4. 记录默认地图。
5. 记录每张图的初始位姿和定位初始化策略。
6. 后续支持禁用地图、删除地图、地图备注等扩展。

### 6.3 地图文件命名规范

建议一期统一如下命名：

- `maps/<map_id>.yaml`
- `maps/<map_id>.pgm`
- `maps/<map_id>.posegraph`
- `maps/<map_id>.data`
- `maps/<map_id>_sc.bin`
- `pcd/<map_id>.pcd`
- `pcd/<map_id>_localization_grounded.pcd`
- `pcd/<map_id>_open3d_grounded.pcd`

说明：

1. 一期至少必须保证 `yaml/pgm/prior_map_pcd` 三项齐全。
2. `posegraph/data/sc.bin` 可按实际链路决定是否强依赖。
3. 现有 `hall_open3d_grounded.pcd` 命名方式需要在新地图中同步保持。

## 7. 点位模型修改

### 7.1 点位新增字段

每个点位新增字段：

```json
{
  "id": "point_a",
  "name": "前台",
  "type": "navigation_target",
  "map_id": "hall1",
  "position": [1.23, 4.56, 0.0],
  "orientation": [0.0, 0.0, 0.707, 0.707],
  "frame_id": "map",
  "properties": {}
}
```

### 7.2 设计说明

1. `map_id` 必填。
2. `frame_id` 一期仍保留 `map`。
3. `map` 的语义为“当前激活地图的全局坐标系”。
4. 如果以后支持跨地图点位复制，则仍保留 `map_id` 不变。

### 7.3 旧数据兼容

已有旧点位数据可能没有 `map_id`，建议加载时兼容：

1. 若点位缺少 `map_id`，自动补为 `default_map_id`。
2. 保存时写回新的完整结构。
3. 对旧客户端未传 `map_id` 的请求，也尽量走兼容逻辑。

## 8. app 和 websocket 协议改造

### 8.1 新增地图查询接口

建议新增命令：

```json
{
  "command_type": "map_management",
  "command_data": {
    "command_type": "get_map_list"
  }
}
```

返回：

```json
{
  "maps": [
    {"map_id": "hall", "display_name": "Hall"},
    {"map_id": "hall1", "display_name": "Hall 1"}
  ],
  "current_map_id": "hall"
}
```

### 8.2 点位设置命令扩展

app 在设置点位时必须带上 `map_id`：

```json
{
  "command_type": "waypoint_management",
  "command_data": {
    "command_type": "set_waypoint",
    "waypoint_data": {
      "id": "point_a",
      "name": "前台",
      "type": "navigation_target",
      "map_id": "hall1",
      "position": [1.23, 4.56, 0.0],
      "orientation": [0.0, 0.0, 0.707, 0.707]
    }
  }
}
```

### 8.3 点位查询命令扩展

建议支持按地图过滤：

```json
{
  "command_type": "waypoint_management",
  "command_data": {
    "command_type": "get_waypoints",
    "map_id": "hall1",
    "include_details": true
  }
}
```

### 8.4 导航命令扩展

当前系统的真实使用方式是：

1. app 侧沿用“多点导航命令”。
2. 但每次实际只下发当前要执行的一个点。
3. 到点后由 app 再决定是否播报、以及是否下发下一点。

因此一期建议继续沿用 `start_multi_point_navigation`，但约定：

1. `waypoint_ids` 必须存在。
2. 一期每次请求里 `waypoint_ids` 只传 1 个当前点。
3. 整轮任务通过 `task_session_id` 串起来。

示例：

```json
{
  "command_type": "navigation_control",
  "command_data": {
    "command_type": "start_multi_point_navigation",
    "waypoint_ids": ["point_a"],
    "map_id": "hall1"
  }
}
```

即使点位本身已有 `map_id`，命令里也建议保留，便于做双重校验。

建议同时引入 `task_session_id`，用于标识“这一轮讲解/巡航任务”：

```json
{
  "command_type": "navigation_control",
  "command_data": {
    "command_type": "start_multi_point_navigation",
    "task_session_id": "tour_20250610_001",
    "waypoint_ids": ["point_a"],
    "map_id": "hall1"
  }
}
```

#### 8.4.1 task_session_id 传递时机

由于当前系统虽然走的是多点导航命令，但整体执行逻辑是：

1. 到达 A 点后，ROS 给 app 返回成功。
2. app 决定是否播报。
3. 不播报或播报完成后，app 再发下一个点位的导航任务。

因此 `task_session_id` 的建议用法是：

1. app 在用户点击“开始执行”时，生成一轮新的 `task_session_id`。
2. 这一轮任务中的所有逐点导航请求，都带同一个 `task_session_id`。
3. 当这一轮任务完成后，下一轮重新开始时，app 生成新的 `task_session_id`。

示例：

- 第一轮任务：`tour_001`
- 第二轮任务：`tour_002`

即使两轮任务都在同一张地图 `hall` 上，也必须使用不同的 `task_session_id`，因为它们在业务语义上是两轮不同任务。

#### 8.4.2 未传 task_session_id 的兼容策略

在 app 完成改造前，服务端可临时兼容：

1. 若未传 `task_session_id`，则沿用现有“当前是否已有活动导航任务”的判断逻辑。
2. 当前任务未结束时，新的开始请求继续返回现有错误：
   - `当前正在执行其他导航任务`
3. app 完成改造后，内部判断逻辑统一切到：
   - `session_status`
   - `task_session_id`
   - `session_map_id`

也就是说，一期建议保留现有报错文案以兼容现网，但底层逐步改成用任务 id 统一判断。

### 8.5 地图状态广播

建议 websocket 增加地图状态推送：

```json
{
  "data_type": "map_status",
  "data": {
    "current_map_id": "hall",
    "target_map_id": "hall1",
    "switch_status": "switching_map",
    "message": "正在切换地图 hall1"
  }
}
```

### 8.6 定位初始化状态广播

建议补一类状态推送：

```json
{
  "data_type": "localization_status",
  "data": {
    "map_id": "hall1",
    "status": "waiting_localization",
    "message": "已注入 hall1 初始位姿，等待定位稳定"
  }
}
```

## 9. ROS 节点职责拆分

### 9.1 新增地图上下文管理节点

建议新增节点：

- `map_context_manager`

建议放在：

- `/home/ubuntu/humanoid_ws/src/humanoid_navigation2/humanoid_navigation2/map_context_manager.py`

职责：

1. 加载地图注册表。
2. 提供地图列表。
3. 维护当前激活地图。
4. 执行地图切换。
5. 发布地图状态。
6. 做地图文件完整性校验。
7. 读取目标地图初始位姿配置。
8. 在切图后协调定位重置和初始化流程。

### 9.2 dynamic_waypoints_manager 职责

只负责：

1. 点位增删改查。
2. 点位持久化。
3. 点位 `map_id` 合法性检查。
4. 把点位数据和导航请求转给状态管理器。

不建议把地图切换逻辑塞进这个节点。

### 9.3 navigation_state_manager_recoverable 职责

在正式启动导航前：

1. 读取目标点位。
2. 解析目标地图 `map_id`。
3. 读取或创建当前任务的 `task_session_id` / `session_map_id`。
4. 判断当前激活地图是否一致。
5. 若不一致，请求 `map_context_manager` 执行切图。
6. 等待切图结果。
7. 等待定位就绪结果。
8. 校验当前请求的 `task_session_id` 是否属于同一轮任务。
9. 地图和定位都准备好后再发 goal。

## 10. 导航前切图流程

### 10.1 核心流程

1. 收到导航请求。
2. 从 `waypoint_ids` 中取当前要执行的唯一点位。
3. 读取点位 `map_id`。
4. 校验命令 `map_id` 和点位 `map_id` 是否一致。
5. 若当前地图已是目标地图，则直接导航。
6. 若不是，则调用地图切换流程。
7. 切图完成后重置定位链路。
8. 根据目标地图配置自动注入预设初始位姿。
9. 定位稳定后开始导航。
10. 若任一步失败，则返回失败状态。

### 10.1.1 session_map_id 锁定规则

当前系统虽然使用多点导航命令，但真实执行方式是 app 每次只下发一个点位。因此一期建议增加任务会话规则：

1. 一轮任务开始后，首个成功进入执行的点位确定本轮 `session_map_id`。
2. 后续同一轮任务中的所有点位，都必须与 `session_map_id` 一致。
3. 如果后续点位的 `map_id` 变成了另一张图，例如从 `hall` 变成 `hall1`，则直接报错并拒绝导航。
4. 地图切换只能发生在新任务开始前，不能发生在当前任务进行中。

### 10.1.2 任务 id 判断规则

建议用 `task_session_id` 统一解释“当前正在执行其他导航任务”这条现有逻辑。

规则如下：

1. 若当前没有活动任务：
   - 收到导航请求后，创建新的任务会话。
   - 记录 `task_session_id` 和 `session_map_id`。
2. 若当前已有活动任务：
   - 若请求里的 `task_session_id` 与当前活动任务相同，则认为是同一轮任务的后续点位。
   - 若请求里的 `task_session_id` 不同，则拒绝执行，并返回“当前任务未结束，不能启动新任务”。
3. 若 app 尚未改造传 `task_session_id`：
   - 继续沿用现有逻辑，返回“当前正在执行其他导航任务”。

### 10.1.3 任务结束后的解锁条件

任务会话的解锁条件建议只保留以下几类，不使用“超时未收到下一点”作为结束判断：

1. app 明确点击“终止任务”。
2. 当前任务被取消。
3. 当前任务失败并退出。
4. 当前任务所有点位完成。
5. 系统进程被停止脚本杀掉、断电、重启。

说明：

1. 不使用超时，是因为现场可能长时间暂停、播报时间不固定、或者启动系统后长时间不立即开始下一点。
2. 暂停不等于任务结束；暂停后继续当前任务时，仍应沿用原来的 `task_session_id`。

### 10.2 状态机

建议状态：

- `ready`
- `switching_map`
- `resetting_localization`
- `waiting_localization`
- `ready_to_navigate`
- `task_active`
- `failed`

说明：

1. `ready`：当前地图和定位状态可正常工作。
2. `switching_map`：正在切换地图，不允许新导航开始。
3. `resetting_localization`：正在清空旧图定位上下文。
4. `waiting_localization`：已注入新图初始位姿，等待定位稳定。
5. `ready_to_navigate`：地图和定位均已就绪，可开始导航。
6. `task_active`：本轮任务已启动，`task_session_id` 与 `session_map_id` 已锁定。
7. `failed`：切图或定位初始化失败。

### 10.3 切图步骤

1. 拒绝新的导航启动请求。
2. 若当前仍在导航，拒绝切图或等待导航结束。
3. 清理当前导航上下文。
4. 重载 2D 地图。
5. 重载 3D prior map。
6. 等待地图相关节点 ready。
7. 更新 `current_map_id`。
8. 重置 prior localization / bridge 等旧图上下文。
9. 读取目标地图 `initial_pose`。
10. 自动注入该初始位姿。
11. 等待 prior localization 稳定。
12. 定位成功后发布切图完成。

### 10.4 切图实现建议

一期不建议追求“无感热切图”，而建议使用“可控重建地图相关节点 + 新图初始化定位”的稳妥方案。

推荐策略：

1. 保留 planner/controller/bt 这些主导航节点。
2. 只对地图和定位相关节点做重启或重建：
   - `map_server`
   - `lifecycle_manager_map`
   - prior localization 相关节点
   - `prior_map_odom_bridge`
3. 切图期间状态机进入 `switching_map`。
4. 切图完成后，进入 `resetting_localization` 和 `waiting_localization`。

这样改动比整套系统重启小，但比强行在线 reload 更稳，也更接近“在新图开机重新定位”的行为。

## 11. 切图后的定位初始化策略

### 11.1 一期定位前提

一期明确假设：

1. 切到新图时，机器人已经被放到该图建图原点附近。
2. 系统无需在整张新图里做全局盲找。
3. 系统只需基于目标地图预设初始位姿重新初始化定位。

### 11.2 为什么必须重置定位状态

切图后不能直接沿用旧图定位状态，否则可能残留：

1. 旧图 `map->odom` 关系。
2. prior localization 内部缓存。
3. 上一张图的置信度和稳定帧计数。
4. 大跳保护、冻结、recovery 等旧状态。

因此切图后必须显式做：

1. 清旧图定位上下文。
2. 注入新图初始位姿。
3. 等待新图定位重新稳定。

### 11.3 一期定位恢复策略

建议一期统一采用：

- `preset_initial_pose`

含义是：

1. 每张图在注册表中都有一组预设初始位姿。
2. 切到该图后，系统自动使用这组位姿初始化定位。
3. 若在限定时间内定位成功，则继续导航。
4. 若失败，则返回错误，并由上层决定是否重试或人工干预。

### 11.4 定位就绪判据

建议满足以下条件后才允许发导航 goal：

1. prior localization 已恢复有效输出。
2. 置信度连续 N 次高于阈值。
3. `map->odom` 已稳定恢复发布。
4. 保持 2 到 3 秒 settle 时间。

若超时仍不满足，返回：

- `localization_not_ready_after_map_switch`

## 12. 建图脚本改造方案

### 12.1 为什么建图脚本也要改

虽然 [start_mapping.sh](/home/ubuntu/humanoid_ws/start_mapping.sh:4) 已支持传入地图名，但仅有“按名字生成文件”还不够。

一期多地图方案要求建图完成后：

1. 地图产物命名和导航侧约定一致。
2. 关键产物完整可用。
3. 新地图自动进入地图注册表。
4. 避免重名误覆盖。
5. 记录该地图的预设初始位姿。

### 12.2 一期建图流程建议

建图流程改为：

1. 传入地图名，例如 `hall1`
2. 启动建图
3. 输出对应地图文件
4. 生成或转换 prior localization 所需的 grounded pcd
5. 校验关键产物是否齐全
6. 记录该地图初始位姿
7. 将 `hall1` 注册到 `map_registry.json`
8. 输出“地图已可用于导航”

### 12.3 start_mapping.sh 要补的点

#### 1. 重名策略

建议新增参数：

- 默认：若地图已存在，则拒绝覆盖
- 可选：`--overwrite` 明确允许覆盖

避免误把已有 `hall1` 全套文件覆盖掉。

#### 2. 产物完整性校验

建图完成后至少校验：

- `maps/<map_id>.yaml`
- `maps/<map_id>.pgm`
- `pcd/<map_id>_open3d_grounded.pcd`

若缺失则不注册地图。

#### 3. 自动注册地图

建图完成后自动写入：

- `config/map_registry.json`

若已存在同名地图：

1. 默认拒绝。
2. `--overwrite` 时才更新注册信息。

#### 4. 命名对齐

当前脚本已有：

- `pcd/<map_id>.pcd`
- `pcd/<map_id>_localization_grounded.pcd`

还需明确是否补齐：

- `pcd/<map_id>_open3d_grounded.pcd`

如果后续 prior localization 实际依赖的是这个命名，则建图脚本必须自动生成它，而不能只保留 `localization_grounded` 版本。

#### 5. 初始位姿登记

建议建图完成后，把本次地图原点或默认启航位姿写入注册表中的：

- `initial_pose.x`
- `initial_pose.y`
- `initial_pose.yaw`

#### 6. 建图完成反馈

建议建图脚本最后输出明确结果：

```text
Map hall1 created successfully.
Registered into map_registry.json.
Initial pose saved.
Ready for navigation.
```

## 13. 还需要修改的其他模块

除建图脚本、点位、导航切图外，一期还建议一起改以下内容。

### 13.1 地图管理命令层

需要新增地图相关命令，而不只是点位命令和导航命令。

建议支持：

1. `get_map_list`
2. `get_current_map`
3. `switch_map`

其中 `switch_map` 主要给测试、运维和后续扩展预留。

### 13.2 地图状态持久化

建议保存最近一次激活地图，例如：

- `/home/ubuntu/humanoid_ws/src/humanoid_navigation2/config/map_runtime_state.json`

用于系统重启后恢复默认激活地图，或至少用于日志追踪。

### 13.3 启动脚本

[start_navigation.sh](/home/ubuntu/humanoid_ws/start_navigation.sh:1) 当前只负责构建和启动整套导航。建议补参数，例如：

```bash
./start_navigation.sh --map hall1
```

这样系统启动时就知道当前默认运行地图是哪一张。

### 13.4 文档和运维说明

需要新增运维说明：

1. 如何新建一张地图。
2. 如何查看当前已注册地图。
3. 如何删除一张地图。
4. 如何指定当前运行地图。
5. 如果切图失败，优先检查哪些文件。
6. 如果定位初始化失败，优先检查哪些状态。

## 14. 失败码与错误处理

建议统一错误码或错误原因文本，至少覆盖以下情况：

1. `map_not_found`
2. `map_file_missing`
3. `map_switch_rejected_navigation_active`
4. `map_switch_timeout`
5. `map_switch_partial_failure`
6. `localization_not_ready_after_map_switch`
7. `waypoint_map_mismatch`
8. `duplicate_map_id`
9. `map_registration_failed`
10. `map_build_output_incomplete`
11. `initial_pose_missing`
12. `initial_pose_injection_failed`
13. `session_map_mismatch`
14. `task_session_mismatch`

补充说明：

1. `session_map_mismatch`
   文案示例：`当前任务已锁定地图 hall，目标点属于 hall1，无法继续导航`
2. `task_session_mismatch`
   文案示例：`当前任务未结束，不能启动新任务`
3. 对旧 app 或现网兼容阶段，可以继续复用已有文案：
   - `当前正在执行其他导航任务`

## 15. 兼容性方案

### 15.1 旧点位数据兼容

1. 若旧点位没有 `map_id`，加载时补 `default_map_id`。
2. 保存时写回新结构。

### 15.2 旧客户端兼容

1. 导航命令没带 `map_id` 时，优先取点位里的 `map_id`。
2. 若点位里也没有，则回退到 `default_map_id`。
3. 后续逐步推动 app 全量带上 `map_id`。

### 15.3 旧地图文件兼容

对于只有 `hall` 的历史环境，初始化 `map_registry.json` 时默认把 `hall` 注册进去。

## 16. 测试方案

### 16.1 功能测试

1. 建图生成 `hall1` 成功。
2. `hall1` 自动注册成功。
3. app 能看到 `hall1`。
4. app 在 `hall1` 上建点位成功。
5. 当前地图为 `hall` 时，导航 `hall1` 点位会先切图再导航。
6. 当前地图已是 `hall1` 时，导航直接开始。
7. 第一轮 `hall` 导航结束后，第二轮 `hall1` 导航前切图成功。
8. 切图后自动注入 `hall1` 初始位姿并恢复定位。
9. 同一轮任务中，后续点位 `map_id` 从 `hall` 变成 `hall1` 时被正确拒绝。
10. 暂停后继续当前任务时，沿用同一个 `task_session_id` 可继续执行。
11. 当前任务未结束时，收到新的 `task_session_id` 会被正确拒绝。
12. 若 `waypoint_ids` 一次传入多个点，一期按协议拒绝并返回错误。

### 16.2 异常测试

1. 地图名重复时建图被拒绝。
2. 地图注册存在但关键文件缺失。
3. 切图时 prior localization 恢复超时。
4. 切图中收到第二条导航请求。
5. 点位地图和命令地图不一致。
6. 目标地图未配置初始位姿。
7. 初始位姿注入失败。
8. 当前任务未结束时收到新任务 id。

### 16.3 回归测试

1. 单地图旧流程仍可正常运行。
2. 老点位数据加载正常。
3. 旧 app 不带 `map_id` 时系统不崩溃。

## 17. 预计改动文件

### 17.1 新增文件

建议新增：

1. `src/humanoid_navigation2/config/map_registry.json`
2. `src/humanoid_navigation2/humanoid_navigation2/map_context_manager.py`
3. `src/humanoid_navigation2/docs/ROS侧多地图一期改造方案.md`
4. 视实现需要，新增 `map_runtime_state.json` 或服务定义

### 17.2 需要修改的现有文件

建议重点修改：

1. [start_mapping.sh](/home/ubuntu/humanoid_ws/start_mapping.sh:1)
2. [start_navigation.sh](/home/ubuntu/humanoid_ws/start_navigation.sh:1)
3. [dynamic_waypoints_manager.py](/home/ubuntu/humanoid_ws/src/humanoid_navigation/humanoid_navigation/dynamic_waypoints_manager.py:352)
4. [navigation_state_manager_recoverable.py](/home/ubuntu/humanoid_ws/src/humanoid_navigation/humanoid_navigation/navigation_state_manager_recoverable.py:937)
5. [websocket_server.py](/home/ubuntu/humanoid_ws/src/humanoid_websocket/humanoid_websocket/websocket_server.py:680)
6. [navigation2.launch.py](/home/ubuntu/humanoid_ws/src/humanoid_navigation2/launch/navigation2.launch.py:43)
7. [setup.py](/home/ubuntu/humanoid_ws/src/humanoid_navigation2/setup.py:1)  
   说明：若新增 `map_context_manager.py` 作为 console script，则需要补 entry point

## 18. 工时评估

以下按“1 名熟悉现有工程的人开发”估算。

### 18.1 设计与方案确认

- 0.5 天

### 18.2 地图注册表与地图上下文节点

- 1 到 1.5 天

### 18.3 点位模型增加 `map_id`

- 0.5 到 1 天

### 18.4 websocket 与命令协议扩展

- 0.5 到 1 天

### 18.5 导航前切图流程接入

- 1 到 1.5 天

### 18.6 定位重置与初始位姿注入

- 1 到 1.5 天

### 18.7 建图脚本补齐

- 1 到 1.5 天

### 18.8 异常保护与状态广播

- 0.5 到 1 天

### 18.9 联调和测试

- 1.5 到 2 天

### 18.10 文档与交付

- 0.5 天

### 18.11 总工时

合计预计：

- 7 到 9 个工作日

更贴近实际的建议排期：

- 开发 5 到 6 天
- 联调测试 2 到 3 天

因为一期不需要做复杂全局重定位，只做“切图后按目标地图预设初始位姿重新初始化”，所以工时比原先需要支持更复杂定位恢复的方案更可控。

## 19. 风险评估

### 19.1 高风险

1. prior localization 切图后恢复不稳定。
2. 切图后 `map->odom` 恢复时间较长。
3. 新地图文件命名和导航侧约定不一致。

### 19.2 中风险

1. 老点位数据兼容。
2. websocket 和 app 联调。
3. 切图中并发命令处理。
4. 初始位姿注入链路与当前定位系统对接的细节。

### 19.3 低风险

1. 地图列表接口。
2. 点位新增 `map_id`。
3. 建图脚本接收地图名。

## 20. 推荐实施顺序

建议按以下顺序落地：

1. 先补地图注册表和命名规范。
2. 再改建图脚本，保证新地图可自动入库，并记录初始位姿。
3. 再改点位模型和 websocket 协议。
4. 再加 `map_context_manager`。
5. 最后把导航前切图、定位重置和初始位姿注入接到状态管理器。
6. 最后把现有“当前正在执行其他导航任务”逻辑收敛到 `task_session_id` 判断。

这样顺序的好处是：

1. 先把“数据基础”和“地图资产入口”建好。
2. 后面导航切图时不会一边写逻辑一边猜文件长什么样。

## 21. 结论

一期除了建图脚本之外，还需要同步修改以下内容：

1. 地图注册表与地图上下文管理。
2. 点位模型和持久化结构。
3. websocket/app 协议。
4. 导航状态管理器的导航前切图流程。
5. 切图后的定位重置和初始位姿注入流程。
6. 地图状态、定位状态广播与失败码。
7. 任务会话管理：`task_session_id`、`session_map_id`、任务解锁规则。
8. 启动脚本和运维说明。

本方案的一期关键约束是：

**地图切换仅发生在机器人已被放置到目标地图建图原点附近的场景下，系统切图后按目标地图预设初始位姿重新初始化定位。**

在这个约束下，一期是一个中等复杂度、可以在 1 到 1.5 周内落地的需求。真正需要重点验证的是“切图后的定位恢复稳定性”，这会决定一期最终工时更接近 7 天还是 9 天。
