# 路线任务启动兼容完整点位与 ID 列表方案

## 1. 背景和目标

当前 `start_route_task` 只支持 APP 后端一次性下发完整 `route_waypoints`。每个路线点必须包含 `waypoint_id / waypoint_role / frame_id / position / orientation / need_broadcast / broadcast_id / stop_and_align / walk_direction` 等字段。

用户希望新增一种更轻的启动方式：导航前 APP 已经通过 `set_waypoint/update_waypoint` 把点位完整写入 ROS 本地 `dynamic_waypoints.json`，开始导航时 APP 只下发有序点位 ID 列表，ROS 根据 ID 从本地点位缓存中补全完整点位属性，然后继续走现有 route task 状态机。

本方案目标是两种方式兼容：

1. 兼容旧方式：APP 继续下发完整 `route_waypoints`，当前逻辑不变。
2. 新增 ID 方式：APP 下发 `route_waypoint_ids`，ROS 按本地点位库补全。
3. 两种方式最终都归一化成同一份内部 `route_waypoints` 快照。
4. 不改变跳步、辅助点、倒走、播报、障碍暂停恢复等已有路线任务语义。
5. 第一版即加入 `waypoints_revision` 点位库版本校验，避免 APP 和 ROS 点位不同步时只发 ID 导致机器人按旧坐标执行。

## 2. 当前源码现状

当前点位设置链路：

1. APP 发送 `data_type="waypoint_management"`。
2. `websocket_server.py` 将 `waypoint_data` 透传到 `/app/waypoint_command`。
3. `dynamic_waypoints_manager.py` 的 `handle_set_waypoint()` / `handle_update_waypoint()` 保存点位。
4. 点位会保存到 `/home/ubuntu/software/Todesk/Files/humanoid_ws/data/dynamic_waypoints.json`。
5. `dynamic_waypoints_manager.py` 发布 `/navigation/waypoints_data`。
6. `navigation_state_manager.py` 订阅 `/navigation/waypoints_data` 并缓存到 `self.waypoints_data`。

当前路线启动链路：

1. APP 发送 `start_route_task`。
2. `dynamic_waypoints_manager.py` 只做轻量校验和 ID 字符串化。
3. `navigation_state_manager.py` 的 `handle_start_route_task()` 读取 `command_data["route_waypoints"]`。
4. `normalize_route_task_waypoints()` 校验完整点位字段。
5. 如果只发 `{"waypoint_id": "11"}`，会因为缺 `position/orientation/waypoint_role` 返回错误。

当前已经可复用的基础能力：

1. `navigation_state_manager.py` 已有 `find_waypoint_data_by_id(waypoint_id)`。
2. `navigation_state_manager.py` 已缓存 `/navigation/waypoints_data`。
3. `normalize_route_task_waypoints()` 已经能统一校验完整点位、拒绝重复 ID、拒绝缺 pose、拒绝缺 role、拒绝只有 transit。

## 3. 推荐协议

### 3.1 旧方式：完整点位快照

旧方式继续保留。

```jsonc
{
  "protocol_version": "2.0",                    // 固定协议版本
  "message_id": "cmd_start_route_full_0001",    // APP 生成的唯一命令 ID
  "message_type": "command",                    // 固定 command
  "data_type": "navigation_control",            // 导航控制入口
  "source": "app",                              // 发送端
  "destination": "ros",                         // 接收端
  "data": {
    "command_type": "start_route_task",         // 开始路线任务
    "request_message_id": "cmd_start_route_full_0001", // 用于匹配 navigation_command_result
    "task_session_id": "session_20260613_001",  // 本次任务会话 ID
    "route_id": "route_exhibition_001",         // 路线 ID
    "route_waypoints": [                        // 完整路线点位快照
      {
        "waypoint_id": "10",                    // 点位 ID
        "waypoint_name": "展厅任务点10",         // 点位名称
        "waypoint_role": "task",                // task=任务点，transit=辅助点
        "frame_id": "map",                      // 坐标系
        "position": [1.0, 2.0, 0.0],             // x/y/z 坐标
        "orientation": [0.0, 0.0, 0.0, 1.0],     // 四元数 x/y/z/w
        "need_broadcast": true,                 // 是否需要 APP 播报
        "broadcast_id": "broadcast_10",         // 播报 ID
        "broadcast_text": "欢迎来到展厅10号点",   // APP 播放文本，ROS 不依赖
        "broadcast_blocking": true,             // 是否等待 APP 播报完成
        "stop_and_align": true,                 // 是否最终停车对齐
        "walk_direction": "forward",            // forward 正走，backward 倒走
        "properties": {                         // 兼容冗余属性
          "waypoint_role": "task",              // 同 waypoint_role
          "walk_direction": "forward"           // 同 walk_direction
        }
      }
    ]
  }
}
```

### 3.2 新方式：只下发 ID 列表

新增 `route_waypoint_ids`。数组顺序就是路线拓扑顺序，ROS 不按数字大小重新排序。

ID 列表模式必须携带 `waypoints_revision`。该字段表示 APP 当前记录的 ROS 点位库版本。ROS 收到后要与本地当前点位库版本比较，不一致则拒绝启动。

```jsonc
{
  "protocol_version": "2.0",                    // 固定协议版本
  "message_id": "cmd_start_route_ids_0001",     // APP 生成的唯一命令 ID
  "message_type": "command",                    // 固定 command
  "data_type": "navigation_control",            // 导航控制入口
  "source": "app",                              // 发送端
  "destination": "ros",                         // 接收端
  "data": {
    "command_type": "start_route_task",         // 开始路线任务
    "request_message_id": "cmd_start_route_ids_0001", // 用于匹配 navigation_command_result
    "task_session_id": "session_20260613_001",  // 本次任务会话 ID
    "route_id": "route_exhibition_001",         // 路线 ID
    "waypoints_revision": "1781285566.123",     // APP 当前记录的 ROS 点位库版本，ID 模式必填
    "route_waypoint_ids": [                     // 有序点位 ID 列表
      "10",                                     // 第 1 个路线点
      "11",                                     // 第 2 个路线点，可以是 transit
      "12",                                     // 第 3 个路线点，可以是 transit
      "13"                                      // 第 4 个路线点
    ]
  }
}
```

### 3.3 两种字段互斥规则

推荐第一版强制互斥：

1. 只允许发 `route_waypoints` 或 `route_waypoint_ids` 其中一种。
2. 两个都不发，返回 `invalid_route_waypoints`。
3. 两个都发，返回 `ambiguous_route_waypoint_source`。

强制互斥可以避免 APP 发了完整点位但同时又发 ID 列表，ROS 不知道应以哪个为准。

### 3.4 点位库版本 waypoints_revision

`waypoints_revision` 是 ROS 点位库版本号，用来保证 APP 和 ROS 对点位库的认知一致。

版本生成建议：

1. ROS 每次成功 `set_waypoint/update_waypoint/delete_waypoint/clear_waypoints` 后刷新 `waypoints_revision`。
2. 第一版可以使用 `time.time()` 字符串，例如 `"1781285566.123"`。
3. `dynamic_waypoints.json` 顶层保存当前 `waypoints_revision`。
4. `/navigation/waypoints_data` 推送中携带当前 `waypoints_revision`。
5. `waypoint_response success` 中也携带当前 `waypoints_revision`。
6. APP 保存点位成功后记录 ROS 返回的最新 `waypoints_revision`。
7. APP 使用 `route_waypoint_ids` 启动路线时必须把记录的 `waypoints_revision` 带上。

ID 模式校验规则：

1. APP 未传 `waypoints_revision`，返回 `missing_waypoints_revision`。
2. APP 传入 revision 与 ROS 当前 revision 不一致，返回 `waypoints_revision_mismatch`。
3. revision 匹配后，ROS 才允许根据 `route_waypoint_ids` 从本地点位库补全。

第一版强制 revision 校验只针对 `route_waypoint_ids` 模式，因为 ID 模式需要 ROS 再去本地点位库查坐标和属性。完整 `route_waypoints` 模式已经把本次路线快照带给 ROS，不依赖本地点位库补全；如果 APP 也传了 revision，ROS 只记录到 `active_route_task` 里用于日志，不参与强校验。

## 4. ROS 侧修改方案

### 4.1 修改 `websocket_server.py`

文件：

`src/humanoid_websocket/humanoid_websocket/websocket_server.py`

在 `route_to_waypoint_manager()` 的 `navigation_control` 分支中透传新增字段：

```python
"route_waypoint_ids": command_data.get("route_waypoint_ids", []),
"waypoints_revision": command_data.get("waypoints_revision", ""),
```

注意：

1. websocket 层不做点位查找。
2. websocket 层只负责把 APP 字段原样透传到 `/app/navigation_command`。
3. `request_message_id` 仍沿用 `command_data.get("request_message_id") or request_message_id`。

### 4.2 修改 `dynamic_waypoints_manager.py`

文件：

`src/humanoid_navigation/humanoid_navigation/dynamic_waypoints_manager.py`

需要改三处。

第一处：`validate_navigation_command()`

当前只检查 `route_waypoints`。修改为检查两种输入：

1. `route_waypoints` 是非空数组。
2. 或 `route_waypoint_ids` 是非空数组。
3. ID 模式缺 `waypoints_revision` 时只记录 warning 并继续转发给状态机，由状态机返回 `missing_waypoints_revision`。
4. 两者都缺，只记录 warning 并继续转发给状态机，由状态机返回统一业务 ack。

第二处：`normalize_navigation_command()`

新增对 `route_waypoint_ids` 的字符串化：

```python
route_waypoint_ids = normalized.get("route_waypoint_ids")
if isinstance(route_waypoint_ids, list):
    normalized["route_waypoint_ids"] = [
        str(item).strip()
        for item in route_waypoint_ids
        if item is not None
    ]
```

同时把 `waypoints_revision` 按字符串归一化，避免数字和字符串比较不一致。

第三处：注释更新

把原来“route_waypoints 已由 APP 后端一次性组好”的注释改成：

1. APP 可下发完整 `route_waypoints`。
2. APP 也可下发 `route_waypoint_ids`。
3. ID 模式必须带 `waypoints_revision`。
4. 具体补全和校验由 `navigation_state_manager.py` 统一处理。

### 4.3 修改 `dynamic_waypoints_manager.py` 的点位版本

文件：

`src/humanoid_navigation/humanoid_navigation/dynamic_waypoints_manager.py`

需要新增点位库版本字段。

建议新增成员：

```python
self.waypoints_revision = ""
```

建议新增函数：

```python
def refresh_waypoints_revision(self):
    """点位库发生成功写操作后刷新版本号。"""
    self.waypoints_revision = f"{time.time():.3f}"
```

保存和加载规则：

1. `save_waypoints_data()` 顶层写入 `"waypoints_revision": self.waypoints_revision`。
2. `load_waypoints_data()` 读取顶层 `"waypoints_revision"`。
3. 老 JSON 没有 revision 时，加载后生成一个新的 revision，并在下次保存时写入。
4. `publish_waypoints_data()` 在 `data` 或 `metadata` 中带上 `waypoints_revision`。
5. `send_app_response()` 的 success 响应中带上 `waypoints_revision`。

写操作刷新规则：

1. `handle_set_waypoint()` 成功写入后刷新 revision，再保存。
2. `handle_update_waypoint()` 成功更新后刷新 revision，再保存。
3. `handle_delete_waypoint()` 成功删除后刷新 revision，再保存。
4. `handle_clear_waypoints()` 成功清空后刷新 revision，再保存。

### 4.4 修改 `navigation_state_manager.py`

文件：

`src/humanoid_navigation/humanoid_navigation/navigation_state_manager.py`

这是核心改动。

#### 4.4.1 新增输入解析函数

建议新增：

```python
def resolve_route_task_waypoints(self, command_data: Dict[str, Any]) -> Tuple[List[Dict[str, Any]], str, str]:
    """把 start_route_task 的输入解析成完整 route_waypoints。

    支持两种来源：
    1. APP 直接下发完整 route_waypoints；
    2. APP 下发 route_waypoint_ids，ROS 从本地点位缓存补全。
    返回值：resolved_waypoints, error_code, message。
    """
```

处理规则：

1. `route_waypoints` 和 `route_waypoint_ids` 都存在且非空，返回 `ambiguous_route_waypoint_source`。
2. `route_waypoints` 存在，直接返回它。
3. `route_waypoint_ids` 存在，先校验 `waypoints_revision`，再调用 `build_route_waypoints_from_ids()` 补全。
4. 都不存在，返回 `invalid_route_waypoints`。

#### 4.4.2 新增点位版本缓存和校验

`waypoints_data_callback()` 需要从 `/navigation/waypoints_data` 中解析并缓存当前点位库版本。

建议新增成员：

```python
self.current_waypoints_revision = ""
```

解析规则：

1. 优先读取 `message_data["data"]["waypoints_revision"]`。
2. 其次读取 `message_data["data"]["metadata"]["waypoints_revision"]`。
3. 再其次读取顶层 `message_data["metadata"]["waypoints_revision"]`。
4. 兼容旧消息没有 revision，此时保持空字符串。

建议新增函数：

```python
def validate_waypoints_revision_for_id_mode(self, command_data: Dict[str, Any]) -> Tuple[str, str]:
    """校验 ID 列表模式的点位库版本。返回 error_code, message。"""
```

校验规则：

1. `waypoints_revision` 缺失，返回 `missing_waypoints_revision`。
2. `self.current_waypoints_revision` 为空，返回 `waypoints_cache_not_ready`。
3. 两者不一致，返回 `waypoints_revision_mismatch`。
4. 一致返回空错误。

#### 4.4.3 新增 ID 补全函数

建议新增：

```python
def build_route_waypoints_from_ids(self, route_waypoint_ids: Any) -> Tuple[List[Dict[str, Any]], str, str]:
    """根据有序 waypoint_id 列表，从 self.waypoints_data 缓存补全 route_waypoints。"""
```

校验规则：

1. `route_waypoint_ids` 必须是 list。
2. 列表不能为空。
3. 每个 ID 用 `route_task_id()` 归一化。
4. 空 ID 返回 `invalid_route_waypoint_ids`。
5. 重复 ID 返回 `duplicate_waypoint_id`。
6. `self.count_cached_waypoints() == 0` 时返回 `waypoints_cache_not_ready`。
7. `find_waypoint_data_by_id()` 找不到 ID 时返回 `waypoint_id_not_found`。
8. 找到后调用 `convert_stored_waypoint_to_route_waypoint()` 转换格式。

#### 4.4.4 新增点位格式转换函数

建议新增：

```python
def convert_stored_waypoint_to_route_waypoint(self, waypoint_id: str, stored_waypoint: Dict[str, Any], source_index: int) -> Dict[str, Any]:
    """把 dynamic_waypoints.json 中的点位格式转换成 route_waypoints 格式。"""
```

转换关系：

| 本地点位字段 | route task 字段 | 说明 |
| --- | --- | --- |
| `id` | `waypoint_id` | 点位 ID |
| `name` | `waypoint_name` | 点位名称 |
| `frame_id` | `frame_id` | 坐标系，缺失默认 `map` |
| `position` | `position` | x/y/z |
| `orientation` | `orientation` | 四元数 |
| `properties.waypoint_role` | `waypoint_role` | 必须是 `task/transit` |
| `properties.need_broadcast` | `need_broadcast` | 缺失默认 false |
| `properties.broadcast_id` | `broadcast_id` | 需要播报时必填 |
| `properties.broadcast_text` | `broadcast_text` | APP 播放文本 |
| `properties.broadcast_blocking` | `broadcast_blocking` | 缺失默认 true |
| `properties.stop_and_align` | `stop_and_align` | task 缺失默认 true |
| `properties.walk_direction` | `walk_direction` | 缺失默认 `forward` |
| 原始点位对象 | `raw_payload` | 便于日志排查 |

转换后的对象仍然交给 `normalize_route_task_waypoints()` 做最终严格校验。

#### 4.4.5 修改 `handle_start_route_task()`

当前逻辑：

```python
route_waypoints = command_data.get("route_waypoints", [])
normalized_waypoints, error_code, message = self.normalize_route_task_waypoints(route_waypoints)
```

改为：

```python
route_waypoints, resolve_error_code, resolve_message = self.resolve_route_task_waypoints(command_data)
if resolve_error_code:
    self.send_route_task_ack("start_route_task", "error", resolve_message, command_data, error_code=resolve_error_code)
    return

normalized_waypoints, error_code, message = self.normalize_route_task_waypoints(route_waypoints)
if error_code:
    self.send_route_task_ack("start_route_task", "error", message, command_data, error_code=error_code)
    return
```

建议在 `active_route_task` 中额外记录来源：

```python
"route_waypoint_source": "full_payload" or "stored_waypoint_ids",
"route_waypoint_ids": [...]
"waypoints_revision": command_data.get("waypoints_revision", "")
```

这样后续日志能分清本次任务是 APP 完整快照启动，还是 ROS 本地补全启动。

## 5. APP 侧修改方案

### 5.1 点位设置阶段

APP 设置点位仍使用现有 `waypoint_management`。

要求：

1. 每个点位必须保存 `id/name/type/frame_id/position/orientation/properties`。
2. 新 route task 属性必须放在 `properties` 中。
3. APP 必须等待 `waypoint_response success` 后，才能认为 ROS 已保存点位。
4. APP 必须从 `waypoint_response` 或 `waypoints_data` 中记录最新 `waypoints_revision`。
5. 如果 APP 批量设置点位，必须等所有点位都成功后再允许开始导航。
6. 批量设置完成后，APP 以最后一次成功响应中的 `waypoints_revision` 作为启动路线时的版本。

点位 `properties` 推荐字段：

```jsonc
{
  "waypoint_role": "task",              // task=任务点，transit=辅助点
  "need_broadcast": true,               // 是否需要 APP 播报
  "broadcast_id": "broadcast_10",       // 播报 ID
  "broadcast_text": "欢迎来到展厅10号点", // APP 播放文本
  "broadcast_blocking": true,           // 是否等待播报完成
  "stop_and_align": true,               // 是否最终停车对齐
  "walk_direction": "forward",          // forward 正走，backward 倒走
  "route_order": 10                     // APP UI 排序字段，ROS 不按它排序
}
```

### 5.2 开始路线任务阶段

APP 后端可以选择两种模式。

完整快照模式：

1. 后端查询 APP 自己的点位库。
2. 组完整 `route_waypoints`。
3. 发给 ROS。

ID 列表模式：

1. 后端按路线 UI 保存顺序生成 `route_waypoint_ids`。
2. 发给 ROS。
3. ROS 从本地点位缓存补全。

ID 列表模式请求：

```jsonc
{
  "protocol_version": "2.0",                    // 固定协议版本
  "message_id": "cmd_start_route_ids_0001",     // APP 唯一命令 ID
  "message_type": "command",                    // 固定 command
  "data_type": "navigation_control",            // 导航控制入口
  "source": "app",                              // 发送端
  "destination": "ros",                         // 接收端
  "data": {
    "command_type": "start_route_task",         // 开始路线任务
    "request_message_id": "cmd_start_route_ids_0001", // 对齐外层 message_id
    "task_session_id": "session_20260613_001",  // 本次路线任务会话 ID
    "route_id": "route_exhibition_001",         // 路线 ID
    "waypoints_revision": "1781285566.123",     // APP 当前记录的 ROS 点位库版本，ID 模式必填
    "route_waypoint_ids": [                     // 按路线执行顺序排列
      "10",                                     // 第 1 个点
      "11",                                     // 第 2 个点
      "12",                                     // 第 3 个点
      "13"                                      // 第 4 个点
    ]
  }
}
```

### 5.3 导航执行期间点位冻结

APP 和 ROS 需要约定：

1. `start_route_task` 成功后，本次任务使用启动瞬间解析出来的路线快照。
2. 导航执行期间，APP 不允许修改本次路线涉及的点位。
3. 如果确实修改了点位，只影响下一次 `start_route_task`。
4. 跳步 `jump_to_waypoint` 不重新查点位库，只在当前 `active_route_task.route_waypoints` 快照内跳转。

这条规则非常重要，能避免“导航中点位被改，机器人路线突然变化”的风险。

## 6. 错误码设计

新增或明确这些错误码：

| 错误码 | 触发条件 | APP 处理建议 |
| --- | --- | --- |
| `ambiguous_route_waypoint_source` | 同时下发 `route_waypoints` 和 `route_waypoint_ids` | 提示后端协议错误，只保留一种输入 |
| `invalid_route_waypoint_ids` | `route_waypoint_ids` 不是数组、为空、包含空 ID | 检查路线配置 |
| `missing_waypoints_revision` | ID 模式未携带 `waypoints_revision` | APP 先同步点位版本，再重新启动 |
| `waypoints_revision_mismatch` | APP 传入 revision 与 ROS 当前 revision 不一致 | 重新同步点位，禁止继续使用旧 ID 列表启动 |
| `waypoints_cache_not_ready` | ROS 尚未收到本地点位缓存 | 提示稍后重试或先查询点位同步状态 |
| `waypoint_id_not_found` | 某个 ID 在 ROS 本地点位库中不存在 | 重新同步点位或检查路线 ID |
| `invalid_waypoint_role` | 点位缺少 `properties.waypoint_role` 或不是 task/transit | 阻止启动，要求补齐点位属性 |
| `missing_waypoint_pose` | 点位缺少 position/orientation 或格式非法 | 阻止启动，要求重新保存点位 |
| `duplicate_waypoint_id` | 同一条路线里重复点位 ID | 检查路线配置 |
| `missing_task_waypoints` | 路线全是 transit，没有 task | 检查路线配置 |
| `missing_broadcast_id` | task 点 `need_broadcast=true` 但没有 `broadcast_id` | 补齐播报配置 |

错误响应仍走：

```jsonc
{
  "protocol_version": "2.0",                    // 固定协议版本
  "message_id": "push_navigation_command_result_0001", // ROS 生成消息 ID
  "message_type": "push",                       // 固定 push
  "data_type": "navigation_status",             // 导航状态类消息
  "source": "data_integration",                 // 数据整合节点
  "destination": "all",                         // 推送所有 APP 客户端
  "data": {
    "event_type": "navigation_command_result",  // 命令业务结果
    "event_data": {
      "request_message_id": "cmd_start_route_ids_0001", // 对应 APP 请求
      "ack_type": "navigation_command_result",  // 固定业务 ack
      "command_type": "start_route_task",       // 原始命令
      "task_session_id": "session_20260613_001", // 会话 ID
      "route_id": "route_exhibition_001",       // 路线 ID
      "status": "error",                        // error 表示拒绝执行
      "result_reason": "",                      // error 时为空
      "error_code": "waypoint_id_not_found",     // 具体错误码
      "message": "route_waypoint_ids[2] waypoint_id not found: 12", // 错误说明
      "event_id": "route_task_session_20260613_001_navigation_command_result_1_1781280000123", // 事件 ID
      "timestamp": 1781280000.123                // 事件时间
    }
  },
  "metadata": {
    "status": "error",                           // error
    "error_code": "waypoint_id_not_found",        // 同 event_data.error_code
    "error_message": "route_waypoint_ids[2] waypoint_id not found: 12" // 错误说明
  }
}
```

## 7. 风险点和解决办法

### 7.1 APP 和 ROS 点位不同步

风险：

APP 认为 11 是新坐标，ROS 本地 `dynamic_waypoints.json` 仍是旧坐标。APP 只发 `11`，ROS 会按旧坐标执行。

解决办法：

1. APP 每次保存点位后必须等待 `waypoint_response success`。
2. APP 批量设置路线点位时，所有点位成功后才允许开始。
3. 导航执行期间禁止修改本次路线涉及的点位。
4. 第一版必须启用 `waypoints_revision` 校验。
5. APP 启动 ID 模式时必须携带最新 `waypoints_revision`。
6. ROS revision 不一致时必须返回 `waypoints_revision_mismatch`，不能继续补全点位。

### 7.2 ROS 点位缓存未就绪

风险：

ROS 刚启动，`navigation_state_manager` 还没收到 `/navigation/waypoints_data`，此时 APP 发送 `route_waypoint_ids` 会查不到点位。

解决办法：

1. `dynamic_waypoints_manager.py` 启动后已经会初始发布点位。
2. `build_route_waypoints_from_ids()` 必须检测缓存为空并返回 `waypoints_cache_not_ready`。
3. APP 收到该错误时延迟重试，或先发 `get_waypoints`/等待 `waypoints_data` 推送。

### 7.3 老点位缺新属性

风险：

当前已有点位 JSON 可能只有 `walk_direction`，没有 `waypoint_role / need_broadcast / stop_and_align`。

解决办法：

1. `waypoint_role` 必须强校验，缺失返回 `invalid_waypoint_role`。
2. `need_broadcast` 缺失默认 false。
3. `broadcast_blocking` 缺失默认 true。
4. `stop_and_align` 对 task 缺失默认 true，对 transit 强制 false。
5. `walk_direction` 缺失默认 forward。
6. APP 点位设置页面要补齐这些属性，避免老点位直接启动失败。

### 7.4 两种输入同时存在

风险：

APP 同时发完整 `route_waypoints` 和 `route_waypoint_ids`，两边内容不一致。

解决办法：

直接返回 `ambiguous_route_waypoint_source`，让 APP 修正协议。

### 7.5 导航中修改点位

风险：

执行中点位变更导致路线语义不稳定。

解决办法：

1. `start_route_task` 时不管来源是完整点位还是 ID 补全，都冻结为 `active_route_task.route_waypoints`。
2. `jump_to_waypoint` 只使用冻结快照，不重新查本地点位库。
3. APP 导航中禁止编辑/删除本次任务点位。

### 7.6 ID 顺序被误认为数字顺序

风险：

APP 或 ROS 按点位数字排序，导致 7-15 之间辅助点吸收错误。

解决办法：

1. `route_waypoint_ids` 数组顺序就是路线顺序。
2. ROS 不按 ID 数值排序。
3. 文档和注释必须反复强调这一点。

## 8. 测试和验收用例

### 8.1 完整快照模式回归

输入完整 `route_waypoints`，验证：

1. 顺序导航正常。
2. task 点停车对齐。
3. transit 点不停车不播报。
4. backward task 仍走倒走逻辑。
5. 任意跳步仍按冻结路线快照计算。

### 8.2 ID 列表模式基础启动

前置：

1. APP 先保存 1-25 个点位。
2. 11、12、19-25 为 transit。
3. 14-16 为 backward task。

输入：

```jsonc
{
  "command_type": "start_route_task",
  "task_session_id": "session_ids_001",
  "route_id": "route_ids_001",
  "waypoints_revision": "1781285566.123",
  "route_waypoint_ids": ["1", "2", "3", "4", "5"]
}
```

验证：

1. ROS 返回 `navigation_command_result success`。
2. active route task 内部保存的是补全后的完整 `route_waypoints`。
3. active route task 内部记录 `route_waypoint_source=stored_waypoint_ids` 和本次 `waypoints_revision`。
4. 后续导航行为与完整快照模式一致。

### 8.3 ID 列表模式吸收辅助点

输入：

```jsonc
{
  "waypoints_revision": "1781285566.123",
  "route_waypoint_ids": ["10", "11", "12", "13"]
}
```

其中：

1. 10 是 task。
2. 11、12 是 transit。
3. 13 是 task。

验证：

1. 10 到 13 的执行段包含 11、12。
2. 11、12 只产生 `waypoint_passed`。
3. 13 才触发最终对齐和播报逻辑。

### 8.4 ID 列表模式跳步

路线：

```jsonc
{
  "waypoints_revision": "1781285566.123",
  "route_waypoint_ids": ["7", "8", "9", "10", "11", "12", "13", "14", "15"]
}
```

其中 11、12 是 transit。

验证：

1. 从 7 跳 15，执行段按数组顺序吸收 11、12。
2. 从 15 跳 7，反向跳按冻结路线反向计算。
3. 跳到 transit 点返回 `target_waypoint_not_task`。

### 8.5 错误用例

必须覆盖：

1. 同时发 `route_waypoints` 和 `route_waypoint_ids`，返回 `ambiguous_route_waypoint_source`。
2. `route_waypoint_ids=[]`，返回 `invalid_route_waypoint_ids`。
3. ID 模式缺 `waypoints_revision`，返回 `missing_waypoints_revision`。
4. ID 模式 `waypoints_revision` 与 ROS 当前版本不一致，返回 `waypoints_revision_mismatch`。
5. 缓存为空，返回 `waypoints_cache_not_ready`。
6. ID 不存在，返回 `waypoint_id_not_found`。
7. 点位缺 `properties.waypoint_role`，返回 `invalid_waypoint_role`。
8. 点位缺 pose，返回 `missing_waypoint_pose`。
9. 重复 ID，返回 `duplicate_waypoint_id`。
10. 全部为 transit，返回 `missing_task_waypoints`。
11. `need_broadcast=true` 但无 `broadcast_id`，返回 `missing_broadcast_id`。

## 9. 推荐排期

| 阶段 | 侧 | 工作内容 | 预计 |
| --- | --- | --- | --- |
| 1 | ROS | websocket 透传 `route_waypoint_ids` 和 `waypoints_revision` | 0.5 小时 |
| 2 | ROS | dynamic_waypoints_manager 增加 `waypoints_revision`、保存/推送版本、归一化 ID 列表 | 1-1.5 小时 |
| 3 | ROS | navigation_state_manager 新增 ID 补全函数、revision 校验和错误码 | 2.5-3.5 小时 |
| 4 | ROS | 静态校验脚本补规则 | 1 小时 |
| 5 | ROS | 动态模拟脚本补完整快照、ID 列表、revision 匹配/不匹配用例 | 2-3 小时 |
| 6 | APP | 后端 start_route_task 支持选择完整快照或 ID 列表模式，并记录/携带最新 `waypoints_revision` | 1.5-2.5 小时 |
| 7 | APP | 前端禁用导航中编辑点位，状态栏展示新增 revision 错误码 | 1 小时 |
| 8 | 联调 | 保存点位、ID 启动、跳步、障碍暂停恢复全链路测试 | 0.5-1 天 |

## 10. 最终建议

建议采用兼容策略：

1. 第一版 ROS 同时支持 `route_waypoints` 和 `route_waypoint_ids`。
2. 第一版 ID 列表模式必须携带 `waypoints_revision`，ROS 校验一致后才允许补全点位。
3. APP 可以先保留完整快照模式作为兜底。
4. ID 列表模式联调稳定后，再作为 APP 默认启动方式。
5. 不建议删除完整 `route_waypoints` 支持，因为它适合调试、回放、跨机器迁移和临时路线测试。

这套方案的关键不是消息大小，而是点位一致性。只要做到“点位保存成功后拿到最新 revision 才能启动”“启动时 revision 必须匹配”和“导航中冻结点位”，ID 列表模式的风险是可控的。
