# 双引擎全局重定位 + Fusion 停车接管 设计方案

日期：2026-05-27
参考 bag：`/home/ubuntu/nav_drift_test3/`
目标：基于 test3 复盘，构建"受控停车 + 双引擎搜索 + 推算先验 + 稳定验收"的定位恢复体系

## 1. 问题回顾

test3 暴露了三条根因的叠加效应：

| # | 问题 | 根因 | 当前状态 |
|---|------|------|----------|
| A | `map->odom` 多发布者竞争 | NDT 拒帧时重发旧 TF，fusion DEGRADED 也发冻结 TF | **已部分修复** (`republish_last_good_tf_on_failure: false`) |
| B | DEGRADED 期间继续走 | 设计初衷是 odom 兜底导航，但边界太宽 (120-180s) | **未修复** |
| C | Recovery 等不到静止 | fusion 驱动机器人继续走，HDL recovery 要求静止 | **未修复** |

## 2. 设计目标

不是回到"NDT 飘了就停等 HDL"的老方案，也不是继续"fusion 无限接管"的现状，
而是在两者之间建立一条受控的恢复链路：

1. NDT 异常时 **fusion 立即接管 TF**（唯一发布者）
2. **导航停车**，不再继续走
3. fusion 用冻结值 + odom 位移 **推算机器人当前位姿** 作为重定位先验
4. **先验喂给双引擎搜索**（SC → HDL fallback），缩小搜索范围
5. **NDT 多帧校验** 初始位姿，通过后才恢复 HEALTHY
6. 恢复后 **继续被中断的 waypoint**，不跳点

新旧对比：

| | 旧方案 NDT+HDL | 当前 fusion | 新方案 |
|---|---|---|---|
| NDT 飘后行为 | 立即停等 HDL | 继续走 (odom) | 立即停 + 先验搜索 |
| TF 发布者 | 只有 NDT | NDT + fusion 竞争 | **fusion 唯一** (DEGRADED+LOST) |
| Recovery 先验 | 无 (全图盲搜) | 无 (全图盲搜) | odom 推算位姿 (误差 1-2m) |
| 搜索结果验证 | 无 | SC 有 GICP+一致性 | SC 先搜 → HDL 兜底 → NDT 多帧校验 |
| 恢复后行为 | 走下一个点 | 继续当前点 | 继续被中断的点 |

## 3. 架构

```
Fast-LIO
  publishes: odom → camera_init/body/base_footprint

lidar_localization (NDT)
  subscribes: /fast_lio/cloud_registered
              /localization/fusion_status    ★ NEW: 监听 fusion 状态
  publishes: /pcl_pose
             /localization/ndt_status
  publishes TF: map→odom (仅 HEALTHY 时)     ★ MOD: 非 HEALTHY 时闭嘴

  NOTE: NDT 独立运行时 (fusion 不参与), 行为完全不变 — 正常发布 TF

localization_odom_fusion
  subscribes: /pcl_pose, /localization/ndt_status, /localization/recovery_status
  publishes: map→odom (DEGRADED/TRANSITIONING/LOST 时, 冻结值)
             /localization/fusion_status     ★ JSON: {state, ndt_error, ...}
             /localization/recovery_requests ★ MOD: 附带推算位姿先验

TF ownership 分工 (Plan B 过渡架构):
  HEALTHY:               NDT 发布 map→odom, fusion 不发布
  DEGRADED/TRANSITIONING/LOST:  NDT 抑制, fusion 发布冻结 map→odom
  切换保护: NDT 监听 fusion_status, 非 HEALTHY → 立即停止 TF;
           fusion 进入非 HEALTHY → 立即发冻结 TF

navigation_state_manager
  subscribes: /localization/fusion_status    ★ 已有: _get_fusion_state()
  owns: 定位不健康 → blocked + zero cmd      ★ MOD: SET-BASED 判断
        缓存被中断的 waypoint + pending request

SC Global Localizer (主引擎)
  订阅: /localization/recovery_requests
  输出: /initialpose (★ 唯一发布者)
  描述子匹配 + GICP 精化, 先验搜索→全局搜索

HDL Global Localization (fallback 引擎)
  订阅: SC 显式触发的 fallback 请求
  输出: 候选位姿 → SC bridge → /initialpose
  enable_runtime_auto_recovery: false  (不直接响应 recovery_request)
  FPFH+RANSAC, 机器人静止后清 buffer→重新采集→搜索
```

## 4. 状态机

### 4.1 Fusion 状态 (简化，无新增状态)

```text
INITIALIZING → HEALTHY → DEGRADED → LOST → HEALTHY
                           ↑                     │
                           └─── TRANSITIONING ←──┘
```

状态行为变化（★ 标记为变更）：

| 状态 | 当前行为 | 新行为 |
|------|----------|--------|
| HEALTHY | NDT 发 TF，fusion 不发 | **同左，不变** |
| DEGRADED | NDT 重发旧 TF + fusion 发冻结 TF | **NDT 不发 TF**（被 fusion_status 抑制），**只有 fusion 发冻结 TF** |
| DEGRADED | 导航继续走 odom | **navigation_state_manager 停车**（cancel goal + zero cmd） |
| LOST | 不发布 TF | **fusion 继续发冻结 TF**（keep TF tree alive） |
| LOST | 等 recovery | 同左 + **附带推算位姿先验** |

### 4.2 Navigation State Manager 行为变化

**核心改动**: 不再等待 `localization_recovery_started` 事件，改为直接监听 fusion 状态变化主动触发停车。

#### 改动 1: DEGRADED 进入 → 立即停车

```text
当前:
  fusion DEGRADED → navigation_state_manager._on_fusion_status() 只更新变量
    → 什么都不做 → 导航继续走 (BUG: 设计意图是停车但缺少触发路径)

新:
  fusion DEGRADED → _on_fusion_status() 检测到 HEALTHY→DEGRADED 跳变
    → _handle_fusion_degraded():
      1. 检查导航状态: EXECUTING/PLANNING → 暂停; IDLE → 不暂停（无导航）
      2. cancel 当前 Nav2 goal
      3. begin_localization_stop_hold() — 持续发零速度
      4. 保留 current_waypoint（不标记完成，不跳下个点）
      5. 通知 APP: "定位异常，已暂停导航"
      6. 等待 fusion 恢复 HEALTHY 后自动 resume
```

关键实现 (`navigation_state_manager_fusion.py`):

```python
# 状态集合定义
HEALTHY_STATES = {"HEALTHY"}
BLOCKED_STATES = {"DEGRADED", "TRANSITIONING", "LOST"}

def _on_fusion_status(self, msg: String):
    # ... 现有解析 ...
    new_state = data.get('state', 'HEALTHY')
    old_state = self.latest_fusion_state

    # ★ SET-BASED 判据: 只要定位不健康，就进入 blocked；只要恢复健康，就解除
    was_blocked = old_state in BLOCKED_STATES
    is_blocked = new_state in BLOCKED_STATES

    if is_blocked and not was_blocked:
        # 进入 blocked: HEALTHY→DEGRADED, INITIALIZING→DEGRADED, 
        #              TRANSITIONING→DEGRADED, HEALTHY→LOST 等全部覆盖
        self._handle_localization_blocked(new_state)
    elif not is_blocked and was_blocked:
        # 退出 blocked: DEGRADED→HEALTHY, LOST→HEALTHY, TRANSITIONING→HEALTHY
        self._handle_fusion_recovered()

    self.latest_fusion_state = new_state


def _handle_localization_blocked(self, blocked_state: str):
    """定位进入不健康状态: 停止导航 + 缓存上下文"""
    if self.current_state in (NavigationState.EXECUTING, NavigationState.PLANNING):
        # 场景A: 导航进行中 → 立即暂停
        self.localization_auto_paused = True
        self.localization_resume_pending = False
        self.localization_recovery_active = True
        self.localization_recovery_reason = f"fusion进入{blocked_state}状态"

        self.current_state = NavigationState.PAUSED
        self.current_detailed_state = "LOCALIZATION_RECOVERY"
        self.pause_time = time.time()
        self.pause_duration_limit = 0
        self.reset_block_detection()
        self.begin_localization_stop_hold()

        if self.current_goal_handle:
            self.cancel_navigation()

        self.publish_status_update("navigation_paused", {
            "pause_source": "fusion_blocked",
            "fusion_state": blocked_state,
            "reason": self.localization_recovery_reason,
            "current_waypoint_id": self.current_waypoint.get("id", ""),
            "waypoint_index": self.current_waypoint_index,
        })
    # else: IDLE 态 → 不暂停（由导航入口门禁拦截新点位）


def _handle_fusion_recovered(self):
    """定位恢复 HEALTHY: resume 被中断的导航 或 执行缓存的新点位"""
    if self.localization_auto_paused:
        self.localization_resume_pending = True
        self.try_resume_after_localization_recovery()
    elif self.pending_navigation_request is not None:
        self._execute_pending_navigation_later()
```

**启动时序兼容**: navigation_state_manager 启动时，`latest_fusion_state` 默认 "HEALTHY"。如果 fusion 启动晚于 navigator，首次 fusion_status 到达时如果非 HEALTHY → 触发 `_handle_localization_blocked()`。如果 fusion 从未启动，latest_fusion_state 永远是 "HEALTHY" → 不影响独立 NDT 模式。

**同时移除** `handle_localization_recovery_started()` 中的 DEGRADED 豁免 (line 1370-1376): 该豁免逻辑已被 set-based 机制取代，不需要单独判断。

#### 改动 2: IDLE 态定位异常 → 拦截新点位

**场景**: 机器人到达路点后语音播报中 → NDT 退化 → 状态变 IDLE → 语音播报完毕 → APP 发下一个点位

**问题**: 如果不对导航启动入口加门禁，IDLE 态下收到新点位会直接启动导航——此时定位还是错的，机器人会在错误位姿下出发。

**修复**: 在 `handle_start_single_navigation`（及 multi-point、exhibition 入口）加融合状态门禁：

```python
def handle_start_single_navigation(self, command_data, request_data):
    # 现有: 检查状态 == IDLE
    if self.current_state != NavigationState.IDLE:
        self.send_acknowledgment("error", "当前正在执行其他导航任务")
        return

    # ★ 新增: 定位不健康时缓存请求，不立即执行
    fusion_state = self._get_fusion_state()
    if fusion_state != "HEALTHY":
        self._cache_navigation_for_recovery(request_data)
        return

    # ... 原有逻辑: 启动导航 ...
```

```python
def _cache_navigation_for_recovery(self, request_data):
    """定位恢复期间收到的新点位请求 → 缓存 + 告知 APP pending
    
    Pending 语义:
      - 只保留最后一个请求 (新请求覆盖旧请求)
      - 覆盖时通知 APP: "pending_overwritten" (避免 APP 以为上一个还在排队)
      - 用户 cancel_navigation 时清除 pending request
      - 恢复后自动执行 (延迟 1s 等 NDT/costmap 稳定)
      - pending 超时 (pending_navigation_timeout: 90s) → 通知 APP 失败
    """
    was_pending = self.pending_navigation_request is not None
    self.pending_navigation_request = json.loads(json.dumps(request_data))
    self.pending_navigation_created_at = time.time()
    self.pending_navigation_reason = "等待定位恢复后自动执行"
    self.send_acknowledgment(
        "navigation_pending_overwritten" if was_pending else "navigation_pending",
        "pending",
        f"定位异常({self._get_fusion_state()})，导航请求已{'覆盖' if was_pending else '缓存'}，定位恢复后自动执行")
```

**Cancel 导航时清除 pending** — 在 `handle_stop_navigation()` 中已有清除逻辑（line 1125-1133），确认覆盖 `pending_navigation_request`。

```python
def _handle_fusion_recovered(self):
    """fusion 恢复到 HEALTHY: resume 被中断的导航 或 执行缓存的新点位"""
    if self.localization_auto_paused:
        # 场景1: 导航进行中被中断 → 恢复当前 waypoint
        self.localization_resume_pending = True
        self.try_resume_after_localization_recovery()
    elif self.pending_navigation_request is not None:
        # 场景2: IDLE 态缓存了新点位 → 延迟 1s 后执行（等 NDT/costmap 稳定）
        self._execute_pending_navigation_later()
    # else: 什么都不做（没有待恢复的任务）
```

#### 完整事件链

```
场景A: 导航进行中 NDT 退化
  NDT 退化 → fusion DEGRADED → _on_fusion_status() set-based 检测
  → is_blocked=true, was_blocked=false → _handle_localization_blocked("DEGRADED")
  → cancel goal + zero cmd + 保留 current_waypoint
  → DEGRADED 持续 → LOST → recovery → HEALTHY
  → is_blocked=false, was_blocked=true → _handle_fusion_recovered()
  → resume current_waypoint

场景B: 到达后语音播报中 NDT 退化
  机器人到达 → navigation_completed → IDLE → 语音播报
  → NDT 退化 → fusion DEGRADED → _handle_localization_blocked("DEGRADED")
  → 状态是 IDLE → 不触发暂停（无导航可停）
  → 语音播报完毕 → APP 发下一个点位
  → handle_start_single_navigation() → is_blocked=true
  → _cache_navigation_for_recovery() → 缓存 + 告知 APP "pending"
  → NDT 恢复 → HEALTHY → _handle_fusion_recovered()
  → _execute_pending_navigation_later() → 1s 后启动导航到新点位

场景C: INITIALIZING → DEGRADED (启动时 NDT 就异常)
  fusion INITIALIZING → 等待超时 → 请求 recovery → DEGRADED
  → is_blocked=true, was_blocked=false → _handle_localization_blocked("DEGRADED")
  → 如果导航正在进行中 → 暂停；如果 IDLE → 拦截入口

场景D: TRANSITIONING → DEGRADED (过渡中 NDT 又退化)
  fusion TRANSITIONING → NDT 又异常 → DEGRADED
  → is_blocked=true, was_blocked=true → 已 blocked，不重复操作
  → 状态保持不变（已暂停，继续等 recovery）

场景E: navigator 晚于 fusion 启动, 首次收到状态就是 DEGRADED
  navigator 启动 → latest_fusion_state="HEALTHY" (默认)
  → 收到第一个 fusion_status: DEGRADED
  → is_blocked=true, was_blocked=false → 触发 blocked
  → 导航入口门禁生效
```

## 5. 关键机制

### 5.1 TF 语义抑制（替代 frequency competition）

```
已有基础设施:
  fusion 已发布 /localization/fusion_status (JSON String)
  {state: "HEALTHY"|"DEGRADED"|...}

新增:
  NDT 订阅 /localization/fusion_status
  fusion_state ∈ {"DEGRADED", "TRANSITIONING", "LOST"} → NDT 抑制所有 TF 发布
  fusion_state == "HEALTHY" → NDT 正常发布 TF

抑制点 (NDT C++ 端):
  broadcaster_.sendTransform(transform_stamped)       // line 1206 — 正常帧
  publishLastGoodTransformIfFresh(reason)             // lines 1043,1053,1140,1163 — 拒帧

实现方式:
  if (fusion_state_ != "HEALTHY") {
    // skip all TF publishing
    return;
  }
```

NDT 作为独立节点的兼容性：
- NDT 初始化时 `fusion_state_` 默认 "HEALTHY"
- **只抑制 DEGRADED/TRANSITIONING/LOST，不抑制 INITIALIZING**（fusion 重启时短暂 INITIALIZING 不会导致 NDT 误抑制）
- 如果 fusion 不在运行，NDT 永远收不到 DEGRADED 信号 → 行为不变

**fusion 崩溃保护 — FUSION_TIMEOUT 状态**:

原方案的问题是: fusion 在 DEGRADED 时崩溃 → 5s 后 NDT 自动恢复 TF → 如果 NDT 仍漂移，错误 TF 重新进入系统 → navigation_state_manager 也收不到 fusion_status 更新 → 不知道发生了什么 → 可能恢复导航。

评审意见正确: 超时不应直接回退 HEALTHY，而应进入一个受控的 `FUSION_TIMEOUT` 状态，区分"fusion 从未运行过"和"fusion 运行过但崩溃了"。

**改进方案**:

NDT C++ 端增加一个标志位 `fusion_ever_received_non_healthy_`:

```cpp
// 订阅 /localization/fusion_status 回调
void fusionStatusCallback(const std_msgs::msg::String::SharedPtr msg) {
    // ... 解析 fusion_state ...
    fusion_state_ = parsed_state;
    last_fusion_status_time_ = this->now();
    fusion_ever_received_ = true;
    if (fusion_state_ != "HEALTHY") {
        fusion_ever_received_non_healthy_ = true;
    }
}

// TF 抑制检查 (在 cloudReceived 各发布点)
bool shouldSuppressTF() {
    // 情况1: fusion 明确说非 HEALTHY → 抑制
    if (fusion_state_ != "HEALTHY" && fusion_state_ != "INITIALIZING") {
        double age = (this->now() - last_fusion_status_time_).seconds();
        if (age <= 5.0) {
            return true;  // fusion 在线且非 HEALTHY → 抑制
        }
        // 情况2: fusion 超时 (>5s) → FUSION_TIMEOUT
        if (fusion_ever_received_non_healthy_) {
            // fusion 曾经在非 HEALTHY 状态运行过 → 可能崩溃了
            // 默认不自动恢复 TF (allow_ndt_tf_when_fusion_timeout: false)
            if (!allow_ndt_tf_when_fusion_timeout_) {
                RCLCPP_WARN_THROTTLE(..., "FUSION_TIMEOUT: suppressing TF");
                return true;  // 继续抑制
            }
        }
        // 情况3: fusion 从未发过非 HEALTHY → 可能是独立 NDT 模式
        // 或 fusion_ever_received_ 从未为 true → 融合不在场 → 不抑制
    }
    return false;  // 允许发布
}
```

**参数**:
```yaml
lidar_localization:
  fusion_status_timeout_sec: 5.0        # fusion 消息超时阈值
  allow_ndt_tf_when_fusion_timeout: false  # 默认 false: 超时继续抑制 TF
```

**设计原理**:
- **fusion 从未运行过** (`fusion_ever_received_ = false`): NDT 完全独立, 始终发布 TF → 兼容旧 launch
- **fusion 运行过但从未非 HEALTHY** (`fusion_ever_received_non_healthy_ = false`): fusion 一直健康, 超时意味着 fusion 可能在 HEALTHY 态崩了 → NDT 接管是安全的
- **fusion 运行过且进入过非 HEALTHY** (`fusion_ever_received_non_healthy_ = true`): fusion 在 DEGRADED/LOST 中崩了 → NDT 可能还在漂移 → 默认继续抑制 TF, 等人工介入

**navigation_state_manager 同步保护**:
navigator 也需要自己的 fusion_status 超时检测:

```python
def _check_fusion_status_timeout(self):
    """fusion_status 超时检测: 如果超过阈值没收消息 + 最后状态是非健康 → 保持 blocked"""
    if self.latest_fusion_state in BLOCKED_STATES:
        age = time.time() - self.last_fusion_status_time
        if age > 5.0:
            # fusion 可能崩溃在非健康态 → 保持 blocked，不自动恢复
            self.get_logger().error(
                f'fusion_status 超时 ({age:.1f}s), 保持 localization_blocked')
            # 不清除 blocked 状态，不 resume 导航
```

### 5.2 推算位姿先验

**核心思路**: 不手工做坐标转换。直接通过 TF 链查询 `map→body` 获取机器人在 map 帧的实时位置，以此作为重定位先验中心。

**为什么可以直接查 TF 链**:

1. P0-2 修复后，LOST 期间 fusion 继续发布冻结 TF → `map→odom` 可用
2. `odom→camera_init` 是静态 TF（身份或近身份）→ 始终可用
3. `camera_init→body` 由 Fast-LIO 实时发布 → 始终可用
4. 三条边齐全 → `map→body` 可查询

**实现** (fusion 端 `_compute_prior_pose()`):

```python
def _compute_prior_pose(self):
    """通过 TF 链直接查询 body 在 map 帧的位置，作为重定位先验中心"""
    try:
        body_in_map = self.tf_buffer.lookup_transform(
            'map', 'body', Time(), timeout=Duration(seconds=0.1))
    except Exception:
        # TF 链断裂 → fallback: 使用冻结时保存的 map_T_body (不是 map_T_odom!)
        # map_T_odom 是 odom 原点在 map 下的位置，不是机器人位置
        # 机器人位置 = frozen_map_T_odom × frozen_odom_T_body
        if self.frozen_map_body is None:
            return None
        frozen_yaw = quat_to_yaw(
            self.frozen_map_body['qx'], self.frozen_map_body['qy'],
            self.frozen_map_body['qz'], self.frozen_map_body['qw'])
        now = time.monotonic()
        return {
            'x': self.frozen_map_body['x'],
            'y': self.frozen_map_body['y'],
            'radius_m': 5.0,          # TF 链断裂 → 用大半径覆盖不确定性
            'yaw_constrained': False,
            'search_mode': 'full_360',
            'source': 'frozen_map_body_no_tf',
            'frozen_age_sec': now - self.frozen_body_stamp,
            'odom_displacement_m': 0.0,   # TF 断了无法推算位移
        }

    prior_x = body_in_map.transform.translation.x
    prior_y = body_in_map.transform.translation.y
    prior_stamp_sec = (body_in_map.header.stamp.sec +
                       body_in_map.header.stamp.nanosec * 1e-9)

    # 搜索半径 = odom 累计位移 + 安全余量
    current_odom = self._lookup_odom_body()
    if current_odom is not None and self.frozen_odom_body is not None:
        displacement = self._compute_odom_displacement(current_odom)
    else:
        displacement = 0.0

    radius = max(displacement + 2.0, 2.0)   # 最小 2m
    radius = min(radius, 5.0)                # 上限 5m

    now = time.monotonic()
    return {
        'x': prior_x,
        'y': prior_y,
        'radius_m': radius,
        'yaw_constrained': False,
        'search_mode': 'full_360',
        'source': 'frozen_tf_chain',
        'odom_displacement_m': displacement,
        'frozen_age_sec': now - self.frozen_body_stamp,
        'ndt_correction_at_freeze_m': self.ndt_correction_at_freeze,
        'stamp': prior_stamp_sec,
    }
```

**冻结快照扩展** — fusion 进入 DEGRADED 时额外保存:

```python
def _enter_degraded(self):
    # ... 现有逻辑: frozen_map_odom, frozen_odom_body ...
    
    # ★ 新增: 保存冻结时的 map_T_body (用于 fallback prior)
    try:
        body_in_map = self.tf_buffer.lookup_transform(
            'map', 'body', Time(), timeout=Duration(seconds=0.1))
        self.frozen_map_body = {
            'x': body_in_map.transform.translation.x,
            'y': body_in_map.transform.translation.y,
            'z': body_in_map.transform.translation.z,
            'qx': body_in_map.transform.rotation.x,
            'qy': body_in_map.transform.rotation.y,
            'qz': body_in_map.transform.rotation.z,
            'qw': body_in_map.transform.rotation.w,
        }
        self.frozen_body_stamp = time.monotonic()
    except Exception:
        self.frozen_map_body = None
        self.frozen_body_stamp = 0.0
    
    # ★ 新增: 记录冻结时的 NDT 状态 (用于 prior 可信度评估)
    self.ndt_correction_at_freeze = self.latest_ndt_correction_translation
```

**为什么 fallback 不能用 `frozen_map_odom` 当机器人位置**:

`frozen_map_odom` 是 `map→odom` 变换，表示 odom 坐标系原点在 map 中的位置。机器人初始化时，odom 原点通常在地图原点附近，但机器人实际位置取决于它走了多远。如果机器人从原点出发走了 30m 到达第一个路点，那么:
- `frozen_map_odom.x ≈ 0` (odom 原点几乎没变)
- 机器人实际位置 ≈ (30, 0) (在 map 帧下)

用 `frozen_map_odom.x` 作为先验会把先验圆心放在地图原点附近，而机器人在 30m 外 → 先验完全错误。正确做法是用冻结时查询的 `map_T_body`（机器人位置）或 `frozen_map_odom × frozen_odom_body` 计算得出。

**精度分析** (基于新方案 DEGRADED→立即停车):

| 场景 | frozen偏差 | odom漂移 | 查询 body 偏差 | 3m 先验覆盖？ |
|------|-----------|---------|---------------|-------------|
| 短期 (<5s) DEGRADED | <0.3m | ~0 | <0.3m | ✓ |
| 中期 (5-30s) 静止 | <0.5m | ~0.05m | <0.55m | ✓ |
| 长期 (>30s) 静止等待 LOST | <0.5m | ~0.1m | <0.6m | ✓ |
| 长期走动 (旧行为，新方案不出现) | <0.5m | ~1-3m | <3.5m | △ |

新方案 DEGRADED 立即停车，80%+ 情况属于短期静止，先验精度极高。

**为什么不用手工坐标转换公式**:

原先设计的手工公式 `prior_x = x_f + dx*cos(yaw_f) - dy*sin(yaw_f)` 假定了 `camera_init` 坐标轴与 standard ROS 的映射关系。但 camera_init 轴 (x=左, y=下, z=后) 与 ROS 轴 (x=前, y=左, z=上) 存在非平凡旋转。手工转换容易出错，尤其在 `odom→camera_init` 非 identity 的场景。

直接查 TF 链的好处:
1. 不管 `camera_init` 轴怎么定义，TF 库自动处理所有旋转
2. 如果将来换传感器 / 改配置，不需要改代码
3. 自然验证: 查不到 TF → 说明真的断了 → fallback 到 frozen_only

**先验不约束 yaw 的理由** (不变):
- 冻结时的 yaw_f 可能已偏 (NDT 开始漂移但未超阈值)
- odom yaw 有累积漂移
- SC (ring-key 循环平移) 和 HDL (FPFH 旋转不变) 天然支持 360° 搜索
- 约束 yaw 反而可能把正确朝向排除在外

先验传递方式：
```json
// /localization/recovery_requests
{
  "reason": "degraded_timeout",
  "prior": {
    "x": 5.8,
    "y": 12.1,
    "radius_m": 3.0,
    "yaw_constrained": false,
    "search_mode": "full_360",
    "source": "frozen_tf_chain",
    "odom_displacement_m": 0.42,
    "frozen_age_sec": 2.1,
    "ndt_correction_at_freeze_m": 0.62,
    "stamp": 1716883201.234
  }
}
// prior.yaw 不传递, 两个引擎各自做 360° 全覆盖搜索
// frozen_age_sec / ndt_correction_at_freeze_m 供 recovery 引擎判断先验可信度
```

### 5.3 双引擎搜索 + 仲裁机制

```
第一引擎: ScanContext (已启用)
  ├── 第一层: odom 先验搜索 (radius=2-3m)
  │     基于 ring-key 描述子匹配 → GICP 精化 → odom gate → 多帧一致性
  ├── 第二层: SC 全局搜索 (radius=全图)
  │     3 次局部失败后触发 'global_recovery_after_attempts: 3'
  └── 超时: 约 2-3s (局部) + 5-10s (全局)

第二引擎: HDL FPFH+RANSAC (仅 SC 显式触发, 不独立响应 recovery_request)
  ├── 前提: 机器人已静止 (由 navigation_state_manager 保证)
  ├── 清 buffer: 丢弃 DEGRADED 期间的污染点云
  ├── 采集 1-2s 新的 stationary scans
  ├── 第一层: 先验半径搜索 (3-5m)
  ├── 第二层: 扩大半径 (10m)
  └── 第三层: 全图搜索
  超时: 15s

双引擎输出统一:
  SC bridge 是唯一 /initialpose 发布者 (见下方仲裁机制)。
  SC 输出最终候选; HDL 只输出候选位姿给 SC bridge，不直接发布 /initialpose。
  NDT 只接收最终仲裁后的 /initialpose，不区分候选来源。
```

#### 仲裁机制

**问题**: SC bridge 和 HDL bridge 都订阅 `/localization/recovery_requests`，如果同时激活会竞争 `/initialpose`。

**方案**: SC bridge 作为主引擎，HDL 作为纯 fallback（不响应 recovery_request）：

```
SC bridge:
  - 订阅 /localization/recovery_requests ✓
  - enable_runtime_auto_recovery: True
  - SC 先验搜索 → GICP 精化 → 发布 /initialpose
  - SC 全局搜索失败 → 通过内部 topic/service 显式触发 HDL

HDL bridge:
  - enable_runtime_auto_recovery: False  ← 不自动响应 recovery_request
  - 只响应 SC bridge 的显式 fallback 调用
  - SC 通过 /localization/hdl_fallback_request topic 触发 HDL
```

**HDL fallback 触发条件**:
- SC 局部搜索失败 (3 次) → SC 全局搜索
- SC 全局搜索失败 (再 3 次) → 触发 HDL
- 或 SC 全局候选中 top1-top2 margin 太小 (歧义) → 触发 HDL

**为什么 SC 为主、HDL 为备**:
- SC 快 (~200ms 描述子匹配)，HDL 慢 (5-15s RANSAC)
- 正常场景 SC 秒级恢复 → 不需要 HDL
- 长廊歧义场景 SC 失败 → HDL 几何鲁棒性兜底
- 不是并行交叉验证 (可能两个结果不一致)，而是顺序 fallback (清晰仲裁)

### 5.4 NDT 恢复校验 + 多层验收

#### NDT 层: initialpose 接收与状态清理

```
当前 initialPoseReceived() 行为 (line 709-778):
  1. 坐标系变换 (line 725-753)
  2. has_last_good_transform_ = false     // 跳过位姿跳变检查
  3. consecutive_rejected_frames_ = 0      // 重置拒帧计数
  4. initialpose_reacquire_active_ = true  // 启用宽松阈值
  5. cloudReceived(last_scan_ptr_)         // ★ 问题: 用旧帧点云

改动:
  (a) 不用旧帧点云 → 等下一帧自然到达
      删除 line 772-775 的强制 cloudReceived(last_scan_ptr_)
      改为只保存 initialpose，让下一帧 point cloud callback 自然触发匹配
      
  (b) NDT 状态清理已足够:
      has_last_good_transform_ = false → 第一帧 pose_jump 检查绕过
      consecutive_rejected_frames_ = 0 → 拒帧计数从零开始
      initialpose_reacquire_active_ → 宽松阈值
```

#### Fusion 层: NDT 连续帧 + TF 跳变

```
恢复验收 (在 fusion._validate_lost_recovery_soft() 中增强):
  
  NDT 指标:
    consecutive_accepted >= lock_recovery_healthy_consecutive_frames (10)
    fitness_score < 0.05
    correction_translation < 0.25m
    correction_yaw < 0.15rad
    inlier_fraction > 0.0 (不能长期为 0 — 长廊虚假健康检测)
  
  TF 指标:
    map->odom 单次跳变 < recovery_pose_jump_max_m (5.0m)
    /pcl_pose 帧间跳变 < 0.1m
    TF 查询稳定，无 lookup timeout
  
  prior 一致性 (软门禁):
    recovery 后的 map->odom 与 frozen 值的差 < odom_displacement + recovery_pose_max_xy_error_m
    NDT error 极低 (<0.03) 时可跳过此检查
```

#### Costmap/Navigation 层 (fusion 或 navigator 端)

```
恢复后稳定验收 (在 navigation_state_manager.try_resume_after_localization_recovery() 中增强):

  Costmap 指标:
    robot pose 在 map bounds 内 (通过 /global_costmap/costmap 或 TF 查询)
    不报 robot/sensor out of bounds
  
  Navigation 状态:
    当前 waypoint 未被误标记完成
    pending request 未被错误消费
  
  恢复后稳定延迟:
    localization_resume_settle_sec: 1.0  (costmap 稳定 + Nav2 重新规划)
```

#### 恢复验收检查清单

| 层级 | 检查项 | 来源 | 阈值 |
|------|--------|------|------|
| NDT | consecutive_accepted | fusion 端 | >= 10 帧 |
| NDT | fitness_score | ndt_status | < 0.05 |
| NDT | correction_translation | ndt_status | < 0.25m |
| NDT | inlier_fraction | ndt_status | > 0.0 |
| TF | map→odom 跳变 | fusion 端 | < 5.0m |
| TF | /pcl_pose 帧间跳变 | fusion 端 | < 0.1m |
| TF | TF 查询稳定性 | fusion 端 | 无 timeout |
| Costmap | robot pose within map | navigator 端 | map bounds |
| Costmap | no out of bounds | navigator 端 | 无 robot/sensor OOB |
| Navigation | waypoint 未被误标 | navigator 端 | current_waypoint 一致 |
| Timing | 稳定延迟 | navigator 端 | 1.0s settle |

## 6. 新 Launch 文件

```
文件: src/humanoid_navigation2/launch/navigation2_fusion_sc_v2.launch.py

变更 (相对 navigation2_fusion_sc.launch.py):
  1. 保留 NDT 节点 (publish_tf 通过参数控制, 默认 true)
  2. NDT 新增参数:
     republish_last_good_tf_on_failure: false (已做)
     max_last_good_tf_age_sec: 0.5 (已做)
     订阅 /localization/fusion_status (通过 ROS args)
     fusion_status_timeout_sec: 5.0         ★ NEW
     allow_ndt_tf_when_fusion_timeout: false ★ NEW
  3. HDL global localization 节点: enable_runtime_auto_recovery: false ★ 不独立响应
  4. SC 参数: global_recovery_after_attempts: 3
  5. fusion 参数:
     - degraded_consecutive_frames: 2 (保持)
     - max_degraded_lock_sec: 30 → 10        ★ 缩短锁定期 (机器人已静止)
     - min_degraded_lock_sec: 30 → 10        ★ 同上
     - max_odom_displacement_m: 30 → 5
     - recovery_pose_soft_gate_enabled: true (保持)
     - lock_recovery_healthy_consecutive_frames: 10
     - lock_recovery_max_correction_m: 0.3
  6. navigation_state_manager 参数:
     - localization_resume_settle_sec: 1.0 (保持)
     - localization_stop_hold_sec: 2.0 (保持)

旧文件保留:
  navigation2_fusion_sc.launch.py — 不改动, 作为回退版本
```

## 7. 分阶段实施

### Phase 1: TF 抑制 + DEGRADED 停车 + LOST TF 保持

**状态**: ✅ **已实施** (2026-05-27)

**目标**: 消除 TF 冲突，DEGRADED 时可靠停车，LOST 时 TF 不断

**实际修改**:
| 文件 | 改动 | 状态 |
|------|------|------|
| `lidar_localization_component.hpp` | 新增 fusion_status 监听成员变量 + shouldSuppressTF() 声明 | ✅ |
| `lidar_localization_component.cpp` | 订阅 `/localization/fusion_status`，shouldSuppressTF() 3-tier 判据 (HEALTHY→FUSION_TIMEOUT→fusion_ever_received_non_healthy_)，TF 发布点抑制 | ✅ |
| `localization_odom_fusion.py` | (a) `_update_lost()` 继续发布 frozen TF；(b) `_enter_degraded()` 保存 frozen_map_body + ndt_correction_at_freeze | ✅ |
| `navigation_state_manager_fusion.py` | (a) SET-BASED `_on_fusion_status` 判据 (BLOCKED_STATES)；(b) `_handle_localization_blocked()` / `_handle_fusion_recovered()`；(c) 3个导航入口门禁 + `_cache_navigation_for_recovery()`；(d) 移除 DEGRADED 豁免；(e) `_check_fusion_status_timeout()` 定时器 | ✅ |
| `navigation2_fusion_sc_v2.launch.py` | 新文件，参数: fusion_status_timeout_sec: 5.0, allow_ndt_tf_when_fusion_timeout: false, min/max_degraded_lock_sec: 10, max_odom_displacement_m: 5 | ✅ |
| `CHANGELOG.md` | Phase 1 变更记录 | ✅ |

**验收**:
```bash
# 1. 检查 /tf publisher 确认无双源
ros2 topic info /tf -v
# 应只有 fusion 一个 publisher (DEGRADED/LOST 期间), NDT 不在列表中

# 2. 模拟 NDT 异常 → DEGRADED → 确认导航暂停
ros2 topic echo /navigation/status | grep -E "LOCALIZATION_RECOVERY|paused"
# 应看到 detailed_state 变为 LOCALIZATION_RECOVERY

# 3. 确认 LOST 时 TF 不断
ros2 topic echo /tf --field transforms | grep -A5 "map.*odom"
# LOST 期间应持续看到 fusion 发布的冻结 TF

# 4. bag 离线检查脚本 (建议)
#  - 统计 map->odom 发布频率 (每帧应来自同一 publisher)
#  - 统计 map->odom 帧间跳变 (应 < 0.1m, 排除 recovery snap)
#  - 统计 /cmd_vel 在 localization_blocked 后归零
#  - 统计 /initialpose 由单一节点发布
```

### Phase 2: 推算位姿先验 + SC 先验搜索

**状态**: ✅ **已实施** (2026-05-27)

**目标**: 用 odom 推算位姿缩小 SC 搜索范围

**实际修改**:
| 文件 | 改动 | 状态 |
|------|------|------|
| `localization_odom_fusion.py` | 新增 `_compute_prior_pose()` 通过 TF chain `map→body` 查询; `_request_recovery()` 附带 prior 到 recovery_requests JSON | ✅ |
| `scancontext_to_initialpose` | `recovery_request_callback` 解析 prior; `publish_recovery_status` 记录 prior 来源; `start_recovery`/`complete_recovery` 管理 prior 生命周期 | ✅ |
| `navigation2_fusion_sc_v2.launch.py` | SC bridge 无新增参数 (prior 通过 recovery_request 动态传递) | ✅ |

**验收**:
```bash
# 检查 recovery_requests 包含 prior
ros2 topic echo /localization/recovery_requests
# 应包含 prior: {x, y, radius_m, source, odom_displacement_m, ...}
```

### Phase 3: HDL 兜底 + NDT 校验增强 + Waypoint 保留

**状态**: ✅ **已实施** (2026-05-27)

**目标**: SC 失败后 HDL 兜底，恢复后正确继续导航

**实际修改**:
| 文件 | 改动 | 状态 |
|------|------|------|
| `navigation2_fusion_sc_v2.launch.py` | 新增 `hdl_bootstrap_to_initialpose` 桥接节点 (monitor_localization=false, allow_full_global_recovery_without_prior=true); SC bridge 新增 `hdl_fallback_request_topic` 参数 | ✅ |
| `scancontext_to_initialpose` | 新增 `_trigger_hdl_fallback()` + HDL publisher; `timer_callback` 全局搜索超限触发 HDL; `ndt_status_callback` NDT 拒绝后触发 HDL | ✅ |
| `hdl_bootstrap_to_initialpose.py` | 新增 `hdl_fallback_callback` 订阅 `/localization/hdl_fallback_request`; fallback 触发时允许无先验全图搜索 (复用现有 1979 行 HDL 流水线, 不新写桥接节点) | ✅ |
| `lidar_localization_component.cpp` | `initialPoseReceived()` 删除 `cloudReceived(last_scan_ptr_)` 调用 — 等下一帧自然到达 | ✅ |
| `navigation_state_manager_fusion.py` | `try_resume_after_localization_recovery()` 恢复前清除 waypoint_arrived 标志 + 验证 current_waypoint 数据完整性 | ✅ |

## 8. 审计闭环 — 内审 + 外审问题汇总

### 8.1 自审发现 (2026-05-27 第一轮)

对照实际代码全流程推演，逐项已融入设计：

| 编号 | 问题 | 级别 | 解决章节 | 核心方案 |
|------|------|------|---------|---------|
| P0-1 | DEGRADED→停车触发链缺失 | 阻断 | §4.2 改动1 | set-based 判据: 非 HEALTHY → `_handle_localization_blocked()` |
| P0-2 | LOST 状态不发 TF | 阻断 | §4.1, §7 Phase1 | `_update_lost()` 继续发布 `frozen_map_odom` |
| P1-1 | 推算先验坐标系统一 | 高 | §5.2 | TF 链查询 `map→body`，fallback 用 `frozen_map_body` |
| P1-2 | fusion_status 超时保护 | 高 | §5.1 | FUSION_TIMEOUT 状态，默认继续抑制 TF |
| P1-3 | 双引擎仲裁 | 高 | §5.3 | SC 唯一 /initialpose 发布者，HDL 仅 fallback |
| P3-1 | 锁定期过长 | 低 | §6 | `min_degraded_lock_sec: 10` |
| P3-2 | NDT 旧帧点云 | 低 | §5.4 | 删除 `cloudReceived(last_scan_ptr_)` |
| P3-3 | APP waypoint 重发 | 低 | §4.2 改动2 | 入口门禁 + pending 缓存 |

### 8.2 外审发现 (2026-05-27, `DUAL_ENGINE_RECOVERY_DESIGN_REVIEW.md`)

| 评审问题 | 级别 | 是否采纳 | 处理 |
|---------|------|---------|------|
| §3 TF ownership 措辞冲突 (Plan A vs Plan B) | 架构 | ✅ 采纳 Plan B | §3 重写, 明确 NDT=HEALTHY / fusion=非HEALTHY 分工 |
| /initialpose 双发布者风险 | 阻断 | ✅ 采纳 | §3, §5.3: SC bridge 是唯一 /initialpose 发布者 |
| DEGRADED 跳变条件太窄 (只检测 HEALTHY→DEGRADED) | 高 | ✅ 采纳 | §4.2: 改为 SET-BASED 判据 (BLOCKED_STATES) |
| fusion 超时回退 HEALTHY 有安全风险 | 高 | ✅ 采纳 | §5.1: 改为 FUSION_TIMEOUT 状态, 默认继续抑制 |
| fallback prior 把 odom 原点当机器人位置 | 阻断 | ✅ 采纳 | §5.2: 改为 frozen_map_body + 保存冻结快照 |
| SC 主引擎不足以防长廊误匹配 | 中 | ⚠️ 部分采纳 | §5.3: 保留 margin-based fallback + NDT 多帧最终防线; top-K 验证纳入 Phase 3 |
| DEGRADED 立即 cancel 过于激进 | 中 | ⚠️ 评估后维持 | `degraded_consecutive_frames: 2` 已提供 ~200ms 防抖, 单帧抖动不会触发 |
| 锁定期 15s 仍偏长 | 低 | ⚠️ 折中 | §6: 降至 10s。2-3s 太短 (仅 20-30 帧), 无法区分真恢复/假恢复 |
| pending 单变量覆盖/取消语义 | 中 | ✅ 采纳 | §4.2 改动2: 明确覆盖通知 + cancel 清除 |
| 恢复验收只查 NDT 不够 | 高 | ✅ 采纳 | §5.4: 扩展为 NDT + TF + Costmap + Navigation 四层验收 |
| 验收命令看不出 publisher | 低 | ✅ 采纳 | §7: 改用 `ros2 topic info /tf -v` + bag 检查脚本 |
| prior 时间不一致 | 低 | ✅ 采纳 | §5.2: prior 带 stamp/frozen_age_sec/ndt_correction_at_freeze_m |

### 8.3 外审后续建议 (预留 Phase 3+)

评审提出的以下建议暂不纳入 Phase 1-2，留作后续增强：

| 建议 | 理由 |
|------|------|
| SC top-K + HDL/GICP 候选验证 | 增加 Phase 3 复杂度。当前 NDT 多帧校验可拦截错误候选 |
| 独立 `recovery_arbiter` 节点 | 新增节点增加维护负担。SC bridge 临时承担 arbiter 足够 |
| DEGRADED soft-block 分级 (先 zero cmd + 短观察再 cancel) | `degraded_consecutive_frames: 2` 已等效防抖。场景倾向于安全优先 |
| pending queue (deque maxlen=3) | APP 一个一个发点位, 单变量覆盖语义已足够 |

### 8.4 Phase 1 修改量 (最终版)

| 文件 | 改动点 | 状态 |
|------|--------|------|
| `lidar_localization_component.hpp` | (a) fusionStatusCallback/shouldSuppressTF 声明; (b) fusion 状态成员变量; (c) 新参数声明 | ✅ |
| `lidar_localization_component.cpp` | (a) 订阅 `/localization/fusion_status`; (b) `shouldSuppressTF()` 3 级判据; (c) FUSION_TIMEOUT 超时逻辑; (d) TF 发布点抑制 (acceptance + rejection) | ✅ |
| `localization_odom_fusion.py` | (a) `_update_lost()` 继续发 frozen TF; (b) `_enter_degraded()` 保存 frozen_map_body + ndt_correction_at_freeze | ✅ |
| `navigation_state_manager_fusion.py` | (a) SET-BASED `_on_fusion_status` + `_handle_localization_blocked()`; (b) `_handle_fusion_recovered()`; (c) 导航入口门禁 + `_cache_navigation_for_recovery()`; (d) 移除 DEGRADED 豁免; (e) `_check_fusion_status_timeout()` | ✅ |
| `navigation2_fusion_sc_v2.launch.py` | 新文件 (~400 行) | ✅ |

**Phase 1 实施完毕** ✅ | **编译**: ✅ 通过 (lidar_localization_ros2)

## 9. 风险与边界

| 风险 | 缓解 |
|------|------|
| DEGRADED 后停车但定位一直不恢复 | LOST 超时 30s 后自动触发 recovery，超时后走人工等待 |
| SC 和 HDL 都搜不到 | HDL 15s 超时，超时后告警，保留当前 waypoint 等人工 |
| 频繁 DEGRADED 导致频繁停车 | `degraded_consecutive_frames: 2` 防止单帧误触发，"锁定期 10-15s" 防止乒乓恢复 |
| 停车位置在障碍物区域 | Nav2 恢复后会 replan，不受停车位置影响 |
| 推算位姿先验偏差过大 | SC 先验搜索失败后自动退回全局搜索 (3 次) |
| 两个引擎结果不一致被 NDT 拒绝 | NDT 的多帧一致性校验会自动滤掉不一致的候选 |
| 导航恢复后走错方向 | 恢复后 Nav2 重新规划，不依赖恢复前的 plan |
| NDT 永久抑制 TF（fusion 崩溃） | FUSION_TIMEOUT 下默认继续抑制 TF，navigator 保持 blocked；需人工重启 fusion 或设 allow_ndt_tf_when_fusion_timeout=true 才降级放行 |
| fusion 重启导致短暂 TF 抑制 | NDT 只抑制 DEGRADED/TRANSITIONING/LOST，不抑制 INITIALIZING |
| SC 和 HDL 同时响应 recovery_request | HDL `enable_runtime_auto_recovery: false`，仅由 SC 显式 fallback 触发 |
| IDLE 态定位异常时 APP 发新点位 | navigation 入口检测 fusion_state != HEALTHY → 缓存请求 + 告知 APP "pending"，恢复后自动执行 |
| fUSION_TIMEOUT 下 NDT 继续抑制，长时间无 TF | navigator + NDT 双端超时检测，5s 阈值。可人工介入重启 fusion |
| 冻结快照 frozen_map_body 查询失败 (启动瞬间 DEGRADED) | 不使用 frozen_map_odom 作为机器人位置；标记 prior unavailable，SC/HDL 直接进入扩大半径或全局搜索。记录 warning 日志 |
| SC bridge 本身崩溃 (唯一 /initialpose 发布者挂了) | Phase 3 后可由 HDL 接管发布。Phase 1-2 期间: fusion 持续重发 recovery_request, SC 重启后自动恢复 |
| 评审建议的部分优化未纳入 Phase 1 | §8.3 已列出预留项。不影响 Phase 1 核心功能 |

## 10. 与文档 `NDT_FUSION_RECOVERY_REDESIGN_TEST3.md` 的关键差异

| 维度 | 文档方案 | 本方案 | 理由 |
|------|----------|--------|------|
| TF 发布者 | fusion 唯一发布（全状态） | NDT 在 HEALTHY，fusion 在 DEGRADED+ | 启动链路更简单，独立 NDT 不受影响 |
| 接管后行为 | BRIDGING 2s → 停 | DEGRADED → 立即停 | 用户反馈：短接管无意义，不如停稳后快恢复 |
| 重定位引擎 | 单一引擎 (HDL/SC) | SC + HDL 双引擎 fallback | 长廊场景 SC 可能歧义，HDL 几何兜底 |
| 先验传递 | 通过 HDL 先验搜索 | 同时支持 SC 和 HDL 使用先验 | 先验不应绑定特定引擎 |
| Recovery 触发 | 导航暂停前置 (P2) | DEGRADED 即停车 | 简化状态机，减少中间态 |
