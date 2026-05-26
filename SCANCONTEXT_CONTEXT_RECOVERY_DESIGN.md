# ScanContext Context 恢复方案 设计与实现文档

> 创建时间: 2026-05-25  
> 工作空间: `/home/ubuntu/humanoid_ws`  
> 目标: 记录本次导航 `context` 恢复功能的设计思路、改动位置、运行逻辑和 bag 验证结果

---

## 一、问题背景

### 1.1 原始问题

机器人在 hall 场景导航时，NDT 在局部几何相似区域会出现漂移、拒帧或 `/pcl_pose` 停止更新。原来的恢复链路主要依赖 HDL bootstrap / navigation context prior，但在部分点位会出现以下问题：

```
NDT 定位失效
    ↓
/pcl_pose stale 或 NDT 连续 rejected
    ↓
navigation_state_manager 发布 context recovery request
    ↓
HDL / 全局定位尝试恢复
    ↓
如果 prior 被污染或环境相似，可能接受错误候选
```

本次新增的 ScanContext context 恢复路线，是把 Scan Context 作为一个非侵入式 sidecar：它不直接抢 TF，只在启动或失锁恢复时提供 `/initialpose` 候选给 NDT。

### 1.2 当前导航 launch 是否已使用 context 功能

当前 `/home/ubuntu/humanoid_ws/src/humanoid_navigation2/launch/navigation2.launch.py` 已经接入 ScanContext context 恢复链路：

- 4 秒后启动 `scancontext_global_localizer_node`
- 6 秒后启动 `scancontext_to_initialpose`
- 5 秒后启动 NDT `lidar_localization`
- ScanContext 不发布 `map->odom`
- NDT 仍然独占主定位输出
- ScanContext 只在启动、定位缺失、外部 recovery request 时发布 `/initialpose`

对应 launch 位置：

```text
src/humanoid_navigation2/launch/navigation2.launch.py
  651: scancontext_global_localizer_node
  692: scancontext_to_initialpose_node
  731: ndt_localization_node
  1147: TimerAction(period=4.0, actions=[scancontext_global_localizer_node])
  1149: TimerAction(period=6.0, actions=[scancontext_to_initialpose_node])
  1154: TimerAction(period=5.0, actions=[ndt_localization_node])
```

---

## 二、整体方案

### 2.1 核心思路

ScanContext 只做“候选位姿生成”，不直接改变定位 TF。

```
Fast-LIO cloud + odom
    ↓
ScanContext sidecar 计算全局候选
    ↓
候选经过 odom gate / ambiguity gate / GICP gate / 多帧一致性
    ↓
scancontext_to_initialpose 转换坐标并发布 /initialpose
    ↓
NDT 接收 initialpose 后重新 scan-to-map
    ↓
只有 NDT 连续 accepted 后，恢复才算成功
```

这样做的边界很清晰：

- `lidar_localization_ros2` 仍然发布 `map->odom`
- ScanContext 不发布 TF
- ScanContext 默认不直接发布 `/initialpose`
- `scancontext_to_initialpose` 是唯一把 ScanContext 候选喂给 NDT 的桥
- recovery 是否成功由 NDT 状态二次验证

### 2.2 为什么不用 ScanContext 直接发布 TF

ScanContext 的 `best_pose` 当前在 Fast-LIO `camera_init` 坐标系中，而 Nav2/NDT 使用 ROS `map` 坐标系。直接发布 TF 风险很高：

- Fast-LIO 原始轴不是标准 ROS 轴：`x=left, y=down, z=backward`
- ScanContext 单次匹配在相似走廊区域可能有错误候选
- 直接改 `map->odom` 会导致机器人在 RViz/Nav2 中瞬移

所以当前设计是：

```
ScanContext best_pose(camera_init)
    ↓ 坐标转换 R_FASTLIO_TO_ROS
/initialpose(map)
    ↓
NDT 自己验证并发布 map->odom
```

---

## 三、文件改动清单

### 3.1 新增/接入文件

| 文件 | 作用 |
|---|---|
| `src/humanoid_scancontext_global_localization/` | 新增 ScanContext sidecar 全局定位包 |
| `src/humanoid_scancontext_global_localization/src/scancontext_global_core.cpp` | Scan Context 描述子、数据库加载、候选搜索 |
| `src/humanoid_scancontext_global_localization/src/scancontext_global_localizer_node.cpp` | 在线 sidecar 节点，订阅 cloud/odom，提供 trigger 服务 |
| `src/humanoid_scancontext_global_localization/scripts/build_scancontext_database.py` | 从 bag 构建 ScanContext 数据库 |
| `src/humanoid_scancontext_global_localization/scripts/offline_bag_nav_validation.py` | 离线 bag 验证脚本 |
| `src/humanoid_scancontext_global_localization/scripts/record_scancontext_validation.py` | 在线触发记录脚本 |
| `src/humanoid_scancontext_global_localization/config/scancontext_global_localization.yaml` | ScanContext sidecar 默认参数 |
| `src/humanoid_navigation2/humanoid_navigation2/scancontext_to_initialpose.py` | ScanContext best_pose 转 `/initialpose` 的桥接节点 |
| `src/humanoid_navigation2/maps/hall_sc_fastlio_registered.bin` | 当前导航使用的 ScanContext 数据库 |

### 3.2 修改/接入位置

| 文件 | 改动 |
|---|---|
| `src/humanoid_navigation2/launch/navigation2.launch.py` | 接入 ScanContext sidecar 和 `scancontext_to_initialpose` |
| `src/humanoid_navigation2/package.xml` | 增加 `humanoid_scancontext_global_localization` 依赖 |
| `src/humanoid_navigation2/setup.py` | 注册 `scancontext_to_initialpose` entry point |

### 3.3 本次顺手修复

| 文件 | 改动 |
|---|---|
| `src/humanoid_navigation2/humanoid_navigation2/localization_odom_fusion.py` | 删除重复残留的 `QoSProfile(...)` 两行，修复 Python `IndentationError` |

说明：这个修复属于 odom 短期接管 fusion 路线的可运行性修复，不属于 ScanContext context 恢复主逻辑。

---

## 四、核心组件详解

### 4.1 scancontext_global_localizer_node

**包**: `humanoid_scancontext_global_localization`  
**节点**: `scancontext_global_localizer`

职责：

1. 订阅 `/fast_lio/cloud_registered`
2. 订阅 `/odom`
3. 将 registered cloud 转回 body-local scan
4. 计算 Scan Context 描述子
5. 在数据库中查找候选 keyframe
6. 对候选做 odom gate / ambiguity gate / GICP refinement
7. 发布候选状态和最佳位姿
8. 提供普通 trigger 和 global trigger 服务

关键 topic / service：

| 名称 | 类型 | 说明 |
|---|---|---|
| `/scancontext_global_localization/candidates` | topic | 候选状态 |
| `/scancontext_global_localization/best_pose` | topic | 最佳候选位姿 |
| `/scancontext_global_localization/trigger` | service | odom-gated 普通触发 |
| `/scancontext_global_localization/trigger_global` | service | 全图恢复触发 |

关键安全参数：

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `sc_distance_threshold` | `0.25` | Scan Context 距离阈值 |
| `max_odom_consistency_distance` | `1.0` | 普通触发 odom gate |
| `min_sc_distance_gap` | `0.03` | 候选置信差距 |
| `max_ambiguous_candidate_distance` | `2.0` | 歧义候选距离 |
| `gicp_fitness_threshold` | `0.6` | 普通 GICP 阈值 |
| `global_recovery_gicp_fitness_threshold` | `0.20` | 全局恢复 GICP 阈值 |
| `global_recovery_required_consistent_results` | `3` | 全局恢复多次一致性 |
| `global_recovery_consistency_window` | `6` | 一致性窗口 |
| `global_recovery_max_refined_odom_consistency_distance` | `0.0` | 全局恢复关闭 odom gate |

### 4.2 scancontext_to_initialpose

**文件**: `src/humanoid_navigation2/humanoid_navigation2/scancontext_to_initialpose.py`

职责：

1. 监听 `/scancontext_global_localization/best_pose`
2. 监听 `/pcl_pose` 是否 stale
3. 监听 `/localization/ndt_status`
4. 监听 `/localization/recovery_requests`
5. 按状态调用 `/trigger` 或 `/trigger_global`
6. 将 Fast-LIO `camera_init` pose 转成 ROS `map` pose
7. 多次发布 `/initialpose`
8. 等待 NDT 连续 accepted
9. 如果 NDT rejected，重新触发恢复

关键参数：

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `global_recovery_after_attempts` | `3` | 普通触发失败 3 次后进入 global trigger |
| `startup_trigger_period_sec` | `2.0` | 启动阶段触发周期 |
| `runtime_trigger_period_sec` | `8.0` | 运行期触发周期 |
| `startup_duration_sec` | `30.0` | 启动阶段持续时间 |
| `localization_pose_stale_sec` | `2.5` | `/pcl_pose` stale 判定 |
| `recovery_settle_sec` | `6.0` | 发布 initialpose 后等待 NDT 验证 |
| `ndt_recovery_required_stable_status_count` | `3` | NDT 连续 accepted 次数 |
| `ndt_rejected_recovery_count` | `2` | NDT rejected 后重试阈值 |
| `publish_repetitions` | `8` | `/initialpose` 重复发布次数 |
| `min_publish_interval_sec` | `12.0` | 防止频繁发布 initialpose |

### 4.3 NDT 主定位节点

**文件**: `src/humanoid_navigation2/launch/navigation2.launch.py`

当前 NDT 仍是主定位：

```text
package: lidar_localization_ros2
executable: lidar_localization_node
topic: /fast_lio/cloud_registered -> /cloud
status: /localization/ndt_status
```

ScanContext 发布 `/initialpose` 后，NDT 自己决定是否接受、跟踪、发布 `map->odom`。

---

## 五、完整逻辑流程

### 5.1 启动阶段

```
1. rslidar + Fast-LIO 启动
      ↓
2. map_server 加载 2D map
      ↓
3. ScanContext sidecar 启动
      - 加载 hall_sc_fastlio_registered.bin
      - 加载 hall.pcd
      - 订阅 /fast_lio/cloud_registered 和 /odom
      ↓
4. NDT lidar_localization 启动
      ↓
5. scancontext_to_initialpose 启动
      - 因 /pcl_pose 还没有更新，进入 startup recovery
      - 每 2s 触发 ScanContext
      ↓
6. ScanContext 找到候选后发布 /best_pose
      ↓
7. scancontext_to_initialpose 转换坐标并发布 /initialpose 8 次
      ↓
8. NDT 接收 initialpose 并开始 scan-to-map
      ↓
9. /localization/ndt_status 连续 accepted 3 帧
      ↓
10. recovery_status 发布 localization_recovered
```

### 5.2 正常导航

```
NDT 正常发布 /pcl_pose 和 map->odom
    ↓
scancontext_to_initialpose 监控 /pcl_pose
    ↓
如果 /pcl_pose 没有 stale，不触发恢复
    ↓
ScanContext sidecar 保持待命，不发布 TF
```

### 5.3 运行中定位失效

```
/pcl_pose stale > 2.5s
    或
/localization/recovery_requests 收到外部 context 请求
    ↓
scancontext_to_initialpose start_recovery()
    ↓
先调用 /scancontext_global_localization/trigger
    ↓
如果普通 trigger 多次无法恢复
    ↓
第 3 次以后调用 /scancontext_global_localization/trigger_global
    ↓
best_pose → /initialpose
    ↓
等待 NDT 验证
```

### 5.4 NDT 验证成功

```
ScanContext 发布 /initialpose
    ↓
NDT scan-to-map accepted
    ↓
/localization/ndt_status state=accepted 连续 3 帧
    ↓
scancontext_to_initialpose 发布 localization_recovered
    ↓
recovery_active=false
```

### 5.5 NDT 验证失败

```
ScanContext 发布 /initialpose
    ↓
NDT rejected
    ↓
rejected_count 达到阈值
    ↓
scancontext_to_initialpose 清空 pending initialpose
    ↓
发布 localization_relocalize_failed
    ↓
重新 start_recovery()
    ↓
直接提高到 global recovery 尝试
```

---

## 六、与 odom 短期接管 fusion 方案的区别

| 维度 | ScanContext context 恢复 | Odom 短期接管 fusion |
|---|---|---|
| 核心目标 | 失锁后给 NDT 提供全局初始位姿 | NDT 漂移时先冻结 `map->odom`，短期靠 odom 继续导航 |
| 是否发布 TF | 不发布 TF | DEGRADED 时发布冻结的 `map->odom` |
| 主定位 | NDT | NDT + fusion 接管 |
| 触发时机 | `/pcl_pose` stale、外部 recovery request、启动缺定位 | NDT matching_error > 0.5 连续 2 帧 |
| 恢复方式 | ScanContext -> `/initialpose` -> NDT 验证 | NDT 自然恢复或 LOST 后请求 HDL/global recovery |
| 风险 | 全局相似区域误匹配 | TF 竞争、冻结坐标下游一致性、长时间 odom 漂移 |
| 当前可运行性 | 已接入 navigation2.launch.py | 已修复语法错误，但仍需实机 fusion bag 验证 |

两条路线可以组合：

```
短时 NDT 异常:
    fusion 冻结 map->odom，odom 兜底

fusion 兜不住进入 LOST:
    发布 /localization/recovery_requests
    ScanContext / HDL 接手重定位
```

---

## 七、Bag 验证结果

### 7.1 验证环境

| 项 | 值 |
|---|---|
| Bag | `/home/ubuntu/nav_drift_test2` |
| Database | `src/humanoid_navigation2/maps/hall_sc_fastlio_registered.bin` |
| Waypoints | `data/dynamic_waypoints.json` (`latest`) |
| 抽样周期 | 2.0s |
| 样本数 | 753 |

### 7.2 ScanContext 离线匹配结果

| 指标 | 结果 |
|---|---:|
| SC accepted | 488/753 (64.8%) |
| Odom gate rejected | 264/753 (35.1%) |
| Ambiguity gate rejected | 0/753 (0.0%) |
| Odom route distance median / p95 | 0.047 / 0.460 m |
| SC route distance median / p95 | 0.591 / 1.031 m |
| SC-vs-odom error median / p95 | 0.904 / 0.983 m |
| SC distance median / p95 | 0.129 / 0.169 |
| Rescue-possible samples | 0 |

说明：`nav_drift_test2` 这包里 odom 本身没有明显跑出路线，所以不是一个天然的真实失锁恢复包；恢复成功率主要通过人为扰动仿真评估。

### 7.3 恢复成功率估计

基于 `debug_monitor/nav_drift_test2_scancontext_rerun_20260525_latest/samples.csv`：

| 指标 | 结果 |
|---|---:|
| Conservative success | 488/753 (64.8%) |
| Global additional success | 3/753 (0.4%) |
| Total estimated success | 491/753 (65.2%) |
| Estimated wrong global publish | 237/753 (31.5%) |
| No recovery publish | 25/753 (3.3%) |

重要限制：这里的 wrong global publish 是离线上界信号，没有真实执行在线 GICP 和 NDT 验证，不能直接等价为实机错误发布率。

### 7.4 随机漂移恢复仿真

随机 12 个点，假设 NDT 接受 `/initialpose` 的 XY 误差阈值为 1.0m：

| 指标 | 结果 |
|---|---:|
| Trials | 12 |
| Success | 8/12 |
| Success rate | 66.7% |

失败样例：

| 起点附近 | 结果 | 说明 |
|---|---|---|
| 点位2 | fail | 最终 global 候选更接近点位3，误差 1.236m |
| 点位4 | fail | 最终跳到点位24 附近，误差 17.579m |
| 点位6 | fail | 最终跳到点位9 附近，误差 11.184m |
| 点位9 | fail | 最终误差 2.281m |

如果把 NDT 接受阈值放宽到 2.0m：

| 指标 | 结果 |
|---|---:|
| Trials | 12 |
| Success | 9/12 |
| Success rate | 75.0% |

放宽阈值会提高成功率，但点位2这类“最终候选接近邻近点位”的情况可信度一般，不建议只靠放宽阈值解决。

---

## 八、当前结论

### 8.1 已经成立的部分

1. `navigation2.launch.py` 已经启用 ScanContext context 恢复链路。
2. ScanContext sidecar 不直接发布 TF，架构上比较安全。
3. `/initialpose` 发布后需要 NDT 连续 accepted，避免单次候选直接污染 `map->odom`。
4. 启动恢复、运行期 `/pcl_pose` stale 恢复、外部 `/localization/recovery_requests` 都已经有入口。
5. 普通 trigger 和 global trigger 分离，普通恢复保守，global 恢复更严格依赖多帧一致性和 GICP。

### 8.2 风险和短板

1. test2bag 上恢复成功率约 65%~75%，不能算高可信。
2. 点位4、点位6、点位9 一带存在明显误匹配风险。
3. 全局恢复如果关闭 odom gate，必须依赖 GICP、多帧一致性和 NDT 验证，否则会有错误候选风险。
4. `nav_drift_test2` 不是理想的真实失锁恢复验证包，因为 odom route 没有明显出界。
5. 目前结果更适合说明“能辅助恢复”，不能证明“可无条件自动全局恢复”。

### 8.3 推荐使用策略

当前建议：

```
启动定位:
    允许 ScanContext 自动提供 initialpose

运行中短暂异常:
    先依赖 NDT 自身和状态监控

运行中失锁:
    允许 ScanContext context recovery 触发
    但必须保留 NDT accepted 验证

全图恢复:
    只在普通 trigger 多次失败后启用
    必须保留 GICP + 多帧一致性 + NDT accepted
```

不建议：

```
ScanContext 单次候选直接发布 map->odom
ScanContext 单次候选绕过 NDT 验证
全局恢复关闭所有 gate 后立即接受
```

---

## 九、运行和验证命令

### 9.1 启动当前导航

```bash
ros2 launch humanoid_navigation2 navigation2.launch.py use_sim_time:=false
```

### 9.2 查看节点

```bash
ros2 node list | grep scancontext
ros2 topic echo /scancontext_global_localization/candidates
ros2 topic echo /scancontext_global_localization/best_pose
ros2 topic echo /localization/recovery_status
```

### 9.3 手动触发普通 ScanContext

```bash
ros2 service call /scancontext_global_localization/trigger std_srvs/srv/Trigger {}
```

### 9.4 手动触发全局恢复

```bash
ros2 service call /scancontext_global_localization/trigger_global std_srvs/srv/Trigger {}
```

### 9.5 离线 bag 验证

```bash
source install/setup.bash
python3 src/humanoid_scancontext_global_localization/scripts/offline_bag_nav_validation.py \
  --bag /home/ubuntu/nav_drift_test2 \
  --database /home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps/hall_sc_fastlio_registered.bin \
  --waypoints /home/ubuntu/humanoid_ws/data/dynamic_waypoints.json \
  --waypoint-group latest \
  --output-dir debug_monitor/nav_drift_test2_scancontext_rerun_20260525_latest \
  --sample-period 2.0 \
  --workers 2 \
  --odom-gate-distance 1.0 \
  --sc-threshold 0.25 \
  --enable-candidate-confidence-gate \
  --min-sc-distance-gap 0.03 \
  --max-ambiguous-candidate-distance 2.0
```

### 9.6 随机漂移恢复仿真

```bash
python3 debug_monitor/simulate_scancontext_drift_recovery.py \
  --samples debug_monitor/nav_drift_test2_scancontext_rerun_20260525_latest/samples.csv \
  --output-dir debug_monitor/nav_drift_test2_simulated_drift_recovery_latest_20260525 \
  --seed 20260525 \
  --count 12 \
  --min-spacing 45 \
  --max-waypoint-dist 1.5 \
  --global-after-attempts 3 \
  --max-attempts 5 \
  --retry-stride 3 \
  --ndt-accept-error 1.0 \
  --global-window 6 \
  --global-required 3 \
  --global-xy-tolerance 0.8 \
  --global-sc-threshold 0.25
```

---

## 十、后续建议

1. 用真正开启 ScanContext context recovery 的实机导航录一包，验证 `/initialpose` 发布后 NDT 是否稳定 accepted。
2. 对点位4、点位6、点位9 附近单独采样，检查候选混淆来源。
3. 全局恢复不要继续放宽 gate，优先增加多帧一致性和在线 GICP/NDT 验证。
4. 如果要和 odom fusion 组合，建议由 fusion 在 LOST 后发布 `/localization/recovery_requests`，ScanContext/HDL 负责恢复，不要让两边同时抢 `map->odom`。
5. 长期更稳的结构是：NDT 或 fusion 独占 `map->odom`，所有全局定位模块只发布候选和 `/initialpose`。
