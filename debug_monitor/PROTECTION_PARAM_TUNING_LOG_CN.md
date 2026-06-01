# prior-map bridge 大跳保护参数测试记录

这个文档用于累计多次 bag 分析后的保护参数建议。每次分析完一组完整导航 bag，都在这里追加一条记录。等累计 5~6 组后，再根据重复出现的问题确定最终实机参数。

## 固定记录格式

每次记录应包含：

- bag 名称和路径
- 分析时间
- 真实导航结果
- 真实最大跳变
- shadow 模拟保护后的最大跳变
- 保护冻结段数量、总时长、冻结期间 odom 位移
- 是否触发 degraded
- 是否发生导航暂停/失败
- 推荐参数
- 本次参数依据
- 下次重点观察点

## 参数含义

```yaml
jump_protection_mode: protect
```

大跳保护总开关：

- `off`：关闭保护，沿用原逻辑。
- `monitor`：只记录 WOULD_HOLD / WOULD_PENDING，不实际拦截。
- `protect`：真实启用保护。

```yaml
max_small_correction_translation: 0.25
max_small_correction_yaw: 0.12
```

小修正阈值。小于这个范围认为是正常地图约束修正，直接接受。

```yaml
nav_medium_correction_translation: 0.50
nav_medium_correction_yaw: 0.20
nav_medium_required_frames: 5
```

导航中的中等修正阈值。`0.25m~0.50m` 的修正不直接跳过去，必须连续稳定 5 帧才接受。

```yaml
nav_large_correction_translation: 0.50
nav_large_correction_yaw: 0.20
allow_nav_large_jump: false
```

导航中的大跳保护。超过 `0.50m` 或 `0.20rad` 时，默认冻结 `map->odom` 更新，只继续发布 last good TF。

```yaml
idle_large_correction_translation: 1.00
idle_large_correction_yaw: 0.35
idle_large_required_frames: 5
allow_idle_large_jump: true
```

空闲/到点讲解阶段的回正策略。导航不 active 时，允许 1m 内的稳定回正，避免大跳保护导致永远回不到正确位姿。

```yaml
hard_reject_translation: 1.00
hard_reject_yaw: 0.50
large_jump_degraded_after_sec: 3.0
```

绝对保护和退化阈值。超过 1m 不自动接受；冻结超过 3s 后发布 degraded，后续应交给状态管理器决定是否暂停/等待/重定位。

## 待比较候选策略

这里记录尚未确定、需要多组 bag 验证的策略。后续每次分析 bag 时，要把这些候选策略纳入比较。

### 候选策略 A：保守 odom 兜底

这是当前更稳妥的建议。思路是：允许 bridge 短时间冻结 `map->odom`，但一旦靠 odom 走得过久或过远，就让状态管理器暂停导航，等待定位恢复。

```yaml
large_jump_degraded_after_sec: 3.0

localization_hold_warn_sec: 3.0
localization_hold_warn_odom_distance: 0.50

localization_hold_pause_sec: 6.0
localization_hold_pause_odom_distance: 1.00

localization_hold_force_relocalize_sec: 10.0
localization_hold_force_relocalize_odom_distance: 1.50
```

判断口径：

- 冻结 `<3s` 且 odom `<0.5m`：允许继续走。
- 冻结 `3~6s` 或 odom `0.5~1.0m`：进入 degraded/watch，但不一定马上停车。
- 冻结 `>6s` 或 odom `>1.0m`：建议暂停导航。
- 冻结 `>10s` 或 odom `>1.5m`：触发重定位/恢复流程。

适用原因：

- test20 中大多数冻结能在 `0.1~0.6s` 内恢复，保守策略不会误伤。
- test21 中点位16/17 出现 `7~10s` 长冻结，保守策略会及时要求状态管理器介入。

### 候选策略 B：放宽 odom 兜底

这是用户提出的思路：目前 test20/test21 的 `/odom` 轨迹看起来连续正常，没有明显 odom 自身跳飞，因此可以考虑允许 `odom` 兜底走更长时间/距离，例如 `2~3m` 或 `30s`，避免过早暂停导航。

```yaml
large_jump_degraded_after_sec: 3.0

localization_hold_warn_sec: 6.0
localization_hold_warn_odom_distance: 1.00

localization_hold_pause_sec: 30.0
localization_hold_pause_odom_distance: 2.00

localization_hold_force_relocalize_sec: 45.0
localization_hold_force_relocalize_odom_distance: 3.00
```

判断口径：

- 冻结 `<6s` 且 odom `<1.0m`：允许继续走。
- 冻结 `6~30s` 或 odom `1.0~2.0m`：进入 degraded/watch，但仍允许 odom 兜底。
- 冻结 `>30s` 或 odom `>2.0m`：再暂停导航。
- 冻结 `>45s` 或 odom `>3.0m`：触发重定位/恢复流程。

需要验证的问题：

- odom 连续不等于全局准确，长时间兜底可能让机器人真实位置和 map 目标逐渐偏离。
- 如果 prior-map 长时间不恢复，Nav2 会一直基于旧 `map->odom + odom` 运行。
- test21 点位16/17 已经出现 `1m+` odom 兜底，如果放宽到 `2~3m / 30s`，需要验证是否仍能安全到点。

后续比较指标：

- 每个 bag 中冻结期间 odom 最大位移。
- 每个 bag 中冻结最长时间。
- 冻结期间是否仍在发 `/cmd_vel`。
- 是否最终在导航中恢复，还是只能到点/空闲后恢复。
- 是否发生路径明显偏移、局部规划失败、目标失败、机器人走偏。
- 放宽策略是否减少不必要暂停，还是增加走偏风险。

---

## 记录 1：nav_drift_test20

### 基本信息

- bag 名称：`nav_drift_test20`
- bag 路径：`/home/ubuntu/nav_drift_test/nav_drift_test20/nav_drift_test20_0.mcap`
- 日志路径：`/home/ubuntu/humanoid_ws/debug_output.txt`
- 分析时间：`2026-05-31 11:12 Asia/Shanghai`
- 分析报告：`debug_monitor/nav_drift_test20_analysis/nav_drift_test20_protection_analysis_cn.md`

### 真实导航结果

- 共 25 个点位。
- 25 个点位全部完成。
- 未发现 `Goal failed`。
- 未发现导航 `paused`。
- 未发现 TF 断裂。
- 当前真实运行中 `jump_protection_mode=monitor`，所以没有真实触发保护冻结。

### 真实定位表现

- 实际记录最大 `map->odom` 单次跳变：`0.864m`。
- 大跳主要集中在：
  - 点位18：导航中最大跳变 `0.783m`
  - 点位19：导航中最大跳变 `0.843m`
  - 点位20：导航中最大跳变 `0.864m`
  - 点位21：导航中最大跳变 `0.573m`
- 点位23 实际导航中最大跳变只有 `0.353m`，但 shadow 重新计算候选时出现过 `0.849m` 级候选，保护会冻结该候选。

### 第一版保护 shadow 模拟结果

- 保护后最大 `map->odom` 跳变：`0.483m`。
- 导航中 `>=0.5m` 大跳接受次数：`0`。
- 保护冻结候选次数：`116`。
- 聚类冻结段数：`6`。
- 冻结总时长：`11.032s`。
- 冻结期间 odom 总位移：`3.115m`。
- degraded 事件：`0`。
- 模拟不会主动暂停导航。
- 模拟会出现短时间 odom 兜底推进。

### 保护冻结重点段

| 点位 | 时间段 | 持续s | 最大候选跳变m | 冻结期间 odom 位移m | 说明 |
|---|---:|---:|---:|---:|---|
| 点位18 | `1780196355.855-1780196358.150` | `2.30` | `0.856` | `0.78` | 拦住 0.8m 级导航中大跳，未超过 degraded 阈值。 |
| 点位19 | `1780196414.349-1780196415.155` | `0.81` | `0.843` | `0.42` | 冻结时间短，风险较低。 |
| 点位20 | `1780196470.862-1780196472.261` | `1.40` | `0.864` | `0.46` | 拦住本包最大跳变。 |
| 点位21 | `1780196483.853-1780196485.360` | `1.51` | `0.742` | `0.20` | 第一段冻结。 |
| 点位21 | `1780196490.368-1780196492.083` | `1.72` | `0.587` | `0.28` | 第二段冻结。 |
| 点位23 | `1780196516.849-1780196520.159` | `3.31` | `0.849` | `0.97` | 最接近 degraded 边界，后续重点观察。 |

### 本次推荐参数

基于 test20，本次建议先不要激进收紧，保持第一版参数：

```yaml
jump_protection_mode: protect

max_small_correction_translation: 0.25
max_small_correction_yaw: 0.12

nav_medium_correction_translation: 0.50
nav_medium_correction_yaw: 0.20
nav_medium_required_frames: 5

nav_large_correction_translation: 0.50
nav_large_correction_yaw: 0.20
allow_nav_large_jump: false

idle_large_correction_translation: 1.00
idle_large_correction_yaw: 0.35
idle_large_required_frames: 5
allow_idle_large_jump: true

hard_reject_translation: 1.00
hard_reject_yaw: 0.50
large_jump_degraded_after_sec: 3.0
```

### 推荐理由

1. `nav_large_correction_translation=0.50`

   test20 中真实风险集中在 `0.57m~0.86m` 的导航中修正。0.50m 能准确覆盖点位18、19、20、21的大跳，同时不会影响大多数正常点位。

2. `allow_nav_large_jump=false`

   test18 曾经暴露过“稳定但错误的局部最优”，仅靠连续帧确认不够。因此导航 active 时不建议自动接受 `>0.50m` 的大跳。

3. `nav_medium_required_frames=5`

   3 帧对稳定错误局部最优不够，5 帧更稳。test20 中 `0.25m~0.50m` 的中等修正没有导致失败，暂时不需要进一步加到 7 帧。

4. `idle_large_correction_translation=1.00`

   大跳保护不能永久拒绝回正。test20 shadow 中有 10 次 idle 回正，范围约 `0.255m~0.404m`，说明空闲阶段允许稳定回正是必要的。

5. `large_jump_degraded_after_sec=3.0`

   点位23 shadow 冻结约 `3.31s`，已经贴近边界。暂时不建议加大到 5s，否则 odom 兜底距离可能过长；也暂时不建议降到 2s，否则点位18这类 2.30s 的短冻结可能过早触发状态管理器。

### 风险点

- 开启 `protect` 后，冻结期间不会暂停导航，机器人会短时间靠 odom 继续推进。
- test20 中冻结期间 odom 累计走了 `3.115m`，其中点位23 单段接近 `0.97m`。
- 如果后续真实开启 `protect`，但状态管理器还不处理 degraded，则长冻结场景仍有风险。
- 点位23 是下一轮重点观察对象：如果多次都出现 `>3s` 冻结，应考虑接入状态管理器暂停逻辑，或单独调大/调小 degraded 策略。

### 下次测试重点

1. 观察点位18、19、20、21是否重复出现 `>0.50m` 导航中大跳。
2. 观察点位23是否重复出现 `>3s` 冻结。
3. 统计冻结期间 odom 位移是否经常超过 `0.50m`。
4. 如果连续多包中 `>3s` 冻结很少，可以考虑先实机小范围开启 `protect`。
5. 如果连续多包中冻结期间 odom 位移经常超过 `0.8m~1.0m`，应优先接 navigation_state_manager 的暂停/等待/恢复逻辑。

---

## 记录 2：nav_drift_test21

### 基本信息

- bag 名称：`nav_drift_test21`
- bag 路径：`/home/ubuntu/nav_drift_test/nav_drift_test21/nav_drift_test21_0.mcap`
- 日志路径：`/home/ubuntu/humanoid_ws/debug_output.txt`
- 分析时间：`2026-05-31 12:58 Asia/Shanghai`
- 分析报告：`debug_monitor/nav_drift_test21_analysis/nav_drift_test21_protection_analysis_cn.md`
- 数据备注：该 mcap 没有 `metadata.yaml` / 消息索引，rosbag2 按文件顺序读取。

### 真实导航结果

- 有效完成点位：点位1~点位17，共 17 个。
- 未发现 `Goal failed`。
- 日志中存在用户暂停/恢复事件：
  - 点位2 前后暂停/恢复两次。
  - 点位8 前暂停/恢复一次。
  - 点位9 前暂停/恢复一次。
  - 点位14 前暂停/恢复一次。
- 未发现 TF 断裂。
- 当前真实运行仍是 `jump_protection_mode=monitor`，没有真实触发 protect 冻结。

### 真实定位表现

- 实际记录最大 `map->odom` 单次跳变：`1.884m`。
- 当前逻辑 shadow 最大跳变：`2.179m`。
- 大跳主要集中在：
  - 点位15：导航中最大跳变 `0.561m`
  - 点位17：导航中最大跳变 `1.884m`
- 点位16 真实导航中最大跳变 `0.253m`，但当前逻辑 shadow 中会出现 `1.414m` 级导航中候选。

### 第一版保护 shadow 模拟结果

- 保护后最大已接受 `map->odom` 跳变：`0.843m`。
- 导航中 `>=0.5m` 大跳接受次数：`0`。
- 保护冻结候选次数：`249`。
- degraded 事件：`76`。
- 聚类冻结段数：`4`。
- 冻结总时长：`24.723s`。
- 冻结期间 odom 总位移：`4.335m`。
- 结论：test21 中 bridge-only protect 风险明显高于 test20，会产生长时间 odom 兜底。

### 保护冻结重点段

| 点位 | 时间段 | 持续s | 最大候选跳变m | degraded次数 | 冻结期间 odom 位移m | 说明 |
|---|---:|---:|---:|---:|---:|---|
| 点位12 | `1780201529.319-1780201533.627` | `4.31` | `0.785` | `14` | `1.70` | 已超过 3s，bridge-only 会继续靠 odom 走。 |
| 点位15 | `1780201763.833-1780201767.446` | `3.61` | `0.710` | `7` | `0.47` | 超过 degraded 阈值，但 odom 位移尚可控。 |
| 点位16 | `1780201844.335-1780201854.149` | `9.81` | `1.553` | `46` | `1.02` | 高风险，必须考虑暂停/等待恢复。 |
| 点位17 | `1780201956.250-1780201963.238` | `6.99` | `2.179` | `9` | `1.14` | 高风险，保护能拦住大跳，但不能让机器人一直靠 odom 走。 |

### 本次推荐参数

参数本身暂时仍建议保持第一版，不建议为了 test21 直接放宽：

```yaml
jump_protection_mode: protect

max_small_correction_translation: 0.25
max_small_correction_yaw: 0.12

nav_medium_correction_translation: 0.50
nav_medium_correction_yaw: 0.20
nav_medium_required_frames: 5

nav_large_correction_translation: 0.50
nav_large_correction_yaw: 0.20
allow_nav_large_jump: false

idle_large_correction_translation: 1.00
idle_large_correction_yaw: 0.35
idle_large_required_frames: 5
allow_idle_large_jump: true

hard_reject_translation: 1.00
hard_reject_yaw: 0.50
large_jump_degraded_after_sec: 3.0
```

但 test21 给出的结论比 test20 更明确：**不能只开 bridge protect，还需要状态管理器处理 degraded。**

### 推荐理由

1. 不建议放宽 `nav_large_correction_translation`

   test21 中点位17 出现 `1.884m~2.179m` 级候选。如果放宽导航中大跳接受，会重新引入明显跳位风险。

2. 不建议打开 `allow_nav_large_jump`

   test18 已证明“连续稳定”不等于“正确”。test21 点位16/17 的大候选也说明导航中不能自动跳过去。

3. 不建议把 `large_jump_degraded_after_sec` 调大

   点位16 已经冻结 `9.81s`，如果把 degraded 时间调大，只会让系统更晚发现风险。

4. 可以考虑未来新增“冻结 odom 位移阈值”

   test21 中点位12、16、17 冻结期间 odom 都超过 `1.0m` 左右。仅靠时间阈值不够，建议状态管理器增加：

   ```yaml
   localization_hold_max_odom_distance: 0.50
   localization_hold_pause_after_sec: 3.0
   ```

   也就是冻结超过 3s 或冻结期间 odom 位移超过 0.5m，就暂停导航等待恢复。

### 风险点

- test21 中 protect shadow 会触发 `76` 次 degraded。
- 点位16/17 的冻结时间太长，bridge-only 会让机器人靠 odom 兜底 `1m+`。
- 如果真实完整开启 protect，但状态管理器不处理 degraded，可能不会 TF 断裂，但会出现“定位没更新，导航继续走”的风险。

### 下次测试重点

1. 继续观察点位16、17是否重复出现 `>1m` 候选跳变。
2. 统计 degraded 是否稳定集中在后半段点位。
3. 验证状态管理器接入 degraded 后，是否能暂停、等待回正、恢复导航。
4. 观察暂停后 prior-map localization 是否能在空闲窗口通过 `idle_large_correction` 找回。
5. 如果接入状态管理器后仍长时间找不回，再考虑更强的重定位流程，而不是放宽 bridge 接受阈值。
