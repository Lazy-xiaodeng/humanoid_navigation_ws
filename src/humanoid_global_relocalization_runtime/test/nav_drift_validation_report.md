# nav_drift_test 全局重定位验证报告

## 验证范围

本报告记录 `humanoid_global_relocalization_runtime` 第一阶段在 `/home/ubuntu/nav_drift_test` 下的实测结果，并补充一个可用外部 bag 的扩展验证。当前只改动并使用当前工作空间：

```text
/home/ubuntu/software/Todesk/Files/humanoid_ws
```

未修改 `/home/ubuntu/humanoid_ws`。

## bag 与话题检查

本轮优先验证以下 bag：

```text
/home/ubuntu/nav_drift_test/nav_drift_test43
/home/ubuntu/nav_drift_test/nav_drift_test44
/home/ubuntu/nav_drift_test/nav_drift_test45
```

三组 bag 均包含：

```text
/fast_lio/cloud_registered
/odom
/robot_realpose
```

三组 bag 均不包含：

```text
/cloud_registered_body
```

因此本轮实测输入模式为 `registered_world`，也就是读取 `/fast_lio/cloud_registered`，再用同一时刻 `/odom` 反变换回 raw body，最后转换到 ROS 标准 `base_footprint` 轴。`body` 模式代码和配置已经保留，但需要后续补充带 `/cloud_registered_body` 的 bag 后才能做同等验证。

为了让 bag 话题检查可复现，新增脚本：

```text
src/humanoid_global_relocalization_runtime/test/scan_bag_inventory.py
```

该脚本递归读取 rosbag2 `metadata.yaml`，输出 CSV 和 Markdown。当前扫描命令：

```bash
python3 src/humanoid_global_relocalization_runtime/test/scan_bag_inventory.py \
  --root /home/ubuntu/nav_drift_test \
  --root /home/ubuntu/fast-lio-bags \
  --root /home/ubuntu/下载/bags \
  --root /home/ubuntu/costmap_clearing_full \
  --root /home/ubuntu/clearing \
  --csv .codex_tmp/bag_inventory_v2.csv \
  --md .codex_tmp/bag_inventory_v2.md
```

扫描结果：

```text
metadata_files: 10
registered_world_with_reference: 4
body_ready: 0
body_with_reference: 0
```

扩展扫描：

```bash
python3 src/humanoid_global_relocalization_runtime/test/scan_bag_inventory.py \
  --root /home/ubuntu \
  --exclude /home/ubuntu/humanoid_ws \
  --exclude /home/ubuntu/software/Todesk/Files/humanoid_ws/.codex_tmp \
  --csv .codex_tmp/bag_inventory_home_scan.csv \
  --md .codex_tmp/bag_inventory_home_scan.md
```

早期扩展结果为 `metadata_files=36`，`registered_world_ready=8`，`registered_world_with_reference=4`，`body_ready=0`，`body_with_reference=0`。其中额外发现的 grid_map 示例 bag 没有全局重定位需要的 `/fast_lio/cloud_registered + /odom`；4 个无 `/robot_realpose` 的 registered_world bag 已做 smoke，但不计入准确率。

为了避免证据矩阵长期依赖旧扫描文件，又新增了 latest inventory 复查文件：

```text
.codex_tmp/bag_inventory_home_scan_latest.csv
.codex_tmp/bag_inventory_nav_drift_latest.csv
```

补录 `nav_drift_test46` 后，当前 latest 复查结果已经变为：排除 `/home/ubuntu/humanoid_ws` 和本包 `.codex_tmp` 后，全 `/home/ubuntu` 为 `metadata_files=32`、`registered_world_ready=4`、`registered_world_with_reference=3`、`body_ready=1`；`/home/ubuntu/nav_drift_test` 当前覆盖 `nav_drift_test44/45/46`，三者均具备 `/fast_lio/cloud_registered + /odom + /robot_realpose`，其中 `nav_drift_test46` 同时包含真实 `/cloud_registered_body`。

无参考位姿 registered_world smoke：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 run humanoid_global_relocalization_runtime global_relocalization_offline_eval \
  --config /home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_global_relocalization_runtime/test/extra_registered_world_no_reference_smoke.yaml
```

当前干净产物为 `.codex_tmp/global_relocalization_extra_no_reference_smoke_v1/global_relocalization_metrics.csv`：4 个无参考 bag、7 个抽样帧、3 类模拟场景，共 21 行，`localized=21/21`，`has_reference=0/21`。该结果只证明输入链路和搜索/精配准流程能跑通，不代表定位准确率。

逐包摘要：

| bag | reg_world | reg_world+ref | body | body+ref | /fast_lio/cloud_registered | /cloud_registered_body | /odom | /robot_realpose |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| /home/ubuntu/clearing | no | no | no | no | 0 | 0 | 0 | 0 |
| /home/ubuntu/costmap_clearing_full | no | no | no | no | 131 | 0 | 0 | 0 |
| /home/ubuntu/fast-lio-bags/hall_mapping | yes | no | no | no | 11493 | 0 | 11500 | 0 |
| /home/ubuntu/fast-lio-bags/hall_mapping_20260605_151228 | yes | no | no | no | 816 | 0 | 816 | 0 |
| /home/ubuntu/fast-lio-bags/hall_mapping_20260605_151705 | yes | no | no | no | 5126 | 0 | 5126 | 0 |
| /home/ubuntu/fast-lio-bags/hall_mapping_20260605_152637 | yes | yes | no | no | 10948 | 0 | 10949 | 3448 |
| /home/ubuntu/nav_drift_test/nav_drift_test43 | yes | yes | no | no | 15045 | 0 | 15045 | 15044 |
| /home/ubuntu/nav_drift_test/nav_drift_test44 | yes | yes | no | no | 25534 | 0 | 25534 | 25533 |
| /home/ubuntu/nav_drift_test/nav_drift_test45 | yes | yes | no | no | 12658 | 0 | 12658 | 12640 |
| /home/ubuntu/下载/bags/hall_mapping | yes | no | no | no | 4500 | 0 | 4500 | 0 |

结论：早期可读 bag metadata 中没有真实 `/cloud_registered_body`，因此先做了合成 body 自洽验证；补录 `nav_drift_test46` 后，真实 body+reference 链路已经具备同场景验证条件。全局只读查找还发现了 `/home/ubuntu/humanoid_ws` 下的 grid_map 示例包和调试输出包，但该目录是正在使用的源码工作空间，本轮没有把它们作为验证输入，也没有对其做任何修改。

`/robot_realpose` 的类型为 `geometry_msgs/msg/PoseWithCovarianceStamped`，本轮用它作为参考位姿。由于 nav_drift_test45 中 `/robot_realpose` 比点云晚约 1.85s 才出现，验证配置使用 `bag_start_frame_skip: 50` 跳过 bag 起始阶段。参考位姿同步使用 bag receive time，而不是只依赖消息 header time。

## 地图与预处理

参与对比的地图包括：

```text
hall.pcd
hall_open3d_grounded.pcd
```

验证中发现，原始 `hall.pcd` 点数少但结构/高度噪声不适合当前 3D-BBS 搜索，部分实验搜索耗时很高且候选质量差。当前推荐使用：

```text
src/humanoid_navigation2/pcd/hall_open3d_grounded.pcd
```

关键预处理结论：

```text
enable_map_z_crop: true
map_min_z: 0.2
map_max_z: 2.5
enable_scan_z_crop: true
scan_min_z: 0.2
scan_max_z: 2.5
```

Z 裁剪非常关键。未裁剪时，地面/天花板/重复平面对 BBS 评分干扰明显，容易把候选推到相似走廊位置；开启地图和 scan 同步 Z 裁剪后，GICP 精配准能稳定把候选拉回真实位置。

## 方法

粗搜索使用 CPU 版 3D-BBS / Branch-and-Bound Scan Matching，整张地图范围内搜索 `map -> base_footprint` 候选，并输出 top-K 粗位姿。

本轮选型的核心判断是：全局重定位不能只依赖局部收敛型定位器。Scan Context/scantext 适合地点召回和闭环候选检索，但单独不能稳定给出可直接重置导航的连续 `map -> base_footprint` 位姿；hdl_localization/NDT 更适合有较好初值时的连续定位修正，面对任意点启动或已经漂移很远的场景，容易受初值和局部极值影响。因此第一阶段直接验证 3D-BBS 这类全局搜索方法，再用 GICP/ICP/NDT 做局部精匹配对比。

精匹配阶段已经支持：

```text
none
icp
gicp
ndt
```

本轮重点结论来自 `none/icp/gicp` 对比。`NDT` 在 broad baseline 中耗时偏高，当前没有成为推荐方案。后续如果要继续验证 NDT，建议单独缩小帧数和地图范围做专项 sweep。

资料来源包括 3D-BBS 官方仓库和论文、Scan Context 官方实现、FAST-LIO 官方仓库以及 hdl_localization 官方仓库。本报告里的最终推荐不是只按论文结论决定，而是按当前 nav_drift bag 的成功率、hard case、资源消耗和在线 smoke 结果共同决定。

## 参数组对比

以下统计只取 `scenario_name=arbitrary_start_no_prior`，避免同一帧的模拟大跳场景重复计数。成功阈值为平移误差小于 `0.80m` 且 yaw 误差小于 `15deg`。

| 参数组 | 成功率 | median 平移误差 | 最大平移误差 | 最大 yaw 误差 | median search | median refine | median total | 最大 total | 峰值 RSS |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| focused / none | 3/15 | 1.101m | - | - | 485.6ms | 0.0ms | 486.3ms | - | - |
| focused / ICP | 13/15 | 0.152m | - | - | 483.6ms | 40.5ms | 511.1ms | - | - |
| focused / GICP | 13/15 | 0.054m | 41.258m | 86.417deg | 554.0ms | 224.5ms | 768.8ms | 2784.1ms | 327.0MB |
| precision / GICP | 15/15 | 0.051m | 0.367m | 3.954deg | 2519.0ms | 2439.7ms | 5133.2ms | 44497.2ms | 432.8MB |
| balanced / GICP | 15/15 | 0.058m | 0.336m | 3.797deg | 1473.8ms | 1598.3ms | 3072.6ms | 12439.6ms | 328.5MB |
| fast_recall / GICP | 14/15 | 0.056m | 5.467m | 176.300deg | 636.2ms | 1705.1ms | 2326.4ms | 5502.7ms | 329.4MB |
| balanced_light / GICP | 14/15 | 0.042m | 10.532m | 3.398deg | 1486.8ms | 1063.8ms | 2504.2ms | 11783.9ms | 329.8MB |
| balanced_iter25 / GICP | 15/15 | 0.058m | 0.336m | 3.797deg | 1557.4ms | 1572.3ms | 3094.5ms | 12412.5ms | 334.8MB |

`balanced_iter25 / GICP` 的最新 CSV 已增加单帧 CPU 增量字段。本轮推荐配置的 median CPU 增量为 `6519.4ms`。由于搜索和精配使用多线程，CPU 增量大于墙钟耗时是正常现象。

## 失败案例分析

`focused_v1` 使用较快的 `1.0m BBS` 和较少 refine 候选，失败 2 帧：

```text
nav_drift_test43: 正确候选在 top-K 后面，max_refine_candidates=5 时没有被精配准到。
nav_drift_test44: 走廊重复结构导致 top10 几乎都在错误区域，1.0m 粗搜索召回/排序不够。
```

`fast_recall_v1` 保留 `1.0m BBS`，把 top-K/refine 增大到 `top30/refine20` 后仍失败 1 帧：

```text
nav_drift_test44 1782179120.592506
平移误差 5.466645m
yaw 误差 176.300147deg
```

这说明 hard case 不是单纯增加 top-K 就能解决，至少需要 `0.75m` 或更细的 BBS 底层分辨率。

`balanced_light_v1` 保留 `0.75m BBS`，但把地图/scan 点云进一步粗化后失败 1 帧：

```text
nav_drift_test45 1782896091.600959
平移误差 10.532247m
```

这说明当前点云密度不能继续粗化到 `map_leaf_size=0.30`、`scan_leaf_size=0.40`，否则 hard case 的几何约束不足。

## 推荐配置

当前推荐使用 `balanced_iter25`：

```text
input_mode: registered_world
registered_world_topic: /fast_lio/cloud_registered
map: hall_open3d_grounded.pcd
map_leaf_size: 0.25
scan_leaf_size: 0.30
enable_map_z_crop: true
map_min_z: 0.2
map_max_z: 2.5
enable_scan_z_crop: true
scan_min_z: 0.2
scan_max_z: 2.5
bbs_min_level_res: 0.75
bbs_max_level: 5
bbs_num_threads: 8
bbs_score_threshold_percentage: 0.12
search_min_xyz: [-35.0, -35.0, -1.0]
search_max_xyz: [35.0, 35.0, 2.0]
search_min_rpy: [-0.05, -0.05, 0.0]
search_max_rpy: [0.05, 0.05, 6.283185307]
top_k: 30
refine_method: gicp
max_refine_candidates: 20
refine_max_iterations: 25
refine_max_correspondence_distance: 1.5
success_translation_thresh: 0.80
success_yaw_thresh_deg: 15.0
```

该配置在当前 3 个 nav_drift bag、15 个抽样帧上达到：

```text
成功率: 15/15
median 平移误差: 0.058m
最大平移误差: 0.336m
最大 yaw 误差: 3.797deg
median total: 3094.5ms
median CPU 增量: 6519.4ms
最大 total: 12412.5ms
峰值 RSS: 334.8MB
```

## nav_drift 30 帧扩展验证

为了避免 15 帧样本过窄，又新增了扩展验证配置：

```text
src/humanoid_global_relocalization_runtime/test/nav_drift_registered_world_extended.yaml
```

该配置保持推荐算法参数不变，只把抽样扩大为每个 bag 10 帧、间隔 250 帧：

```text
bag_start_frame_skip: 50
max_bag_frames: 10
bag_frame_stride: 250
```

扩展验证结果：

```text
总帧数: 30
localized: 30/30
有效参考成功率: 27/30
median 平移误差: 0.060m
median yaw 误差: 0.197deg
median search: 2824.0ms
median refine: 1567.3ms
median total: 4441.4ms
median CPU 增量: 10698.2ms
峰值 RSS: 371.9MB
```

按 bag 拆分：

```text
nav_drift_test43: 8/10 成功，最大失败误差 34.15m。
nav_drift_test44: 10/10 成功。
nav_drift_test45: 9/10 成功，最大失败误差 26.08m。
```

失败帧：

```text
nav_drift_test43 1782119604.896394:
  final=(21.90, 17.37, 68.04deg)
  reference=(-0.55, -8.37, -128.31deg)
  error=34.15m / 163.66deg

nav_drift_test43 1782119654.897294:
  final=(12.95, 18.58, 112.81deg)
  reference=(-1.25, -10.56, -98.99deg)
  error=32.41m / 148.20deg

nav_drift_test45 1782896166.601948:
  final=(5.15, 13.66, 34.72deg)
  reference=(-1.70, -11.50, 122.21deg)
  error=26.08m / 87.50deg
```

候选分析：

```text
nav_drift_test43 1782119604:
  top30 都集中在错误区域；top100 里仍没有足够接近真实位姿的候选。

nav_drift_test43 1782119654:
  top100 中最近候选平移约 2.90m，但 yaw 仍差 61.72deg，GICP 最终仍选到错误区域。

nav_drift_test45 1782896166:
  推荐配置 top30 中有接近真实区域的粗候选，但 GICP fitness 最终选择错误 rank20；
  增大 refine_candidates 到 30 仍然选错，说明单帧 fitness 在重复结构里不可靠。
```

针对 hard frame 的补充消融：

```text
0.5m BBS + top60/refine40:
  nav45 hard frame 仍失败，且单帧 total 约 54.6s，不适合作为推荐方向。

0.75m BBS + top100/refine60:
  nav43 两个 hard frame 仍失败，说明单纯扩大 top-K 不足以解决重复结构歧义。

7 帧累积 scan:
  nav43 1782119604 被救回，误差 0.094m / 4.27deg。
  nav43 1782119654 仍失败，误差 40.07m / 58.23deg。
  nav45 1782896166 仍失败，误差 26.05m / 87.72deg。
```

候选质量特征分析：

新增脚本：

```text
src/humanoid_global_relocalization_runtime/test/analyze_candidate_quality.py
```

该脚本把主指标 CSV 和候选明细 CSV 合并，按每一帧统计：

```text
refined_candidate_rank: 最终被 GICP 选中的粗候选排名。
refine_fitness_score: GICP/NDT/ICP 精配准 fitness。
score_margin: top1 与 topK 最后一名的 BBS 分数差。
top_tie_count: 与 top1 分数完全相同的候选数量。
unique_score_count: topK 中不同 BBS 分数数量。
xy_spread_m: topK 候选在平面位置上的离散程度。
yaw_spread_deg: topK 候选 yaw 的圆形标准差。
nearest_candidate_*: topK 中离参考位姿最近的候选误差和排名。
```

30 帧扩展验证的统计结果：

```text
成功帧 n=27:
  median 平移误差: 0.058m
  median yaw 误差: 0.169deg
  median refined_rank: 8
  median fitness: 0.019
  median xy_spread: 0.506m
  median yaw_spread: 2.007deg
  median nearest_candidate_trans: 0.231m
  median nearest_candidate_yaw: 0.519deg
  median nearest_candidate_rank: 3

失败帧 n=3:
  median 平移误差: 32.412m
  median yaw 误差: 148.199deg
  median refined_rank: 10
  median fitness: 0.057
  median xy_spread: 7.822m
  median yaw_spread: 65.077deg
  median nearest_candidate_trans: 2.897m
  median nearest_candidate_yaw: 61.720deg
  median nearest_candidate_rank: 21
```

这个分析能说明失败帧整体更容易出现候选分散、正确候选排名靠后、fitness 偏差变大的现象，但它不能直接变成一个安全可靠的单帧拒绝阈值：

```text
nav43 1782119604:
  top30 全部在错误区域，BBS 分数高度并列，候选看起来并不“发散”，但实际错到 34m。

nav43 1782119654:
  top100 中虽然出现相对接近真实位置的候选，但 yaw 仍差 61.72deg，GICP 最终仍选错。

nav45 1782896166:
  top30 中有接近真实区域的候选，nearest_candidate_trans 约 0.54m、yaw 约 1.54deg、rank26；
  但单帧 GICP fitness 选择了错误 rank20。
```

因此，`fitness`、`refined_rank`、`候选分散度`、`nearest_candidate_rank` 这些指标可以作为后续候选拒绝和状态机的特征输入，但不能单独承担“是否发布全局重置”的最终判据。真正接入导航前，更稳妥的方向是：

```text
1. 连续多帧分别做全局搜索，不立即重置定位。
2. 对多帧 top-K 候选按空间位置和 yaw 做聚类。
3. 只接受跨时间稳定出现、能被当前 odom 短程运动关系解释的候选簇。
4. 对候选簇再做局部地图重投影、局部 scan 一致性或代价地图可行性检查。
5. 通过状态管理器只在“定位已丢失/漂移明显”时触发重定位，避免正常定位时被单帧假阳性覆盖。
```

多帧 `map->odom` 一致性分析：

为了验证上面的方向是否真的能覆盖当前 hard case，又新增脚本：

```text
src/humanoid_global_relocalization_runtime/test/analyze_temporal_consistency.py
```

该脚本会从原始 bag 读取 `/odom`，把每个 3D-BBS 粗候选 `map->base_footprint` 转成它隐含的：

```text
T_map_odom = T_map_base * inverse(T_odom_base)
```

然后在当前帧前后各 2 个抽样帧的窗口内，统计这个 `map->odom` 假设能被多少帧的 top-K 候选共同支持。该逻辑已经同步做成 C++ 核心模块：

```text
include/humanoid_global_relocalization_runtime/temporal_consistency.hpp
src/temporal_consistency.cpp
```

离线 evaluator 会在 bag 批处理结束后直接输出：

```text
global_relocalization_temporal_consistency.csv
```

Python 脚本保留为独立交叉检查工具。分析参数：

```text
window_before: 2
window_after: 2
xy_gate: 1.0m
yaw_gate: 12.0deg
```

30 帧扩展验证的统计结果：

```text
成功帧 n=27:
  median selected_support_frames: 4
  median best_support_frames: 5

失败帧 n=3:
  median selected_support_frames: 1
  median best_support_frames: 1
```

逐帧关键结论：

```text
nav43 1782119604:
  GICP 选中的错误 rank10 只有 1 帧支持；当前帧 top-K 内也没有更稳定的正确簇。

nav43 1782119654:
  GICP 选中的错误 rank4 只有 1 帧支持；当前帧 top-K 内也没有更稳定的正确簇。

nav45 1782896166:
  GICP 选中的错误 rank20 只有 1 帧支持；
  但 rank26 候选形成 5 帧支持的稳定 map->odom 簇，且前面候选质量分析里它接近真实位姿。
```

这说明多帧一致性至少有两类价值：

```text
1. 对 nav43 两个 hard frame，它可以拒绝“单帧突然出现、跨帧不稳定”的错误重置，避免把导航拉到错误区域。
2. 对 nav45 hard frame，它不仅能拒绝 GICP 选错的 rank20，还有机会从 top-K 中选出跨帧稳定的 rank26 正确簇。
```

因此，后续在线化时不建议只做“单帧结果通过/拒绝”，而应该把 `map->odom` 时序一致性作为重定位状态机的核心模块。

C++ 内置模块回归：

新增回归配置：

```text
src/humanoid_global_relocalization_runtime/test/nav_drift_registered_world_extended_cpp_temporal.yaml
```

输出目录：

```text
.codex_tmp/global_relocalization_nav_drift_extended_cpp_temporal_v1
```

该配置复用 30 帧扩展验证的推荐参数，单独重新跑完整 BBS+GICP。结果：

```text
localized: 30/30
有效参考成功率: 27/30
median 平移误差: 0.060m
median yaw 误差: 0.197deg
median search: 2714.0ms
median refine: 1571.7ms
median total: 4290.0ms
median CPU 增量: 10461.2ms
峰值 RSS: 368.7MB
```

C++ 输出的 `global_relocalization_temporal_consistency.csv` 与 Python 分析结论一致：

```text
成功帧 n=27:
  median selected_support_frames: 4
  median best_support_frames: 5

失败帧 n=3:
  median selected_support_frames: 1
  median best_support_frames: 1

失败帧细节:
  nav43 1782119604: selected rank10，selected_support_frames=1，best rank1，best_support_frames=1。
  nav43 1782119654: selected rank4， selected_support_frames=1，best rank1，best_support_frames=1。
  nav45 1782896166: selected rank20，selected_support_frames=1，best rank26，best_support_frames=5。
```

随后把 C++ temporal CSV 扩展出 coarse candidate 对参考位姿的误差字段，并重新跑 30 帧回归：

```text
输出目录: .codex_tmp/global_relocalization_nav_drift_extended_cpp_temporal_v2
localized: 30/30
有效参考成功率: 27/30
median 平移误差: 0.060m
median yaw 误差: 0.197deg
median total: 4239.9ms
median CPU 增量: 10337.5ms
峰值 RSS: 371.9MB
```

新增字段统计：

```text
成功帧 n=27:
  median selected_candidate_translation_error: 0.278m
  median selected_candidate_yaw_error: 0.934deg
  median best_seed_translation_error: 0.236m
  median best_seed_yaw_error: 1.955deg

失败帧 n=3:
  median selected_candidate_translation_error: 31.801m
  median selected_candidate_yaw_error: 145.291deg
  median best_seed_translation_error: 11.245m
  median best_seed_yaw_error: 86.137deg
```

hard frame 逐帧结论：

```text
nav43 1782119604:
  selected rank10: 34.05m / 179.19deg，support=1
  best rank1:      35.02m / 169.56deg， support=1
  结论：当前 top-K 没有稳定正确簇，应拒绝重置。

nav43 1782119654:
  selected rank4: 31.80m / 145.29deg，support=1
  best rank1:     11.24m / 86.14deg， support=1
  结论：best_seed 比 selected 稍好但仍明显错误，应拒绝重置。

nav45 1782896166:
  selected rank20: 25.03m / 88.46deg，support=1
  best rank26:      0.54m / 1.54deg， support=5
  结论：多帧一致性可以从 top-K 中选出正确区域候选，后续再接共享 GICP 精配准有机会把该帧救回。
```

temporal decision 策略模拟：

随后新增 `global_relocalization_temporal_decisions.csv`，用于模拟后续在线恢复策略：

```text
1. 先根据多帧 map->odom 一致性选择 best_seed。
2. 只有 best_support_frames >= temporal_consistency_online_min_support_frames 时才 accept。
3. accept 后对 best_seed 执行共享 refiner，也就是配置中的 GICP/ICP/NDT。
4. 用 refine 后位姿计算最终误差和 refined_success。
```

先使用阈值：

```text
temporal_consistency_online_min_support_frames: 3
```

重新跑 30 帧回归：

```text
输出目录: .codex_tmp/global_relocalization_nav_drift_extended_cpp_temporal_v3
主 metrics:
  localized: 30/30
  原始单帧有效参考成功率: 27/30
  median 平移误差: 0.060m
  median yaw 误差: 0.197deg
  median total: 4235.2ms
  median CPU 增量: 10325.1ms
  峰值 RSS: 371.9MB

temporal decision:
  accept: 27
  reject: 3
  accepted refined success: 27/27
  accepted median refined trans: 0.062m
  accepted median refined yaw: 0.118deg
```

hard/boundary case：

```text
nav43 1782119604:
  decision=reject，reason=support_below_threshold
  best rank1 support=1，coarse error=35.02m / 169.56deg
  结论：危险错误候选被拒绝。

nav43 1782119654:
  decision=reject，reason=support_below_threshold
  best rank1 support=1，coarse error=11.24m / 86.14deg
  结论：危险错误候选被拒绝。

nav45 1782896166:
  decision=accept，best rank26 support=5
  best coarse error=0.54m / 1.54deg
  refined error=0.088m / 0.067deg
  结论：原始单帧失败被 temporal decision + GICP 救回。

nav43 1782119629:
  decision=reject，best rank1 support=2
  best coarse error=0.71m / 0.19deg
  结论：这是保守阈值带来的误拒；该帧原本可成功，但支持帧数未达到 3。
```

因此当前默认阈值的行为是：

```text
优点：能拒绝两个危险 nav43 hard failure，并救回 nav45 hard failure。
代价：会误拒一个低支持但可成功的边界帧。
后续优化方向：围绕 online_min_support_frames、历史窗口长度、局部 refine fitness 和定位状态机触发条件做联合调参，而不是单独放宽支持阈值。
```

随后显式把阈值降为：

```text
temporal_consistency_online_min_support_frames: 2
```

重新跑完整 30 帧回归：

```text
输出目录: .codex_tmp/global_relocalization_nav_drift_extended_cpp_temporal_support2_real_v1
主 metrics:
  localized: 30/30
  原始单帧有效参考成功率: 27/30
  median 平移误差: 0.060m
  median yaw 误差: 0.197deg
  median total: 4272.8ms
  median CPU 增量: 10438.5ms
  峰值 RSS: 370.2MB

temporal decision:
  accept: 28
  reject: 2
  accepted refined success: 28/28
  accepted median refined trans: 0.063m
  accepted median refined yaw: 0.134deg
  accepted max refined trans: 0.353m
  accepted max refined yaw: 4.034deg
```

support>=2 的 hard/boundary case：

```text
nav43 1782119604:
  decision=reject，best rank1 support=1
  coarse error=35.02m / 169.56deg
  结论：危险错误候选仍被拒绝。

nav43 1782119654:
  decision=reject，best rank1 support=1
  coarse error=11.24m / 86.14deg
  结论：危险错误候选仍被拒绝。

nav43 1782119629:
  decision=accept，best rank1 support=2
  coarse error=0.71m / 0.19deg
  refined error=0.077m / 0.296deg
  结论：support>=3 误拒的边界成功帧被找回。

nav45 1782896166:
  decision=accept，best rank26 support=5
  coarse error=0.54m / 1.54deg
  refined error=0.088m / 0.067deg
  结论：原始单帧失败仍被救回。
```

上面这组 `support>=2` 结果使用的是离线双向窗口：

```text
temporal_consistency_window_before: 2
temporal_consistency_window_after: 2
```

它适合分析“如果等待前后若干帧，top-K 里是否存在稳定正确簇”，但严格来说包含未来帧，不应直接等同在线系统可用结果。

随后新增因果时序配置：

```text
src/humanoid_global_relocalization_runtime/test/nav_drift_registered_world_extended_cpp_temporal_causal.yaml
```

关键差异：

```text
temporal_consistency_window_before: 4
temporal_consistency_window_after: 0
temporal_consistency_online_min_support_frames: 2
输出目录: .codex_tmp/global_relocalization_nav_drift_extended_cpp_temporal_causal_v1
```

该配置只使用当前帧和历史帧，更接近在线 debug 节点和后续导航状态机的信息边界。完整 30 帧回归结果：

```text
主 metrics:
  localized: 30/30
  原始单帧有效参考成功率: 27/30
  median 平移误差: 0.060m
  median yaw 误差: 0.197deg
  median total: 4475.2ms
  median CPU 增量: 10672.7ms
  峰值 RSS: 375.6MB

causal temporal decision:
  accept: 25
  reject: 5
  accepted refined success: 25/25
  accepted median refined trans: 0.063m
  accepted median refined yaw: 0.151deg
  accepted max refined trans: 0.337m
  accepted max refined yaw: 3.802deg
```

因果 hard/boundary case：

```text
nav43 1782119604:
  decision=reject，best rank1 support=1
  coarse error=35.02m / 169.56deg
  结论：危险错误候选仍被拒绝。

nav43 1782119654:
  decision=reject，best rank1 support=1
  coarse error=11.24m / 86.14deg
  结论：危险错误候选仍被拒绝。

nav45 1782896166:
  decision=accept，best rank26 support=5
  coarse error=0.54m / 1.54deg
  refined error=0.088m / 0.067deg
  结论：不借助未来帧，仅用历史窗口也能救回该 hard frame。

nav43 1782119629:
  decision=accept，best rank1 support=4
  coarse error=0.71m / 0.19deg
  refined error=0.077m / 0.296deg
  结论：support>=3 曾误拒的边界成功帧，在 4 帧历史因果窗口下可被接受。
```

因果配置额外 reject 的 3 帧是每个 bag 的首帧或历史不足帧：

```text
nav_drift_test43 1782119429.893823
nav_drift_test44 1782178970.591691
nav_drift_test45 1782896041.599820
```

因此当前推荐在线确认策略为：`support>=2`，并使用只看历史的因果窗口。它在现有 30 帧验证里能拒绝两个危险 hard failure、救回 nav45 hard failure，代价是启动/恢复刚开始时需要等待至少两帧稳定支持。后续新增更多 bag 后仍需要继续复核该阈值，尤其是更长走廊、更多重复结构和真实 `/cloud_registered_body` 输入。

为避免后续调参时人工漏看 hard case，新增门禁脚本：

```text
src/humanoid_global_relocalization_runtime/test/verify_temporal_decisions.py
```

该脚本读取 `global_relocalization_temporal_decisions.csv`，自动检查：

```text
总行数为 30。
accept 为 28，reject 为 2。
accepted refined success 为 28/28。
两个 nav43 危险 hard failure 必须 reject。
nav45 hard failure 必须 accept，且 refined 后满足成功阈值。
```

当前 support>=2 CSV 校验输出：

```text
rows=30 accept=28 reject=2 accepted_success=28
accepted_refined median_trans=0.062502 max_trans=0.353460 median_yaw=0.134454 max_yaw=4.034419
[verify_temporal_decisions] PASS
```

在线 debug 滑动窗口：

在 C++ 内置离线模块基础上，`global_relocalization_node` 也已接入同一套多帧一致性逻辑。在线搜索每次得到 top-K 候选后，会把候选、同帧 `/odom` 和预处理 scan 放入滑动窗口，并在达到以下条件后发布 debug verified pose：

```text
temporal_consistency_online_min_support_frames: 2
temporal_consistency_online_max_history_frames: 10
verified_pose_topic: /global_relocalization/verified_candidate
verified_map_odom_topic: /global_relocalization/verified_map_to_odom
recovery_status_topic: /global_relocalization/recovery_status
```

`verified_candidate` 类型为 `geometry_msgs/PoseStamped`，语义是“当前滑动窗口里跨帧支持最强、并经过 refine 的 `map->base_footprint` 候选”。`verified_map_to_odom` 也是 `geometry_msgs/PoseStamped`，语义是由 `refined_map_to_base * inverse(current_odom_to_base)` 计算出的 `map->odom` 恢复量。它们仍然只用于 RViz/debug，不发布 TF，不注入 `initialpose`，不会改变导航闭环。后续真正接入时，应由导航状态管理器在定位丢失/漂移状态下触发重定位，并在 verified 候选连续稳定后再调用 bridge 专用入口。

当前实现已在第一层 selected support/refine 之外，补入在线 trajectory recovery 尝试：当 selected 支持不足或 refine fitness 超阈值时，节点会对滑动窗口中的历史 top-K 候选计算 scan-map trajectory likelihood；只有满足严格 average overlap/margin，或满足“单帧高置信 + 轨迹候选与单帧位姿一致”补救规则时，才发布 `verified_trajectory` 或 `verified_trajectory_single_agreement` 状态。该能力仍只输出 debug PoseStamped，不发布 TF。

后续又把 evaluator 中的 ICP/GICP/NDT 精配准逻辑抽成共享模块：

```text
include/humanoid_global_relocalization_runtime/refiner.hpp
src/refiner.cpp
```

离线 evaluator 和在线 debug 节点现在共用同一套 `refine_single_candidate/refine_candidates`。在线节点发布 `/global_relocalization/verified_candidate` 前，会对多帧一致性选出的候选再执行一次配置中的 `refine_method`，例如 GICP，然后发布精配准后的 debug 位姿。这样在线 debug 输出的 verified pose 与离线评估的精配准逻辑保持一致。

扩展验证结论：

1. `balanced_iter25` 仍是当前最均衡的单帧配置，但 30 帧扩展验证证明它不是“可直接无保护接入导航重置”的最终策略。
2. hard case 主要来自走廊/重复结构造成的全局歧义，单帧 GICP fitness 可能把错误候选评为更优。
3. 候选质量指标能暴露部分风险，但不能形成可靠的单帧硬阈值；必须把它们放进多帧一致性和状态机里使用。
4. 多帧 `map->odom` 一致性分析已经证明：当前 3 个失败帧的错误最终候选都缺少跨帧支持，其中 nav45 还存在跨帧稳定的正确候选簇。
5. v2 粗候选误差字段进一步证明：nav45 可通过 best_seed rank26 进入正确区域，而 nav43 两帧没有可靠正确簇，应优先拒绝重置。
6. temporal decision 对比证明：support>=2 在当前 30 帧上优于 support>=3，可接受 28 帧且 28/28 精配准成功，并仍拒绝两个危险 nav43 failure。
7. 后续接入 bridge/nav 前，必须增加多帧一致性确认、候选拒绝、局部定位状态联合判断，避免单帧错误候选直接发布为 `map -> odom` 或 initialpose。
8. 多帧累积 scan 有一定潜力，但简单 7 帧累积不能稳定解决全部 hard case；当前证据更支持“多帧候选聚类/一致性投票”，而不是只把点云堆在一起。

## 扩展 bag 验证

除 `/home/ubuntu/nav_drift_test` 外，已额外筛选当前机器上的 rosbag metadata。结果如下：

```text
/home/ubuntu/clearing: 只有 /airy_points、IMU 和 TF，没有 /fast_lio/cloud_registered、/odom、/robot_realpose，不能用于当前 registered_world/body 验证。
/home/ubuntu/costmap_clearing_full: 有 /fast_lio/cloud_registered，但没有 /odom 和 /robot_realpose，不能直接按 registered_world 链路验证。
/home/ubuntu/fast-lio-bags/hall_mapping: 有 /fast_lio/cloud_registered 和 /odom，但没有 /robot_realpose，只能做无真值验证。
/home/ubuntu/fast-lio-bags/hall_mapping_20260605_151228: 有 /fast_lio/cloud_registered 和 /odom，但没有 /robot_realpose，只能做无真值验证。
/home/ubuntu/fast-lio-bags/hall_mapping_20260605_151705: 有 /fast_lio/cloud_registered 和 /odom，但没有 /robot_realpose，只能做无真值验证。
/home/ubuntu/fast-lio-bags/hall_mapping_20260605_152637: 有 /fast_lio/cloud_registered、/odom、/robot_realpose，可计算参考误差。
/home/ubuntu/下载/bags/hall_mapping: 有 /fast_lio/cloud_registered 和 /odom，但没有 /robot_realpose，只能做无真值验证。
```

没有在可读 metadata 中发现 `/cloud_registered_body`。

对 `/home/ubuntu/fast-lio-bags/hall_mapping_20260605_152637` 做了 5 帧扩展验证。该 bag 很大，`/robot_realpose` 从后半段才开始出现，实测第一个可对齐点云约为 cloud index `7980`，所以配置使用 `bag_start_frame_skip: 7980`。

开启参考位姿合理性过滤后的统计为：

```text
localized: 5/5
有效参考帧: 3/5
raw success: 3/5
有效参考成功率 success_over_reference_rate: 3/3
median 平移误差: 0.039m
median yaw 误差: 1.582deg
median total: 6009.3ms
median CPU 增量: 16776.3ms
峰值 RSS: 242.6MB
```

逐帧检查 `reference_x_m/reference_y_m/reference_yaw_deg` 后发现：

```text
1780645913.204300: reference=(0.030, 0.082, -7.496deg)，成功，误差 0.039m / 1.582deg。
1780645963.205729: reference=(0.035, 0.080, -7.512deg)，成功，误差 0.036m / 1.596deg。
1780646013.206258: reference=(-1.426, -7.801, 148.207deg)，成功，误差 0.044m / 0.546deg。
1780646311.412325: reference=(-6775.895, 1630.489, 54.976deg)，触发 reference_xy_out_of_range(abs_limit=100)。
1780646361.413350: reference=(-15496.725, 3345.906, -146.561deg)，触发 reference_xy_out_of_range(abs_limit=100)。
```

因此该扩展 bag 只能得出两个结论：

1. 参考位姿正常的前 3 帧中，推荐配置 `3/3` 成功，说明算法链路可以泛化到 nav_drift_test 之外的同类 bag。
2. 后 2 帧不能作为算法失败计入最终成功率，因为 `/robot_realpose` 自身跳到几千米/上万米量级，属于参考源异常或 frame/数据记录问题。

对没有 `/robot_realpose` 的其它 bag 也做了 registered_world 链路冒烟验证，每个 bag 最多抽 2 帧。由于没有参考位姿，这组结果只证明数据链路能跑通，不评价准确率：

```text
/home/ubuntu/fast-lio-bags/hall_mapping: 2/2 localized。
/home/ubuntu/fast-lio-bags/hall_mapping_20260605_151228: 1/1 localized，另一个抽样点未形成有效评估帧。
/home/ubuntu/fast-lio-bags/hall_mapping_20260605_151705: 2/2 localized。
/home/ubuntu/下载/bags/hall_mapping: 2/2 localized；该 bag 的点云 header 与 /odom header 约有 0.11s 到 0.13s 偏差，因此 smoke 配置把 `odom_time_tolerance_sec` 从 0.05s 放宽到 0.20s。
```

无真值 smoke 的汇总：

```text
抽样记录: 7
localized: 7/7
reference: 0/7
median total: 4956.1ms
median CPU 增量: 13415.2ms
峰值 RSS: 232.8MB
```

## 合成 body 链路验证

早期所有可读 bag metadata 中都没有真实 `/cloud_registered_body`。为了先验证本包 `input_mode=body` 分支是否和 `registered_world` 分支在坐标链路上自洽，新增测试脚本：

```text
src/humanoid_global_relocalization_runtime/test/make_synthetic_body_bag.py
```

该脚本从原始 bag 读取：

```text
/fast_lio/cloud_registered
/odom
/robot_realpose
```

然后执行：

```text
p_body = inverse(T_camera_init_body) * p_camera_init
```

并写出小规模派生 bag：

```text
.codex_tmp/synthetic_body_nav_drift_test43
.codex_tmp/synthetic_body_nav_drift_test44
.codex_tmp/synthetic_body_nav_drift_test45
```

每个派生 bag 内容：

```text
/cloud_registered_body: 5 条，frame_id=body
/odom: 5 条
/robot_realpose: 5 条
```

生成命令：

```bash
source /opt/ros/jazzy/setup.bash
python3 src/humanoid_global_relocalization_runtime/test/make_synthetic_body_bag.py \
  --input-bag /home/ubuntu/nav_drift_test/nav_drift_test44 \
  --output-bag /home/ubuntu/software/Todesk/Files/humanoid_ws/.codex_tmp/synthetic_body_nav_drift_test44 \
  --skip 50 --stride 500 --max-frames 5 \
  --odom-tolerance 0.05 --reference-tolerance 0.05 --overwrite

python3 src/humanoid_global_relocalization_runtime/test/make_synthetic_body_bag.py \
  --input-bag /home/ubuntu/nav_drift_test/nav_drift_test45 \
  --output-bag /home/ubuntu/software/Todesk/Files/humanoid_ws/.codex_tmp/synthetic_body_nav_drift_test45 \
  --skip 50 --stride 500 --max-frames 5 \
  --odom-tolerance 0.05 --reference-tolerance 0.05 --overwrite
```

使用 `synthetic_body_nav_drift_test43.yaml` 运行 body 模式，结果为：

```text
input_mode: body
localized: 5/5
有效参考成功率: 5/5
median 平移误差: 0.048m
median yaw 误差: 0.210deg
median total: 2914.9ms
median CPU 增量: 6182.1ms
峰值 RSS: 161.4MB
```

逐帧与 registered_world 同帧结果对比：

```text
1782119429.893823: body err 0.047728m / 0.184460deg；registered_world err 0.047728m / 0.184461deg。
1782119479.894553: body err 0.038723m / 0.118806deg；registered_world err 0.038724m / 0.118806deg。
1782119529.895285: body err 0.041866m / 0.275316deg；registered_world err 0.041866m / 0.275316deg。
1782119579.896152: body err 0.061244m / 0.209532deg；registered_world err 0.061244m / 0.209532deg。
1782119629.896734: body err 0.061453m / 0.240621deg；registered_world err 0.061452m / 0.240618deg。
```

为了让这组对比不只停留在人工读表，又新增脚本：

```text
src/humanoid_global_relocalization_runtime/test/compare_input_modes.py
```

当前脚本校验输出：

```text
matched=5 max_xy_diff=0.000001000 median_xy_diff=0.000000000 max_yaw_diff=0.000003000 median_yaw_diff=0.000000000 max_error_diff=0.000001000 max_rank_diff=0
[compare_input_modes] PASS
```

结论：

1. 本包 body 输入分支、raw body 到 `base_footprint` 的轴转换、BBS/GICP 搜索和参考评估链路与 registered_world 分支一致。
2. 该验证使用的是从 registered_world 派生的合成 body 数据，不等价于真实 Fast-LIO `/cloud_registered_body` 发布链路。后续已经用 `nav_drift_test46` 补做真实 body 验证。

为了扩大 body 分支覆盖，又新增三包配置：

```text
src/humanoid_global_relocalization_runtime/test/synthetic_body_nav_drift_all.yaml
```

运行命令：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 run humanoid_global_relocalization_runtime global_relocalization_offline_eval \
  --config /home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_global_relocalization_runtime/test/synthetic_body_nav_drift_all.yaml
```

当前结果：

```text
bag_frames: 15
metrics rows: 45
localized: 45/45
success: 45/45
reference: 45/45
任意点启动 median total: 2961.7ms
峰值 RSS: 172.4MB
```

按 bag 的任意点启动结果：

```text
synthetic_body_nav_drift_test43: 5/5 成功，median 平移误差 0.047728m，median yaw 误差 0.209532deg。
synthetic_body_nav_drift_test44: 5/5 成功，median 平移误差 0.062678m，median yaw 误差 0.139641deg。
synthetic_body_nav_drift_test45: 5/5 成功，median 平移误差 0.043277m，median yaw 误差 0.327300deg。
```

该三包结果把 body 工程链路覆盖从 `nav43` 扩展到 `nav43/44/45`，但仍然属于合成 body 自洽验证；真实 `/cloud_registered_body` 的话题发布时延、坐标约定和驱动链路由后续 `nav_drift_test46` 复核。

## 真实 body 双链路验证

补录 bag：

```text
/home/ubuntu/nav_drift_test/nav_drift_test46
```

该 bag 具备：

```text
/fast_lio/cloud_registered: 36363
/cloud_registered_body: 36363
/odom: 36363
/robot_realpose: 36353
```

时间戳检查显示 `/fast_lio/cloud_registered`、`/cloud_registered_body` 和 `/odom` 的 header stamp 同帧一致，`/cloud_registered_body` 的 `frame_id=body`。`/robot_realpose` 比点云晚约 1 秒开始，因此 10 个抽样帧中第 1 帧没有参考位姿，后 9 帧可计算误差。

真实 body 验证产物：

```text
.codex_tmp/global_relocalization_real_body_validation_v1/global_relocalization_metrics.csv
```

结果：

```text
arbitrary_start_no_prior:
  localized: 10/10
  有参考帧成功: 9/9
  median 平移误差: 0.078m
  max 平移误差: 0.091m
  median yaw 误差: 0.090deg
  max yaw 误差: 0.170deg
  median total: 2684.2ms
  max total: 12116.7ms
  peak RSS max: 340.6MB
```

同一个 bag 又跑了 registered_world 对照：

```text
.codex_tmp/global_relocalization_nav46_registered_world_v1/global_relocalization_metrics.csv
```

两路同 stamp 对照：

```text
matched: 10
max_xy_diff: 0.000042m
median_xy_diff: 0.0000005m
max_yaw_diff: 0.000151deg
max_error_diff: 0.000032m
success_mismatch: 0
```

随后又把同一个 `nav_drift_test46` 扩大为跨全包 30 帧抽样，抽样间隔 `1200`，分别跑真实 body 和 registered_world 对照：

```text
body 30 帧产物：
.codex_tmp/global_relocalization_real_body_validation_30frame_v1/global_relocalization_metrics.csv

registered_world 30 帧产物：
.codex_tmp/global_relocalization_nav46_registered_world_30frame_v1/global_relocalization_metrics.csv
```

30 帧真实 body 结果：

```text
arbitrary_start_no_prior:
  localized: 30/30
  有参考帧成功: 27/29
  median 平移误差: 0.101m
  max 平移误差: 4.211m
  median yaw 误差: 0.229deg
  max yaw 误差: 171.415deg
  median total: 5548.6ms
  median CPU: 15238.0ms
  peak RSS max: 363.7MB

localization_jump_3m:
  localized: 30/30
  有参考帧成功: 27/29

localization_jump_5m_90deg:
  localized: 30/30
  有参考帧成功: 27/29
```

30 帧 registered_world 对照结果：

```text
arbitrary_start_no_prior:
  localized: 30/30
  有参考帧成功: 27/29
  median 平移误差: 0.101m
  max 平移误差: 4.157m
  median yaw 误差: 0.317deg
  max yaw 误差: 170.750deg
  median total: 5414.7ms
  median CPU: 15111.0ms
  peak RSS max: 357.1MB
```

30 帧两路同 stamp 对照：

```text
matched: 90
max_xy_diff: 0.074169m
median_xy_diff: 0.000002m
max_yaw_diff: 0.665205deg
max_error_diff: 0.053625m
success_mismatch: 0
```

两个失败时间戳在真实 body 和 registered_world 上完全一致：

```text
1782966610.292374: body 4.211m / 171.415deg，registered_world 4.157m / 170.750deg。
1782967810.317188: body 2.896m / 25.895deg，registered_world 2.896m / 25.899deg。
```

结论：在 `nav_drift_test46` 上，真实 `/cloud_registered_body` 与 `/fast_lio/cloud_registered + /odom` 反变换链路输出高度一致；30 帧扩展里的失败不是输入话题或坐标轴问题，而是当前全局匹配在重复结构中选到了相似候选。后续优化重点应放在候选拒绝、多帧确认和降低全图搜索 CPU 代价上。

## nav46 资源优化 sweep

为了回应 CPU 占用过高的问题，在真实 `nav_drift_test46` body 30 帧上新增资源 sweep 脚本：

```text
src/humanoid_global_relocalization_runtime/test/run_nav46_resource_sweep.py
```

汇总产物：

```text
.codex_tmp/nav46_resource_sweep_summary.csv
```

结果：

```text
baseline_body30:
  localized: 30/30
  reference_success: 27/29
  accepted_ref_success: 27/27
  median total: 5548.6ms
  median CPU: 15238.0ms
  thread_count: 10-10

threads4_refine10:
  localized: 30/30
  reference_success: 27/29
  accepted_ref_success: 27/27
  median total: 4125.3ms
  median CPU: 9911.3ms
  thread_count: 6-6

threads2_refine8:
  localized: 30/30
  reference_success: 27/29
  accepted_ref_success: 27/27
  median total: 4087.9ms
  median CPU: 6784.7ms
  thread_count: 4-4

coarser_scan_threads4:
  localized: 30/30
  reference_success: 27/29
  accepted_ref_success: 25/26
  median total: 3163.5ms
  median CPU: 7236.9ms
  thread_count: 6-6
```

结论：

1. `threads2_refine8` 是当前推荐低负载档：保持 temporal accepted `27/27` 成功，同时 median CPU 从 `15238.0ms` 降到 `6784.7ms`，线程数从 `10` 降到 `4`。
2. `threads4_refine10` 也安全，但 CPU 降幅不如 2 线程档。
3. `coarser_scan_threads4` 虽然 wall time 更低，但出现一个被 temporal 接受后仍失败的恢复点 `1782968170.324651`，误差 `1.377m / 0.103deg`，因此不作为推荐配置。
4. 主 YAML、在线 debug YAML 和真实 body 自动验证脚本已采用 `bbs_num_threads=2`、`max_refine_candidates=8`，保持 `scan_leaf_size=0.30` 不变。

## 在线 debug 节点验证

为了确认在线节点不仅能启动，还能实际从点云话题触发搜索、维护多帧窗口并发布 verified debug pose，新增：

```text
src/humanoid_global_relocalization_runtime/test/synthetic_body_online_debug.yaml
src/humanoid_global_relocalization_runtime/test/run_online_debug_smoke.py
```

该测试使用合成 body 小 bag：

```text
.codex_tmp/synthetic_body_nav_drift_test43
```

运行命令：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
python3 src/humanoid_global_relocalization_runtime/test/run_online_debug_smoke.py --timeout 180
```

实测日志关键行：

```text
loaded config=.../synthetic_body_online_debug.yaml input_mode=body enable_online_search=true
built 3D-BBS map index: map_points=34965 occupancy_voxels=13188 build_ms=7.273
online debug search subscribed cloud=/cloud_registered_body odom=/odom
trajectory recovery candidate rejected: support=1/2 overlap=0.991199 margin=0.035204 single_agree=(0.193m, 0.804deg)
temporal selected candidate not verified yet: best_rank=1 best_support=1 selected_rank=1 selected_support=1/2
temporal selected candidate verified: selected_rank=1 selected_support=2 best_rank=1 best_support=2 map_odom=(0.210, 0.054, -8.14deg) refine=gicp converged=true fitness=0.035719 refine_ms=75.900
verified_candidate x=0.194 y=0.009 z=0.000
verified_map_to_odom x=0.210 y=0.054 yaw=-8.138
recovery_status ... state=verified ... refined_converged=true ...
PASS verified=1 map_odom=1 status=2
```

结论：

1. 在线 debug 节点在 `enable_online_search=true` 时可以加载地图、订阅 `/cloud_registered_body` 和 `/odom`。
2. 第一帧 trajectory recovery 因 support=1/2 暂不发布 verified，第二帧达到 support=2 后发布 `/global_relocalization/verified_candidate` 和 `/global_relocalization/verified_map_to_odom`。
3. verified 发布前执行了共享 GICP refine，且本次收敛。
4. 该测试仍使用合成 body bag，只证明在线运行链路和 verified debug 发布链路可用，不替代真实 `/cloud_registered_body` 准确率验证。

为了确认新增的在线 trajectory recovery 分支不是只存在于离线 CSV，又新增专项脚本：

```text
src/humanoid_global_relocalization_runtime/test/run_online_trajectory_recovery_smoke.py
```

该脚本会基于 `synthetic_body_online_debug.yaml` 生成临时配置：

```text
.codex_tmp/online_trajectory_recovery_smoke.yaml
```

临时配置把 `temporal_consistency_online_max_refine_fitness` 压到 `0.0`，使普通第一层 `state=verified` 分支必然因为 fitness 阈值失败；随后要求 `run_online_debug_smoke.py` 必须收到：

```text
state=verified_trajectory_single_agreement
```

运行命令：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
python3 src/humanoid_global_relocalization_runtime/test/run_online_trajectory_recovery_smoke.py --timeout 180
```

实测关键日志：

```text
trajectory recovery candidate rejected: support=1/2 overlap=0.967641 margin=0.967641 single_agree=(0.198m, 2.284deg)
trajectory recovery candidate rejected: support=2/2 overlap=0.962862 margin=0.001594 single_agree=(8.344m, 1.446deg)
trajectory recovery verified: state=verified_trajectory_single_agreement seed=1 rank=1 support=3 overlap=0.989154 margin=0.028138 single_agree=(0.171m, 1.039deg)
recovery_status ... state=verified_trajectory_single_agreement ... refined_fitness=0.0191528
PASS verified=1 map_odom=1 status=4
```

输出证据：

```text
.codex_tmp/online_trajectory_recovery_smoke_pose.csv
```

证据校验命令：

```bash
python3 src/humanoid_global_relocalization_runtime/test/verify_online_trajectory_recovery_smoke.py
```

结论：新增在线 trajectory recovery 分支已被真实 ROS 节点执行。它会拒绝支持不足或与单帧精配不一致的候选，并在 support/overlap/margin/单帧一致性同时满足后发布 `verified_trajectory_single_agreement` debug 恢复量。

为了确认在线节点在“不能安全发布恢复位姿”时能给上层状态机明确提示，又新增主动恢复状态专项脚本：

```text
src/humanoid_global_relocalization_runtime/test/run_online_active_view_status_smoke.py
src/humanoid_global_relocalization_runtime/test/verify_online_active_view_status_smoke.py
```

该脚本临时关闭 `trajectory_single_agreement_fallback_enable`，并把 strict overlap 提高，让普通 verified 和 fallback verified 都无法发布。实测输出：

```text
state=need_active_view_margin_below_threshold
recovery_hint=active_view
trajectory_attempted=true
trajectory_support=2
trajectory_average_overlap=0.962862
trajectory_margin=0.0015939
trajectory_single_agree_xy=8.34378
```

结论：当当前历史窗口出现候选歧义或证据不足时，在线节点不会强行发布 `verified_candidate`，而是通过 `recovery_status` 给出 `need_active_view_*` 和 `recovery_hint=active_view`；verified 状态还会记录 `map_odom_x/y/yaw_deg`，并由在线 smoke verifier 校验它与 `/verified_map_to_odom` PoseStamped 一致。后续应由导航状态机低速移动/转动采集新视角后再触发本包重定位。

在线 smoke 覆盖矩阵记录在：

```text
src/humanoid_global_relocalization_runtime/test/online_smoke_evidence.csv
```

当前覆盖：

```text
registered_world + nav_drift_test44: PASS，verified_candidate x=0.004 y=0.047，map_odom x=0.035 y=0.093 yaw=-10.764deg。
registered_world + nav_drift_test45: PASS，verified_candidate x=0.068 y=0.032，map_odom x=0.010 y=0.014 yaw=-4.209deg。
registered_world + nav_drift_test46: PASS，verified_candidate x=-0.008 y=0.103，map_odom x=0.068 y=0.083 yaw=-14.005deg。
body + real_body_nav_drift_test46: PASS，verified_candidate x=-0.003 y=0.113，map_odom x=0.073 y=0.093 yaw=-14.020deg。
body + synthetic_body_nav_drift_test43: PASS，verified_candidate x=0.193 y=0.009，map_odom x=0.206 y=0.052 yaw=-8.172deg。
body + synthetic_body_nav_drift_test44: PASS，verified_candidate x=-0.004 y=0.036，map_odom x=0.041 y=0.088 yaw=-10.692deg。
body + synthetic_body_nav_drift_test45: PASS，verified_candidate x=0.240 y=-0.201，map_odom x=0.195 y=-0.218 yaw=-3.626deg。
```

矩阵重跑命令：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
python3 src/humanoid_global_relocalization_runtime/test/run_online_smoke_matrix.py
```

矩阵校验命令：

```bash
python3 src/humanoid_global_relocalization_runtime/test/verify_online_smoke_evidence.py
```

为了同时验证真实 nav_drift 的 `registered_world` 在线链路，又新增配置：

```text
src/humanoid_global_relocalization_runtime/test/nav_drift_registered_world_online_debug.yaml
```

运行命令：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
python3 src/humanoid_global_relocalization_runtime/test/run_online_debug_smoke.py \
  --config src/humanoid_global_relocalization_runtime/test/nav_drift_registered_world_online_debug.yaml \
  --bag /home/ubuntu/nav_drift_test/nav_drift_test44 \
  --timeout 180
```

实测关键日志：

```text
loaded config=.../nav_drift_registered_world_online_debug.yaml input_mode=registered_world enable_online_search=true
built 3D-BBS map index: map_points=34965 occupancy_voxels=13188 build_ms=8.116
online debug search subscribed cloud=/fast_lio/cloud_registered odom=/odom
trajectory recovery candidate rejected: support=1/2 overlap=0.992944 margin=0.368952 single_agree=(0.040m, 1.331deg)
temporal selected candidate not verified yet: best_rank=1 best_support=1 selected_rank=1 selected_support=1/2
skip registered_world cloud because synced odom was not found
temporal selected candidate verified: selected_rank=1 selected_support=2 best_rank=1 best_support=2 map_odom=(0.029, 0.087, -10.67deg) refine=gicp converged=true fitness=0.025353 refine_ms=75.443
verified_candidate x=-0.002 y=0.040 z=0.000
verified_map_to_odom x=0.029 y=0.087 yaw=-10.673
PASS verified=1 map_odom=1 status=2
```

结论：

1. 真实 nav_drift registered_world 在线链路可运行：`/fast_lio/cloud_registered + /odom -> raw body 反变换 -> base_footprint -> BBS -> temporal -> GICP -> verified_candidate + verified_map_to_odom`。
2. 因果时序窗口 `window_after=0` 下，第二个有效搜索达到 support=2 后成功发布 `/global_relocalization/verified_candidate` 和 `/global_relocalization/verified_map_to_odom`，后续搜索支持度继续增强到 support=3。
3. 中间出现一帧 odom 同步失败并被跳过，这是正确保护行为，避免用错误 odom 把 registered_world 点云反变换坏。
4. 这仍是 debug 输出验证，只发布 `map->odom` PoseStamped 调试消息，不发布 TF，不注入 `initialpose`。

## 恢复消费者仿真

为了在真正接入 bridge/nav 前先验证“状态机如果消费这些恢复量，会不会放行错误恢复”，新增：

```text
src/humanoid_global_relocalization_runtime/test/simulate_recovery_consumer.py
```

模拟接收规则：

```text
best_support_frames >= 2
refined_converged == true
refined_fitness <= 0.12
```

该规则只使用线上可获得字段，不使用真值。真值只用于离线统计 false accept。

输出产物：

```text
.codex_tmp/recovery_consumer_simulation.csv
```

结果：

```text
nav46_real_body_30frame: consumer_accept=25 consumer_reject=5 false_accept=0
nav46_registered_world_30frame: consumer_accept=24 consumer_reject=6 false_accept=0
nav_drift_registered_world_causal: consumer_accept=23 consumer_reject=7 false_accept=0
online_smoke_recovery_interface: publish=7/7 consumer_accept=7 consumer_reject=0
online_active_view_status_interface: publish=0/1 consumer_accept=0 consumer_reject=1
```

结论：当前 `support>=2 + refine 收敛 + fitness<=0.12` 规则在已有真值验证集中没有放行错误恢复；最新 online smoke 中 7 个 verified 场景都能提供 `verified_map_to_odom`、`state=verified` 和一致的 `map_odom_x/y/yaw_deg` 状态字段，且 7/7 通过消费者安全门控。`need_active_view_*` 状态也被模拟消费者明确拒绝，不会被当作可注入恢复量，只能作为主动采集新视角提示。

## 当前限制

1. 真实 `/cloud_registered_body` 已在 `nav_drift_test46` 上完成第一组同场景验证，但目前真实 body 覆盖仍只有 1 个 bag；后续还需要更多路线、遮挡和启动点。
2. CPU 版 3D-BBS 在 hard frame 上仍会出现 10s 级尾延迟，后续若要在线实时恢复，需要增加触发策略、超时策略、多帧确认，或接入 GPU/更强剪枝。
3. 当前第一阶段只输出候选、CSV 和 debug verified pose，不发布 `map -> odom`，不注入 `initialpose`，不会改变导航状态。
4. nav_drift 有效成功率统计来自历史 3 个 bag、15/30 个抽样帧，以及最新 `nav_drift_test46` 真实 body 对照；扩展 hall_mapping bag 中参考正常的 3 帧也全部成功；无真值 smoke 覆盖了其它 cloud+odom bag 的输入链路。但这仍不足以代表所有楼层/所有场景。后续应补充更多路线、更多遮挡、更多启动点，以及更多真实 `/cloud_registered_body` bag。
5. 门禁已经加入真实 body 后续约束：latest inventory 如果发现真实 `/cloud_registered_body`，目标证据矩阵会要求 `.codex_tmp/global_relocalization_real_body_validation_v1/global_relocalization_metrics.csv` 存在并覆盖这些 body bag；如果同时有 `/robot_realpose`，对应参考帧成功率必须不低于 `0.90`。当前还额外要求 `nav_drift_test46` 的 30 帧 body/registered_world 对照满足两路 `30/30 localized`、`27/29` 有参考帧成功且 `success_mismatch=0`。
   真实 body 补录后的推荐复现命令为：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
python3 src/humanoid_global_relocalization_runtime/test/run_validation_gates.py
python3 src/humanoid_global_relocalization_runtime/test/run_real_body_validation.py --run
python3 src/humanoid_global_relocalization_runtime/test/run_validation_gates.py
```

   `run_real_body_validation.py` 会根据 latest inventory 自动生成 `.codex_tmp/real_body_validation_config.yaml`，并把输出写到门禁要求的 `.codex_tmp/global_relocalization_real_body_validation_v1`。

## 代码回归记录

共享 refiner 抽取后，重新跑了 nav_drift smoke 配置，输出目录为：

```text
.codex_tmp/global_relocalization_refiner_smoke_v1
```

结果：

```text
bag_frames: 2
localized: 2/2
gicp: 1/1 success，median trans 0.037m，median yaw 3.084deg，median total 595.5ms。
none: 1/1 localized 但 success=0，说明粗定位对照仍按预期保留误差。
```

同时 `colcon build --packages-select humanoid_global_relocalization_runtime --cmake-args -DCMAKE_BUILD_TYPE=Release` 和在线 debug launch 冒烟启动均通过。

## 复现命令

编译：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select humanoid_global_relocalization_runtime --cmake-args -DCMAKE_BUILD_TYPE=Release
```

运行推荐参数：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 run humanoid_global_relocalization_runtime global_relocalization_offline_eval \
  --config src/humanoid_global_relocalization_runtime/test/nav_drift_registered_world_balanced_iter25.yaml
```

汇总：

```bash
python3 src/humanoid_global_relocalization_runtime/test/summarize_eval_csv.py \
  .codex_tmp/global_relocalization_nav_drift_balanced_iter25_v5_validref/global_relocalization_metrics.csv
```

候选质量分析：

```bash
python3 src/humanoid_global_relocalization_runtime/test/analyze_candidate_quality.py \
  .codex_tmp/global_relocalization_nav_drift_extended_v1/global_relocalization_metrics.csv \
  .codex_tmp/global_relocalization_nav_drift_extended_v1/global_relocalization_candidates.csv
```

多帧一致性分析：

```bash
source /opt/ros/jazzy/setup.bash
python3 src/humanoid_global_relocalization_runtime/test/analyze_temporal_consistency.py \
  .codex_tmp/global_relocalization_nav_drift_extended_v1/global_relocalization_metrics.csv \
  .codex_tmp/global_relocalization_nav_drift_extended_v1/global_relocalization_candidates.csv
```

C++ 内置多帧一致性回归：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 run humanoid_global_relocalization_runtime global_relocalization_offline_eval \
  --config src/humanoid_global_relocalization_runtime/test/nav_drift_registered_world_extended_cpp_temporal.yaml
```

temporal decision 门禁校验：

```bash
python3 src/humanoid_global_relocalization_runtime/test/verify_temporal_decisions.py \
  .codex_tmp/global_relocalization_nav_drift_extended_cpp_temporal_support2_real_v1/global_relocalization_temporal_decisions.csv
```

因果 temporal decision 门禁校验：

```bash
python3 src/humanoid_global_relocalization_runtime/test/verify_temporal_decisions.py \
  .codex_tmp/global_relocalization_nav_drift_extended_cpp_temporal_causal_v1/global_relocalization_temporal_decisions.csv \
  --expected-accept 25 \
  --expected-reject 5 \
  --expect-causal-warmup-rejects
```

registered_world/body 同帧一致性校验：

```bash
python3 src/humanoid_global_relocalization_runtime/test/compare_input_modes.py \
  --registered-csv .codex_tmp/global_relocalization_nav_drift_extended_cpp_temporal_support2_real_v1/global_relocalization_metrics.csv \
  --body-csv .codex_tmp/global_relocalization_synthetic_body_nav43_v1/global_relocalization_metrics.csv
```

中文文档/注释契约校验：

```bash
python3 src/humanoid_global_relocalization_runtime/test/check_documentation_contract.py
```

工作空间边界校验：

```bash
python3 src/humanoid_global_relocalization_runtime/test/check_workspace_boundaries.py
```

目标证据矩阵校验：

```bash
python3 src/humanoid_global_relocalization_runtime/test/check_goal_evidence.py
```

该脚本把“只改当前工作空间、测试代码独立目录、C++ 包、中文 YAML/源码说明、bag inventory、真实 nav_drift 30 帧、因果时序一致性、合成 body 链路、在线 smoke”等目标要求逐项映射到当前证据。当前真实 `/cloud_registered_body` bag 缺失会显示为 `KNOWN_LIMIT`，不是失败，也不会被当成真实 body 准确率通过。

精匹配方法和多地图 sweep 证据校验：

```bash
python3 src/humanoid_global_relocalization_runtime/test/verify_refine_sweep.py
```

资源消耗统计字段校验：

```bash
python3 src/humanoid_global_relocalization_runtime/test/verify_resource_metrics.py
```

第一阶段关键验证产物总门禁：

```bash
python3 src/humanoid_global_relocalization_runtime/test/run_validation_gates.py
```

该总门禁会先只读刷新 `.codex_tmp/bag_inventory_home_scan_latest.*` 和 `.codex_tmp/bag_inventory_nav_drift_latest.*`，再运行文档契约、工作空间边界、目标证据矩阵、refine sweep、资源字段、online smoke、temporal decision 和输入模式一致性检查。

在线 debug 节点冒烟启动：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 launch humanoid_global_relocalization_runtime global_relocalization_debug.launch.py
```

## nav46 60 点压力验证

用户补录 `nav_drift_test46` 后，新增压力验证脚本：

```text
src/humanoid_global_relocalization_runtime/test/run_nav46_stress_validation.py
```

该脚本沿 bag46 时间轴按 `skip=100`、`stride=600` 每路抽取 60 帧，分别验证：

```text
body:             /cloud_registered_body
registered_world: /fast_lio/cloud_registered + /odom 反变换
```

测试仍使用当前低负载推荐参数：

```text
bbs_num_threads: 2
max_refine_candidates: 8
scan_leaf_size: 0.30
temporal_consistency_window_before: 4
temporal_consistency_window_after: 0
temporal_consistency_online_min_support_frames: 2
```

旧版 temporal decision 会选择 `best_seed_rank`，也就是当前帧 top-K 中跨帧支持度最高的候选，再对它做 GICP 精配准。bag46 60 点扩大测试发现该策略不安全：

```text
body:             temporal_false_accept = 2
registered_world: temporal_false_accept = 2
```

其中 `1782966920.298724` 这一帧单帧 selected/GICP 已经正确，但 best_seed 选择了另一个稳定错误簇；`1782967100.302426` 则是重复结构导致 selected 和 best_seed 都错误。两路输入失败时间戳一致，说明问题不是 `/cloud_registered_body` 话题、轴转换或 registered_world 反变换，而是重复结构下“任意稳定簇”不能直接发布。

随后把恢复门控改为：只有当前最终候选 `selected_candidate` 自身的 `selected_support_frames >= 2` 才 accept；`best_seed` 仍写入 CSV 作为诊断字段，但不再替换 selected 发布。重跑同一批 60 点后的结果：

| 输入模式 | localized | 单帧有真值成功 | temporal accept | temporal reject | accepted success | false accept | median total | median CPU | peak RSS |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| body | 60/60 | 58/60 | 55 | 5 | 55/55 | 0 | 4193.6ms | 6720.5ms | 418.3MB |
| registered_world | 60/60 | 57/59 | 55 | 5 | 54/54 | 0 | 4383.2ms | 6999.4ms | 427.3MB |

本轮问题帧产物：

```text
.codex_tmp/nav46_stress_validation/summary_60f_s600_skip100.csv
.codex_tmp/nav46_stress_validation/problem_frames_60f_s600_skip100.csv
```

结论：

1. 当前参数具备较好的全局召回能力：两路输入 60/60 都能给出候选。
2. selected 门控后没有误接受，适合继续作为接入 bridge/nav 前的保守策略。
3. 仍有 5/60 帧被拒绝，其中 2 帧本身是单帧明显错误，3 帧是 selected 支持不足的保守拒绝。上线策略应优先保持“拒绝不确定帧”，再考虑用更多 scan 累积、语义/强度特征或更严格二次验证去救回这些保守拒绝。
4. body 与 registered_world 的准确率和失败模式基本一致，后续主要优化方向是重复结构歧义与资源耗时，而不是两路话题关系。

## nav46 随机 80 点位置分析

为了更接近“任意位置启动”和“定位漂移后随机触发恢复”，继续使用同一脚本做显式随机帧抽样：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
python3 src/humanoid_global_relocalization_runtime/test/run_nav46_stress_validation.py \
  --run --mode both --random-count 80 --seed 20260702
```

这次不再按固定 stride 均匀抽点，而是从 bag46 的点云帧序号中随机抽取 80 个 index，并把这些 index 写入自动生成的 YAML `bag_sample_frame_indices`。该方式可以复现同一批随机点，也便于后续针对问题帧单独回放。

随机 80 点第一轮发现：仅靠 `selected_support>=2` 仍有 1 个 false accept。问题帧为：

```text
stamp: 1782964959.563418
reference: (-0.10, 0.01, -9.7deg)
final:     ( 1.35, 8.70, -7.7deg)
error:      8.82m / 2.0deg
selected_support_frames: 2
refine_fitness: 0.121
```

因此新增精配质量门控：

```text
temporal_consistency_online_max_refine_fitness: 0.12
```

门控语义是：`selected_support>=2` 证明当前候选在时序上稳定，`refined_fitness<=0.12` 再证明当前 scan 与地图几何贴合；二者同时满足才发布 verified debug 恢复量。

复测同一批随机点后的结果：

| 输入模式 | localized | 单帧有真值成功 | temporal accept | temporal reject | accepted success | false accept | median total | median CPU | peak RSS |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| body | 80/80 | 74/80 | 68 | 12 | 68/68 | 0 | 3394.4ms | 5575.1ms | 493.1MB |
| registered_world | 80/80 | 74/80 | 68 | 12 | 68/68 | 0 | 3372.4ms | 5406.2ms | 489.7MB |

问题帧产物：

```text
.codex_tmp/nav46_stress_validation/summary_rand80_seed20260702.csv
.codex_tmp/nav46_stress_validation/problem_frames_rand80_seed20260702.csv
```

单帧失败主要位置如下，两路输入位置完全一致：

| stamp | 参考位置 x/y/yaw | 错误输出 x/y/yaw | 单帧误差 | decision reason |
|---|---|---|---|---|
| 1782964959.563418 | (-0.10, 0.01, -9.7deg) | (1.35, 8.70, -7.7deg) | 8.82m / 2.0deg | refine_fitness_above_threshold |
| 1782966058.480960 | (3.46, 7.97, -153.3deg) | (24.07, 12.34, 113.9deg) | 21.06m / 92.8deg | selected_support_below_threshold |
| 1782966183.283410 | (-1.31, -8.25, -75.0deg) | (21.05, 16.92, 88.2deg) | 33.67m / 163.2deg | selected_support_below_threshold |
| 1782966183.683642 | (-1.29, -8.29, -82.6deg) | (22.55, 18.14, 76.2deg) | 35.60m / 158.8deg | selected_support_below_threshold |
| 1782967301.406556 | (-0.23, -1.11, -107.7deg) | (8.23, 8.89, 92.5deg) | 13.10m / 159.8deg | selected_support_below_threshold |
| 1782968359.328737 | (9.59, 13.84, -177.8deg) | (10.31, 13.92, 158.4deg) | 0.73m / 23.7deg | selected_support_below_threshold |

结论：

1. 失败主要不是随机散布，而是落在几个重复结构/转角区域；`(-1.3, -8.3)` 附近连续两帧失败，是当前最明显的局部问题区。
2. `(-0.10, 0.01)` 附近曾出现稳定但错误的跨帧候选，fitness 门控能把它拒掉。
3. `(9.59, 13.84)` 这一帧平移接近阈值内，但 yaw 误差 23.7deg，说明后续还需要关注朝向歧义，而不只是 x/y。
4. 所有问题点在 body 与 registered_world 两路上同步出现，因此后续优化应优先针对候选歧义、局部结构相似和二次验证，而不是纠结点云话题选择。

## nav46 随机 80 点长历史二阶段救回验证

为了回答“短历史保守 reject 的点能不能再救回来”，把随机 80 点中 `temporal_reject` 的 12 个时间点单独转换成 trajectory likelihood 目标：

```text
src/humanoid_global_relocalization_runtime/test/nav46_rand80_reject_matches.csv
```

然后使用约 24 秒历史窗口重跑：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
python3 src/humanoid_global_relocalization_runtime/test/run_trajectory_likelihood_validation.py \
  --run \
  --bag /home/ubuntu/nav_drift_test/nav_drift_test46 \
  --matches src/humanoid_global_relocalization_runtime/test/nav46_rand80_reject_matches.csv \
  --output-root .codex_tmp/trajectory_rand80_rejects_long_history \
  --modes body registered_world
```

旧版 trajectory 接受条件只检查 `support_frames`、`margin` 和单帧支持阈值。结果显示两路完全一致：12 个短历史 reject 中，长历史能把 11 个选到正确位置，其中包括原单帧误差 `20.8m`、`33.7m`、`35.6m`、`13.1m`、`13.0m` 的困难点；但最后一个点也被误接受，候选误差为 `1.96m / 6.11deg`。

这个危险点的关键特征是 `average_overlap=0.902`，而同批其它 11 个正确救回点最低为 `0.970`。同时，bag44/45/46 的 69 个长历史成功样本在 `trajectory_likelihood_min_margin=0.0015` 时最低 `average_overlap` 约为 `0.968`。因此新增参数：

```text
trajectory_likelihood_min_average_overlap: 0.95
```

该参数和 `trajectory_likelihood_min_overlap_ratio` 的区别是：`min_overlap_ratio` 判断某一帧是否支持候选，`min_average_overlap` 判断整段历史是否整体贴合地图。新增门控后，复算随机 80 reject 结果为：

| 输入模式 | 短历史 reject 点 | 长历史 accept | accept success | false accept | 继续 reject 的成功点 |
|---|---:|---:|---:|---:|---:|
| body | 12 | 11 | 11 | 0 | 0 |
| registered_world | 12 | 11 | 11 | 0 | 0 |

为了验证新代码行为，不只做 CSV 事后复算，又用新二进制单独重跑危险点：

```text
.codex_tmp/nav46_rand80_reject12_match.csv
.codex_tmp/trajectory_rand80_reject12_avg_gate
```

结果两路都从旧版 `decision=accept` 变为 `decision=candidate`，也就是不会作为二阶段恢复结果发布：

```text
body:             trajectory=0 err=1.960729m/6.108467deg overlap=0.902171 decision=candidate
registered_world: trajectory=0 err=1.960729m/6.108467deg overlap=0.902171 decision=candidate
```

三包长历史回归的阈值影响如下，说明 `0.95` 不会伤到当前 69 个已验证成功点：

```text
avg_overlap>=0.900: accept=69 accept_success=69 false_accept=0 reject_success=0
avg_overlap>=0.930: accept=69 accept_success=69 false_accept=0 reject_success=0
avg_overlap>=0.950: accept=69 accept_success=69 false_accept=0 reject_success=0
avg_overlap>=0.970: accept=68 accept_success=68 false_accept=0 reject_success=1
avg_overlap>=0.980: accept=66 accept_success=66 false_accept=0 reject_success=3
```

这组三包结果已经固化为门禁脚本：

```bash
python3 src/humanoid_global_relocalization_runtime/test/verify_trajectory_long_history_bags.py
```

门禁要求 `nav_drift_test44/45/46` 长历史 trajectory likelihood 都能全量成功，且所有 accepted 行均为成功：bag44 从单帧 `15/25` 到轨迹 `25/25`，bag45 从 `12/22` 到 `22/22`，bag46 从 `18/22` 到 `22/22`，三包 `false_accept=0`。这条门禁用于防止后续优化只对 nav46 随机 80 点成立，却损伤 44/45 的长历史恢复能力。

结论：短历史在线门控仍适合作为第一层快速安全恢复；当它 reject 时，长历史 trajectory likelihood 可作为第二层 recovery，当前随机 80 的困难点能从 `accept=68/80` 提高到等效 `accept=79/80`，并保持已验证数据中 `false_accept=0`。唯一剩下的 reject 是一个整体 overlap 明显偏低的候选，应继续保守拒绝，等待主动恢复动作采集新视角。

## nav46 最后拒绝点主动恢复验证

为了继续确认最后 `1/80` 是否真的无解，又把 `run_trajectory_likelihood_validation.py` 扩展为支持带符号 offset：

```text
正数 offset: target_index - offset，表示目标之前的历史帧
0:          target_index，表示目标帧
负数 offset: target_index - offset，表示目标之后的新视角帧
```

这样可以离线模拟“机器人停住后，低速移动/转动采集新视角，再做第三层主动恢复”。验证命令：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
python3 src/humanoid_global_relocalization_runtime/test/run_trajectory_likelihood_validation.py \
  --run \
  --bag /home/ubuntu/nav_drift_test/nav_drift_test46 \
  --matches .codex_tmp/nav46_rand80_reject12_match.csv \
  --output-root .codex_tmp/trajectory_rand80_reject12_active_view_v2 \
  --modes body registered_world \
  --history-offsets 240 200 160 120 80 40 0 -40 -80 -120 -160 -200 -240
```

第一轮主动恢复显示：加入未来新视角后，最佳轨迹候选已经回到正确区域，误差为 `0.631871m / 0.418965deg`，但平均 overlap 降到 `0.867512`，低于严格的 `trajectory_likelihood_min_average_overlap=0.95`，所以仍被保守拒绝。直接降低 average overlap 不安全，因为旧 7 帧历史窗口里的错误候选 `average_overlap=0.902` 会被放行。

因此新增第三层补救规则：

```text
trajectory_single_agreement_fallback_enable: true
trajectory_single_agreement_max_fitness: 0.04
trajectory_single_agreement_max_xy_m: 1.0
trajectory_single_agreement_max_yaw_deg: 3.0
trajectory_single_agreement_min_overlap: 0.80
trajectory_single_agreement_min_margin: 0.005
```

含义是：严格 trajectory 门控失败时，只有当单帧 GICP fitness 很低，并且轨迹最佳候选与单帧最终位姿在 `x/y/yaw` 上高度一致，才允许第三层主动恢复放行。该规则不是简单相信单帧，也不是简单降低 overlap，而是要求两个独立证据互相同意。

回归结果：

| 窗口 | 输入模式 | 轨迹候选误差 | average overlap | single agreement | decision |
|---|---|---:|---:|---:|---|
| 旧 7 帧历史窗口 | body | 1.960729m / 6.108467deg | 0.902171 | 不满足 | candidate |
| 旧 7 帧历史窗口 | registered_world | 1.960729m / 6.108467deg | 0.902171 | 不满足 | candidate |
| 主动恢复 13 帧窗口 | body | 0.631871m / 0.418965deg | 0.867512 | 满足，xy 约 0.615m | accept |
| 主动恢复 13 帧窗口 | registered_world | 0.631871m / 0.418965deg | 0.867512 | 满足，xy 约 0.615m | accept |

对应产物：

```text
.codex_tmp/trajectory_rand80_reject12_avg_gate_v2
.codex_tmp/trajectory_rand80_reject12_active_view_v2
```

更新后的三层结论：

```text
第一层：短历史 selected support + refine fitness
  nav46 随机 80: accept=68/80, accepted_success=68/68, false_accept=0

第二层：长历史 trajectory likelihood + strict average overlap
  可额外救回 11/12 个短历史 reject，false_accept=0

第三层：主动恢复新视角 + 单帧高置信一致性 fallback
  可救回最后 1/12 个短历史 reject，旧历史错误候选仍拒绝

等效结果：
  nav46 随机 80 双路输入均可恢复 80/80，当前验证证据中 false_accept=0
```

对应门禁脚本：

```bash
python3 src/humanoid_global_relocalization_runtime/test/verify_nav46_three_stage_recovery.py
python3 src/humanoid_global_relocalization_runtime/test/run_nav46_hardframe_recovery_validation.py
```

为了避免 hard-frame 重跑时对历史辅助帧也逐帧做完整 trajectory likelihood，新增离线参数：

```text
trajectory_likelihood_center_frame_indices
```

该参数为空时保持原行为；填入目标 bag cloud frame index 后，历史/未来帧仍参与窗口打分，但只有目标帧会作为中心帧输出 trajectory likelihood。`run_nav46_hardframe_recovery_validation.py --run` 会自动写入这 12 个目标 index。最新完整重跑结果：

```text
long_history_accept=22 long_success=22 long_false_accept=0
active_accept=2 active_success=2 active_false_accept=0
```

这说明目标帧过滤没有改变补救结论，但能把 trajectory likelihood 输出压到目标难例数量级；当前耗时主要仍来自 3D-BBS 对历史/目标帧逐帧粗搜索。

继续做 top-K 成本对照：

```bash
python3 src/humanoid_global_relocalization_runtime/test/run_trajectory_likelihood_validation.py \
  --run \
  --bag /home/ubuntu/nav_drift_test/nav_drift_test46 \
  --matches src/humanoid_global_relocalization_runtime/test/nav46_rand80_reject_matches.csv \
  --output-root .codex_tmp/trajectory_rand80_rejects_long_history_top30_probe \
  --modes body registered_world \
  --history-offsets 240 200 160 120 80 40 0 \
  --top-k 30 \
  --trajectory-max-candidates 30 \
  --trajectory-min-average-overlap 0.95
```

结果：长历史 top30 两路仍然都是 `11/12` 安全救回，`false_accept=0`。被救回的点包含原单帧误差 `8.8m`、`21.1m / 92.8deg`、`33.7m / 163.2deg`、`35.6m / 158.8deg`、`13.1m / 159.8deg` 等真正危险失败点。唯一剩下的仍是 `rand80_reject_12`，且被正确保守拒绝：

```text
top30 long history:
  body / registered_world: 11/12 accept_success, false_accept=0
  rand80_reject_12: overlap≈0.903, decision=candidate
```

再对最后一个点做主动新视角 top30 对照：

```bash
python3 src/humanoid_global_relocalization_runtime/test/run_trajectory_likelihood_validation.py \
  --run \
  --bag /home/ubuntu/nav_drift_test/nav_drift_test46 \
  --matches src/humanoid_global_relocalization_runtime/test/nav46_rand80_reject_matches.csv \
  --ids rand80_reject_12 \
  --output-root .codex_tmp/trajectory_rand80_reject12_active_view_top30_probe \
  --modes body registered_world \
  --history-offsets 240 200 160 120 80 40 0 -40 -80 -120 -160 -200 -240 \
  --top-k 30 \
  --trajectory-max-candidates 30 \
  --trajectory-min-average-overlap 0.95
```

结果：主动新视角 top30 仍保守拒绝，候选误差约 `1.63m / 4.10deg`、margin 约 `0.0023`，不满足发布门控；而 top60 主动新视角可恢复到 `0.632m / 0.419deg` 并 accept。

因此当前更合理的成本策略是：

```text
第一层：短历史 selected support + refine fitness，使用常规 top30
第二层：长历史 trajectory likelihood，仍使用 top30，已救回 11/12 hard reject
第三层：只有仍 need_active_view 的最后歧义点，主动采集新视角后升到 top60 深搜
```

这条策略兼顾了用户关心的 CPU 问题和“精度/效果第一”：绝大多数恢复不需要 top60，只有 top30 明确保守拒绝且状态机已经进入主动恢复时，才付出深搜成本。

资源代价：

```text
旧 7 帧窗口 v2:
  body median total 7140.6ms, median CPU 11413.3ms, max RSS 330.3MB
  registered_world median total 7462.2ms, median CPU 11835.3ms, max RSS 330.9MB

主动恢复 13 帧窗口 v2:
  body median total 5056.9ms, median CPU 8033.3ms, max RSS 331.0MB
  registered_world median total 4486.7ms, median CPU 7129.9ms, max RSS 330.8MB
```

注意：第三层依赖目标之后的新视角，不能用于“刚开机完全静止且没有历史/新视角”的瞬时恢复；它对应的是状态机进入 recovery 后，让机器人低速采样几帧再重定位。
