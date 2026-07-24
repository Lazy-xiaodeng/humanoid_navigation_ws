# humanoid_global_relocalization_runtime

## 功能包作用

`humanoid_global_relocalization_runtime` 是机器人导航系统中的全局重定位运行层 C++ 功能包。它在机器人任意点启动、局部定位大跳、漂移或丢失后，使用当前 3D LiDAR scan、Fast-LIO odom 和先验 PCD 地图搜索正确的 `map -> base_footprint` 位姿，并输出经过多层门控确认的恢复候选。

当前节点默认不直接发布 `map -> odom` TF，也不直接注入 `initialpose`。它会发布 `recovery_pose`、`recovery_map_to_odom` 和 `recovery_status`，由后续恢复状态机或定位桥审核后再决定是否真正改写导航闭环。这样可以把“搜索候选”和“改变机器人定位状态”分成两个清晰边界。

## 正式集成入口

正式系统集成使用 [relocalization_runtime.yaml](/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_global_relocalization_runtime/config/relocalization_runtime.yaml)。这份配置只保留在线运行需要的参数，默认 `enable_relocalization=true`，适合由恢复状态机按需启动或放在低频恢复通道中运行。

离线 bag 回归、CSV 统计、参数扫描使用 [relocalization_validation.yaml](/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_global_relocalization_runtime/config/relocalization_validation.yaml) 或 `test/` 下脚本生成的临时 YAML。当前 `config/` 目录只保留这两份固定入口：一份正式运行，一份测试验证。

上线时建议只把这些接口接入上层系统：

- 输入：`/fast_lio/cloud_registered`、`/odom`，或确认轴系后的 `/cloud_registered_body`。
- 输出：`/global_relocalization/recovery_pose`、`/global_relocalization/recovery_map_to_odom`、`/global_relocalization/recovery_status`。
- 运维观察：`/global_relocalization/candidates`、`/global_relocalization/candidate_markers`、`/global_relocalization/aligned_scan`。

不要让导航栈直接消费 `/global_relocalization/candidates`，也不要把本节点输出自动等同于 TF 重置。恢复状态机应先检查 `recovery_status` 中的 `state`、`refined_converged`、`refined_fitness`、`selected_support`、`trajectory_*` 字段，再决定是否应用 `recovery_map_to_odom`。

当前正式运行模板默认启用的核心链路是：

```text
3D-BBS global search
  + GICP top8 refinement
  + temporal map->odom support gate
  + trajectory likelihood fallback
  + precision recovery layer on reject / weak-accept risk
```

正式默认层不常开 2.5D。高精度层在默认层拒绝或弱接受风险分 `precision_layer_trigger_risk_score` 达标时介入，使用 `Scan Context DB + scan0.25 + GICP top30` 做复核。Scan Context 在默认层仍关闭；只有当部署地图已经有稳定的 descriptor/keyframe 数据库时，才填写 `precision_layer_scan_context_database_path` 或全局 `scan_context_database_path`。

正式源码还包含一个多初值连续跟踪兜底层，参数入口为 `enable_multi_seed_recovery`，默认关闭。开启后，它只在默认层没有发布、精度层已经实际尝试仍未发布（或精度层数据库不可用）、trajectory 也未裁决时运行。该层先把同一失败帧中已经精配的 BBS/SOLiD 候选按来源和位姿聚类，为每个候选建立独立 `map->odom` 轨迹；后续帧使用 odom 传播预测，首次执行局部 SE(2) 搜索和粗 GICP，此后执行 0.10m 细 GICP。滚动 12 帧内至少 9 帧有效，并且至少两条同来源、同初始簇轨迹的 `map->odom` 在 `0.08m/2deg` 内一致时，才发布 `accepted_multi_seed` 候选。后续仍必须经过 RO trusted commit 和 Bridge 单一 TF 发布链路。

多初值计算在后台执行，最多一个批次在途。`cancel`、地图切换或新 attempt 会递增代次，旧后台结果即使晚到也会被丢弃。每个精配线程使用独立 PCL 配准器，只共享只读地图和已经预处理的当前 scan，避免阻塞 ROS 主线程或产生配准器数据竞争。

## 上下游关系

上游输入分为两种模式，由 [relocalization_runtime.yaml](/home/ubuntu/software/Todesk/Files/humanoid_ws/src/humanoid_global_relocalization_runtime/config/relocalization_runtime.yaml) 参数选择：

- `registered_world`：订阅 `/fast_lio/cloud_registered` 和 `/odom`。点云位于 Fast-LIO 的 `camera_init` raw world 坐标系，需要用同一时刻 `/odom` 反变换回 raw body。
- `body`：订阅 `/cloud_registered_body`。点云位于 Fast-LIO raw body 坐标系，不需要反算 odom，但仍需要转换到 ROS 标准 `base_footprint`。

两种输入都会统一成：

```text
base_footprint 标准轴当前 scan
x 前，y 左，z 上
```

下游输出：

- `/global_relocalization/candidates`：top-K 全局候选位姿，类型 `geometry_msgs/PoseArray`。
- `/global_relocalization/candidate_markers`：top-K 候选可视化 marker，类型 `visualization_msgs/MarkerArray`。
- `/global_relocalization/aligned_scan`：当前 scan 按最佳候选对齐后的点云，类型 `sensor_msgs/PointCloud2`。
- `/global_relocalization/recovery_pose`：确认后的 `map -> base_footprint` 恢复候选，类型 `geometry_msgs/PoseStamped`。
- `/global_relocalization/recovery_map_to_odom`：确认后的 `map -> odom` 恢复量候选，类型 `geometry_msgs/PoseStamped`。
- `/global_relocalization/recovery_status`：恢复状态和拒绝原因，类型 `std_msgs/String`。

## 坐标系关系

Fast-LIO 当前链路里：

```text
/fast_lio/cloud_registered = /cloud_registered remap 后的话题
frame_id = camera_init
点云 = T_camera_init_body * p_body

/cloud_registered_body
frame_id = body
点云 = p_body
```

两者理论关系为：

```text
p_body = inverse(T_camera_init_body) * p_camera_init
```

但两者仍然都是 Fast-LIO raw 轴，不是 ROS 标准轴。当前工程约定：

```text
raw/body:       x 左，y 下，z 后
base_footprint: x 前，y 左，z 上
```

因此本包在算法前统一做：

```text
p_base.x = -p_raw.z
p_base.y =  p_raw.x
p_base.z = -p_raw.y
```

## 工作原理

## 方案选型依据

调研和本地 bag 验证后，本包没有把 scantext、hdl_localization 或纯 NDT 当成主恢复方案，原因如下：

- `scantext/Scan Context` 一类方法更适合做地点召回或闭环候选检索。它能告诉系统“这帧 scan 像地图里的哪个地方”，但通常还需要额外的局部配准或位姿优化才能得到可直接重置定位的 `map -> base_footprint` 连续位姿。
- `hdl_localization` 一类 NDT 定位器适合在已有较好初值时做连续定位修正。它依赖预测位姿附近的局部收敛，机器人任意点启动或定位已经大幅漂移时，单靠 NDT 容易收敛到局部错误位置。
- `3D-BBS` 的目标正好是单帧 3D LiDAR scan 相对先验 3D 点云地图的全局定位。它用分支定界在全地图搜索粗位姿，比“拿一个错误初值直接跑局部配准”更适合救任意点启动和大跳丢失。
- 本包采用 `3D-BBS 粗搜索 + Top-K 候选 + GICP/ICP/NDT 精匹配 + 多帧 map->odom 一致性确认`。这样既保留全局召回能力，又避免单帧重复结构把错误候选直接注入导航。

因此当前运行层采用“全局候选召回 + 局部精配准 + 多帧/轨迹门控”的结构：先尽量找到正确候选，再严格拒绝危险候选。

## 算法流程

本包 vendoring 并链接 KOKIAOKI/3d_bbs 的官方 CPU 后端，CUDA 后端当前关闭。3D-BBS 的输入是重力方向大致对齐的一帧 3D scan 和预建点云地图，核心是分支定界：

1. 先把 PCD 地图构造成多分辨率 voxel map。
2. 在整张地图范围内枚举粗粒度平移和姿态格子。
3. 用粗分辨率 voxel 命中数量作为上界评分，低分区域提前剪枝。
4. 对高分区域继续细分到更高分辨率。
5. 保留 top-K 高分 leaf 候选，输出每个候选的 `map -> base_footprint` 粗位姿和 voxel 命中分数。
6. 可选对 top-K 候选分别使用 ICP/GICP/NDT 做局部精配准，再按 fitness 选择最终位姿。

vendored CPU 后端已经扩展出 `set_top_k()`、`get_top_poses()` 和 `get_top_scores()`，因此运行节点可以输出 top-K 候选，离线回归工具也可以复用同一套候选数据。

多帧一致性模块会把每个候选换算成隐含 `map->odom`，并统计该假设在历史窗口中被多少帧 top-K 候选共同支持。运行节点每次搜索成功后，会把当前 top-K 候选、同帧 `/odom` 和预处理后的 scan 放入滑动窗口；发布恢复候选前优先要求“当前最终候选 selected candidate 自身”的隐含 `map->odom` 在历史窗口中达到 `temporal_consistency_online_min_support_frames`。`best_seed` 只作为审计字段，不会直接替换 selected 发布，因为扩大压测证明 best_seed 可能形成稳定错误簇。

如果第一层 selected 支持不足，或 selected 精配后 fitness 超过 `temporal_consistency_online_max_refine_fitness`，节点会进入第二层 trajectory recovery：对滑动窗口内的历史 top-K 候选计算 trajectory likelihood，用 `trajectory_likelihood_min_average_overlap`、`trajectory_likelihood_min_margin` 和 `trajectory_single_agreement_*` 门控筛选候选。通过后发布到 `/global_relocalization/recovery_*` 话题；不通过时只发布 reject 状态。

在 selected 支持帧不足且 trajectory 也没有安全发布时，节点还提供一个很窄的 `single_frame_high_confidence_fallback`：只有当前单帧精配准 fitness 低于 `single_frame_high_confidence_max_fitness` 才会接受。它用于补救“单帧已经非常贴图、只是历史支持还没攒够”的冷启动点；重复走廊里 fitness 较高或候选歧义明显的帧仍会被拒绝，等待长历史或主动恢复新视角。

新增的 Scan Context 召回模块位于 `scan_context.hpp/.cpp`。离线 evaluator 可通过 `enable_scan_context_recall=true` 从 bag 中按 `scan_context_keyframe_stride` 抽 keyframe 建描述子库，查询当前 scan 的 top-K 相似 keyframe，并把这些 keyframe 位姿转成额外 GICP seed。该能力不直接替代 3D-BBS，而是作为高精度恢复层候选补充：第一阶段使用当前较轻的 `scan_leaf_size=0.30 + max_refine_candidates=8`；当默认层拒绝或弱接受风险分达到阈值时，再启用 `scan_leaf_size=0.25 + max_refine_candidates=30 + Scan Context top-K`。bag46 回归中，`risk>=3` 触发精度层复核可把 174 个综合样本的 `0.3m/5deg` 恢复成功从轻默认单独的 `150/174` 提升到 `165/174`，误接受从 `9` 降到 `5`。当前 C++ 验证版会临时缓存 keyframe 点云消息建库，峰值内存约 1.3GB；上线前应改为建图阶段离线生成轻量 descriptor/keyframe 文件，运行时只加载描述子和 keyframe 位姿。

STD/BTC 三角描述子方向已新增离线验证脚本 `test/run_triangle_descriptor_validation.py`。该脚本不是 HKU-MARS STD/BTC 的完整移植，而是先用“关键点三角边长直方图 + keyframe seed + yaw 多假设 + GICP”验证三角几何指纹能否作为第三阶段兜底。bag46 hard52 对照中，Scan Context DB 版主阈值 `0.3m/5deg` 为 `32/52`，当前简化三角描述子为 `30/52`，两者 union 为 `33/52`：它额外救回 `5878` 这一帧，但会错过 `3301/29762/35520` 三个 Scan Context 已成功帧。因此当前结论是：简化三角直方图不适合替代 Scan Context 或直接进入主链路；完整 STD/BTC 若要继续尝试，应重点移植其稳定关键点、局部二进制描述和描述子匹配后的几何一致性验证，而不是只用全局三角边长直方图。

随后又在 `.codex_tmp/official_std_standalone` 中拉取并编译了 HKU-MARS 官方 STD/BTC 核心，使用 ROS1 stub 保留官方 `Generate*Descs -> SearchLoop -> Add*Descs` 完整算法链路。对 bag46 hard52 做了三种 STD 输入验证：单帧默认 Livox 参数、单帧室内化参数、10 帧 odom 对齐 submap 且保留完整高度/30m 半径。前两种输入几乎全部 `desc=0`；完整高度 10 帧 submap 偶尔能提到 `1~4` 个 descriptor，但 52 个目标中只有 1 个被 localized，且误差约 `7.37m / 70.63deg`，属于错误接受。BTC 使用同一份完整高度 10 帧 submap 时能提取较多 binary/triangle descriptor，但官方 `SearchLoop` 对 52 个目标全部拒绝，`localized=0/52`。因此当前证据表明：官方 STD/BTC 完整体与当前 Airy/Fast-LIO 室内点云、短历史 submap 和防泄漏 keyframe 库的匹配度不足，不能作为提高冷启动成功率的直接路线。

```text
/global_relocalization/recovery_pose
/global_relocalization/recovery_map_to_odom
/global_relocalization/recovery_status
```

`recovery_pose` 类型为 `geometry_msgs/PoseStamped`，语义是精配准后的 `map->base_footprint` 候选。`recovery_map_to_odom` 也是 `geometry_msgs/PoseStamped`，语义是 `refined_map_to_base * inverse(current_odom_to_base)` 得到的 `map->odom` 恢复量。节点不会直接发布 TF，不会直接注入 `initialpose`，也不会直接影响导航。

`recovery_pose` 发布前会复用同一套 `refiner` 模块，对被多帧确认的候选执行一次配置中的 `refine_method`，例如 GICP。也就是说，运行态输出的是“多帧一致性或 trajectory recovery 选出的候选 + 局部精配准/单帧一致性确认”后的位姿，而不是纯 BBS 粗格子中心。

## 第三方代码

`third_party/3d_bbs` 来自 KOKIAOKI/3d_bbs，保留其 MIT License。当前 CMake 中强制 `BUILD_CUDA=OFF`，只构建 `cpu_bbs3d`。我们在 vendored CPU 后端上做了很小的工程扩展：保留 top-K leaf 候选并导出位姿/分数；算法主体仍然是 3D-BBS 的分支定界搜索。

## 外部资料

本包方案参考了以下公开资料，并结合当前机器人室内 bag 重新验证参数：

- 3D-BBS 官方仓库：`https://github.com/KOKIAOKI/3d_bbs`
- 3D-BBS 论文：`https://arxiv.org/abs/2310.10023`
- Scan Context 官方实现：`https://github.com/gisbi-kim/scancontext_tro`
- STD 官方仓库：`https://github.com/hku-mars/STD`
- STD 论文：`https://arxiv.org/abs/2209.12435`
- BTC Descriptor 官方仓库：`https://github.com/hku-mars/btc_descriptor`
- KISS-Matcher 官方仓库：`https://github.com/MIT-SPARK/KISS-Matcher`
- BEVPlace++ 官方仓库：`https://github.com/zjuluolun/BEVPlace2`
- RING++ 官方仓库：`https://github.com/lus6-Jenny/RING`
- NDT-Map-Code 官方仓库：`https://github.com/SlamCabbage/NDTMC`
- mcl_3dl 官方仓库：`https://github.com/at-wat/mcl_3dl`
- Reliable-loc 官方仓库：`https://github.com/zouxianghong/Reliable-loc`
- Area Graph 论文：`https://arxiv.org/abs/2308.05593`
- FAST-LIO 官方仓库：`https://github.com/hku-mars/FAST_LIO`
- hdl_localization 官方仓库：`https://github.com/koide3/hdl_localization`

## 使用方法

先编译：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select humanoid_global_relocalization_runtime
```

正式运行态配置使用 `config/relocalization_runtime.yaml`，测试验证配置使用 `config/relocalization_validation.yaml`。安装后的 share 目录只包含这两份 YAML、launch 和 README；bag 回归、资源统计和参数扫描工具保留在源码 `test/` 目录中，用于上线前复测，不作为运行态安装接口。

扫描当前机器上的 bag 话题能力时，可使用源码中的辅助脚本：

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

该脚本只读取 `metadata.yaml`，会输出每个 bag 是否具备 `/fast_lio/cloud_registered + /odom`、是否有 `/robot_realpose`、是否有真实 `/cloud_registered_body`。新增 bag 后建议先跑这个脚本，再决定能做 registered_world、body 还是无真值 smoke。

如果需要刷新目标证据矩阵中的“当前机器最新 bag 状态”，使用下面两个命令生成 latest inventory。它们只读扫描 metadata，不播放 bag，也不会写入 `/home/ubuntu/humanoid_ws`：

```bash
source /opt/ros/jazzy/setup.bash
python3 src/humanoid_global_relocalization_runtime/test/scan_bag_inventory.py \
  --root /home/ubuntu \
  --exclude /home/ubuntu/humanoid_ws \
  --exclude /home/ubuntu/software/Todesk/Files/humanoid_ws/.codex_tmp \
  --csv .codex_tmp/bag_inventory_home_scan_latest.csv \
  --md .codex_tmp/bag_inventory_home_scan_latest.md

python3 src/humanoid_global_relocalization_runtime/test/scan_bag_inventory.py \
  --root /home/ubuntu/nav_drift_test \
  --csv .codex_tmp/bag_inventory_nav_drift_latest.csv \
  --md .codex_tmp/bag_inventory_nav_drift_latest.md
```

启动全局重定位运行层节点：

```bash
source install/local_setup.bash
ros2 launch humanoid_global_relocalization_runtime global_relocalization_runtime.launch.py
```

默认 `enable_relocalization=false`，节点只加载配置，不做重计算。需要实时运行重定位时，在 YAML 中打开 `enable_relocalization`。打开后节点会按 `input_mode` 订阅点云链路，发布：

- `/global_relocalization/candidates`：`geometry_msgs/PoseArray`，top-K 候选位姿。
- `/global_relocalization/candidate_markers`：`visualization_msgs/MarkerArray`，箭头和 rank/score 文本。
- `/global_relocalization/aligned_scan`：best 粗候选对齐后的当前 scan 点云。
- `/global_relocalization/recovery_pose`：`geometry_msgs/PoseStamped`，多帧一致性确认并经过配置精配准后的 `map->base_footprint` 恢复候选。
- `/global_relocalization/recovery_map_to_odom`：`geometry_msgs/PoseStamped`，由 recovery `map->base_footprint` 和同帧 `/odom` 计算出的 `map->odom` 恢复量候选。
- `/global_relocalization/recovery_status`：`std_msgs/String`，记录 `accepted`、`accepted_trajectory`、`accepted_trajectory_single_agreement`、`reject_selected_support_below_threshold` 或 `need_active_view_*` 等状态，以及支持帧数、候选 rank、fitness、`map_odom_x/y/yaw_deg`、trajectory overlap/margin、`recovery_hint` 等审计信息。`need_active_view_*` 表示当前证据不足以安全发布恢复位姿，上层状态机应考虑主动采集新视角后再触发。

这些话题是恢复候选和状态输出；节点本身不会发布 TF，也不会注入 `initialpose`。

运行态回归工具：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
python3 src/humanoid_global_relocalization_runtime/test/run_online_smoke_matrix.py
python3 src/humanoid_global_relocalization_runtime/test/verify_online_smoke_evidence.py
```

这些脚本用于回放已有 bag、检查 `recovery_pose`、`recovery_map_to_odom` 和 `recovery_status` 是否按预期输出。它们属于开发回归入口，不随安装包进入运行态 share 目录。
运行态回归矩阵结果保存在 `src/humanoid_global_relocalization_runtime/test/online_smoke_evidence.csv`。

长回归完成后，校验 temporal decision CSV：

```bash
python3 src/humanoid_global_relocalization_runtime/test/verify_temporal_decisions.py \
  .codex_tmp/global_relocalization_nav_drift_extended_cpp_temporal_support2_real_v1/global_relocalization_temporal_decisions.csv
```

历史 support>=2 回归门禁曾证明 best_seed 策略可以救回部分 hard case，但 bag46 60 点扩大压测发现该策略会产生稳定错误簇误接受。当前代码已改为 selected candidate 自身支持度门控；旧 best_seed CSV 字段只保留作诊断，不作为发布对象。

校验只看历史帧的因果 temporal decision CSV：

```bash
python3 src/humanoid_global_relocalization_runtime/test/verify_temporal_decisions.py \
  .codex_tmp/global_relocalization_nav_drift_extended_cpp_temporal_causal_v1/global_relocalization_temporal_decisions.csv \
  --expected-accept 25 \
  --expected-reject 5 \
  --expect-causal-warmup-rejects
```

当前因果回归口径仍只看历史帧；后续门禁应以 selected candidate 支持度为准，优先保证 `false_accept=0`，再逐步考虑是否引入更强的候选二次验证来救回 selected 错误但 top-K 内有正确稳定簇的场景。

单独校验 registered_world 与合成 body 同帧一致性：

```bash
python3 src/humanoid_global_relocalization_runtime/test/compare_input_modes.py \
  --registered-csv .codex_tmp/global_relocalization_nav_drift_extended_cpp_temporal_support2_real_v1/global_relocalization_metrics.csv \
  --body-csv .codex_tmp/global_relocalization_synthetic_body_nav43_v1/global_relocalization_metrics.csv
```

当前 5 帧对齐结果为 `max_xy_diff=0.000001m`、`max_yaw_diff=0.000003deg`、`max_error_diff=0.000001m`，说明由 registered_world 派生出的合成 body 链路与原链路几乎逐位一致。该结论只证明本包坐标转换/输入分支自洽，仍不能替代真实 `/cloud_registered_body` bag。

校验中文文档/注释契约：

```bash
python3 src/humanoid_global_relocalization_runtime/test/check_documentation_contract.py
```

该脚本会检查功能源码和测试脚本文件头、主 YAML 顶层参数中文注释、README 与验证报告关键章节，防止后续改代码时丢失可维护性和复现说明。

校验工作空间边界：

```bash
python3 src/humanoid_global_relocalization_runtime/test/check_workspace_boundaries.py
```

该脚本会检查运行相关源码、配置、测试脚本和 bag inventory，防止误把 `/home/ubuntu/humanoid_ws` 作为输入、输出或依赖路径；文档里只允许以“未修改/未使用”的语义提到该目录。

校验目标证据矩阵：

```bash
python3 src/humanoid_global_relocalization_runtime/test/check_goal_evidence.py
```

该脚本把用户目标中的显式要求映射到当前代码、配置、bag inventory、验证 CSV 和门禁脚本。若 latest inventory 没有真实 `/cloud_registered_body`，它会把真实 body 项标记为 `KNOWN_LIMIT`，避免把合成 body 自洽验证误认为真实 body 准确率；若 latest inventory 发现真实 body bag，它会要求存在 `.codex_tmp/global_relocalization_real_body_validation_v1/global_relocalization_metrics.csv`，并检查真实 body bag 是否已被覆盖。若该 bag 同时有 `/robot_realpose`，还会要求对应参考帧成功率不低于 `0.90`，并额外检查 30 帧 body/registered_world 对照的成功失败是否一致。当前 `nav_drift_test46` 已满足这一项。

补录真实 `/cloud_registered_body` bag 后，先运行总门禁刷新 latest inventory，再运行真实 body 验证脚本：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
python3 src/humanoid_global_relocalization_runtime/test/run_validation_gates.py
python3 src/humanoid_global_relocalization_runtime/test/run_real_body_validation.py --run
python3 src/humanoid_global_relocalization_runtime/test/run_validation_gates.py
```

`run_real_body_validation.py` 会从 `.codex_tmp/bag_inventory_home_scan_latest.csv` 中提取真实 body bag，生成 `.codex_tmp/real_body_validation_config.yaml`，并把结果写入 `.codex_tmp/global_relocalization_real_body_validation_v1`。如果 latest inventory 仍没有真实 body bag，它会输出 `KNOWN_LIMIT` 并正常退出。

校验精匹配方法和多地图 sweep 证据：

```bash
python3 src/humanoid_global_relocalization_runtime/test/verify_refine_sweep.py
```

该脚本读取历史 focused/baseline CSV：focused 用于确认 `none -> ICP -> GICP` 的误差改善关系，baseline 用于确认 `NDT` 和 `hall.pcd/hall_open3d_grounded.pcd` 多地图候选机制确实跑通过。

校验资源消耗统计字段：

```bash
python3 src/humanoid_global_relocalization_runtime/test/verify_resource_metrics.py
```

该脚本检查推荐 30 帧、因果 30 帧和合成 body 结果中的 `total_ms`、`delta_user_cpu_ms`、`delta_system_cpu_ms`、`rss_mb`、`peak_rss_mb`、`thread_count` 等字段，确保资源记录没有在后续改动中丢失。

关键验证产物总门禁：

```bash
python3 src/humanoid_global_relocalization_runtime/test/run_validation_gates.py
```

该脚本会先只读刷新 latest bag inventory，然后检查源码/配置/报告是否齐全，读取 bag inventory，验证真实 nav_drift registered_world 30 帧主指标、temporal decision hard case、合成 body 链路结果。它会明确提示当前是否缺少真实 `/cloud_registered_body` bag，避免把合成 body 自洽验证误认为真实 body 准确率。

## CSV 指标说明

主指标 CSV 每行对应一次 scan 在一个模拟重定位场景下的最终结果：

- `scenario_name`：模拟场景名，例如任意点启动、3m 大跳、5m+90 度大跳。
- `simulated_prior_*`：模拟外部定位先验偏差。3D-BBS 不使用这个先验，因此它只用于统计分类。
- `input_mode`：输入模式，`registered_world` 或 `body`。
- `localized`：3D-BBS 是否认为定位成功。
- `score`：voxel 命中分数。
- `score_ratio`：`score / scan_points`，越高表示 scan 与地图重合越多。
- `final_x_m` / `final_y_m` / `final_z_m` / `final_yaw_deg`：best 候选经可选 GICP 后的最终输出位姿。
- `reference_x_m` / `reference_y_m` / `reference_z_m` / `reference_yaw_deg`：本帧匹配到的参考位姿。该字段用于判断误差是算法问题，还是 bag 内参考源本身异常。
- `reference_rejected_reason`：如果参考位姿被合理性检查拒绝，会记录拒绝原因，例如超出室内地图范围；被拒绝的参考不参与有效成功率。
- `translation_error_m`：如果配置了参考位姿，则记录估计位姿和平面参考位姿的平移误差。
- `yaw_error_deg`：如果配置了参考位姿，则记录 yaw 误差。
- `map_points` / `scan_points`：地图和 scan 进入搜索前的点数。
- `build_index_ms` / `search_ms` / `refine_ms` / `total_ms`：各阶段耗时。
- `user_cpu_ms` / `system_cpu_ms`：进程累计 CPU 时间，用于确认整体进程资源趋势。
- `delta_user_cpu_ms` / `delta_system_cpu_ms`：本帧评估前后的 CPU 时间差，更适合比较单次全局重定位的 CPU 消耗。
- `rss_mb` / `peak_rss_mb`：当前和峰值常驻内存。
- `thread_count`：当前进程线程数。

候选明细 CSV 每行对应一个 3D-BBS 粗搜索候选：

- `rank`：候选排名，1 为最高分。
- `candidate_x_m` / `candidate_y_m` / `candidate_z_m` / `candidate_yaw_deg`：粗搜索候选位姿。
- `score` / `score_ratio`：该候选的 voxel 命中分数和比例。

汇总 CSV 每次离线运行追加一行：

- `total_frames` / `localized_frames` / `success_frames`：本次评估帧数、3D-BBS 成功给出候选的帧数、满足参考位姿阈值的帧数。
- `localized_rate`：`localized_frames / total_frames`，用于没有真值时先观察算法是否稳定给出候选。
- `success_rate`：`success_frames / total_frames`，会受无参考或参考被拒绝帧影响。
- `success_over_reference_rate`：`success_frames / reference_frames`，只按有效参考帧计算，更适合评估算法真实成功率。

时序一致性 CSV 每行对应一帧的候选稳定性后处理：

- `selected_rank`：单帧精配准最终选中的 3D-BBS 粗候选 rank。
- `selected_support_frames`：在时序窗口内，有多少帧 top-K 候选支持这个被选中的 `map->odom` 假设。
- `selected_median_rank`：支持该假设的各帧中，最靠前候选 rank 的中位数；越小表示该假设在多帧中排名越靠前。
- `best_seed_rank`：当前帧 top-K 中跨帧支持度最高的候选 rank。
- `best_support_frames`：最佳时序簇被多少帧支持。
- `selected_map_odom_x/y/yaw`：被单帧精配准选中候选隐含的 `map->odom`，用于判断重置假设是否跨帧稳定。
- `selected_candidate_translation_error_m` / `selected_candidate_yaw_error_deg`：单帧精配准最终选择的粗候选，在 BBS 粗位姿层面对参考位姿的误差。该字段用于判断“被选中的候选本身是否在正确区域”，不代表最终 GICP 后误差。
- `best_seed_translation_error_m` / `best_seed_yaw_error_deg`：跨帧支持最强候选在 BBS 粗位姿层面对参考位姿的误差。它能直接评估多帧一致性是否把候选从错误区域拉回正确区域。

## nav_drift_test 当前验证结论

完整记录见 [test/nav_drift_validation_report.md](test/nav_drift_validation_report.md)。历史推荐参数回归主要使用的 bag 为：

```text
/home/ubuntu/nav_drift_test/nav_drift_test43
/home/ubuntu/nav_drift_test/nav_drift_test44
/home/ubuntu/nav_drift_test/nav_drift_test45
```

这些 bag 均包含 `/fast_lio/cloud_registered`、`/odom`、`/robot_realpose`，但不包含 `/cloud_registered_body`。因此当前实测结论只覆盖 `registered_world` 输入模式；`body` 输入模式需要后续补录带 `/cloud_registered_body` 的 bag 再验证。

随后补录了真实 body bag：

```text
/home/ubuntu/nav_drift_test/nav_drift_test46
```

该 bag 同时包含 `/fast_lio/cloud_registered`、`/cloud_registered_body`、`/odom`、`/robot_realpose`。最新 latest inventory 显示当前 `/home/ubuntu/nav_drift_test` 覆盖 `nav_drift_test44/45/46`，其中 `nav_drift_test46` 是第一组真实 body+reference 验证 bag。

无 `/robot_realpose` 的 4 个 registered_world bag 不能计算准确率，但已通过 `extra_registered_world_no_reference_smoke.yaml` 做链路冒烟验证：7 个抽样帧、3 类模拟场景，共 21 行全部 `localized=1`。这部分只证明输入链路、BBS 搜索、GICP 精配准和资源统计可以跑通，不纳入最终成功率。

当前推荐运行配置已经同步到 `config/relocalization_runtime.yaml`，核心参数为：

```text
map: hall_open3d_grounded.pcd
map_leaf_size: 0.25
scan_leaf_size: 0.30
enable_map_z_crop: true
enable_scan_z_crop: true
z_crop: 0.2m ~ 2.5m
bbs_min_level_res: 0.75
bbs_max_level: 5
bbs_num_threads: 8
top_k: 30
refine_method: gicp
max_refine_candidates: 20
refine_max_iterations: 25
refine_max_correspondence_distance: 1.5
temporal_consistency_online_min_support_frames: 2
temporal_consistency_online_max_refine_fitness: 0.12
single_frame_high_confidence_max_fitness: 0.02
trajectory_likelihood_min_average_overlap: 0.95
trajectory_likelihood_min_margin: 0.0015
```

在 3 个 nav_drift bag、15 个抽样帧上的当前结果：

| 参数组 | 成功率 | median 平移误差 | 最大平移误差 | 最大 yaw 误差 | median total | median CPU 增量 | 最大 total | 峰值 RSS |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| focused / GICP | 13/15 | 0.054m | 41.258m | 86.417deg | 768.8ms | 未记录 | 2784.1ms | 327.0MB |
| precision / GICP | 15/15 | 0.051m | 0.367m | 3.954deg | 5133.2ms | 未记录 | 44497.2ms | 432.8MB |
| balanced / GICP | 15/15 | 0.058m | 0.336m | 3.797deg | 3072.6ms | 未记录 | 12439.6ms | 328.5MB |
| fast_recall / GICP | 14/15 | 0.056m | 5.467m | 176.300deg | 2326.4ms | 未记录 | 5502.7ms | 329.4MB |
| balanced_light / GICP | 14/15 | 0.042m | 10.532m | 3.398deg | 2504.2ms | 未记录 | 11783.9ms | 329.8MB |
| balanced_iter25 / GICP | 15/15 | 0.058m | 0.336m | 3.797deg | 3094.5ms | 6519.4ms | 12412.5ms | 334.8MB |

扩大抽样后，推荐配置在 `nav_drift_test43/44/45` 每包 10 帧、共 30 帧上的结果为：

```text
localized: 30/30
有效参考成功率: 27/30
median 平移误差: 0.060m
median yaw 误差: 0.197deg
median total: 4441.4ms
median CPU 增量: 10698.2ms
峰值 RSS: 371.9MB
```

3 个失败帧集中在重复结构 hard case。针对这些帧做了补充验证：`0.5m BBS + top60/refine40`、`0.75m BBS + top100/refine60` 和简单 7 帧累积 scan 都不能稳定全部救回；其中一个 nav43 hard frame 可由 7 帧累积恢复，但另一个 nav43 和 nav45 hard frame 仍失败。因此第一版推荐配置适合作为离线验证和后续接入基础，但真正上线前还需要多帧一致性确认、候选验证/拒绝策略，避免单帧重复结构误匹配直接重置定位。

对 30 帧扩展验证又做了候选质量分析，脚本为：

```text
test/analyze_candidate_quality.py
```

该脚本会把主指标 CSV 和 top-K 候选 CSV 合并，统计最终 refine rank、fitness、top-K 分数并列数量、候选空间分散度、离参考位姿最近的候选排名等指标。当前结果显示：失败帧整体上更容易出现候选分散、正确候选排名靠后、fitness 偏高等现象；但是这些现象不能直接变成可靠的单帧硬阈值。典型反例如下：

- nav43 有一帧 top30 全部集中在错误区域，BBS 分数高度并列，看起来并不发散，但最终错到 34m。
- nav45 有一帧 top30 中存在接近真实位姿的 rank26 候选，但 GICP fitness 选择了错误 rank20。
- 部分成功帧本身也会出现较大的候选分散度或较靠后的 refine rank。

因此后续上线策略不应该是“单帧搜索到一个最佳候选就立刻重置 `map -> odom`”。更稳妥的方向是连续多帧搜索、对 top-K 候选做时序聚类和运动一致性检查，再结合定位状态机决定是否发布全局重置。

当前还新增了离线多帧一致性分析脚本：

```text
test/analyze_temporal_consistency.py
```

它会从原 bag 读取 `/odom`，把每个粗候选换算成隐含的 `map->odom`，再检查当前帧前后各 2 个抽样帧内这个 `map->odom` 假设是否稳定出现。这个逻辑现在已经同步内置到 C++ evaluator；Python 脚本保留为独立交叉检查工具。C++ 回归配置：

```text
test/nav_drift_registered_world_extended_cpp_temporal.yaml
```

30 帧 C++ 回归验证的结果为：

```text
成功帧:
  median selected_support_frames = 4
  median best_support_frames = 5

失败帧:
  median selected_support_frames = 1
  median best_support_frames = 1
```

这说明 3 个失败帧里，GICP 最终选中的错误候选都缺少跨帧支持。更重要的是，nav45 的 hard frame 中，GICP 选错的 rank20 只有 1 帧支持，而 rank26 正确区域候选形成了 5 帧支持的稳定 `map->odom` 簇。因此多帧候选聚类不仅可以拒绝错误重置，也有机会从 top-K 中重新挑出更可信的候选。

随后又把 temporal CSV 扩展出 selected/best_seed 粗候选误差字段。30 帧 C++ 回归中，nav45 hard frame 的单帧错误 rank20 粗候选误差为 `25.03m / 88.46deg`，而多帧 best_seed rank26 粗候选误差为 `0.54m / 1.54deg`。nav43 两个 hard frame 的 best_seed 仍然不接近真值，因此更适合被时序一致性策略拒绝，而不是强行发布重置。

最新 evaluator 还会输出 `global_relocalization_temporal_decisions.csv`，用于模拟在线恢复策略：只有 `best_support_frames >= temporal_consistency_online_min_support_frames` 时才接受 best_seed，并对该候选执行共享 GICP/ICP/NDT 精配准。离线双向窗口 `window_before=2/window_after=2` 可作为诊断上限：30 帧回归中 `support>=2` 时 `28` 帧 accepted 且 `28/28` refined success，`2` 帧 rejected；其中 nav45 hard frame 被 accepted 并 refine 到 `0.088m / 0.067deg`，两个危险 nav43 hard frame 被 rejected。

为了避免把未来帧信息带入在线结论，又新增因果配置 `test/nav_drift_registered_world_extended_cpp_temporal_causal.yaml`，使用 `window_before=4/window_after=0/support>=2`。该配置只看当前和历史帧，30 帧结果为 `25` 帧 accepted、`5` 帧 rejected、`25/25` refined success；仍然救回 nav45 hard frame，并拒绝两个危险 nav43 hard frame。额外 reject 的 3 帧是每个 bag 的首帧或历史不足帧，说明真实在线接入时需要等待至少两帧稳定支持再发布恢复候选。主配置默认采用这个因果窗口口径。

结论：

- `none` 粗定位不能直接作为最终输出，重复走廊环境里误匹配明显。
- ICP 速度快，但在 hard case 上稳定性不如 GICP。
- `1.0m BBS` 即使增加到 `top30/refine20` 仍会失败 1 帧，说明 hard case 需要 `0.75m` 或更细的粗搜索分辨率。
- 继续粗化地图/scan 点云会导致 hard case 丢失几何约束，因此当前不推荐 `map_leaf_size=0.30`、`scan_leaf_size=0.40`。
- 当前推荐 `balanced_iter25`，它在 15 帧基准中保持 15/15 成功率，并比 precision 显著降低 median total 和峰值 RSS；在 30 帧扩展验证中暴露 3 个单帧 hard case，因此后续接入时必须加入多帧确认或候选拒绝策略。
- 候选质量指标可以作为拒绝策略的输入特征，但不能单独决定是否重置定位；当前证据更支持“多帧候选聚类 + odom 短程运动一致性 + 状态机触发”的方案。
- 多帧 `map->odom` 一致性已经在当前 hard case 上显示出区分度：错误最终候选通常只有单帧支持，而正确或更稳定的候选簇会跨多个抽样帧出现。
- 在线运行节点已能维护滑动窗口，并对 accepted 候选执行共享 GICP/ICP/NDT 精配准后发布 `recovery_pose` 和 `recovery_map_to_odom`；后续接入 bridge/nav 时，需要把触发条件、确认阈值和恢复动作放进导航状态机。
- CPU 版仍存在 10s 级尾延迟；后续在线接入前需要增加触发策略、超时策略、多帧确认，或评估 GPU/更强剪枝。

额外扩展验证：

- `/home/ubuntu/fast-lio-bags/hall_mapping_20260605_152637` 也包含 `/fast_lio/cloud_registered`、`/odom`、`/robot_realpose`。
- 该 bag 的 `/robot_realpose` 从后半段才开始出现，前 3 个参考正常抽样帧均成功，误差约 `0.036m~0.044m`。
- 后 2 个抽样帧的 `/robot_realpose` 跳到几千米/上万米量级，明显超出室内地图范围，已被 `reference_xy_out_of_range` 过滤，不作为算法失败结论。
- 其它只有 `/fast_lio/cloud_registered` 和 `/odom`、没有 `/robot_realpose` 的 bag 已做无真值 smoke：7 条抽样记录中 5 条 localized，2 条因为点云与 `/odom` 时间同步失败跳过。这组结果只证明输入链路覆盖，不计入准确率。
- 在没有真实 `/cloud_registered_body` bag 前，已用 `make_synthetic_body_bag.py` 从 `nav_drift_test43/44/45` 派生了三个 5 帧 `/cloud_registered_body` 小 bag。`nav43` 单包用于和 registered_world 同帧逐位对比，结果几乎一致，`5/5` 成功；三包 body 扩展验证覆盖 15 帧、3 类模拟场景，共 `45/45` 成功，证明本包 body 输入分支和轴转换链路在三个优先 bag 上是自洽的。
- 真实 `nav_drift_test46` body 验证已完成：快速 10 帧抽样中 `10/10 localized`，9 个有参考帧全部成功，任意点启动 median 平移误差 `0.078m`、median yaw `0.090deg`。随后扩展为跨全包 30 帧抽样，真实 body 和 registered_world 都是 `30/30 localized`，三类场景各自 `27/29` 有参考帧成功；两路失败时间戳完全一致，`success_mismatch=0`，说明失败来自重复结构候选混淆，不是 `/cloud_registered_body` 话题轴或反变换问题。30 帧 body 任意点启动 median 平移误差 `0.101m`、median yaw `0.229deg`、median total `5548.6ms`、median CPU `15238.0ms`、peak RSS max `363.7MB`。
- 资源 sweep 已在 `nav_drift_test46` 真实 body 30 帧上完成。当前推荐 `bbs_num_threads=2`、`max_refine_candidates=8`、`scan_leaf_size=0.30`：temporal accepted 仍为 `27/27` 成功，median total `4087.9ms`，median CPU `6784.7ms`，线程数 `4-4`。更粗的 `scan_leaf_size=0.40` 虽然更快，但 accepted_ref_success 降到 `25/26`，因此暂不采用。
- nav46 60 点双路扩大压测脚本为 `test/run_nav46_stress_validation.py`。旧 best_seed 发布策略在 body 和 registered_world 两路各出现 `2` 个 temporal false accept；改为 selected candidate 自身 `selected_support>=2` 后，两路均为 `60/60 localized`、`accept=55`、`reject=5`、`false_accept=0`。body 为 `single_ref_success=58/60`、`accepted_success=55/55`、median total `4193.6ms`、median CPU `6720.5ms`、peak RSS `418.3MB`；registered_world 为 `single_ref_success=57/59`、`accepted_success=54/54`、median total `4383.2ms`、median CPU `6999.4ms`、peak RSS `427.3MB`。这说明两路输入效果基本一致，当前主要风险是重复结构候选歧义，不是 `/cloud_registered_body` 话题或轴转换。
- nav46 随机 80 点双路压测使用 `--random-count 80 --seed 20260702`。单靠 `selected_support>=2` 时两路各出现 `1` 个 false accept；增加 `temporal_consistency_online_max_refine_fitness: 0.12` 后，两路均为 `80/80 localized`、`single_ref_success=74/80`、`accept=68`、`reject=12`、`accepted_success=68/68`、`false_accept=0`。主要单帧失败位置集中在 `(-0.10, 0.01)`、`(3.46, 7.97)`、`(-1.31, -8.25)`、`(-1.29, -8.29)`、`(-0.23, -1.11)`、`(9.59, 13.84)` 附近；其中 `(-1.3, -8.3)` 连续两帧失败，属于当前最明显的局部问题区域。
- 针对随机 80 点中被短历史门控 reject 的 12 帧，新增长历史 trajectory likelihood 二阶段验证。使用约 24 秒历史窗口后，两路都能把 11 个短历史 reject 选回正确位置，包括原单帧误差 `20m~35m` 的困难点；旧门控会误接受 1 个 `average_overlap=0.902` 的候选，因此新增 `trajectory_likelihood_min_average_overlap: 0.95`。新门控下两路均为 `accept=11/12`、`accept_success=11/11`、`false_accept=0`；同阈值对 bag44/45/46 共 69 个长历史成功样本无召回损失。
- 为避免只对 nav46 单包调参过拟合，新增三包长历史门禁 `test/verify_trajectory_long_history_bags.py`。它固定检查 `nav_drift_test44/45/46` 的长历史产物：bag44 从单帧 `15/25` 提升到轨迹 `25/25`，bag45 从 `12/22` 提升到 `22/22`，bag46 从 `18/22` 提升到 `22/22`，三包 accepted false_accept 均为 `0`。
- 对最后 `1/12` 个长历史仍拒绝的点，进一步用带符号 offset 模拟主动恢复新视角：历史 24 秒 + 目标后 24 秒。加入新视角后最佳轨迹候选变为正确区域，误差 `0.632m / 0.419deg`，但 average overlap 低于严格阈值。为避免简单降阈值放过旧错误候选，新增 `trajectory_single_agreement_fallback_enable` 及相关门控：只有单帧 GICP fitness 很低，且轨迹候选与单帧最终位姿在 x/y/yaw 上高度一致时才放行。回归结果中，旧 7 帧错误窗口仍为 `candidate`，主动恢复 13 帧窗口 body/registered_world 都变为 `accept`。因此当前 nav46 随机 80 可形成三层恢复：短历史恢复 68、长历史再救 11、主动恢复新视角救最后 1，等效 `80/80`，当前验证证据中 `false_accept=0`。
- hard-frame 补救现在有独立一键脚本 `test/run_nav46_hardframe_recovery_validation.py` 和固定 fixture `test/nav46_rand80_reject_matches.csv`。脚本默认只复核已有产物；加 `--run` 会重新从 bag46 生成长历史和主动视角证据。为避免离线重排对辅助历史帧也逐帧计算，脚本会写入 `trajectory_likelihood_center_frame_indices`，让历史/未来帧只作为上下文，trajectory likelihood 只对目标难例帧输出。最新重跑结果为长历史 `22/22` 安全接受、`false_accept=0`，主动视角 `2/2` 接受。
- top-K 对照结论：长历史二阶段用 `top30` 已能救回两路 11/12 个短历史拒绝点，且没有误接受；剩下的 `rand80_reject_12` 在主动新视角下用 `top30` 仍会保守拒绝，`top60` 才能找到正确候选并通过单帧一致性 fallback。因此当前推荐恢复策略是“先 top30 快速尝试，仍需主动恢复时再 top60 深搜”，而不是常态每次都 top60。
- 静止冷启动描述子召回实验新增 `test/run_scan_context_keyframe_recall.py`。该脚本用真实 bag keyframe scan 构造 Scan Context 风格描述子库，并排除目标前后 `240` 个 cloud index，避免把当前帧附近数据直接泄漏进数据库。点位 8 在原始单帧 BBS 中候选全部落到错误区域，静止冷启动失败；但 keyframe 描述子召回的 top1 已到 `1.048m / 6.762deg`，top20 第 8 名为 `0.229m / 0.927deg`。25 个动态点位整体上，top20 中 `25/25` 都有 `1m` 内位置候选，严格 `1m/15deg` 为 `22/25`；失败的点位 1、6、10 都是位置正确但 yaw 略超阈值，后续应交给局部 yaw 搜索、3D-BBS 或 GICP 修正。反向实验表明，直接从全局 PCD 裁虚拟局部地图生成描述子会因为包含不可见结构而召回错误区域，因此第一版推荐使用建图/巡航 bag 的真实 keyframe scan 建库。
- 坐标修正后，描述子召回脚本与 C++ evaluator 一样使用完整 `/odom` 反变换和 Fast-LIO raw body -> base 轴转换。点位 8 的 keyframe 描述子 top1 直接变为 `0.229m / 0.927deg`。新增 `test/run_descriptor_fusion_validation.py` 后，使用“描述子 top10 + yaw 偏移 `[-20,-10,0,10,20]` + Open3D GICP”验证当前冷启动困难点：点位 2 恢复到 `0.107m / 0.149deg`，点位 4 恢复到 `0.014m / 1.849deg`，点位 8 恢复到 `0.124m / 0.153deg`。点位 12 的描述子 top1 本身为 `0.740m / 7.738deg`，但无约束 GICP 会滑到 `1.606m / 22.927deg`，说明最终融合必须加入 seed drift gate 或局部 3D-BBS score 验证：GICP 只能在描述子 seed 附近小范围修正，不能让它跨重复结构自由滑动。
- 描述子融合验证已补上 seed drift gate 和融合评分：先要求 GICP 结果相对 descriptor seed 不超过 `1.5m / 35deg`，再按 `GICP rmse + 0.5 * descriptor_distance + 0.03 * descriptor_rank` 选择候选，避免“只信 descriptor rank”或“只信 GICP rmse”在重复走廊里选错。使用 `nav_drift_test44/45/46` 的 25 个动态点位重跑后，主链路静止冷启动为 `52/75`，主链路最终恢复为 `71/75`；对主链路拒绝的 4 个点再执行 descriptor keyframe + GICP fusion，`4/4` 全部救回，组合最终结果为 `75/75`。4 个兜底点误差分别为 bag44 点位2 `0.043m / 0.452deg`、bag45 点位1 `0.055m / 3.210deg`、bag45 点位2 `0.044m / 0.802deg`、bag46 点位8 `0.127m / 1.001deg`。汇总报告写在 `.codex_tmp/waypoint_pose_validation_multi_bags/registered_world_3bag_25pts_with_descriptor_fusion_report.csv`。当前这仍是离线验证链路，证明“静止 descriptor 召回兜底”值得融合进在线 C++ 节点；还没有把该兜底直接接入 bridge/nav 或发布最终 TF。
- 为了单独评估“机器人放在任意点开机、不主动旋转”的静止冷启动能力，又对三包中主链路冷启动失败的 `23` 个点全部跑了 descriptor fusion。结果为 bag44 `9/11`、bag45 `5/9`、bag46 `3/3`，总计额外救回 `17/23`；因此静止冷启动组合成功率从 `52/75` 提升到 `69/75`。组合冷启动可计算误差样本的 median 为 `0.093m / 0.604deg`，最大为 `0.787m / 6.821deg`。仍失败的 6 个点为 bag44 点位13/22、bag45 点位9/11/13/23，主要表现为重复结构 yaw 歧义或跳到约 `23m` 外的相似区域，不能通过放宽 fitness 或 yaw 阈值硬收。冷启动融合报告写在 `.codex_tmp/waypoint_pose_validation_multi_bags/registered_world_3bag_25pts_cold_start_with_descriptor_fusion_report.csv`。
- OPD / LiDAR-Localization-100FPS 和 TEASER++ 完整功能试验已完成。OPD 采用官方 40x40 占据模板、候选站位、yaw 离散和倒排投票流程；在 `bag46 hard52` 上细化到 `1.0m` 候选栅格、`120` 个 yaw、`top60` 后，严格 `0.3m/5deg` 只有 top1 `3/52`、top60 `9/52`，`0.5m/10deg` 为 `14/52`，建库和查询峰值 RSS 约 `1.1GB`。同参数对 `rand100` 为 top1 `10/100`、top60 `32/100`，对 `edge100` 为 top1 `20/100`、top60 `40/100`。TEASER++ 使用官方 FPFH + Matcher + RobustRegistrationSolver，再接 GICP 精修；接 OPD top3 后 hard52 严格 `0.3m/5deg` 为 `6/52`，rand100 为 `8/100`，edge100 为 `3/100`，单候选 median `96ms~124ms`。接现有 SC/BBS top3 的 hard52 也只有 `5/52`。结论是：OPD 可作为研究参考，但不适合当前室内重复结构地图的主召回；TEASER++ 在当前稀疏 scan 与局部地图 patch 上容易被 FPFH 错对应带偏，暂不作为线上候选验证器。当前仍推荐“真实 keyframe Scan Context 召回 + 3D-BBS/GICP + 时序/轨迹/主动新视角门控”的主线。
- KISS-Matcher 官方完整 C++ 核心已在 `.codex_tmp/upstream_kiss_mcl_area/KISS-Matcher` 拉取、编译和临时安装，并新增独立验证器 `.codex_tmp/kiss_mcl_area_validation/kiss_cli/kiss_candidate_validator.cpp`。验证器对每个 OPD 或 SC/BBS top3 候选裁剪局部地图 patch，调用官方 `KISSMatcherConfig -> estimate()` 完成 Faster-PFH、ROBIN 图剪枝和鲁棒求解，再用 KISS 处理后的点云做 GICP 精修。结果显示 KISS 可运行但不适合作为当前主候选验证器：`hard52 + OPD top3 + 0.45m` 为 `0.2m/3deg 3/52`、`0.3m/5deg 8/52`、`0.5m/10deg 11/52`，median 单候选 `118.5ms`，峰值 RSS 约 `371MB`，CPU 约 `2.55` 核；`hard52 + SC/BBS top3 + 0.45m` 为 `1/52`、`5/52`、`9/52`；改成官方默认更细的 `0.30m` 后为 `4/52`、`7/52`、`9/52`，但 median 单候选增至 `213.6ms`，CPU 约 `3.30` 核，峰值 RSS 约 `427MB`。在更大抽样上，`rand100 + OPD top3 + 0.45m` 只有 `0.3m/5deg 5/100`，`edge100 + OPD top3 + 0.45m` 只有 `3/100`。主要失败模式仍是重复走廊/边角局部 patch 几何相似，Faster-PFH 对应和图剪枝会给出自洽但错误的刚体解。因此 KISS-Matcher 当前只保留为研究对照，不并入线上恢复链路。
- 虚拟 LiDAR scan 数据库完整验证脚本已新增为 `test/run_virtual_lidar_relocalization_validation.py`。该脚本从全局 PCD 地图生成可站立候选位置，在每个候选上做 360 度首次命中射线模拟，建立 range/scan-context 描述子库，再用静止 MCL 多粒子假设、可见性似然、禁穿墙比例和 GICP 精修排序候选。对 `bag46 hard52` 的完整验证结果为 `0.2m/3deg 1/52`、`0.3m/5deg 2/52`、`0.5m/10deg 2/52`，耗时 `5分30秒`，CPU 约 `1.78` 核，峰值 RSS 约 `400MB`。失败样本大量表现为远处重复走廊/墙角拥有很高 GICP fitness，例如第一个 hard 点被选到约 `19.7m/67deg` 外的相似区域。因此结论是：仅由全局地图合成虚拟 scan 的召回能力不足，不能替代真实 keyframe Scan Context；它最多可作为后续拓扑/区域门控的研究输入。
- BEVPlace++ 官方完整链路已拉取到 `.codex_tmp/upstream_virtual_bev/BEVPlace2`，并新增 `test/run_bevplace_official_validation.py` 做数据适配。验证保留官方 REIN、NetVLAD、local feature 和 rigid RANSAC，使用官方 checkpoint `runs/Aug08_10-17-29/model_best.pth.tar`；由于当前机器是 CPU-only，脚本只做 `.cuda()` no-op 包装和 BEV 图像向量化生成。第一个 `bag46 hard52` 目标的真实最近候选在 BEVPlace++ 全局描述子中排名 `860/1344`，top10 全部落在 `13m~16m` 外的相似区域；完整 top5 RANSAC 位姿估计误差为 `31.903m / 127.484deg`。首次生成 1344 个数据库 BEV 描述子耗时 `7分48秒`，CPU 约 `2.66` 核，峰值 RSS 约 `1.35GB`。因此官方 KITTI/NCLT 权重不能直接迁移到当前室内 Airy/Fast-LIO 地图；若未来继续 BEVPlace++，应先用本机器人建图/巡航数据重新训练或微调，再重新验证。
- RING++ 官方仓库已拉取到 `.codex_tmp/upstream_ring_ndtmc/RING`，并新增 `test/run_ring_official_validation.py`。当前机器无 `nvidia-smi/nvcc`，官方 RING++ point-feature Cython 分支和 torch-radon CUDA 路线不能完整运行；脚本验证的是官方 RING occupancy CPU 分支，仍调用上游 `generate_bev -> generate_RING_cpu -> fast_corr -> solve_translation` 完成地点召回、yaw 和 2D 平移估计。官方 fast_gicp Python 扩展因 bundled pybind11 不兼容 Python 3.12、且 PCL 1.14 中 `pcl_isfinite` API 变化而编译失败，因此本次没有伪装执行 refinement。对 `bag46 hard52` 真实 keyframe scan 数据库的结果为：top1 `0.2m/3deg 8/52`、`0.3m/5deg 9/52`、`0.5m/10deg 17/52`；top20 oracle `0.2m/3deg 9/52`、`0.3m/5deg 13/52`、`0.5m/10deg 28/52`。全量 hard52 耗时 `7分54秒`，平均 CPU 约 `8.32` 核，峰值 RSS 约 `1.10GB`。第一个 hard 点使用全点输入复测仍为 `3.718m / 18.025deg`，说明失败不是点采样造成。结论是：RING 可作为研究对照，但当前 occupancy 分支召回率和 yaw 精度都弱于已有真实 keyframe Scan Context + GICP 融合，不建议并入主恢复链路。
- NDT-Map-Code 官方 C++ 核心已拉取到 `.codex_tmp/upstream_ring_ndtmc/NDTMC`，并在 PCL 1.14 下用 `boost::make_shared -> pcl::make_shared` 兼容补丁完成编译。新增临时验证器 `.codex_tmp/ring_ndtmc_validation/ndtmc_keyframe_validator.cpp`，直接调用官方 `NDTMC::createNDTMC()` 和 `NDTMC::distanceBtnNDTScanContext()`，只负责读取我们的 keyframes/targets CSV 和 PCD 数据。对 `bag46 hard52` 真实 keyframe scan 数据库，top1 为 `0.2m/3deg 5/52`、`0.3m/5deg 5/52`、`0.5m/10deg 6/52`；top20 oracle 为 `7/52`、`7/52`、`8/52`。资源开销较低，耗时约 `1分01秒`，CPU 约 `0.98` 核，峰值 RSS 约 `68MB`，但召回准确率明显不足。因此 NDTMC 暂不适合作为当前室内重定位的主候选召回器。
- mcl_3dl、Reliable-loc 和 Area Graph 方向已做完整落地可行性检查。mcl_3dl 是 ROS1/catkin 点云 MCL 节点，依赖 `roscpp/pcl_ros/tf2/mcl_3dl_msgs` 和 ROS service，全量使用需要先做 ROS2/ament 迁移、消息/服务替换、bag topic 适配和地图接口改造；它的思想适合后续做“恢复专用粒子多假设验证”，但不能直接在当前 ROS2 bag 上完整运行。Reliable-loc 是训练/评估一体的深度地点识别与 Monte Carlo sequential localization 工程，官方 README 要求 docker、TEASER++、多套 CUDA 扩展、特定数据集和权重，完整跑我们的室内 bag 需要先转换数据集并准备/训练模型，不适合作为当前无需训练的上线增强。Area Graph 是拓扑/区域约束思路，当前更适合作为我们自建语义区域层：从地图中离线标注走廊、房间、边界、禁穿墙区域，对候选做区域一致性和可见性门控；它不是一个可以直接替换现有 C++ 包的即插即用配准器。
- Area Graph/可见性门控方向已新增离线验证脚本 `test/run_area_visibility_gate_validation.py`。该脚本不重新召回候选，而是读取现有 `bag46 hard52` 三角描述子 + GICP 候选明细，把当前 scan 按每个候选位姿投到地图里，计算端点贴图比例、射线提前撞墙比例和候选站位是否落入障碍。完整 hard52 结果表明，硬门控不适合直接用于自动恢复：例如 `endpoint_near>=0.45` 且 `wall_cross<=0.30` 只接受 `5/52`，其中 `0.3m/5deg` 成功 `3/5`，大量正确候选被地图墙体厚度、稠密障碍点和动态残留误伤。改成软重排序后有小幅收益：原三角描述子/GICP 选点为 `0.2m/3deg 25/52`、`0.3m/5deg 30/52`、`0.5m/10deg 37/52`，加入端点贴图奖励和穿墙比例惩罚后提升到 `28/52`、`36/52`、`45/52`，median 误差为 `0.113m / 1.490deg`，但最大错误仍有 `36.315m / 174.689deg`。该离线脚本全候选评估耗时 `92.08s`、平均 CPU `1.27` 核、峰值 RSS `443.9MB`。因此该方向建议作为候选软排序和诊断特征接入，不建议作为单独的硬接受/拒绝条件；站位清空检查当前只保留诊断，不参与默认排序。
- 2D/2.5D BBS 全图搜索方向已新增离线验证脚本 `test/run_bbs_2p5d_validation.py`。该脚本从全局 PCD 地图生成多高度层二维占据距离场，按 `x/y/yaw` 在整个地图上做多分辨率分支搜索，最终对 top 候选做空间/角度 NMS、禁穿墙/站位清空重排序和 Open3D GICP 精修。对 `bag46 hard52` 全量 52 点的最终配置为：`base_resolution=0.20m`、高度层 `0.2:0.8/0.8:1.5/1.5:2.2`、yaw 步长 `5deg`、最终 yaw 加密 `2.5deg`、每层保留 `12000` 候选、最终 NMS `2.0m/30deg`、GICP 精修 top16。直接自动接受结果为 `0.2m/3deg 12/52`、`0.3m/5deg 18/52`、`0.5m/10deg 26/52`，median 约 `2.214m / 5.840deg`，说明它不能单独作为最终重定位发布器；但 top16 候选池 oracle 非常强：seed 口径为 `25/52`、`39/52`、`52/52`，GICP 后为 `29/52`、`37/52`、`52/52`。全量耗时 `123.31s`，平均 CPU `1.69` 核，峰值 RSS `444.4MB`，单帧 median 大约 `2.2s~2.4s`。结论是：BBS_2.5D 是目前最有价值的“全图兜底候选召回层”，尤其适合静止冷启动/定位丢失时给 trajectory likelihood 或主动新视角模块提供多假设；但不能只按单帧分数直接发布 initialpose，否则重复走廊仍会误接受。
- BBS_2.5D 又在 `nav_drift_test46` 上做了三组随机 100 点压测，点位来自 `/fast_lio/cloud_registered` 与 `/robot_realpose` 精确同步抽样，结果目录为 `.codex_tmp/bbs_2p5d_random_sets/rand100_seed20260707~20260709`。把最终单帧选中的候选当作“静止冷启动直接发布”时，300 点总成功率为 `0.2m/3deg 101/300`、`0.3m/5deg 112/300`、`0.5m/10deg 114/300`，全部样本 median 误差为 `7.987m / 26.288deg`；这说明单帧 top1 仍然不能安全直发。成功样本自身很准，三组 `0.3m/5deg` 成功样本 median 约 `0.053m~0.069m / 0.150deg~0.280deg`，问题主要是重复结构下会选到远处相似区域。若把 BBS_2.5D 作为“定位丢失恢复候选池”而不是直接发布器，top16 候选池在 300 点中的 oracle 为：seed 口径 `0.2m/3deg 178/300`、`0.3m/5deg 266/300`、`0.5m/10deg 300/300`，GICP 后 `0.2m/3deg 266/300`、`0.3m/5deg 292/300`、`0.5m/10deg 300/300`。资源开销为 median total `2030.9ms`、median BBS `1141.0ms`、峰值 RSS `444.9MB`，三次整包平均 CPU 约 `1.88~2.00` 核。当前上线含义是：BBS_2.5D 应作为恢复层深搜兜底，为后续多帧轨迹似然、主动新视角或状态机确认提供 top-K 多假设；冷启动静止场景也必须经过候选确认后再发布 `initialpose/map->odom`，不能单靠单帧分数自动接受。
- 2D/2.5D BBS 已接入 C++ 运行态候选池，新增核心文件为 `include/humanoid_global_relocalization_runtime/bbs2d_search.hpp` 和 `src/bbs2d_search.cpp`。运行态参数位于 `config/relocalization_runtime.yaml` 的“2D/2.5D BBS 深搜候选召回”段，正式默认 `enable_bbs2d_recall: false`；它保留为没有 Scan Context DB 时的兜底召回层，不作为日常默认层。打开后流程为：3D-BBS 先给基础候选，Scan Context 按原逻辑补充 keyframe seed，最后 2.5D BBS 追加深搜候选。追加后的候选统一进入 `BbsResult.candidates`，继续由 GICP、`selected_support`、trajectory likelihood 和主动新视角门控确认，不会绕过安全发布逻辑。临时 1 帧 smoke 使用 `.codex_tmp/bbs2d_online_integration_smoke.yaml` 打开 2.5D，输出 `.codex_tmp/bbs2d_online_integration_smoke`，结果为 `localized=1`、融合候选 `17` 个、`search_ms=2344.98`、`total_ms=3054.77`、RSS `272.6MB`；该 smoke 无 `/robot_realpose` 同步参考，因此只验证链路跑通和候选融合，不计入准确率结论。
- SpectralGV 官方仓库已拉取到 `.codex_tmp/upstream_spectral_mcl/SpectralGV`，并新增 `test/run_spectral_gv_validation.py`。该脚本保留官方 SpectralGV 的核心流程：局部特征最近邻匹配、空间一致性 adjacency graph、power iteration 主特征向量和 spectral consistency score；为了完整跑我们自己的 bag，局部特征使用 Open3D FPFH 生成。候选 keyframe 已从 `nav_drift_test46` exact 导出到 `.codex_tmp/spectral_gv/candidate_keyframes.csv`，覆盖三角候选 CSV 中出现的 `287` 个唯一 keyframe。对 `bag46 hard52` 的完整验证结果为：同候选池 `weight=0` 基线 `0.2m/3deg 24/52`、`0.3m/5deg 28/52`、`0.5m/10deg 35/52`；加入 SpectralGV 软重排序后最佳为 `27/52`、`31/52`、`39/52`，median `0.118m / 2.041deg`，最大错误仍为 `25.168m / 171.284deg`。全量 `1040` 个 target-keyframe pair 耗时约 `25.97s`，平均 CPU `2.05` 核，峰值 RSS `582.6MB`。`sgv_distance_threshold` 在 `0.35/0.55/0.80/1.10` 上 sweep 后严格和实用口径都没有继续突破，只在可救回口径偶尔提升到 `40/52`。结论是：SpectralGV + FPFH 能带来小幅候选重排序收益，但无法解决当前主要重复走廊/墙角歧义；若未来要继续这个方向，应换成官方论文依赖的学习型局部特征，而不是继续调 FPFH。
- MCL-DLF 官方仓库已拉取到 `.codex_tmp/upstream_spectral_mcl/mcl-dlf`，完整 DLF 分支当前不能在本机直接运行。官方入口 `main.py` 依赖 `MinkowskiEngine` 和 `MinkUNeXt` 权重；本机环境为 Python `3.12.3`、torch `2.11.0+cpu`、无 `nvidia-smi/nvcc`，仓库内也没有 `.pth/.pt/.ckpt` 权重文件。尝试安装 `MinkowskiEngine==0.5.4` 的 CPU-only 临时构建时，先后遇到 build isolation 看不到 torch、PEP668 保护、以及 Python 3.12 下缺少 `numpy.distutils` 的构建阻塞；即使用 `.codex_tmp/minkowski_build_deps` 临时加入 `numpy==1.26.4`，Python 3.12 仍无法导入 `numpy.distutils`。因此这不是数据适配失败，而是官方完整 DLF 运行环境不满足。MCL-DLF 的思想仍值得保留：用 keyframe 拓扑粒子 MCL 做多假设，再用深度局部特征 fine localization；但要完整验证，需要单独准备 Python 3.9/3.10、MinkowskiEngine 可编译环境、CUDA 或明确 CPU 补丁、以及用我们机器人 bag 训练/获得 MinkUNeXt 权重。

校验 nav46 随机 80 点三层恢复证据：

```bash
python3 src/humanoid_global_relocalization_runtime/test/verify_nav46_three_stage_recovery.py
python3 src/humanoid_global_relocalization_runtime/test/verify_trajectory_long_history_bags.py
python3 src/humanoid_global_relocalization_runtime/test/run_nav46_hardframe_recovery_validation.py
python3 src/humanoid_global_relocalization_runtime/test/run_nav46_hardframe_recovery_validation.py --run
```

## 恢复消费者仿真

为了在真正接入 bridge/nav 前先验证“哪些恢复量会被状态机接收”，新增：

```text
src/humanoid_global_relocalization_runtime/test/simulate_recovery_consumer.py
```

该脚本只读取已有 CSV，不连接导航系统。模拟接收规则为：`selected_support>=2`、`refined_converged=true`、`refined_fitness<=0.12`。在有真值的数据集上，任何被模拟接收但 `refined_success!=1` 的帧都会算作 false accept。

当前产物：

```text
.codex_tmp/recovery_consumer_simulation.csv
```

当前结果：

```text
nav46_real_body_30frame: consumer_accept=25 consumer_reject=5 false_accept=0
nav46_registered_world_30frame: consumer_accept=24 consumer_reject=6 false_accept=0
nav_drift_registered_world_causal: consumer_accept=23 consumer_reject=7 false_accept=0
online_smoke_recovery_interface: publish=7/7 consumer_accept=7 consumer_reject=0
online_active_view_status_interface: publish=0/1 consumer_accept=0 consumer_reject=1
```

## 后续接入导航方案

验证通过后再进入第二阶段：

1. 继续补充更多包含 `/cloud_registered_body` 的 bag，把 registered_world/body 同场景对比从 `nav_drift_test46` 扩展到更多路线和遮挡场景。
2. 扩大 bag 覆盖范围，增加转角、遮挡、不同启动点和连续漂移恢复验证。
3. 增加真实 keyframe scan 描述子库，把静止冷启动 scan 先召回到若干候选区域，再由局部 yaw 搜索、3D-BBS/GICP 和 trajectory likelihood 负责精确验证。
4. 增加多帧候选聚类模块，把连续几帧 top-K 候选换算成隐含 `map->odom`，再按 `x/y/yaw` 聚成时序稳定簇。
5. 增加候选拒绝模块，综合 fitness、候选分散度、候选排名、scan-map 重合度、局部可行性等特征，但不让任何单一特征独立决定重置。
6. 增加 `/prior_localization/relocalization_pose` 或 service，让 bridge 明确区分“普通小修正”和“全局重定位重置”。
7. 重定位通过多帧一致性确认后，同时发布给 RoboSense `/prior_localization/manual_initialpose` 和 bridge 专用入口。
8. 保持 `map -> odom` 仍由 `prior_map_odom_bridge_cpp` 独占发布。
