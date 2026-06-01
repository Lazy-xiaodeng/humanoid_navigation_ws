# ro/op 降采样 0.1 bag 验证结论

验证时间：2026-06-01

验证对象：
- ro：`robosense_lidar_localization` 集成链路，bag 配置中 `map_voxel_leaf_size=0.10`，`lidar_matcher.leaf_size=0.10`。
- op：Open3D bag 链路，`voxelsize_fine=0.10`；`voxelsize_coarse=0.01` 原本已经比 0.1 更细，本次未改粗配准参数。
- bridge：仍为 `jump_protection_mode=monitor`，旋转/SpinToPose guard 开启；正式实机配置未切到 0.1。

## 参数是否生效

RoboSense 日志确认：
- 旧 monitor 基线地图 KD-tree 下采样后点数：`51023`。
- 本次 `map_voxel_leaf_size=0.10` 后地图点数：`626750`。

也就是说 ro 参与最近邻搜索的地图点约增加到 `12.3x`，特征/细节确实更密，但计算压力也明显增加。

Open3D 日志确认本轮进入了 `voxel_size: 0.1` 的细尺度配准分支；节点内部仍会打印 0.2/0.3 多尺度信息，这是 Open3D 节点自身的多尺度流程，不代表 launch 参数没有生效。

## 关键指标

| bag | 方法 | 配置 | samples | map->odom 最大平移跳变(m) | map->odom 最大 yaw 跳变(deg) | prior 最大 yaw 跳变(deg) | 主要状态 |
|---|---|---:|---:|---:|---:|---:|---|
| test23 | ro | 0.30/0.10 基线 | 956 | 1.2315 | 4.13 | 94.07 | ACCEPTED 780 / REJECTED 151 |
| test23 | ro | 0.10/0.10 本次 | 956 | 1.2038 | 4.30 | 77.86 | ACCEPTED 774 / REJECTED 145 |
| test23 | op | fine 0.20 基线 | 959 | 1.0932 | 2.87 | 76.22 | ACCEPTED 785 / REJECTED 133 |
| test23 | op | fine 0.10 本次 | 961 | 1.1301 | 4.61 | 75.00 | ACCEPTED 772 / REJECTED 152 |
| test24 | ro | 0.30/0.10 基线 | 857 | 1.7069 | 3.95 | 77.35 | ACCEPTED 671 / REJECTED 101 |
| test24 | ro | 0.10/0.10 本次 | 857 | 1.7335 | 3.32 | 83.31 | ACCEPTED 660 / REJECTED 167 |
| test24 | op | fine 0.20 基线 | 862 | 0.4153 | 7.33 | 74.86 | SPIN_GUARD 714 |
| test24 | op | fine 0.10 本次 | 861 | 0.1433 | 7.38 | 70.36 | SPIN_GUARD 758 |
| test25 | ro | 0.30/0.10 基线 | 1023 | 1.7296 | 8.45 | 92.59 | ACCEPTED 812 / REJECTED 169 |
| test25 | ro | 0.10/0.10 本次 | 1023 | 1.0468 | 6.01 | 0.01* | SPIN_GUARD 837 / PENDING 174 |
| test25 | op | fine 0.20 基线 | 1031 | 2.3808 | 6.40 | 78.89 | ACCEPTED 804 / REJECTED 152 |
| test25 | op | fine 0.10 本次 | 1029 | 1.7298 | 5.51 | 81.00 | ACCEPTED 822 / REJECTED 163 |

`*` test25 ro 本次 prior 最大 yaw 近似为 0，不代表定位突然极好，而是该段大量处于 `SPIN_GUARD/PENDING`，有效接受的 ro 全局定位帧很少。

## 判断

1. `0.1` 会让地图特征更丰富，但没有带来稳定、可复现的整体提升。

2. ro：
   - test23 轻微改善：平移跳变 `1.2315 -> 1.2038`，prior yaw `94.07 -> 77.86`。
   - test24 变差：平移跳变 `1.7069 -> 1.7335`，prior yaw `77.35 -> 83.31`，REJECTED 从 `101` 增到 `167`。
   - test25 表面改善，但主要是 SpinToPose guard/PENDING 占比极高，不能当作 ro 定位本身更稳。

3. op：
   - test23 变差：map->odom yaw `2.87 -> 4.61`，REJECTED 增多。
   - test24 map->odom 平移跳变变小，但该包大部分时间都在 `SPIN_GUARD`，只能说明保护/状态条件下输出更保守。
   - test25 有改善：map->odom 平移 `2.3808 -> 1.7298`，yaw `6.40 -> 5.51`。

4. 所以“降采样越小越好”在这套数据上不成立。0.1 增加了点密度，但也增加了重复/局部平面点和计算负载，匹配不一定更有判别性；在走廊、墙面、原地旋转附近，过密点云反而可能让局部最优更明显。

## 建议

正式链路暂时不要把 ro 地图 KD-tree voxel 从 0.30 改成 0.10。

建议下一轮只做较小网格搜索：
- ro `map_voxel_leaf_size`: `0.20`, `0.15`
- ro `lidar_matcher.leaf_size`: 先保持 `0.10`
- op `voxelsize_fine`: `0.15`, `0.20`

判断标准不要只看最大跳变，还要同时看：
- `ACCEPTED/REJECTED/PENDING/SPIN_GUARD` 分布；
- map->odom 轨迹是否出现孤立大台阶；
- prior 全局定位轨迹是否在原地旋转和 SpinToPose 前后出现角度复位；
- 运行负载是否还能满足实机实时性。

## 输出文件

- 汇总报告：`/home/ubuntu/humanoid_ws/debug_monitor/integrated_voxel01_validation_20260601/integrated_ro_vs_open3d_report.md`
- 轨迹图目录：`/home/ubuntu/humanoid_ws/debug_monitor/integrated_voxel01_validation_20260601/plots`
- 本轮脚本：`/home/ubuntu/humanoid_ws/debug_monitor/run_integrated_voxel01_bag_validation.sh`
