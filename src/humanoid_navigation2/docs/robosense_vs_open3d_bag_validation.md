# RoboSense 与 Open3D Bag 验证流程

目标：在暂不集成 RoboSense 到当前 `/home/ubuntu/humanoid_ws` 的前提下，直接使用外部工作空间 `/home/ubuntu/exp_code/robosense_localization`，对 `nav_drift_test23/24/25` 做离线回放验证，并与当前 Open3D prior-map 定位方法对比。

## 1. 当前可运行性结论

外部 RoboSense 工作空间可以单独跑起来。

已确认：

- `/home/ubuntu/exp_code/robosense_localization` 最近一次 `colcon build` 成功。
- 已安装可执行文件：`lidar_localization/lidar_localization_node`。
- 节点实际硬编码路径是当前有效路径 `/home/ubuntu/exp_code/robosense_localization/lidar_localization/config/config.yaml`，不是旧的 `/home/xichen/...`。
- `nav_drift_test23`、`nav_drift_test24` 已经有全量 RoboSense 回放结果。
- `nav_drift_test25` 已做 60 秒 smoke，能加载地图、订阅 bag 数据并输出 `/lidar_pose_xyz`。

已有 RoboSense 结果：

```text
/home/ubuntu/exp_code/robosense_localization/out/nav_drift_test23_current_node_full
  poses_1406s.csv
  odom_vs_global_current_node.html

/home/ubuntu/exp_code/robosense_localization/out/nav_drift_test24_current_node_full
  poses_1258s.csv
  odom_vs_global_current_node.html
```

本次 smoke 结果：

```text
/tmp/robosense_test25_smoke/poses_60s.csv
```

smoke 样本数：

```text
1655 lines，包括 header
```

注意：在当前 sandbox 环境里日志会出现 FastDDS `getifaddrs: Operation not permitted` / UDP socket 报警，但 60 秒 smoke 仍然正常收到数据并输出定位结果。实机终端直接运行一般不会有这个 sandbox 限制。

## 2. Bag 路径

```text
/home/ubuntu/nav_drift_test/nav_drift_test23
/home/ubuntu/nav_drift_test/nav_drift_test24
/home/ubuntu/nav_drift_test/nav_drift_test25
```

三个 bag 都包含关键对比话题：

```text
/odom
/fast_lio/cloud_registered
/robot_realpose
/navigation/status
/prior_localization/odom
/prior_localization/confidence
```

其中 `/prior_localization/odom` 和 `/prior_localization/confidence` 是录包时的 Open3D prior-map 定位输出。若只需要和“当时实机跑的 Open3D”对比，可以直接从 bag 里抽取这些话题；若要验证“当前代码/当前参数”的 Open3D，则按第 4 节重新跑 Open3D 离线回放。

## 3. RoboSense 单独回放

外部工作空间已有脚本：

```text
/home/ubuntu/exp_code/robosense_localization/tools/run_tj_lidar_localization_test.sh
```

脚本会启动：

- `tools/fastlio_odom_to_base.py`
- `tools/registered_cloud_localization_relay.py`
- `tools/pose_csv_recorder.py`
- `ros2 run lidar_localization lidar_localization_node`
- `ros2 bag play`

推荐先用 `registered` 输入模式，因为当前配置针对 `/fast_lio/cloud_registered` 和 `/odom` 做了转换：

```bash
cd /home/ubuntu/exp_code/robosense_localization

./tools/run_tj_lidar_localization_test.sh \
  60 \
  3.0 \
  /tmp/robosense_test25_smoke \
  /home/ubuntu/nav_drift_test/nav_drift_test25 \
  registered
```

参数含义：

```text
60          回放 bag 中前 60 秒
3.0         3 倍速
OUT_DIR     输出目录
BAG         bag 路径
registered 订阅 /fast_lio/cloud_registered；脚本内部转成定位器需要的 body cloud
```

全量验证命令：

```bash
cd /home/ubuntu/exp_code/robosense_localization

./tools/run_tj_lidar_localization_test.sh \
  1406 \
  3.0 \
  /home/ubuntu/humanoid_ws/debug_monitor/robosense_nav_drift_test23_full \
  /home/ubuntu/nav_drift_test/nav_drift_test23 \
  registered

./tools/run_tj_lidar_localization_test.sh \
  1258 \
  3.0 \
  /home/ubuntu/humanoid_ws/debug_monitor/robosense_nav_drift_test24_full \
  /home/ubuntu/nav_drift_test/nav_drift_test24 \
  registered

./tools/run_tj_lidar_localization_test.sh \
  1510 \
  3.0 \
  /home/ubuntu/humanoid_ws/debug_monitor/robosense_nav_drift_test25_full \
  /home/ubuntu/nav_drift_test/nav_drift_test25 \
  registered
```

输出文件：

```text
poses_<duration>s.csv
lidar_localization_node.log
fastlio_odom_to_base.log
registered_cloud_localization_relay.log
pose_csv_recorder.log
rosbag_play.log
```

生成 HTML 轨迹图：

```bash
cd /home/ubuntu/exp_code/robosense_localization

python3 tools/make_odom_vs_lidar_html.py \
  /home/ubuntu/humanoid_ws/debug_monitor/robosense_nav_drift_test25_full/poses_1510s.csv \
  /home/ubuntu/exp_code/robosense_localization/lidar_localization/map/hall_open3d_grounded.pcd \
  /home/ubuntu/humanoid_ws/debug_monitor/robosense_nav_drift_test25_full/odom_vs_robosense.html
```

## 4. Open3D 当前代码离线回放

如果要验证当前工作空间里的 Open3D 方法，而不是 bag 中录下来的旧输出，使用：

```text
src/humanoid_navigation2/launch/prior_map_bag_test.launch.py
```

关键点：播放 bag 时只播放 Open3D 需要的输入话题，不播放 bag 中旧的 `/prior_localization/*`，因此当前 Open3D 输出不会被旧输出干扰。

启动 Open3D + bridge：

```bash
cd /home/ubuntu/humanoid_ws
source install/setup.bash

ros2 launch humanoid_navigation2 prior_map_bag_test.launch.py
```

另一个终端启动监控：

```bash
cd /home/ubuntu/humanoid_ws
source install/setup.bash

ros2 run humanoid_navigation2 prior_map_bag_monitor \
  --ros-args \
  -p output_dir:=/home/ubuntu/humanoid_ws/debug_monitor/open3d_nav_drift_test25_current \
  -p robot_pose_topic:=/robot_realpose
```

第三个终端播放 bag 输入话题，避免播放 bag 里旧的 `/prior_localization/*` 干扰当前 Open3D 输出：

```bash
cd /home/ubuntu/humanoid_ws
source install/setup.bash

ros2 bag play /home/ubuntu/nav_drift_test/nav_drift_test25 \
  --clock \
  --topics /odom /fast_lio/cloud_registered /robot_realpose /navigation/status \
  --rate 3.0 \
  --disable-keyboard-controls
```

输出文件：

```text
debug_monitor/open3d_nav_drift_test25_current/samples.csv
debug_monitor/open3d_nav_drift_test25_current/summary.json
```

对 test23/test24 同理替换 bag 路径和 `output_dir`。

## 5. 对比指标

沿用之前 bag 验证文档中的报告格式，重点比较：

```text
bag 名称:
方案:
参数:

整体统计:
  samples:
  ACCEPTED:
  REJECTED:
  PENDING:
  low_confidence:
  no_confidence:
  spin_to_pose_freeze_tf:
  confirmed_large_correction:

map->odom 跳变:
  总次数:
  accepted 跳变次数:
  pending 跳变次数:
  最大 accepted dxy:
  最大 pending dxy:

点位统计:
  点位 - 是否 completed/paused/failed - 最终距离 - accepted dxy 总量 - 最大 accepted dxy - pending dxy 总量

结论:
  是否比基线更稳:
  是否误伤导航:
  是否建议保留:
  下一轮需要改什么:
```

RoboSense 额外关注：

- `lidar_localization_node.log` 中 `match_score`
- `valid_pair_ratio`
- `prior_delta_xy`
- `prior_delta_yaw`
- `status`，CSV 中 `lidar_localization` 行的最后一列
- 是否频繁出现 `LOW_ACCURACY` 或 `valid_pair_ratio: 0`

Open3D 额外关注：

- `/prior_localization/confidence`
- bridge 状态中的 `ACCEPTED` / `REJECTED` / `PENDING`
- 是否在 `SpinToPose` 或原地旋转阶段触发大跳

## 6. 推荐验证顺序

1. 使用已有 RoboSense test23/test24 结果先看 HTML 和 CSV。
2. 对 test25 跑 RoboSense 全量回放。
3. 对 test23/24/25 跑当前 Open3D isolated 回放，保证和当前代码一致。
4. 分别生成每个 bag 的报告。
5. 如果 RoboSense 在三个 bag 上比 Open3D 更稳定，再考虑集成到 `/home/ubuntu/humanoid_ws/src/robosense_lidar_localization`。

## 7. 当前初步判断

RoboSense 方法可以作为独立工作空间先验证，不需要现在就移植进主工作空间。

从 test25 60 秒 smoke 看，启动初期有几帧 `valid_pair_ratio: 0` 和 `NOT_CONVERGED`，随后进入稳定匹配，后段 `valid_pair_ratio` 约 `0.995`，`match_score` 约 `0.029`。这说明它能跑通，但是否比 Open3D 更适合导航，还必须看完整 test23/24/25 的跳变、失配和点位结果。
