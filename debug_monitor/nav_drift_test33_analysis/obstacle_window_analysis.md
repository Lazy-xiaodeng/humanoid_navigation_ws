# nav_drift_test33 第25点障碍窗口细查

## 结论

- 分析窗口：`1780389417.367-1780389480.950`；重点障碍窗口：`1780389438.017-1780389475.384`。
- `/navigation/status` 在窗口内 active 样本 123/6283，paused 样本 0；日志显示 `navigation_obstacle_blocked` 后 Nav2 goal 没有被取消，BT 继续执行 wait recovery 并重试 FollowPath。
- 重点窗口内 `/cmd_vel` 最大平面速度 `0.500 m/s`，最大正向 vx `0.500 m/s`，最大角速度 `0.220 rad/s`；存在多段非零速度，不是一直停住。
- 窗口内 `/odom` 位移 `6.00 m`，`/robot_realpose` 位移 `13.25 m`；重点障碍窗口内 `/odom` 位移 `2.09 m`，`/robot_realpose` 位移 `3.33 m`。
- local costmap 前方检测窗口样本 112 帧，其中前方有 occupied/lethal 的帧 0，前方完全 clear 的帧 112，最大 cost 0，最大 lethal cell 数 0。
- bridge 事件统计：`{'ACCEPTED': 636}`；窗口内最大 `map->odom` 跳变 `0.025 m`。

## 解释

- 这次“先停、播报障碍、随后又走”的直接机制是 Nav2 BT 的恢复行为：RPP 连续检测 collision 后 FollowPath abort，BT 进入 wait；等待结束后重新计算/跟踪路径，状态管理器只上报 `navigation_obstacle_blocked`，没有取消 Nav2 当前 goal。
- 从定位侧看，第 25 点窗口没有 REJECTED/HOLD，也没有 `map->odom` 大跳，因此不像是定位漂移导致机器人忽略障碍。
- 从 costmap 侧看，重点窗口里如果出现前方 clear 帧，就说明局部代价地图在某些重试时刻已经不再认为正前方被占用；这会让 RPP 重新输出速度。该结论依赖 local costmap 的栅格坐标和 `/odom` 位姿反算，不能替代真实闭环复现。

## `/cmd_vel` 非零段

| start | end | duration | max_v | max_vx | max_wz | samples |
|---:|---:|---:|---:|---:|---:|---:|
| 1780389417.411 | 1780389429.786 | 12.38 | 0.500 | 0.500 | 0.329 | 100 |
| 1780389429.801 | 1780389431.401 | 1.60 | 0.000 | 0.000 | 1.000 | 9 |
| 1780389433.689 | 1780389434.889 | 1.20 | 0.000 | 0.000 | 1.000 | 7 |
| 1780389435.144 | 1780389437.892 | 2.75 | 0.500 | 0.500 | 0.559 | 23 |
| 1780389459.009 | 1780389459.510 | 0.50 | 0.500 | 0.500 | 0.182 | 5 |
| 1780389459.760 | 1780389459.760 | 0.00 | 0.500 | 0.500 | 0.151 | 1 |
| 1780389460.135 | 1780389462.760 | 2.63 | 0.500 | 0.500 | 0.220 | 22 |
| 1780389473.319 | 1780389478.096 | 4.78 | 0.500 | 0.500 | 0.417 | 39 |
| 1780389478.109 | 1780389480.709 | 2.60 | 0.000 | 0.000 | 1.000 | 14 |

## `/cmd_vel` 正向 vx 段

| start | end | duration | max_vx | samples |
|---:|---:|---:|---:|---:|
| 1780389417.411 | 1780389429.786 | 12.38 | 0.500 | 100 |
| 1780389435.144 | 1780389437.892 | 2.75 | 0.500 | 23 |
| 1780389459.009 | 1780389459.510 | 0.50 | 0.500 | 5 |
| 1780389459.760 | 1780389459.760 | 0.00 | 0.500 | 1 |
| 1780389460.135 | 1780389462.760 | 2.63 | 0.500 | 22 |
| 1780389473.319 | 1780389478.096 | 4.78 | 0.500 | 39 |

## `/cmd_vel` 归零段

| start | end | duration | samples |
|---:|---:|---:|---:|
| 1780389429.786 | 1780389429.786 | 0.00 | 1 |
| 1780389431.601 | 1780389431.601 | 0.00 | 1 |
| 1780389435.089 | 1780389435.089 | 0.00 | 1 |
| 1780389438.018 | 1780389438.391 | 0.37 | 4 |
| 1780389448.449 | 1780389448.953 | 0.50 | 5 |
| 1780389459.635 | 1780389459.635 | 0.00 | 1 |
| 1780389459.888 | 1780389460.010 | 0.12 | 2 |
| 1780389462.885 | 1780389463.270 | 0.39 | 4 |
| 1780389478.096 | 1780389478.096 | 0.00 | 1 |
| 1780389480.909 | 1780389480.909 | 0.00 | 1 |

## local costmap 前方窗口

| t | frame | sampled | occupied | lethal | unknown | max_cost |
|---:|---|---:|---:|---:|---:|---:|
| 1780389438.068 | odom_ground | 520 | 0 | 0 | 0 | 0 |
| 1780389438.468 | odom_ground | 520 | 0 | 0 | 0 | 0 |
| 1780389438.682 | odom_ground | 520 | 0 | 0 | 0 | 0 |
| 1780389439.069 | odom_ground | 519 | 0 | 0 | 0 | 0 |
| 1780389439.470 | odom_ground | 520 | 0 | 0 | 0 | 0 |
| 1780389439.868 | odom_ground | 520 | 0 | 0 | 0 | 0 |
| 1780389440.280 | odom_ground | 520 | 0 | 0 | 0 | 0 |
| 1780389440.676 | odom_ground | 517 | 0 | 0 | 0 | 0 |
| 1780389441.077 | odom_ground | 520 | 0 | 0 | 0 | 0 |
| 1780389441.277 | odom_ground | 521 | 0 | 0 | 0 | 0 |
| 1780389441.669 | odom_ground | 520 | 0 | 0 | 0 | 0 |
| 1780389442.069 | odom_ground | 521 | 0 | 0 | 0 | 0 |
| 1780389442.468 | odom_ground | 521 | 0 | 0 | 0 | 0 |
| 1780389442.669 | odom_ground | 521 | 0 | 0 | 0 | 0 |
| 1780389443.069 | odom_ground | 519 | 0 | 0 | 0 | 0 |
| 1780389443.469 | odom_ground | 519 | 0 | 0 | 0 | 0 |
| 1780389443.869 | odom_ground | 520 | 0 | 0 | 0 | 0 |
| 1780389444.269 | odom_ground | 521 | 0 | 0 | 0 | 0 |
| 1780389444.668 | odom_ground | 520 | 0 | 0 | 0 | 0 |
| 1780389445.069 | odom_ground | 518 | 0 | 0 | 0 | 0 |
| 1780389445.269 | odom_ground | 519 | 0 | 0 | 0 | 0 |
| 1780389445.668 | odom_ground | 520 | 0 | 0 | 0 | 0 |
| 1780389446.068 | odom_ground | 520 | 0 | 0 | 0 | 0 |
| 1780389446.269 | odom_ground | 518 | 0 | 0 | 0 | 0 |
| 1780389446.669 | odom_ground | 519 | 0 | 0 | 0 | 0 |
| 1780389447.069 | odom_ground | 521 | 0 | 0 | 0 | 0 |
| 1780389447.269 | odom_ground | 521 | 0 | 0 | 0 | 0 |
| 1780389447.469 | odom_ground | 519 | 0 | 0 | 0 | 0 |
| 1780389447.882 | odom_ground | 520 | 0 | 0 | 0 | 0 |
| 1780389448.269 | odom_ground | 521 | 0 | 0 | 0 | 0 |
| 1780389448.670 | odom_ground | 522 | 0 | 0 | 0 | 0 |
| 1780389448.889 | odom_ground | 522 | 0 | 0 | 0 | 0 |
| 1780389449.286 | odom_ground | 518 | 0 | 0 | 0 | 0 |
| 1780389449.666 | odom_ground | 522 | 0 | 0 | 0 | 0 |
| 1780389449.868 | odom_ground | 520 | 0 | 0 | 0 | 0 |
| 1780389450.081 | odom_ground | 520 | 0 | 0 | 0 | 0 |
| 1780389450.467 | odom_ground | 521 | 0 | 0 | 0 | 0 |
| 1780389450.668 | odom_ground | 521 | 0 | 0 | 0 | 0 |
| 1780389451.068 | odom_ground | 521 | 0 | 0 | 0 | 0 |
| 1780389451.269 | odom_ground | 520 | 0 | 0 | 0 | 0 |
| 1780389451.670 | odom_ground | 520 | 0 | 0 | 0 | 0 |
| 1780389452.067 | odom_ground | 521 | 0 | 0 | 0 | 0 |
| 1780389452.268 | odom_ground | 520 | 0 | 0 | 0 | 0 |
| 1780389452.669 | odom_ground | 520 | 0 | 0 | 0 | 0 |
| 1780389453.069 | odom_ground | 521 | 0 | 0 | 0 | 0 |
| 1780389453.468 | odom_ground | 521 | 0 | 0 | 0 | 0 |
| 1780389453.669 | odom_ground | 518 | 0 | 0 | 0 | 0 |
| 1780389454.069 | odom_ground | 519 | 0 | 0 | 0 | 0 |
| 1780389454.468 | odom_ground | 520 | 0 | 0 | 0 | 0 |
| 1780389454.868 | odom_ground | 521 | 0 | 0 | 0 | 0 |
| 1780389455.269 | odom_ground | 519 | 0 | 0 | 0 | 0 |
| 1780389455.669 | odom_ground | 518 | 0 | 0 | 0 | 0 |
| 1780389456.068 | odom_ground | 519 | 0 | 0 | 0 | 0 |
| 1780389456.469 | odom_ground | 519 | 0 | 0 | 0 | 0 |
| 1780389456.869 | odom_ground | 519 | 0 | 0 | 0 | 0 |
| 1780389457.069 | odom_ground | 519 | 0 | 0 | 0 | 0 |
| 1780389457.468 | odom_ground | 518 | 0 | 0 | 0 | 0 |
| 1780389457.868 | odom_ground | 520 | 0 | 0 | 0 | 0 |
| 1780389458.269 | odom_ground | 521 | 0 | 0 | 0 | 0 |
| 1780389458.689 | odom_ground | 519 | 0 | 0 | 0 | 0 |
| 1780389459.087 | odom_ground | 520 | 0 | 0 | 0 | 0 |
| 1780389459.289 | odom_ground | 519 | 0 | 0 | 0 | 0 |
| 1780389459.494 | odom_ground | 521 | 0 | 0 | 0 | 0 |
| 1780389459.694 | odom_ground | 520 | 0 | 0 | 0 | 0 |
| 1780389460.084 | odom_ground | 520 | 0 | 0 | 0 | 0 |
| 1780389460.293 | odom_ground | 520 | 0 | 0 | 0 | 0 |
| 1780389460.682 | odom_ground | 519 | 0 | 0 | 0 | 0 |
| 1780389460.884 | odom_ground | 519 | 0 | 0 | 0 | 0 |
| 1780389461.296 | odom_ground | 510 | 0 | 0 | 0 | 0 |
| 1780389461.685 | odom_ground | 518 | 0 | 0 | 0 | 0 |
| 1780389462.080 | odom_ground | 520 | 0 | 0 | 0 | 0 |
| 1780389462.466 | odom_ground | 515 | 0 | 0 | 0 | 0 |
| 1780389462.675 | odom_ground | 520 | 0 | 0 | 0 | 0 |
| 1780389462.880 | odom_ground | 522 | 0 | 0 | 0 | 0 |
| 1780389463.270 | odom_ground | 521 | 0 | 0 | 0 | 0 |
| 1780389463.670 | odom_ground | 521 | 0 | 0 | 0 | 0 |
| 1780389464.075 | odom_ground | 519 | 0 | 0 | 0 | 0 |
| 1780389464.469 | odom_ground | 520 | 0 | 0 | 0 | 0 |
| 1780389464.669 | odom_ground | 518 | 0 | 0 | 0 | 0 |
| 1780389464.870 | odom_ground | 521 | 0 | 0 | 0 | 0 |

## bridge / map->odom

- bridge reasons：`{'small_correction': 636}`

| t | dxy | dyaw |
|---:|---:|---:|
| 1780389427.407 | 0.020 | 0.002 |
| 1780389461.507 | 0.024 | 0.002 |
| 1780389462.307 | 0.025 | -0.001 |

## ROS 日志时间线

| t | event |
|---:|---|
| 1780389417.368 | 开始导航到路点: 点位24 (1/1), walk_direction=forward |
| 1780389420.036 | Passing new path to controller. |
| 1780389422.536 | Passing new path to controller. |
| 1780389425.035 | Passing new path to controller. |
| 1780389427.535 | Passing new path to controller. |
| 1780389429.786 | Reached the goal! |
| 1780389431.640 | Goal succeeded |
| 1780389431.642 | Nav2确认到达路点: 点位24 |
| 1780389433.658 | 开始导航到路点: 点位25 (1/1), walk_direction=forward |
| 1780389437.766 | Passing new path to controller. |
| 1780389438.018 | RegulatedPurePursuitController detected collision ahead! |
| 1780389438.141 | RegulatedPurePursuitController detected collision ahead! |
| 1780389438.267 | RegulatedPurePursuitController detected collision ahead! |
| 1780389438.391 | RegulatedPurePursuitController detected collision ahead! |
| 1780389438.391 | Controller patience exceeded |
| 1780389438.409 | Running wait |
| 1780389443.983 | 收到状态管理器确认: navigation_obstacle_blocked - error |
| 1780389443.984 | 📤 立即推送导航事件: navigation_obstacle_blocked |
| 1780389443.985 | ������ 转发导航确认: navigation_obstacle_blocked - error |
| 1780389448.409 | wait completed successfully |
| 1780389448.449 | RegulatedPurePursuitController detected collision ahead! |
| 1780389448.574 | RegulatedPurePursuitController detected collision ahead! |
| 1780389448.699 | RegulatedPurePursuitController detected collision ahead! |
| 1780389448.824 | RegulatedPurePursuitController detected collision ahead! |
| 1780389448.950 | RegulatedPurePursuitController detected collision ahead! |
| 1780389448.950 | Controller patience exceeded |
| 1780389448.969 | Running wait |
| 1780389458.969 | wait completed successfully |
| 1780389459.635 | RegulatedPurePursuitController detected collision ahead! |
| 1780389459.888 | RegulatedPurePursuitController detected collision ahead! |
| 1780389460.010 | RegulatedPurePursuitController detected collision ahead! |
| 1780389461.634 | Passing new path to controller. |
| 1780389462.885 | RegulatedPurePursuitController detected collision ahead! |
| 1780389463.009 | RegulatedPurePursuitController detected collision ahead! |
| 1780389463.135 | RegulatedPurePursuitController detected collision ahead! |
| 1780389463.270 | RegulatedPurePursuitController detected collision ahead! |
| 1780389463.270 | Controller patience exceeded |
| 1780389463.289 | Running wait |
| 1780389473.291 | wait completed successfully |
| 1780389475.384 | ✅ 机器人阻塞后已恢复运动，速度: 0.3884 m/s (pose_delta)，允许后续阻塞重新上报 |
| 1780389475.944 | Passing new path to controller. |
| 1780389478.096 | Reached the goal! |
| 1780389480.948 | Goal succeeded |
| 1780389480.950 | Nav2确认到达路点: 点位25 |

## 数据限制

- bag 中包含 local costmap：`True`；包含 prior 候选位姿/置信度：`False`。
- 前方 costmap 统计使用 `/local_costmap/costmap` 栅格、`/odom` 或 `/robot_realpose` 最近位姿，将机器人前方 0.10-1.40m、左右 0.50m 作为检测区域；本 bag 的 local costmap frame 为 `odom_ground`，其 XY 与 `/robot_realpose` 更一致，因此该 frame 使用 `/robot_realpose` 反算。
- 该区域不是 Nav2 footprint/collision checker 的完整等价实现，只用于判断前方代价是否大体持续存在。
- 本报告是离线分析，不是闭环 Nav2 回放。
