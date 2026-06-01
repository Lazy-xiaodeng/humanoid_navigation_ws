# live_20260524_165144

该目录是一次调试、回放、监控或对比实验的结果归档。目录名保留了实验主题、bag/test 编号或日期，便于追溯。

## 使用方法

- 优先查看 `summary.json`、`report.md`、`*_report.md`、`*.html` 或 `plots/` 下图片。
- `samples.csv`、`poses_*.csv`、`*.jsonl` 用于后续脚本复算和统计。
- `ros_logs/`、`*.log` 用于排查当次 launch、节点启动、TF 或定位状态。

## 文件说明

| 文件 | 用途 | 使用方法/注意事项 |
|---|---|---|
| `bw_airy_points.log` | 进程或 ROS 节点运行日志；排查启动、TF、定位和回放问题时查看。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `bw_airy_points_filtered.log` | 进程或 ROS 节点运行日志；排查启动、TF、定位和回放问题时查看。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `echo_diagnostics.log` | 进程或 ROS 节点运行日志；排查启动、TF、定位和回放问题时查看。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `echo_hdl_relocalize_prior.log` | 进程或 ROS 节点运行日志；排查启动、TF、定位和回放问题时查看。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `echo_initialpose.log` | 进程或 ROS 节点运行日志；排查启动、TF、定位和回放问题时查看。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `echo_rosout.log` | 进程或 ROS 节点运行日志；排查启动、TF、定位和回放问题时查看。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `hz_airy_points.log` | 进程或 ROS 节点运行日志；排查启动、TF、定位和回放问题时查看。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `hz_airy_points_filtered.log` | 进程或 ROS 节点运行日志；排查启动、TF、定位和回放问题时查看。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `hz_airy_points_for_elevation.log` | 进程或 ROS 节点运行日志；排查启动、TF、定位和回放问题时查看。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `hz_fast_lio_cloud_registered.log` | 进程或 ROS 节点运行日志；排查启动、TF、定位和回放问题时查看。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `hz_hdl_bootstrap_odom.log` | 进程或 ROS 节点运行日志；排查启动、TF、定位和回放问题时查看。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `hz_odom.log` | 进程或 ROS 节点运行日志；排查启动、TF、定位和回放问题时查看。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `hz_pcl_pose.log` | 进程或 ROS 节点运行日志；排查启动、TF、定位和回放问题时查看。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `hz_pose.log` | 进程或 ROS 节点运行日志；排查启动、TF、定位和回放问题时查看。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `hz_robot_realpose.log` | 进程或 ROS 节点运行日志；排查启动、TF、定位和回放问题时查看。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `hz_tf.log` | 进程或 ROS 节点运行日志；排查启动、TF、定位和回放问题时查看。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `meta.txt` | 文本日志、说明或元数据；直接打开或用 rg 检索关键字。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `pids.tsv` | 归档文件；根据扩展名和所在目录判断用途，默认不参与当前正式启动链路。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `tail_ros_node_logs.log` | 进程或 ROS 节点运行日志；排查启动、TF、定位和回放问题时查看。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `tail_start_navigation.log` | 进程或 ROS 节点运行日志；排查启动、TF、定位和回放问题时查看。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `tf_echo_map_base_footprint.log` | 进程或 ROS 节点运行日志；排查启动、TF、定位和回放问题时查看。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `tf_echo_map_ground_base_footprint.log` | 进程或 ROS 节点运行日志；排查启动、TF、定位和回放问题时查看。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `tf_echo_map_odom.log` | 进程或 ROS 节点运行日志；排查启动、TF、定位和回放问题时查看。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `tf_echo_odom_base_footprint.log` | 进程或 ROS 节点运行日志；排查启动、TF、定位和回放问题时查看。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `tf_monitor_map_base_footprint.log` | 进程或 ROS 节点运行日志；排查启动、TF、定位和回放问题时查看。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `tf_monitor_map_ground_base_footprint.log` | 进程或 ROS 节点运行日志；排查启动、TF、定位和回放问题时查看。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `tf_monitor_map_odom.log` | 进程或 ROS 节点运行日志；排查启动、TF、定位和回放问题时查看。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
| `tf_monitor_odom_base_footprint.log` | 进程或 ROS 节点运行日志；排查启动、TF、定位和回放问题时查看。 | 路径已归档到本目录；如脚本依赖绝对工作空间路径，保留原 `/home/ubuntu/humanoid_ws` 引用。 |
