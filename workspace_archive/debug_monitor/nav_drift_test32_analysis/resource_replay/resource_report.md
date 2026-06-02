# nav_drift_test32 bag 回放资源占用

- bag: `/home/ubuntu/nav_drift_test/nav_drift_test32`
- rate: 3.0x
- wall time: 102.2s
- return code: 0

| 排名 | 进程/节点 | 最大CPU% | 平均CPU% | 最大RSS MB | 命令 |
|---:|---|---:|---:|---:|---|
| 1 | rosbag2_player | 40.23 | 16.36 | 153.2 | `/usr/bin/python3 /opt/ros/jazzy/bin/ros2 bag play /home/ubuntu/nav_drift_test/nav_drift_test32 --clock --rate 3.0 --read-ahead-queue-size 100 --playback-duration 300.0 --disable-keyboard-controls` |
| 2 | ros2 | 10.94 | 10.94 | 0.0 | `ros2` |
