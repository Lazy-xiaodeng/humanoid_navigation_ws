# nav_drift_test31 bag 回放资源占用

- bag: `/home/ubuntu/nav_drift_test/nav_drift_test31`
- rate: 3.0x
- wall time: 102.7s
- return code: 0

| 排名 | 进程/节点 | 最大CPU% | 平均CPU% | 最大RSS MB | 命令 |
|---:|---|---:|---:|---:|---|
| 1 | rosbag2_player | 49.12 | 15.92 | 166.0 | `/usr/bin/python3 /opt/ros/jazzy/bin/ros2 bag play /home/ubuntu/nav_drift_test/nav_drift_test31 --clock --rate 3.0 --read-ahead-queue-size 100 --playback-duration 300.0 --disable-keyboard-controls` |
| 2 | ros2 | 1.00 | 1.00 | 0.0 | `ros2` |
