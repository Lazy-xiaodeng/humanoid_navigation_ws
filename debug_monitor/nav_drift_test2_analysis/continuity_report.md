# Nav Bag Continuity Analysis

- Bag: `/home/ubuntu/nav_drift_test2`
- Jump thresholds: distance>=0.50 m, speed>=2.00 m/s, yaw>=35.0 deg, dt<=0.50 s

## Summary

| topic | samples | jump events | max step | p95 step | max speed | p95 speed | max yaw step |
|---|---:|---:|---:|---:|---:|---:|---:|
| /odom | 15474 | 110 | 0.0917 m | 0.0304 m | 0.8783 m/s | 0.3048 m/s | 176.93 deg |
| /robot_realpose | 10131 | 28 | 14.5789 m | 0.0631 m | 145.8096 m/s | 0.6319 m/s | 52.69 deg |
| /initialpose | 194 | 0 | 0.0000 m | 0.0000 m | 0.0000 m/s | 0.0000 m/s | 0.00 deg |

## Recovery Topics

- Recovery requests: 23
- Recovery status messages: 758

Detailed jump events are in `jump_events.csv`.
