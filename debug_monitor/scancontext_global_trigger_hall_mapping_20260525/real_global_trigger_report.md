# Real Global Trigger Validation

- CSV: `/home/ubuntu/humanoid_ws/debug_monitor/scancontext_global_trigger_hall_mapping_20260525/global_trigger.csv`
- Bag: `/home/ubuntu/fast-lio-bags/hall_mapping`
- Mode: real ROS node + `/trigger_global` + real GICP + 3x bag playback
- Duration: 180s wall time, trigger period 2s

## Summary

- Records: 87
- Accepted/published global poses: 1/87 (1.1%)
- Accepted XY error max: 0.0331 m
- Accepted yaw error max: 0.019 deg
- Accepted GICP fitness max: 0.008150
- Non-published candidates with XY error <= 1m: 29
- Non-published candidates with XY error > 1m and GICP fitness <= 0.20: 57

## Interpretation

- The current global recovery path is safe in this moving-bag run because it published only one pose, and that pose was correct.
- The recovery success rate is not acceptable yet: only 1.1% of global trigger attempts published a pose.
- More importantly, GICP fitness alone is not discriminative enough in this map: many wrong global candidates still have fitness <= 0.20.
- Because the bag is moving, this test is stricter than real recovery, where navigation should pause and the robot should be stationary. However, the wrong low-fitness candidates mean a stationary repeated trigger at an ambiguous location could still accept the wrong cluster.

## Worst Wrong Low-Fitness Candidates

| error_xy_m | gicp_fitness | keyframe | sc_distance |
|---:|---:|---:|---:|
| 26.028 | 0.000000 | 436 | 0.000372 |
| 25.586 | 0.128455 | 528 | 0.000131 |
| 25.569 | 0.138438 | 513 | 0.006335 |
| 25.089 | 0.143465 | 529 | 0.012555 |
| 25.056 | 0.000000 | 554 | 0.000000 |
| 24.560 | 0.000000 | 480 | 0.000316 |
| 23.731 | 0.135809 | 514 | 0.000035 |
| 20.779 | 0.000000 | 394 | 0.000007 |
| 20.776 | 0.000000 | 394 | 0.000019 |
| 20.775 | 0.000000 | 394 | 0.000008 |
| 20.773 | 0.000000 | 394 | 0.000001 |
| 19.454 | 0.000000 | 675 | 0.000000 |
