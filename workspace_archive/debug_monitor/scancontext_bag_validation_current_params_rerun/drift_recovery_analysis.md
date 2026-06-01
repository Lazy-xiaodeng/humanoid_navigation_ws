# Current Params Drift / Recovery Analysis

Bag: `/home/ubuntu/fast-lio-bags/hall_mapping`

Database: `/home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps/hall_sc_fastlio_registered.bin`

Output directory: `/home/ubuntu/humanoid_ws/debug_monitor/scancontext_bag_validation_current_params_rerun`

## Parameters

- `sc_threshold=0.25`
- `odom_gate_distance=1.0`
- `exclude_keyframe_window=3`
- `enable_candidate_confidence_gate=true`
- `min_sc_distance_gap=0.03`
- `max_ambiguous_candidate_distance=2.0`
- `workers=2`

## Overall Result

- Samples: 743
- Accepted SC matches: 481 / 743 (64.7%)
- Odom-gate rejected: 228 / 743 (30.7%)
- Ambiguity-gate rejected: 0 / 743
- Accepted SC-vs-odom max error: 0.9994 m
- Accepted SC-vs-odom p95 error: 0.9116 m
- Odom route-distance p95: 2.2314 m
- SC route-distance p95 for accepted matches: 1.6404 m

Interpretation:

- Accepted ScanContext results did not produce a dangerous global jump in this run.
- The largest accepted SC-vs-odom error stayed just below the 1.0 m odom gate.
- The main route-level drift is odom/trajectory drift near route segments and waypoints, especially near 点位22.
- With the current safety gate, ScanContext is stable but conservative: it avoids wrong jumps, but it will not forcibly correct odom if odom is already several meters off route.

## Drift Episodes

Route drift is defined as `odom_route_dist > 1.0 m`.

| episode | samples | nearest waypoint span | max odom route dist | max near | recovery | rescue samples |
|---:|---|---|---:|---|---|---:|
| 1 | 58-67 | 点位1 -> 点位1 | 1.040 m | 点位1 | sample 68 near 点位1 | 1 |
| 2 | 69-69 | 点位1 -> 点位1 | 1.024 m | 点位1 | sample 70 near 点位1 | 1 |
| 3 | 94-105 | 点位2 -> 点位6 | 2.173 m | 点位6 | sample 106 near 点位6 | 4 |
| 4 | 142-154 | 点位4 -> 点位6 | 1.640 m | 点位5 | sample 155 near 点位6 | 0 |
| 5 | 172-178 | 点位7 -> 点位7 | 1.445 m | 点位7 | sample 179 near 点位7 | 0 |
| 6 | 208-251 | 点位24 -> 点位8 | 2.362 m | 点位23 | sample 252 near 点位8 | 0 |
| 7 | 256-262 | 点位9 -> 点位9 | 1.107 m | 点位9 | sample 263 near 点位9 | 3 |
| 8 | 274-276 | 点位10 -> 点位10 | 1.153 m | 点位10 | sample 277 near 点位10 | 2 |
| 9 | 292-298 | 点位23 -> 点位10 | 1.578 m | 点位23 | sample 299 near 点位10 | 1 |
| 10 | 302-342 | 点位23 -> 点位22 | 4.120 m | 点位22 | sample 343 near 点位22 | 1 |
| 11 | 503-504 | 点位16 -> 点位16 | 1.117 m | 点位16 | sample 505 near 点位16 | 1 |
| 12 | 531-543 | 点位17 -> 点位17 | 1.677 m | 点位18 | sample 544 near 点位17 | 5 |
| 13 | 567-603 | 点位20 -> 点位15 | 1.406 m | 点位16 | sample 604 near 点位15 | 0 |
| 14 | 606-609 | 点位14 -> 点位14 | 1.389 m | 点位14 | sample 610 near 点位13 | 0 |
| 15 | 617-618 | 点位21 -> 点位14 | 1.232 m | 点位14 | sample 619 near 点位14 | 1 |
| 16 | 703-706 | 点位24 -> 点位1 | 2.129 m | 点位24 | sample 707 near 点位1 | 0 |

## Point-Level Notes

- First off-route event: sample 58, near 点位1, odom route distance 1.013 m.
- Worst drift: sample 329, near 点位22, odom route distance 4.120 m.
- Final sample: sample 742, near 点位1, odom route distance 0.066 m, SC accepted, SC-vs-odom error 0.007 m.
- Not reached within 1 m in this bag trajectory: 点位5, 点位8, 点位9, 点位10, 点位20, 点位24.

## Conclusion

From origin through the end of the bag, localization does show route-level drift in several sections, but the run recovers back to the route after every detected drift episode. The most significant drift happens around 点位22. The current ScanContext configuration is stable in the sense that accepted relocalization does not jump far away from odom. However, because odom consistency gating is enabled at 1.0 m, ScanContext is not allowed to perform a large global correction when odom itself is several meters off route.

For navigation stability, keep this safe mode for normal operation. Add a separate recovery mode only when the robot is clearly lost: temporarily relax odom gate, require multi-frame agreement, and require strict GICP fitness before publishing any correction to Nav2.
