# Session Handoff (2026-05-25) - ScanContext + Fast-LIO Bags

This file is a compact “handoff note” so a new chat session can continue from the current state without re-deriving assumptions.

## Goal Recap

- Identify the coordinate axes / frames of the mapping bag point cloud used for building the map.
- Make ScanContext-based relocalization work with Fast-LIO mapping bags (`/fast_lio/cloud_registered` + `/odom`).
- Build a usable ScanContext database from the correct mapping bag.
- Build a strict, bag-based validation pipeline (2 CPU cores) to quantify drift / failure points and whether “relocalization rescue” can recover.
- Iterate parameters/code for stability (avoid catastrophic wrong relocalizations in symmetric corridors).

## Key Coordinate/Frame Facts

- Fast-LIO in this workspace uses a non-ROS-standard body/world axes convention typically described as:
  - In `camera_init/body`: `x = left`, `y = down`, `z = back`
- ROS navigation convention is: `x = forward`, `y = left`, `z = up`
- Conversion used across this workspace:
  - `x_ros = -z_fastlio`
  - `y_ros =  x_fastlio`
  - `z_ros = -y_fastlio`
- Important: `src/humanoid_navigation2/humanoid_navigation2/save_pcd_map.py` does **no TF conversion**.
  - Therefore `pcd/hall.pcd` is saved in **Fast-LIO coordinates** (the frame of `/fast_lio/cloud_registered`, typically `camera_init`).

## Why `/fast_lio/cloud_registered` Needs Special Handling

- `/fast_lio/cloud_registered` points are already in the “world” frame (`camera_init`) and are *registered*.
- For ScanContext descriptors (place recognition), we want a body-local / sensor-local cloud per keyframe.
- So we must “unregister” each cloud back into local coordinates using `/odom` pose at the same time:
  - `p_local = R^T * (p_world - t)` (using the odom pose `(R,t)` of base in world)

## Code Changes Already Made

Package: `src/humanoid_scancontext_global_localization`

- Defaults updated to match Fast-LIO:
  - `cloud_topic: /fast_lio/cloud_registered`
  - `odom_topic: /odom`
- Added `cloud_frame_mode` parameter:
  - `registered`: treat incoming cloud as world-registered and transform back to local using `/odom`
  - `local`: treat incoming cloud as already local (legacy case)
- Node now subscribes to `/odom` when `cloud_frame_mode=registered`.
- Yaw correction bug fix: yaw correction now rotates in the configured horizontal plane axes, not hardcoded x/y.
- Published `map_frame` default is `"camera_init"` to avoid accidentally publishing a ROS `/map`-frame pose without explicit conversion. (Nav2 `/initialpose` requires consistent frame strategy; do not mix frames silently.)

Important params (final “safe mode” used for strict validation):

- `sc_distance_threshold: 0.25`
- `enable_odom_consistency_gate: true`
- `max_odom_consistency_distance: 1.0`  (meters, in horizontal plane)
- `max_cloud_odom_time_diff_sec: 0.35`
- `enable_candidate_confidence_gate: true`
- `min_sc_distance_gap: 0.03`
- `max_ambiguous_candidate_distance: 2.0`
- `max_refined_odom_consistency_distance: 1.5`

Rationale:

- In corridor-like environments, ScanContext can produce symmetric false matches.
- Odom-gating prevents catastrophic wrong relocalization at the cost of lower acceptance rate.
- Candidate ambiguity gating rejects cases where close-scored candidates are spatially far apart.
- The final refined-pose odom gate catches GICP results that drift away from the odom-consistent hypothesis.
- This improves navigation stability (better to reject than “jump” to a wrong corridor).

## Database: Old vs New

Existing database found:

- `src/humanoid_navigation2/maps/hall_sc.bin`
  - This was considered *format-compatible* but **not semantically compatible** (axis + registered/local mismatch).

New database built from the *actual mapping bag*:

- Bag used: `/home/ubuntu/fast-lio-bags/hall_mapping` (about 44G)
- Output DB (correct for Fast-LIO registered cloud pipeline):
  - `/home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps/hall_sc_fastlio_registered.bin`
- DB header check:
  - `sectors=90`, `rings=24`, `max_range=60.0`, `keyframes=743`

## Validation Pipeline (Strict, Offline, 2 Cores)

Offline validator script:

- `/home/ubuntu/humanoid_ws/src/humanoid_scancontext_global_localization/scripts/offline_bag_nav_validation.py`

Waypoints source:

- `/home/ubuntu/humanoid_ws/data/dynamic_waypoints.json`
  - Contains duplicates for 点位1..12; script supports choosing “latest” per number.

Final rerun outputs directory:

- `/home/ubuntu/humanoid_ws/debug_monitor/scancontext_bag_validation_final_rerun`
  - `samples.csv` (per-sample metrics)
  - `waypoints.csv` (waypoint coverage)
  - `summary.json` (stats)
  - `report.md` (human-readable report)

Latest stability-gate rerun outputs directory:

- `/home/ubuntu/humanoid_ws/debug_monitor/scancontext_bag_validation_stability_gates_20260525`
  - Same strict settings as the final rerun, plus candidate ambiguity gate fields.
  - `ambiguity_gate_rejected_count=0` in this specific strict run, meaning the existing `odom_gate=1.0` already removed the far ambiguous candidates for these samples.

Strict-mode notes:

- Strict mode excludes matching against the same or neighboring keyframes to prevent trivial self-match when validating on the same bag used to build the DB.
- Parameter: `--exclude-keyframe-window 3` (exclude +/-3 around current index).

Example command (high precision, 2 worker processes):

```bash
source /home/ubuntu/humanoid_ws/install/setup.bash
python3 /home/ubuntu/humanoid_ws/src/humanoid_scancontext_global_localization/scripts/offline_bag_nav_validation.py \
  --bag /home/ubuntu/fast-lio-bags/hall_mapping \
  --db /home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps/hall_sc_fastlio_registered.bin \
  --cloud-topic /fast_lio/cloud_registered \
  --odom-topic /odom \
  --interval 2.0 \
  --exclude-keyframe-window 3 \
  --sc-threshold 0.25 \
  --odom-gate 1.0 \
  --workers 2 \
  --out /home/ubuntu/humanoid_ws/debug_monitor/scancontext_bag_validation_final_rerun
```

## Latest Validation Summary (Final Rerun)

From `.../summary.json` (key points):

- Total samples: 743
- Accepted (passes thresholds): 481 (64.7%)
- Rejected by odom gate: 228 (30.7%)
- SC-vs-odom horizontal error:
  - median ~0.019 m
  - p95 ~0.912 m
- Odom-to-route distance:
  - median ~0.483 m
  - p95 ~2.231 m
- “Rescue possible” samples (heuristic): 20

Waypoints not reached within 1m in this bag run (likely the bag trajectory didn’t pass through them):

- 点位5 / 点位8 / 点位9 / 点位10 / 点位20 / 点位24

Interpretation:

- With strict validation, ScanContext alone can mis-match badly in symmetric areas.
- With odom consistency gating + stricter SC threshold, catastrophic jumps were largely eliminated.
- Tradeoff: acceptance rate drops; this is intended for stable navigation.

## Recommended Next Improvements (If Continuing)

- Add “global recovery mode”:
  - When odom is suspected wrong, temporarily relax odom gate but require multi-frame confirmation and a strict GICP fitness check before accepting a jump.
- Add confidence checks:
  - If top-N candidates disagree widely, treat as low confidence and reject.
- Add explicit frame alignment path for Nav2:
  - If publishing `/initialpose` into Nav2 `/map`, define and apply a consistent `camera_init <-> map` transform (do not silently mix frames).

## Files Mentioned By User (Reference)

- PCD map: `/home/ubuntu/humanoid_ws/src/humanoid_navigation2/pcd/hall.pcd`
- Old DB: `/home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps/hall_sc.bin`
- New DB: `/home/ubuntu/humanoid_ws/src/humanoid_navigation2/maps/hall_sc_fastlio_registered.bin`
