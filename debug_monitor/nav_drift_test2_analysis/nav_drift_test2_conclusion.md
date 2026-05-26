# nav_drift_test2 Conclusion

Bag: `/home/ubuntu/nav_drift_test2`

Continuity output: `/home/ubuntu/humanoid_ws/debug_monitor/nav_drift_test2_analysis`

ScanContext output: `/home/ubuntu/humanoid_ws/debug_monitor/nav_drift_test2_scancontext_current_params`

## Bag Topics

- `/odom`: 15474 messages
- `/fast_lio/cloud_registered`: 15474 messages
- `/robot_realpose`: 10131 messages
- `/initialpose`: 194 messages
- `/localization/recovery_requests`: 23 messages
- `/localization/recovery_status`: 758 messages

## Odom Continuity

`/odom` translation is continuous in this run.

- Max adjacent translation step: `0.0774 m`
- P95 adjacent translation step: `0.0304 m`
- Max adjacent speed: `0.8783 m/s`
- P95 adjacent speed: `0.3048 m/s`
- Translational jumps over `0.2 m`: `0`

The yaw-only jump count is high if interpreted as ROS planar yaw, but the Fast-LIO frame here is non-standard, so this yaw extraction is not a reliable indicator of map-frame heading drift. The important result for RViz position jumping is the translation continuity: `/odom` does not show position teleporting.

## Existing Navigation Pose Behavior

`/robot_realpose` does show map-level jumps:

- Max `/robot_realpose` jump: `14.5789 m` at stamp `1779696271.407644`
- Other large jumps: `4.3505 m`, `4.1540 m`, `1.9930 m`
- Recovery status includes:
  - `localization_recovery_started`: 49
  - `localization_relocalize_accepted`: 24
  - `localization_initialpose_published`: 24
  - `localization_recovered`: 3

This means the bag already contains map-level pose jumps from the existing localization/recovery stack. Those jumps are not caused by `/odom` translation.

## Current ScanContext Params On This Bag

Offline check used current safety params:

- `sc_threshold=0.25`
- `odom_gate_distance=1.0`
- `enable_candidate_confidence_gate=true`
- `min_sc_distance_gap=0.03`
- `max_ambiguous_candidate_distance=2.0`
- `workers=2`

Results:

- Samples: `753`
- Accepted: `488 / 753` (`64.8%`)
- Odom-gate rejected: `264 / 753` (`35.1%`)
- Ambiguity-gate rejected: `0`
- Odom route distance p95: `0.4604 m`
- First odom off route over `1.0 m`: none
- Accepted SC-vs-odom max error: `0.9999 m`
- Accepted SC-vs-odom over `1.0 m`: `0`
- Far wrong examples, such as `35 m` SC-vs-odom candidates, were rejected by odom gate.

## Answer To The Navigation Question

If the current ScanContext sidecar is used with the current safe params, it should not make RViz suddenly jump to another far-away place during this bag:

- `/odom` itself does not teleport in translation.
- Current SC accepts only candidates within the `1.0 m` odom gate.
- Far wrong ScanContext matches exist in the raw candidates, but they are rejected.
- Online mode is stricter than this offline check if GICP refinement is enabled.

The important caveat is integration:

- With `publish_initialpose=false`, this sidecar only publishes `/scancontext_global_localization/best_pose`; it will not move Nav2 or RViz by itself.
- If `publish_initialpose=true` is enabled without a correct `camera_init -> map` conversion, RViz/Nav2 can still jump because of a frame mismatch.
- If this sidecar is wired into Nav2 correctly, the expected correction size in this bag is bounded by the current `1.0 m` odom gate, not a tens-of-meters jump.
