# humanoid_scancontext_global_localization

Sidecar Scan Context++ style global localization package.

Defaults are intentionally non-invasive:

- subscribes to `/fast_lio/cloud_registered`
- publishes `/scancontext_global_localization/candidates`
- publishes `/scancontext_global_localization/best_pose`
- does not publish `/initialpose` unless `publish_initialpose:=true`
- does not publish TF

Coordinate convention:

- This robot's Fast-LIO frame is non-standard: `x=left`, `y=down`, `z=backward`.
- `/fast_lio/cloud_registered` is in Fast-LIO `camera_init`/world coordinates.
- The node and database builder transform registered clouds back into the
  odom/body-local frame with `/odom` before computing Scan Context.
- Scan Context is configured to use horizontal axes `x/z` and relative height `-y`.
- Published `best_pose` is in `camera_init`; keep `publish_initialpose:=false`
  unless a camera_init-to-Nav2-map conversion is added.
- The database builder and online node must use the same axis parameters.
- Descriptor height is relative to the lowest valid point in each scan, so small
  LiDAR mounting height changes, such as 7-8 cm between mapping and walking, are
  mostly removed before matching.
- The online node uses conservative safety gates by default: cloud/odom timestamp
  consistency, odom-distance gating, ambiguous-candidate rejection, and a final
  odom consistency check after GICP refinement.

Build a database from a mapping/navigation bag:

```bash
ros2 run humanoid_scancontext_global_localization build_scancontext_database.py \
  --bag /home/ubuntu/下载/bags/hall_mapping \
  --output /home/ubuntu/humanoid_ws/data/scancontext/hall_mapping_sc.bin \
  --cloud-topic /fast_lio/cloud_registered \
  --odom-topic /odom \
  --cloud-frame-mode registered \
  --interval 1.5 \
  --horizontal-axis-1 0 \
  --horizontal-axis-2 2 \
  --vertical-axis 1 \
  --vertical-sign -1.0
```

Run the sidecar:

```bash
ros2 launch humanoid_scancontext_global_localization scancontext_global_localization.launch.py \
  database_path:=/home/ubuntu/humanoid_ws/data/scancontext/hall_mapping_sc.bin \
  pcd_map_path:=/home/ubuntu/humanoid_ws/src/humanoid_navigation2/pcd/hall.pcd \
  cloud_topic:=/fast_lio/cloud_registered \
  odom_topic:=/odom \
  cloud_frame_mode:=registered \
  publish_initialpose:=false
```

Recommended stability defaults for this hall map:

- `sc_distance_threshold=0.25`
- `max_odom_consistency_distance=1.0`
- `min_sc_distance_gap=0.03`
- `max_ambiguous_candidate_distance=2.0`
- `max_refined_odom_consistency_distance=1.5`

Trigger one query from the latest cloud:

```bash
ros2 service call /scancontext_global_localization/trigger std_srvs/srv/Trigger {}
```

Record a validation run:

```bash
ros2 run humanoid_scancontext_global_localization record_scancontext_validation.py \
  --output /tmp/scancontext_validation.csv \
  --duration 120 \
  --trigger-period 2.0 \
  --odom-topic /odom
```
