# ScanContext Recovery Success Estimate

- Source samples: `debug_monitor/nav_drift_test2_scancontext_rerun_20260525/samples.csv`
- Samples: 1463
- Conservative success: 934/1463 (63.8%)
- Global additional success: 13/1463 (0.9%)
- Total estimated success: 947/1463 (64.7%)
- Estimated wrong global publish: 475/1463 (32.5%)
- No recovery publish: 41/1463 (2.8%)

This estimates the new recovery state machine from offline SC samples. It does not include real GICP fitness, so wrong-publish risk is an upper-bound signal rather than a final safety proof.

## Waypoints

| waypoint | samples | conservative | global add | total | wrong global | no publish |
|---|---:|---:|---:|---:|---:|---:|
| 点位1 | 52 | 51 | 0 | 51 | 0 | 1 |
| 点位2 | 56 | 8 | 2 | 10 | 44 | 2 |
| 点位3 | 60 | 21 | 1 | 22 | 31 | 7 |
| 点位4 | 67 | 1 | 0 | 1 | 60 | 6 |
| 点位5 | 30 | 0 | 0 | 0 | 30 | 0 |
| 点位6 | 56 | 2 | 0 | 2 | 45 | 9 |
| 点位7 | 37 | 6 | 0 | 6 | 29 | 2 |
| 点位8 | 37 | 0 | 0 | 0 | 33 | 4 |
| 点位9 | 82 | 1 | 0 | 1 | 77 | 4 |
| 点位10 | 61 | 52 | 2 | 54 | 7 | 0 |
| 点位11 | 10 | 10 | 0 | 10 | 0 | 0 |
| 点位12 | 7 | 7 | 0 | 7 | 0 | 0 |
| 点位13 | 867 | 755 | 7 | 762 | 105 | 0 |
| 点位21 | 3 | 3 | 0 | 3 | 0 | 0 |
