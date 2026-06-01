# ScanContext Recovery Success Estimate

- Source samples: `debug_monitor/nav_drift_test2_scancontext_rerun_20260525_latest/samples.csv`
- Samples: 753
- Conservative success: 488/753 (64.8%)
- Global additional success: 3/753 (0.4%)
- Total estimated success: 491/753 (65.2%)
- Estimated wrong global publish: 237/753 (31.5%)
- No recovery publish: 25/753 (3.3%)

This estimates the new recovery state machine from offline SC samples. It does not include real GICP fitness, so wrong-publish risk is an upper-bound signal rather than a final safety proof.

## Waypoints

| waypoint | samples | conservative | global add | total | wrong global | no publish |
|---|---:|---:|---:|---:|---:|---:|
| 点位1 | 27 | 27 | 0 | 27 | 0 | 0 |
| 点位2 | 29 | 4 | 0 | 4 | 22 | 3 |
| 点位3 | 31 | 10 | 0 | 10 | 19 | 2 |
| 点位4 | 34 | 1 | 0 | 1 | 29 | 4 |
| 点位5 | 15 | 0 | 0 | 0 | 14 | 1 |
| 点位6 | 29 | 2 | 0 | 2 | 24 | 3 |
| 点位7 | 19 | 4 | 0 | 4 | 13 | 2 |
| 点位8 | 19 | 0 | 0 | 0 | 15 | 4 |
| 点位9 | 42 | 0 | 0 | 0 | 41 | 1 |
| 点位10 | 32 | 29 | 1 | 30 | 2 | 0 |
| 点位11 | 5 | 5 | 0 | 5 | 0 | 0 |
| 点位12 | 4 | 4 | 0 | 4 | 0 | 0 |
| 点位13 | 446 | 391 | 2 | 393 | 53 | 0 |
| 点位21 | 2 | 2 | 0 | 2 | 0 | 0 |
