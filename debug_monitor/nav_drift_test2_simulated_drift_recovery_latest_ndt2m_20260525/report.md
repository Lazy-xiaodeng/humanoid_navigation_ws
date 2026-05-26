# Simulated Drift Recovery Trials

- Source samples: `debug_monitor/nav_drift_test2_scancontext_rerun_20260525_latest/samples.csv`
- Random seed: `20260525`
- Trials: 12
- Success: 9/12 (75.0%)
- Simulated NDT accepts initialpose if XY error <= 2.00 m

This simulates the current feedback loop from offline ScanContext samples. It does not execute live NDT; use it as a repeatable bag-level risk test.

| sample | nearest wp | wp dist | result | attempts | final mode | final error | final wp | reason |
|---:|---|---:|---|---:|---|---:|---|---|
| 3 | 点位1 | 0.036 | success | 1 | conservative | 0.039 | 点位1 | conservative accepted by simulated NDT |
| 50 | 点位2 | 0.187 | success | 5 | global | 1.236 | 点位3 | global accepted by simulated NDT |
| 103 | 点位4 | 0.190 | fail | 5 | global | 17.579 | 点位24 | max attempts reached |
| 167 | 点位6 | 0.510 | fail | 5 | global | 11.184 | 点位9 | max attempts reached |
| 226 | 点位9 | 0.194 | fail | 5 | global | 2.281 | 点位9 | max attempts reached |
| 272 | 点位10 | 0.264 | success | 1 | conservative | 0.890 | 点位10 | conservative accepted by simulated NDT |
| 324 | 点位13 | 0.510 | success | 1 | conservative | 0.902 | 点位13 | conservative accepted by simulated NDT |
| 392 | 点位13 | 0.833 | success | 1 | conservative | 0.944 | 点位13 | conservative accepted by simulated NDT |
| 457 | 点位13 | 0.909 | success | 1 | conservative | 0.904 | 点位13 | conservative accepted by simulated NDT |
| 558 | 点位13 | 0.924 | success | 1 | conservative | 0.915 | 点位13 | conservative accepted by simulated NDT |
| 627 | 点位13 | 0.918 | success | 1 | conservative | 0.909 | 点位13 | conservative accepted by simulated NDT |
| 736 | 点位13 | 0.923 | success | 1 | conservative | 0.897 | 点位13 | conservative accepted by simulated NDT |
