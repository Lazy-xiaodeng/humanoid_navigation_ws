# Simulated Drift Recovery Trials

- Source samples: `debug_monitor/nav_drift_test2_scancontext_rerun_20260525/samples.csv`
- Random seed: `20260525`
- Trials: 12
- Success: 10/12 (83.3%)
- Simulated NDT accepts initialpose if XY error <= 1.00 m

This simulates the current feedback loop from offline ScanContext samples. It does not execute live NDT; use it as a repeatable bag-level risk test.

| sample | nearest wp | wp dist | result | attempts | final mode | final error | final wp | reason |
|---:|---|---:|---|---:|---|---:|---|---|
| 6 | 点位1 | 0.037 | success | 1 | conservative | 0.045 | 点位1 | conservative accepted by simulated NDT |
| 101 | 点位2 | 0.191 | fail | 5 | global | 1.082 | 点位3 | max attempts reached |
| 206 | 点位4 | 0.193 | fail | 5 | global | 17.599 | 点位23 | max attempts reached |
| 336 | 点位7 | 0.965 | success | 1 | conservative | 0.450 | 点位7 | conservative accepted by simulated NDT |
| 543 | 点位10 | 0.252 | success | 1 | conservative | 0.904 | 点位10 | conservative accepted by simulated NDT |
| 649 | 点位13 | 0.473 | success | 1 | conservative | 0.989 | 点位13 | conservative accepted by simulated NDT |
| 784 | 点位13 | 0.838 | success | 4 | global | 0.954 | 点位13 | global accepted by simulated NDT |
| 915 | 点位13 | 0.889 | success | 1 | conservative | 0.898 | 点位13 | conservative accepted by simulated NDT |
| 1117 | 点位13 | 0.919 | success | 1 | conservative | 0.903 | 点位13 | conservative accepted by simulated NDT |
| 1255 | 点位13 | 0.912 | success | 1 | conservative | 0.904 | 点位13 | conservative accepted by simulated NDT |
| 1326 | 点位13 | 0.924 | success | 1 | conservative | 0.906 | 点位13 | conservative accepted by simulated NDT |
| 1420 | 点位13 | 0.919 | success | 1 | conservative | 0.897 | 点位13 | conservative accepted by simulated NDT |
