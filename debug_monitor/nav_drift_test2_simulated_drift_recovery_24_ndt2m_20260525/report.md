# Simulated Drift Recovery Trials

- Source samples: `debug_monitor/nav_drift_test2_scancontext_rerun_20260525/samples.csv`
- Random seed: `20260526`
- Trials: 18
- Success: 15/18 (83.3%)
- Simulated NDT accepts initialpose if XY error <= 2.00 m

This simulates the current feedback loop from offline ScanContext samples. It does not execute live NDT; use it as a repeatable bag-level risk test.

| sample | nearest wp | wp dist | result | attempts | final mode | final error | final wp | reason |
|---:|---|---:|---|---:|---|---:|---|---|
| 0 | 点位1 | 0.036 | success | 1 | conservative | 0.045 | 点位1 | conservative accepted by simulated NDT |
| 115 | 点位2 | 0.514 | success | 1 | conservative | 0.065 | 点位2 | conservative accepted by simulated NDT |
| 181 | 点位4 | 0.251 | fail | 5 | global | 17.552 | 点位24 | max attempts reached |
| 290 | 点位6 | 0.485 | fail | 5 | global | 12.711 | 点位8 | max attempts reached |
| 354 | 点位7 | 0.513 | success | 4 | global | 0.442 | 点位7 | global accepted by simulated NDT |
| 473 | 点位9 | 0.202 | fail | 5 | global | 2.214 | 点位9 | max attempts reached |
| 545 | 点位10 | 0.255 | success | 1 | conservative | 0.901 | 点位10 | conservative accepted by simulated NDT |
| 611 | 点位13 | 0.459 | success | 1 | conservative | 0.944 | 点位13 | conservative accepted by simulated NDT |
| 710 | 点位13 | 0.450 | success | 1 | conservative | 0.960 | 点位13 | conservative accepted by simulated NDT |
| 773 | 点位13 | 0.850 | success | 4 | global | 1.071 | 点位13 | global accepted by simulated NDT |
| 842 | 点位13 | 0.859 | success | 4 | global | 1.055 | 点位13 | global accepted by simulated NDT |
| 925 | 点位13 | 0.892 | success | 1 | conservative | 0.902 | 点位13 | conservative accepted by simulated NDT |
| 993 | 点位13 | 0.887 | success | 1 | conservative | 0.896 | 点位13 | conservative accepted by simulated NDT |
| 1105 | 点位13 | 0.923 | success | 1 | conservative | 0.899 | 点位13 | conservative accepted by simulated NDT |
| 1206 | 点位13 | 0.916 | success | 1 | conservative | 0.902 | 点位13 | conservative accepted by simulated NDT |
| 1269 | 点位13 | 0.929 | success | 1 | conservative | 0.999 | 点位14 | conservative accepted by simulated NDT |
| 1331 | 点位13 | 0.916 | success | 1 | conservative | 0.898 | 点位13 | conservative accepted by simulated NDT |
| 1394 | 点位13 | 0.919 | success | 1 | conservative | 0.896 | 点位13 | conservative accepted by simulated NDT |
