# Protected Policy Impact Estimate

这不是 Nav2 闭环重跑；它只估算第一版保护策略下，哪些时段会冻结 map->odom 更新，以及原 bag 中这段时间机器人是否在动。

## Summary

- hold clusters: 4
- total hold duration: 24.723s
- total odom motion during holds: 4.335m
- total moving-cmd hold duration: 24.723s

## Clusters

| label | phase | start | end | dur | max dx | degraded | odom xy | cmd moving ratio | max cmd v | max cmd w |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 点位12 | nav | 1780201529.319 | 1780201533.627 | 4.308 | 0.785 | 14 | 1.704 | 1.00 | 0.500 | 0.571 |
| 点位15 | nav | 1780201763.833 | 1780201767.446 | 3.614 | 0.710 | 7 | 0.474 | 1.00 | 0.500 | 0.519 |
| 点位16 | nav | 1780201844.335 | 1780201854.149 | 9.813 | 1.553 | 46 | 1.018 | 0.99 | 0.500 | 1.000 |
| 点位17 | nav | 1780201956.250 | 1780201963.238 | 6.988 | 2.179 | 9 | 1.139 | 0.98 | 0.500 | 1.000 |
