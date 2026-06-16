# 前端Mock示例全集

## 1. 文档目的

本文档用于给前端提供一组可以直接照着落成 mock JSON 文件的示例全集，方便：

1. 页面原型开发
2. 状态机联调前演示
3. 按钮可用规则验证
4. 双状态页面切换验证

说明：

1. 这里的示例以“整页状态快照”思路组织。
2. 每个场景都尽量对应一个可落盘的 JSON 文件。
3. 字段结构尽量贴近《控制台前端状态管理设计》和《控制台与ROS联调JSON消息清单》。

## 2. 建议文件列表

建议前端项目 `mock/` 目录至少放这些文件：

1. `idle.json`
2. `running.json`
3. `broadcast_wait.json`
4. `manual_paused.json`
5. `obstacle_paused.json`
6. `manual_escape_wait_resume.json`
7. `localization_recovering.json`
8. `map_switching.json`
9. `failed.json`
10. `completed.json`

## 3. `idle.json`

适用场景：

1. 页面初始待命
2. 机器人在线但未开始任务

```json
{
  "session": {
    "userId": "u001",
    "userName": "操作员A",
    "role": "operator",
    "selectedRobotId": "XR-102",
    "selectedRobotName": "小贝机器人 XR-102"
  },
  "connection": {
    "wsConnected": true,
    "wsConnecting": false,
    "lastHeartbeatAt": 1781395200000,
    "backendReachable": true,
    "robotOnline": true,
    "latencyMs": 28,
    "lastError": null
  },
  "robot": {
    "batteryPercent": 78,
    "linearSpeed": 0.0,
    "angularSpeed": 0.0,
    "headingDeg": 135.6,
    "pose": {
      "x": 23.45,
      "y": 16.87,
      "yawDeg": 135.6
    },
    "health": {
      "navModule": "ok",
      "localizationModule": "ok",
      "obstacleModule": "ok",
      "powerModule": "ok",
      "commModule": "ok"
    }
  },
  "navigation": {
    "mainState": "idle",
    "reasonCode": "",
    "reasonText": "",
    "startedAt": null,
    "pausedAt": null,
    "resumedAt": null,
    "finishedAt": null,
    "canStart": true,
    "canPause": false,
    "canResume": false,
    "canStop": false,
    "canJump": false
  },
  "routeTask": {
    "routeTaskId": null,
    "routeName": null,
    "currentWaypointId": null,
    "nextWaypointId": null,
    "jumpTargetWaypointId": null,
    "currentWaypointName": null,
    "nextWaypointName": null,
    "jumpTargetWaypointName": null,
    "sequence": [],
    "progressPercent": null
  },
  "map": {
    "currentMapId": "hall_a_floor_1",
    "currentMapName": "A栋一层导览图",
    "availableMaps": [
      {
        "mapId": "hall_a_floor_1",
        "mapName": "A栋一层导览图"
      },
      {
        "mapId": "hall_b_floor_1",
        "mapName": "B栋一层实验室图"
      }
    ],
    "switching": false,
    "switchTargetMapId": null,
    "switchTargetMapName": null,
    "mode": "2d",
    "followRobot": true,
    "zoomLevel": 1
  },
  "broadcast": {
    "waiting": false,
    "currentBroadcastId": null,
    "currentBroadcastText": null,
    "waitingSince": null,
    "lastFinishedAt": null
  },
  "obstacle": {
    "blocked": false,
    "blockedSince": null,
    "reason": null,
    "snapshotUrl": null,
    "guidanceText": null
  },
  "recording": {
    "recording": false,
    "startedAt": null,
    "savePath": null,
    "fileName": null,
    "lastError": null
  },
  "logs": {
    "items": [],
    "maxSize": 300
  },
  "ui": {
    "jumpDialogOpen": false,
    "waypointEditorOpen": false,
    "mapSwitchDialogOpen": false,
    "selectedWaypointId": null,
    "routePanelExpanded": true,
    "logPanelExpanded": true
  }
}
```

## 4. `running.json`

适用场景：

1. 正常运行态页面
2. 双状态效果图上半部分主要参考

```json
{
  "session": {
    "userId": "u001",
    "userName": "操作员A",
    "role": "operator",
    "selectedRobotId": "XR-102",
    "selectedRobotName": "小贝机器人 XR-102"
  },
  "connection": {
    "wsConnected": true,
    "wsConnecting": false,
    "lastHeartbeatAt": 1781395205000,
    "backendReachable": true,
    "robotOnline": true,
    "latencyMs": 28,
    "lastError": null
  },
  "robot": {
    "batteryPercent": 78,
    "linearSpeed": 0.42,
    "angularSpeed": 0.12,
    "headingDeg": 135.6,
    "pose": {
      "x": 23.45,
      "y": 16.87,
      "yawDeg": 135.6
    },
    "health": {
      "navModule": "ok",
      "localizationModule": "ok",
      "obstacleModule": "ok",
      "powerModule": "ok",
      "commModule": "ok"
    }
  },
  "navigation": {
    "mainState": "running",
    "reasonCode": "",
    "reasonText": "",
    "startedAt": 1781395100000,
    "pausedAt": null,
    "resumedAt": null,
    "finishedAt": null,
    "canStart": false,
    "canPause": true,
    "canResume": false,
    "canStop": true,
    "canJump": true
  },
  "routeTask": {
    "routeTaskId": "NAV-20240614-001",
    "routeName": "展厅上午导览",
    "currentWaypointId": "13",
    "nextWaypointId": "14",
    "jumpTargetWaypointId": null,
    "currentWaypointName": "任务点13",
    "nextWaypointName": "辅助点14",
    "jumpTargetWaypointName": null,
    "sequence": [
      {
        "waypointId": "10",
        "waypointName": "任务点10",
        "role": "task",
        "walkDirection": "forward",
        "needBroadcast": true,
        "status": "completed"
      },
      {
        "waypointId": "11",
        "waypointName": "辅助点11",
        "role": "transit",
        "walkDirection": "forward",
        "needBroadcast": false,
        "status": "completed"
      },
      {
        "waypointId": "12",
        "waypointName": "辅助点12",
        "role": "transit",
        "walkDirection": "forward",
        "needBroadcast": false,
        "status": "completed"
      },
      {
        "waypointId": "13",
        "waypointName": "任务点13",
        "role": "task",
        "walkDirection": "forward",
        "needBroadcast": true,
        "status": "running"
      },
      {
        "waypointId": "14",
        "waypointName": "辅助点14",
        "role": "transit",
        "walkDirection": "forward",
        "needBroadcast": false,
        "status": "pending"
      },
      {
        "waypointId": "16",
        "waypointName": "任务点16",
        "role": "task",
        "walkDirection": "forward",
        "needBroadcast": true,
        "status": "pending"
      }
    ],
    "progressPercent": 62.5
  },
  "map": {
    "currentMapId": "hall_a_floor_1",
    "currentMapName": "A栋一层导览图",
    "availableMaps": [
      {
        "mapId": "hall_a_floor_1",
        "mapName": "A栋一层导览图"
      },
      {
        "mapId": "hall_b_floor_1",
        "mapName": "B栋一层实验室图"
      }
    ],
    "switching": false,
    "switchTargetMapId": null,
    "switchTargetMapName": null,
    "mode": "2d",
    "followRobot": true,
    "zoomLevel": 1
  },
  "broadcast": {
    "waiting": false,
    "currentBroadcastId": null,
    "currentBroadcastText": null,
    "waitingSince": null,
    "lastFinishedAt": 1781395095000
  },
  "obstacle": {
    "blocked": false,
    "blockedSince": null,
    "reason": null,
    "snapshotUrl": null,
    "guidanceText": null
  },
  "recording": {
    "recording": false,
    "startedAt": null,
    "savePath": null,
    "fileName": null,
    "lastError": null
  },
  "logs": {
    "items": [
      {
        "id": "log_001",
        "timestamp": 1781395190000,
        "level": "success",
        "category": "nav",
        "title": "路线任务已启动",
        "detail": "开始执行展厅上午导览"
      },
      {
        "id": "log_002",
        "timestamp": 1781395200000,
        "level": "info",
        "category": "nav",
        "title": "已到达任务点13",
        "detail": "已切换至等待播报前状态"
      }
    ],
    "maxSize": 300
  },
  "ui": {
    "jumpDialogOpen": false,
    "waypointEditorOpen": false,
    "mapSwitchDialogOpen": false,
    "selectedWaypointId": "13",
    "routePanelExpanded": true,
    "logPanelExpanded": true
  }
}
```

## 5. `broadcast_wait.json`

适用场景：

1. 到达任务点后等待 APP 播报完成
2. 非异常态，但流程被阻塞

建议在 `running.json` 基础上改以下字段：

1. `navigation.mainState = "broadcast_wait"`
2. `navigation.reasonCode = "waiting_broadcast_finished"`
3. `navigation.reasonText = "等待 APP 播报完成"`
4. `navigation.canPause = true`
5. `navigation.canJump = false`
6. `broadcast.waiting = true`
7. `broadcast.currentBroadcastId = "broadcast_13"`
8. `broadcast.currentBroadcastText = "欢迎来到展厅13号点"`
9. `broadcast.waitingSince = 1781395210400`

## 6. `manual_paused.json`

适用场景：

1. 用户点击暂停后的页面状态

建议在 `running.json` 基础上改以下字段：

1. `navigation.mainState = "manual_paused"`
2. `navigation.reasonCode = "manual_pause"`
3. `navigation.reasonText = "用户手动暂停导航"`
4. `navigation.canPause = false`
5. `navigation.canResume = true`
6. `navigation.canJump = false`
7. `robot.linearSpeed = 0.0`
8. `robot.angularSpeed = 0.0`

## 7. `obstacle_paused.json`

适用场景：

1. 动态障碍物触发自动暂停
2. 双状态效果图下半部分主要参考

建议在 `running.json` 基础上改以下字段：

1. `navigation.mainState = "obstacle_paused"`
2. `navigation.reasonCode = "dynamic_obstacle_blocked"`
3. `navigation.reasonText = "检测到前方障碍物，导航已自动暂停"`
4. `navigation.canPause = false`
5. `navigation.canResume = true`
6. `navigation.canJump = false`
7. `robot.linearSpeed = 0.0`
8. `robot.angularSpeed = 0.0`
9. `robot.batteryPercent = 61`
10. `robot.pose.x = 21.34`
11. `robot.pose.y = 16.26`
12. `robot.pose.yawDeg = 132.1`
13. `robot.health.localizationModule = "warn"`
14. `obstacle.blocked = true`
15. `obstacle.reason = "dynamic_obstacle"`
16. `obstacle.blockedSince = 1781395210000`
17. `obstacle.guidanceText = "请确认前方障碍是否已移除，必要时人工辅助脱困"`

## 8. `manual_escape_wait_resume.json`

适用场景：

1. 机器人脱困后，等待人工点击继续

建议在 `obstacle_paused.json` 基础上改以下字段：

1. `navigation.mainState = "manual_escape_wait_resume"`
2. `navigation.reasonCode = "manual_escape_wait_resume"`
3. `navigation.reasonText = "人工脱困完成，等待点击继续"`
4. `obstacle.reason = "manual_escape"`
5. `obstacle.guidanceText = "请确认机器人周边无碰撞风险后，再点击继续"`

## 9. `localization_recovering.json`

适用场景：

1. 定位置信度下降
2. 切图后正在恢复定位

建议在 `running.json` 基础上改以下字段：

1. `navigation.mainState = "localization_recovering"`
2. `navigation.reasonCode = "localization_recovering"`
3. `navigation.reasonText = "定位置信度下降，正在恢复中"`
4. `navigation.canPause = false`
5. `navigation.canResume = false`
6. `navigation.canJump = false`
7. `robot.linearSpeed = 0.0`
8. `robot.angularSpeed = 0.0`
9. `robot.health.localizationModule = "warn"`
10. `robot.headingDeg = 132.1`

## 10. `map_switching.json`

适用场景：

1. 用户发起切图后
2. 页面进入“切图中、等待定位恢复”的状态

建议在 `idle.json` 基础上改以下字段：

1. `navigation.mainState = "map_switching"`
2. `navigation.reasonCode = "map_switching"`
3. `navigation.reasonText = "正在切换地图并重启定位链路"`
4. `navigation.canStart = false`
5. `map.switching = true`
6. `map.switchTargetMapId = "hall_b_floor_1"`
7. `map.switchTargetMapName = "B栋一层实验室图"`

## 11. `failed.json`

适用场景：

1. 导航执行失败
2. 页面进入错误态并等待人工处理

建议在 `running.json` 基础上改以下字段：

1. `navigation.mainState = "failed"`
2. `navigation.reasonCode = "navigation_failed"`
3. `navigation.reasonText = "导航失败，请检查环境、定位或底层导航模块"`
4. `navigation.canPause = false`
5. `navigation.canResume = false`
6. `navigation.canStop = true`
7. `navigation.canJump = false`
8. `robot.linearSpeed = 0.0`
9. `robot.angularSpeed = 0.0`
10. `robot.health.navModule = "error"`

## 12. `completed.json`

适用场景：

1. 路线任务已全部完成

建议在 `running.json` 基础上改以下字段：

1. `navigation.mainState = "completed"`
2. `navigation.reasonCode = "route_task_completed"`
3. `navigation.reasonText = "路线任务已完成"`
4. `navigation.canStart = true`
5. `navigation.canPause = false`
6. `navigation.canResume = false`
7. `navigation.canStop = false`
8. `navigation.canJump = false`
9. `routeTask.progressPercent = 100`
10. `routeTask.sequence` 中全部点改为 `completed`

## 13. 使用建议

前端使用这些 mock 时，建议：

1. 做一个开发态场景切换器
2. 支持场景一键切换
3. 支持自动轮播场景
4. 支持从 `idle -> running -> broadcast_wait -> running -> completed` 的连续流程演示

## 14. 推荐补充

如果后续需要更丝滑地给前端落地，我还建议再补：

1. 一份 `mock/` 目录下的真实 JSON 文件
2. 一份 `TypeScript 类型定义示例`

这样前端就可以直接复制进项目里用。
