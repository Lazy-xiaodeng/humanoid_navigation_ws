#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Route task 语义模拟器。

用法：
1. 先启动 navigation_state_manager。
2. 在同一个 ROS_DOMAIN_ID 下运行本脚本。

本脚本模拟 APP 发送 start_route_task / jump_to_waypoint / broadcast_finished，
同时模拟 TF、odom、机器人 ready 和定位 healthy。点位都放在当前位置附近，
用于验证路线任务状态机语义，不替代真实地图/控制器通行测试。
"""

import argparse
import json
import time
from typing import Callable, Dict, List

import rclpy
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


ROUTE_EVENT_TYPES = {
    "navigation_command_result",
    "route_task_started",
    "waypoint_passed",
    "jump_updated",
    "final_align_started",
    "final_align_completed",
    "broadcast_requested",
    "task_waypoint_completed",
    "route_task_completed",
    "navigation_failed",
}


class RouteTaskSemanticSim(Node):
    def __init__(self):
        super().__init__("route_task_semantic_sim")
        self.cmd_pub = self.create_publisher(String, "/navigation/requests", 10)
        self.waypoints_pub = self.create_publisher(String, "/navigation/waypoints_data", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.robot_pub = self.create_publisher(String, "/robot_status_raw", 10)
        self.loc_pub = self.create_publisher(String, "/localization/prior_map_odom_bridge_status", 10)
        self.status_sub = self.create_subscription(String, "/navigation/status", self.on_status, 100)
        # 状态管理器现在会在 through 或近距离快捷路径之后，再发 NavigateToPose 完成最终 yaw 对齐。
        # 模拟器提供一个立即成功的 Nav2 action server，只验证状态机语义，不模拟真实运动控制。
        self.nav_to_pose_server = ActionServer(self, NavigateToPose, "navigate_to_pose", self.execute_nav_to_pose)
        self.nav_through_server = ActionServer(
            self, NavigateThroughPoses, "navigate_through_poses", self.execute_nav_through
        )
        self.tf_pub = TransformBroadcaster(self)
        self.events: List[Dict] = []
        self.create_timer(0.05, self.publish_inputs)

    def execute_nav_to_pose(self, goal_handle):
        goal_handle.succeed()
        return NavigateToPose.Result()

    def execute_nav_through(self, goal_handle):
        goal_handle.succeed()
        return NavigateThroughPoses.Result()

    def publish_inputs(self):
        now = self.get_clock().now().to_msg()

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = "map"
        tf.child_frame_id = "base_footprint"
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation.w = 1.0
        self.tf_pub.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

        self.robot_pub.publish(String(data=json.dumps({
            "robot_status": "ready",
            "robot_state": "ready",
            "motion_busy": False,
        })))
        self.loc_pub.publish(String(data="ACCEPTED route_task_semantic_sim"))

    def on_status(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if payload.get("event_type") in ROUTE_EVENT_TYPES:
            self.events.append(payload)

    def reset_events(self):
        self.events = []

    def command(self, command_data: Dict):
        msg = String()
        msg.data = json.dumps({
            "request_type": "navigation_command",
            "command_data": command_data,
        }, ensure_ascii=False)
        self.cmd_pub.publish(msg)

    def publish_waypoints_cache(self, route_waypoints: List[Dict], revision: str = "sim_revision_001"):
        """模拟 dynamic_waypoints_manager 发布本地点位库。

        ID 列表模式下，navigation_state_manager 不再从 APP 消息里直接拿坐标，
        而是根据 route_waypoint_ids 到这份缓存里补全点位，所以测试必须先发布缓存。
        """
        stored_waypoints = {}
        for waypoint in route_waypoints:
            waypoint_id = str(waypoint["waypoint_id"])
            properties = {
                "waypoint_role": waypoint.get("waypoint_role", "task"),
                "need_broadcast": waypoint.get("need_broadcast", False),
                "broadcast_id": waypoint.get("broadcast_id", ""),
                "broadcast_blocking": waypoint.get("broadcast_blocking", True),
                "stop_and_align": waypoint.get("stop_and_align", waypoint.get("waypoint_role") == "task"),
                "walk_direction": waypoint.get("walk_direction", "forward"),
            }
            stored_waypoints[waypoint_id] = {
                "id": waypoint_id,
                "name": f"waypoint_{waypoint_id}",
                "type": "navigation_target",
                "frame_id": waypoint.get("frame_id", "map"),
                "position": waypoint.get("position", [0.0, 0.0, 0.0]),
                "orientation": waypoint.get("orientation", [0.0, 0.0, 0.0, 1.0]),
                "properties": properties,
            }

        msg = String()
        msg.data = json.dumps({
            "protocol_version": "2.0",
            "message_type": "push",
            "data_type": "waypoints_data",
            "source": "route_task_semantic_sim",
            "destination": "all",
            "data": {
                "update_type": "semantic_sim",
                "timestamp": time.time(),
                "waypoints_revision": revision,
                "data": {
                    "waypoints": {
                        "navigation_target": stored_waypoints,
                    }
                },
                "metadata": {
                    "total_count": len(stored_waypoints),
                    "waypoints_revision": revision,
                },
            },
            "metadata": {
                "status": "success",
                "waypoints_revision": revision,
            },
        }, ensure_ascii=False)
        self.waypoints_pub.publish(msg)


def wp(
    waypoint_id: str,
    role: str = "task",
    need_broadcast: bool = False,
    walk_direction: str = "forward"
) -> Dict:
    waypoint = {
        "waypoint_id": waypoint_id,
        "waypoint_role": role,
        "frame_id": "map",
        "position": [0.0, 0.0, 0.0],
        "orientation": [0.0, 0.0, 0.0, 1.0],
        "need_broadcast": bool(need_broadcast),
        "broadcast_id": f"broadcast_{waypoint_id}" if need_broadcast else "",
        "broadcast_blocking": bool(need_broadcast),
        "stop_and_align": role == "task",
    }
    if role == "task" and walk_direction:
        waypoint["walk_direction"] = walk_direction
    return waypoint


def event_data(event: Dict) -> Dict:
    return event.get("event_data", {})


def spin_until(node: RouteTaskSemanticSim, predicate: Callable[[], bool], timeout: float = 8.0) -> bool:
    end = time.time() + timeout
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def assert_no_navigation_failed(node: RouteTaskSemanticSim):
    failed = [event_data(e) for e in node.events if e.get("event_type") == "navigation_failed"]
    assert not failed, f"navigation_failed events present: {failed}"


def command_result_by_request(node: RouteTaskSemanticSim, request_message_id: str) -> Dict:
    for event in reversed(node.events):
        data = event_data(event)
        if (
            event.get("event_type") == "navigation_command_result" and
            data.get("request_message_id") == request_message_id
        ):
            return data
    return {}


def wait_command_result(node: RouteTaskSemanticSim, request_message_id: str, timeout: float = 4.0) -> Dict:
    assert spin_until(node, lambda: bool(command_result_by_request(node, request_message_id)), timeout), (
        f"command result not received: {request_message_id}"
    )
    return command_result_by_request(node, request_message_id)


def scenario_bidirectional_jump(node: RouteTaskSemanticSim) -> Dict:
    node.reset_events()
    session = f"semantic_session_{int(time.time() * 1000)}"
    route_id = "semantic_route"

    route = [
        wp("B", "task", True),
        wp("C", "task", False),
        wp("D", "task", False),
        wp("E", "transit", False),
        wp("F", "transit", False),
        wp("G", "task", True),
        wp("H", "task", False),
    ]

    node.command({
        "command_type": "start_route_task",
        "request_message_id": "start",
        "task_session_id": session,
        "route_id": route_id,
        "route_waypoints": route,
    })
    assert spin_until(node, lambda: any(
        e.get("event_type") == "broadcast_requested" and event_data(e).get("waypoint_id") == "B"
        for e in node.events
    )), "B broadcast not requested"

    node.command({
        "command_type": "jump_to_waypoint",
        "request_message_id": "jump_B_to_G",
        "task_session_id": session,
        "route_id": route_id,
        "target_waypoint_id": "G",
        "interrupt_broadcast": True,
    })
    assert spin_until(node, lambda: any(
        e.get("event_type") == "broadcast_requested" and event_data(e).get("waypoint_id") == "G"
        for e in node.events
    )), "G broadcast not requested after forward jump"

    node.command({
        "command_type": "jump_to_waypoint",
        "request_message_id": "jump_G_to_B",
        "task_session_id": session,
        "route_id": route_id,
        "target_waypoint_id": "B",
        "interrupt_broadcast": True,
    })
    assert spin_until(node, lambda: len([
        e for e in node.events
        if e.get("event_type") == "broadcast_requested" and event_data(e).get("waypoint_id") == "B"
    ]) >= 2), "B broadcast not requested after reverse jump"

    node.command({
        "command_type": "broadcast_finished",
        "request_message_id": "finish_B",
        "task_session_id": session,
        "route_id": route_id,
        "waypoint_id": "B",
        "broadcast_id": "broadcast_B",
        "broadcast_result": "completed",
    })
    assert spin_until(node, lambda: len([
        e for e in node.events
        if e.get("event_type") == "broadcast_requested" and event_data(e).get("waypoint_id") == "G"
    ]) >= 2), "route did not continue to G after B completion"

    node.command({
        "command_type": "broadcast_finished",
        "request_message_id": "finish_G",
        "task_session_id": session,
        "route_id": route_id,
        "waypoint_id": "G",
        "broadcast_id": "broadcast_G",
        "broadcast_result": "completed",
    })
    assert spin_until(node, lambda: any(e.get("event_type") == "route_task_completed" for e in node.events)), (
        "route did not complete"
    )

    assert_no_navigation_failed(node)
    jump_events = [event_data(e) for e in node.events if e.get("event_type") == "jump_updated"]
    completed_events = [event_data(e) for e in node.events if e.get("event_type") == "task_waypoint_completed"]
    summary = [event_data(e) for e in node.events if e.get("event_type") == "route_task_completed"][-1]
    segment_ids = [
        event_data(e).get("segment_id", "")
        for e in node.events
        if event_data(e).get("segment_id")
    ]
    compact_segment_ids = []
    for segment_id in segment_ids:
        if not compact_segment_ids or compact_segment_ids[-1] != segment_id:
            compact_segment_ids.append(segment_id)

    assert len(jump_events) >= 2, f"expected two jump_updated events, got {jump_events}"
    assert jump_events[0].get("execution_waypoint_ids") == ["E", "F", "G"], jump_events[0]
    assert jump_events[1].get("execution_waypoint_ids") == ["F", "E", "B"], jump_events[1]
    for jump_event in jump_events:
        assert jump_event.get("target_waypoint_id") not in jump_event.get("skipped_task_ids", []), jump_event
    assert set(summary.get("completed_task_ids", [])) == {"B", "C", "D", "G", "H"}, summary
    assert not (set(summary.get("completed_task_ids", [])) & set(summary.get("skipped_task_ids", []))), summary
    assert "C" not in summary.get("skipped_task_ids", []), summary
    assert "D" not in summary.get("skipped_task_ids", []), summary
    assert len(compact_segment_ids) == len(set(compact_segment_ids)), compact_segment_ids

    return {
        "jump_updated": jump_events,
        "task_waypoint_completed": completed_events,
        "route_task_completed": summary,
        "compact_segment_ids": compact_segment_ids,
        "event_count": len(node.events),
    }


def scenario_jump_to_no_broadcast_shortcut(node: RouteTaskSemanticSim) -> Dict:
    node.reset_events()
    session = f"no_broadcast_jump_session_{int(time.time() * 1000)}"
    route_id = "no_broadcast_jump_route"

    node.command({
        "command_type": "start_route_task",
        "request_message_id": "start",
        "task_session_id": session,
        "route_id": route_id,
        "route_waypoints": [wp("A", "task", True), wp("B", "task", False), wp("C", "task", False)],
    })
    assert spin_until(node, lambda: any(
        e.get("event_type") == "broadcast_requested" and event_data(e).get("waypoint_id") == "A"
        for e in node.events
    )), "A broadcast not requested"

    node.command({
        "command_type": "jump_to_waypoint",
        "request_message_id": "jump_A_to_B",
        "task_session_id": session,
        "route_id": route_id,
        "target_waypoint_id": "B",
        "interrupt_broadcast": True,
    })
    assert spin_until(node, lambda: any(e.get("event_type") == "route_task_completed" for e in node.events)), (
        "route did not complete"
    )

    assert_no_navigation_failed(node)
    command_results = [event_data(e) for e in node.events if e.get("event_type") == "navigation_command_result"]
    jump_events = [event_data(e) for e in node.events if e.get("event_type") == "jump_updated"]
    completed = [event_data(e) for e in node.events if e.get("event_type") == "task_waypoint_completed"]
    summary = [event_data(e) for e in node.events if e.get("event_type") == "route_task_completed"][-1]

    assert any(
        result.get("command_type") == "jump_to_waypoint" and result.get("status") == "success"
        for result in command_results
    ), command_results
    assert jump_events, "jump_updated missing for no-broadcast shortcut jump"
    assert jump_events[-1].get("target_waypoint_id") == "B", jump_events
    assert "B" not in jump_events[-1].get("skipped_task_ids", []), jump_events[-1]
    assert summary.get("completed_task_ids") == ["B", "C"], summary
    assert summary.get("skipped_task_ids") == ["A"], summary
    assert summary.get("summary", {}).get("completed_count") == 2, summary
    assert summary.get("summary", {}).get("skipped_count") == 1, summary

    return {
        "command_results": command_results,
        "jump_updated": jump_events,
        "task_waypoint_completed": completed,
        "route_task_completed": summary,
        "event_count": len(node.events),
    }


def scenario_transit_final_align_and_backward(node: RouteTaskSemanticSim) -> Dict:
    """验证辅助点 through 后只对最终 task 对齐，倒走 task 使用 reverse BT。"""
    node.reset_events()
    session = f"transit_backward_session_{int(time.time() * 1000)}"
    route_id = "transit_backward_route"

    route = [
        wp("10", "task", True),
        wp("11", "transit", False),
        wp("12", "transit", False),
        wp("13", "task", True),
        wp("14", "task", True, walk_direction="backward"),
    ]

    node.command({
        "command_type": "start_route_task",
        "request_message_id": "tb_start",
        "task_session_id": session,
        "route_id": route_id,
        "route_waypoints": route,
    })
    assert spin_until(node, lambda: any(
        e.get("event_type") == "broadcast_requested" and event_data(e).get("waypoint_id") == "10"
        for e in node.events
    )), "10 broadcast not requested"

    node.command({
        "command_type": "broadcast_finished",
        "request_message_id": "tb_finish_10",
        "task_session_id": session,
        "route_id": route_id,
        "waypoint_id": "10",
        "broadcast_id": "broadcast_10",
        "broadcast_result": "completed",
    })
    assert spin_until(node, lambda: any(
        e.get("event_type") == "broadcast_requested" and event_data(e).get("waypoint_id") == "13"
        for e in node.events
    )), "13 broadcast not requested"

    node.command({
        "command_type": "broadcast_finished",
        "request_message_id": "tb_finish_13",
        "task_session_id": session,
        "route_id": route_id,
        "waypoint_id": "13",
        "broadcast_id": "broadcast_13",
        "broadcast_result": "completed",
    })
    assert spin_until(node, lambda: any(
        e.get("event_type") == "broadcast_requested" and event_data(e).get("waypoint_id") == "14"
        for e in node.events
    )), "14 broadcast not requested"

    node.command({
        "command_type": "broadcast_finished",
        "request_message_id": "tb_finish_14",
        "task_session_id": session,
        "route_id": route_id,
        "waypoint_id": "14",
        "broadcast_id": "broadcast_14",
        "broadcast_result": "completed",
    })
    assert spin_until(node, lambda: any(e.get("event_type") == "route_task_completed" for e in node.events)), (
        "transit/backward route did not complete"
    )

    assert_no_navigation_failed(node)
    passed = [event_data(e) for e in node.events if e.get("event_type") == "waypoint_passed"]
    final_align_started = [event_data(e) for e in node.events if e.get("event_type") == "final_align_started"]
    broadcasts = [event_data(e) for e in node.events if e.get("event_type") == "broadcast_requested"]
    summary = [event_data(e) for e in node.events if e.get("event_type") == "route_task_completed"][-1]

    assert [event.get("waypoint_id") for event in passed] == ["11", "12"], passed
    assert [event.get("waypoint_id") for event in broadcasts] == ["10", "13", "14"], broadcasts
    assert [event.get("waypoint_id") for event in final_align_started] == ["10", "13", "14"], final_align_started
    assert not any(event.get("waypoint_id") in {"11", "12"} for event in final_align_started), final_align_started
    align_14 = [event for event in final_align_started if event.get("waypoint_id") == "14"][-1]
    assert align_14.get("walk_direction") == "backward", align_14
    assert align_14.get("behavior_tree"), align_14
    assert summary.get("completed_task_ids") == ["10", "13", "14"], summary
    assert summary.get("skipped_task_ids") == [], summary

    return {
        "waypoint_passed": passed,
        "final_align_started": final_align_started,
        "broadcast_requested": broadcasts,
        "route_task_completed": summary,
        "event_count": len(node.events),
    }


def build_route_1_to_25() -> List[Dict]:
    """构造综合路线：11/12/19-25 为辅助点，14-16 为倒走任务点。"""
    route = []
    transit_ids = {11, 12, 19, 20, 21, 22, 23, 24, 25}
    backward_ids = {14, 15, 16}
    for index in range(1, 26):
        waypoint_id = str(index)
        if index in transit_ids:
            route.append(wp(waypoint_id, "transit", False))
        else:
            route.append(
                wp(
                    waypoint_id,
                    "task",
                    True,
                    walk_direction="backward" if index in backward_ids else "forward",
                )
            )
    return route


def finish_broadcast(node: RouteTaskSemanticSim, session: str, route_id: str, waypoint_id: str, request_prefix: str):
    """完成指定 task 的播报，并等待 ROS 接受回执。"""
    node.command({
        "command_type": "broadcast_finished",
        "request_message_id": f"{request_prefix}_finish_{waypoint_id}",
        "task_session_id": session,
        "route_id": route_id,
        "waypoint_id": waypoint_id,
        "broadcast_id": f"broadcast_{waypoint_id}",
        "broadcast_result": "completed",
    })
    result = wait_command_result(node, f"{request_prefix}_finish_{waypoint_id}")
    assert result.get("status") == "success", result


def jump_to_task(node: RouteTaskSemanticSim, session: str, route_id: str, target_id: str, request_id: str):
    """发送 jump_to_waypoint，并等待 jump_updated。"""
    before_count = len([e for e in node.events if e.get("event_type") == "jump_updated"])
    node.command({
        "command_type": "jump_to_waypoint",
        "request_message_id": request_id,
        "task_session_id": session,
        "route_id": route_id,
        "target_waypoint_id": target_id,
        "interrupt_broadcast": True,
    })
    result = wait_command_result(node, request_id)
    assert result.get("status") == "success", result
    assert spin_until(node, lambda: len([
        e for e in node.events if e.get("event_type") == "jump_updated"
    ]) > before_count), f"jump_updated not received for {target_id}"
    return [event_data(e) for e in node.events if e.get("event_type") == "jump_updated"][-1]


def wait_broadcast_for(node: RouteTaskSemanticSim, waypoint_id: str, min_count: int = 1):
    assert spin_until(node, lambda: len([
        e for e in node.events
        if e.get("event_type") == "broadcast_requested" and event_data(e).get("waypoint_id") == waypoint_id
    ]) >= min_count), f"broadcast not requested for {waypoint_id}"


def scenario_route_1_to_25_mixed_segments_and_jumps(node: RouteTaskSemanticSim) -> Dict:
    """综合验证 1-25 路线的辅助点、倒走点和正反向跳点。

    关注点：
    - 11/12/19-25 只允许 waypoint_passed，不允许 final_align/broadcast；
    - 14/15/16 必须使用 backward final align；
    - 正向跳和反向跳都要按 route 数组顺序吸收区间内 transit。
    """
    node.reset_events()
    session = f"route_1_25_session_{int(time.time() * 1000)}"
    route_id = "route_1_25_mixed"

    node.command({
        "command_type": "start_route_task",
        "request_message_id": "r125_start",
        "task_session_id": session,
        "route_id": route_id,
        "route_waypoints": build_route_1_to_25(),
    })
    wait_broadcast_for(node, "1")

    # 顺序执行到 10，覆盖普通 task -> 普通 task。
    for waypoint_id in [str(i) for i in range(1, 10)]:
        finish_broadcast(node, session, route_id, waypoint_id, "r125_seq")
        wait_broadcast_for(node, str(int(waypoint_id) + 1))

    # 10 -> 11(transit) -> 12(transit) -> 13(task)，覆盖任务点经过辅助点到任务点。
    finish_broadcast(node, session, route_id, "10", "r125_seq")
    wait_broadcast_for(node, "13")

    # 13 -> 14(backward)，覆盖任务点到倒走点。
    finish_broadcast(node, session, route_id, "13", "r125_seq")
    wait_broadcast_for(node, "14")

    # 14 -> 15 -> 16 都是倒走任务点，覆盖连续倒走点。
    finish_broadcast(node, session, route_id, "14", "r125_seq")
    wait_broadcast_for(node, "15")
    finish_broadcast(node, session, route_id, "15", "r125_seq")
    wait_broadcast_for(node, "16")

    # 从倒走点 16 反向跳到 10，区间内必须反向吸收 12/11。
    # 这专门覆盖“倒走点发起，经过辅助点到任务点”的切换。
    jump_16_to_10 = jump_to_task(node, session, route_id, "10", "r125_jump_16_to_10")
    wait_broadcast_for(node, "10", min_count=2)

    # 再从 10 正向跳到 18，区间内必须正向吸收 11/12，并跳过 13-17。
    jump_10_to_18 = jump_to_task(node, session, route_id, "18", "r125_jump_10_to_18")
    wait_broadcast_for(node, "18")

    # 从 18 再反向跳到 10，验证普通 task 反向跳也会重新吸收 12/11。
    jump_18_to_10 = jump_to_task(node, session, route_id, "10", "r125_jump_18_to_10")
    wait_broadcast_for(node, "10", min_count=3)

    # 最后正向跳回 18，作为完成路线前的收尾目标。
    jump_10_to_18_again = jump_to_task(node, session, route_id, "18", "r125_jump_10_to_18_again")
    wait_broadcast_for(node, "18", min_count=2)

    # 从 18 跳到 23 是非法目标，因为 23 是 transit，必须拒绝。
    node.command({
        "command_type": "jump_to_waypoint",
        "request_message_id": "r125_jump_to_transit_23",
        "task_session_id": session,
        "route_id": route_id,
        "target_waypoint_id": "23",
        "interrupt_broadcast": True,
    })
    jump_to_transit = wait_command_result(node, "r125_jump_to_transit_23")
    assert jump_to_transit.get("status") == "error", jump_to_transit
    assert jump_to_transit.get("error_code") == "target_waypoint_not_task", jump_to_transit

    # 18 是最后一个 task，播报完成后 19-25 只是尾部 transit，不应再生成新 task。
    finish_broadcast(node, session, route_id, "18", "r125_seq")
    assert spin_until(node, lambda: any(e.get("event_type") == "route_task_completed" for e in node.events)), (
        "route 1-25 did not complete"
    )

    assert_no_navigation_failed(node)
    passed = [event_data(e) for e in node.events if e.get("event_type") == "waypoint_passed"]
    align_started = [event_data(e) for e in node.events if e.get("event_type") == "final_align_started"]
    broadcasts = [event_data(e) for e in node.events if e.get("event_type") == "broadcast_requested"]
    summary = [event_data(e) for e in node.events if e.get("event_type") == "route_task_completed"][-1]

    transit_ids = {"11", "12", "19", "20", "21", "22", "23", "24", "25"}
    align_ids = [item.get("waypoint_id") for item in align_started]
    broadcast_ids = [item.get("waypoint_id") for item in broadcasts]
    assert not any(waypoint_id in transit_ids for waypoint_id in align_ids), align_started
    assert not any(waypoint_id in transit_ids for waypoint_id in broadcast_ids), broadcasts

    backward_aligns = [
        item for item in align_started
        if item.get("waypoint_id") in {"14", "15", "16"}
    ]
    assert backward_aligns, align_started
    for item in backward_aligns:
        assert item.get("walk_direction") == "backward", item
        assert item.get("behavior_tree"), item

    assert jump_16_to_10.get("execution_waypoint_ids") == ["12", "11", "10"], jump_16_to_10
    assert jump_16_to_10.get("segment_direction") == "backward", jump_16_to_10
    assert jump_18_to_10.get("execution_waypoint_ids") == ["12", "11", "10"], jump_18_to_10
    assert jump_18_to_10.get("segment_direction") == "backward", jump_18_to_10
    assert jump_10_to_18.get("execution_waypoint_ids") == ["11", "12", "18"], jump_10_to_18
    assert jump_10_to_18.get("segment_direction") == "forward", jump_10_to_18
    assert jump_10_to_18_again.get("execution_waypoint_ids") == ["11", "12", "18"], jump_10_to_18_again
    assert jump_10_to_18_again.get("segment_direction") == "forward", jump_10_to_18_again

    passed_ids = [item.get("waypoint_id") for item in passed]
    assert "11" in passed_ids and "12" in passed_ids, passed_ids
    assert summary.get("completed_waypoint_id") == "18", summary
    assert "18" in summary.get("completed_task_ids", []), summary

    return {
        "jump_16_to_10": jump_16_to_10,
        "jump_10_to_18": jump_10_to_18,
        "jump_18_to_10": jump_18_to_10,
        "jump_10_to_18_again": jump_10_to_18_again,
        "jump_to_transit_result": jump_to_transit,
        "passed_ids": passed_ids,
        "final_align_ids": align_ids,
        "broadcast_ids": broadcast_ids,
        "route_task_completed": summary,
        "event_count": len(node.events),
    }


def scenario_protocol_errors(node: RouteTaskSemanticSim) -> Dict:
    """验证错误命令不会误推进 route task 状态。"""
    node.reset_events()
    session = f"protocol_error_session_{int(time.time() * 1000)}"
    route_id = "protocol_error_route"
    route = [
        wp("A", "task", True),
        wp("T1", "transit", False),
        wp("B", "task", True),
        wp("C", "task", False),
    ]

    node.command({
        "command_type": "start_route_task",
        "request_message_id": "err_start",
        "task_session_id": session,
        "route_id": route_id,
        "route_waypoints": route,
    })
    assert spin_until(node, lambda: any(
        e.get("event_type") == "broadcast_requested" and event_data(e).get("waypoint_id") == "A"
        for e in node.events
    )), "A broadcast not requested"

    node.command({
        "command_type": "start_route_task",
        "request_message_id": "err_duplicate_start",
        "task_session_id": f"{session}_duplicate",
        "route_id": route_id,
        "route_waypoints": route,
    })
    duplicate_start_result = wait_command_result(node, "err_duplicate_start")
    assert duplicate_start_result.get("status") == "error", duplicate_start_result
    assert duplicate_start_result.get("error_code") == "route_task_already_running", duplicate_start_result

    node.command({
        "command_type": "jump_to_waypoint",
        "request_message_id": "same_target_A",
        "task_session_id": session,
        "route_id": route_id,
        "target_waypoint_id": "A",
        "interrupt_broadcast": True,
    })
    same_target_result = wait_command_result(node, "same_target_A")
    assert same_target_result.get("status") == "success", same_target_result
    assert same_target_result.get("result_reason") == "already_current_target", same_target_result

    node.command({
        "command_type": "jump_to_waypoint",
        "request_message_id": "err_jump_bad_session",
        "task_session_id": f"{session}_bad",
        "route_id": route_id,
        "target_waypoint_id": "B",
        "interrupt_broadcast": True,
    })
    bad_session_jump_result = wait_command_result(node, "err_jump_bad_session")
    assert bad_session_jump_result.get("status") == "error", bad_session_jump_result
    assert bad_session_jump_result.get("error_code") == "invalid_task_session", bad_session_jump_result

    node.command({
        "command_type": "jump_to_waypoint",
        "request_message_id": "err_jump_bad_route",
        "task_session_id": session,
        "route_id": f"{route_id}_bad",
        "target_waypoint_id": "B",
        "interrupt_broadcast": True,
    })
    bad_route_jump_result = wait_command_result(node, "err_jump_bad_route")
    assert bad_route_jump_result.get("status") == "error", bad_route_jump_result
    assert bad_route_jump_result.get("error_code") == "invalid_route_id", bad_route_jump_result

    node.command({
        "command_type": "jump_to_waypoint",
        "request_message_id": "err_jump_missing_route",
        "task_session_id": session,
        "target_waypoint_id": "B",
        "interrupt_broadcast": True,
    })
    missing_route_jump_result = wait_command_result(node, "err_jump_missing_route")
    assert missing_route_jump_result.get("status") == "error", missing_route_jump_result
    assert missing_route_jump_result.get("error_code") == "invalid_route_id", missing_route_jump_result
    assert missing_route_jump_result.get("route_id") == "", missing_route_jump_result

    node.command({
        "command_type": "broadcast_finished",
        "request_message_id": "err_broadcast_bad_session",
        "task_session_id": f"{session}_bad",
        "route_id": route_id,
        "waypoint_id": "A",
        "broadcast_id": "broadcast_A",
        "broadcast_result": "completed",
    })
    bad_session_broadcast_result = wait_command_result(node, "err_broadcast_bad_session")
    assert bad_session_broadcast_result.get("status") == "error", bad_session_broadcast_result
    assert bad_session_broadcast_result.get("error_code") == "invalid_task_session", bad_session_broadcast_result

    node.command({
        "command_type": "broadcast_finished",
        "request_message_id": "err_broadcast_bad_route",
        "task_session_id": session,
        "route_id": f"{route_id}_bad",
        "waypoint_id": "A",
        "broadcast_id": "broadcast_A",
        "broadcast_result": "completed",
    })
    bad_route_broadcast_result = wait_command_result(node, "err_broadcast_bad_route")
    assert bad_route_broadcast_result.get("status") == "error", bad_route_broadcast_result
    assert bad_route_broadcast_result.get("error_code") == "invalid_route_id", bad_route_broadcast_result

    node.command({
        "command_type": "broadcast_finished",
        "request_message_id": "err_broadcast_missing_route",
        "task_session_id": session,
        "waypoint_id": "A",
        "broadcast_id": "broadcast_A",
        "broadcast_result": "completed",
    })
    missing_route_broadcast_result = wait_command_result(node, "err_broadcast_missing_route")
    assert missing_route_broadcast_result.get("status") == "error", missing_route_broadcast_result
    assert missing_route_broadcast_result.get("error_code") == "invalid_route_id", missing_route_broadcast_result
    assert missing_route_broadcast_result.get("route_id") == "", missing_route_broadcast_result

    node.command({
        "command_type": "jump_to_waypoint",
        "request_message_id": "err_jump_transit",
        "task_session_id": session,
        "route_id": route_id,
        "target_waypoint_id": "T1",
        "interrupt_broadcast": True,
    })
    transit_result = wait_command_result(node, "err_jump_transit")
    assert transit_result.get("status") == "error", transit_result
    assert transit_result.get("error_code") == "target_waypoint_not_task", transit_result

    node.command({
        "command_type": "jump_to_waypoint",
        "request_message_id": "err_jump_no_interrupt",
        "task_session_id": session,
        "route_id": route_id,
        "target_waypoint_id": "B",
        "interrupt_broadcast": "false",
    })
    no_interrupt_result = wait_command_result(node, "err_jump_no_interrupt")
    assert no_interrupt_result.get("status") == "error", no_interrupt_result
    assert no_interrupt_result.get("error_code") == "interrupt_broadcast_false_not_supported", no_interrupt_result

    # 上面这些错误/同目标请求不应推进当前 A 的等待播报态。
    assert not any(e.get("event_type") == "jump_updated" for e in node.events), node.events
    assert len([
        e for e in node.events
        if e.get("event_type") == "broadcast_requested" and event_data(e).get("waypoint_id") == "A"
    ]) == 1, node.events

    node.command({
        "command_type": "broadcast_finished",
        "request_message_id": "err_wrong_broadcast",
        "task_session_id": session,
        "route_id": route_id,
        "waypoint_id": "B",
        "broadcast_id": "broadcast_B",
        "broadcast_result": "completed",
    })
    wrong_broadcast_result = wait_command_result(node, "err_wrong_broadcast")
    assert wrong_broadcast_result.get("status") == "error", wrong_broadcast_result
    assert wrong_broadcast_result.get("error_code") == "broadcast_context_mismatch", wrong_broadcast_result

    node.command({
        "command_type": "broadcast_finished",
        "request_message_id": "err_unsupported_result",
        "task_session_id": session,
        "route_id": route_id,
        "waypoint_id": "A",
        "broadcast_id": "broadcast_A",
        "broadcast_result": "failed",
    })
    unsupported_result = wait_command_result(node, "err_unsupported_result")
    assert unsupported_result.get("status") == "error", unsupported_result
    assert unsupported_result.get("error_code") == "unsupported_broadcast_result", unsupported_result

    node.command({
        "command_type": "broadcast_finished",
        "request_message_id": "finish_A",
        "task_session_id": session,
        "route_id": route_id,
        "waypoint_id": "A",
        "broadcast_id": "broadcast_A",
        "broadcast_result": "completed",
    })
    finish_a_result = wait_command_result(node, "finish_A")
    assert finish_a_result.get("status") == "success", finish_a_result
    assert spin_until(node, lambda: any(
        e.get("event_type") == "broadcast_requested" and event_data(e).get("waypoint_id") == "B"
        for e in node.events
    )), "B broadcast not requested after valid A finish"

    node.command({
        "command_type": "broadcast_finished",
        "request_message_id": "duplicate_A",
        "task_session_id": session,
        "route_id": route_id,
        "waypoint_id": "A",
        "broadcast_id": "broadcast_A",
        "broadcast_result": "completed",
    })
    duplicate_result = wait_command_result(node, "duplicate_A")
    assert duplicate_result.get("status") == "success", duplicate_result
    assert duplicate_result.get("result_reason") == "duplicate_broadcast_finished", duplicate_result

    # duplicate A 不应完成 B，也不应结束路线。
    assert not any(
        e.get("event_type") == "task_waypoint_completed" and event_data(e).get("waypoint_id") == "B"
        for e in node.events
    ), node.events
    assert not any(e.get("event_type") == "route_task_completed" for e in node.events), node.events

    node.command({
        "command_type": "broadcast_finished",
        "request_message_id": "finish_B",
        "task_session_id": session,
        "route_id": route_id,
        "waypoint_id": "B",
        "broadcast_id": "broadcast_B",
    })
    finish_b_result = wait_command_result(node, "finish_B")
    assert finish_b_result.get("status") == "success", finish_b_result
    assert spin_until(node, lambda: any(e.get("event_type") == "route_task_completed" for e in node.events)), (
        "route did not complete after valid B finish"
    )

    assert_no_navigation_failed(node)
    command_results = [event_data(e) for e in node.events if e.get("event_type") == "navigation_command_result"]
    completed = [event_data(e) for e in node.events if e.get("event_type") == "task_waypoint_completed"]
    summary = [event_data(e) for e in node.events if e.get("event_type") == "route_task_completed"][-1]
    assert summary.get("completed_task_ids") == ["A", "B", "C"], summary
    assert summary.get("skipped_task_ids") == [], summary

    return {
        "command_results": command_results,
        "task_waypoint_completed": completed,
        "route_task_completed": summary,
        "event_count": len(node.events),
    }


def scenario_start_validation(node: RouteTaskSemanticSim) -> Dict:
    """验证未启动状态和 start_route_task 坏包不会创建残留 route task。"""
    node.reset_events()
    route_id = "start_validation_route"

    node.command({
        "command_type": "jump_to_waypoint",
        "request_message_id": "not_running_jump",
        "task_session_id": "missing_session",
        "route_id": route_id,
        "target_waypoint_id": "A",
        "interrupt_broadcast": True,
    })
    not_running_jump = wait_command_result(node, "not_running_jump")
    assert not_running_jump.get("status") == "error", not_running_jump
    assert not_running_jump.get("error_code") == "route_task_not_running", not_running_jump

    node.command({
        "command_type": "broadcast_finished",
        "request_message_id": "not_running_broadcast",
        "task_session_id": "missing_session",
        "route_id": route_id,
        "waypoint_id": "A",
        "broadcast_id": "broadcast_A",
        "broadcast_result": "completed",
    })
    not_running_broadcast = wait_command_result(node, "not_running_broadcast")
    assert not_running_broadcast.get("status") == "error", not_running_broadcast
    assert not_running_broadcast.get("error_code") == "route_task_not_running", not_running_broadcast

    bad_starts = [
        (
            "missing_session_start",
            {
                "command_type": "start_route_task",
                "request_message_id": "missing_session_start",
                "route_id": route_id,
                "route_waypoints": [wp("A", "task", False)],
            },
            "missing_task_session_id",
        ),
        (
            "missing_route_start",
            {
                "command_type": "start_route_task",
                "request_message_id": "missing_route_start",
                "task_session_id": "start_validation_session",
                "route_waypoints": [wp("A", "task", False)],
            },
            "missing_route_id",
        ),
        (
            "bad_waypoints_type_start",
            {
                "command_type": "start_route_task",
                "request_message_id": "bad_waypoints_type_start",
                "task_session_id": "start_validation_session",
                "route_id": route_id,
                "route_waypoints": "not-an-array",
            },
            "invalid_route_waypoints",
        ),
        (
            "bad_waypoint_role_start",
            {
                "command_type": "start_route_task",
                "request_message_id": "bad_waypoint_role_start",
                "task_session_id": "start_validation_session",
                "route_id": route_id,
                "route_waypoints": [wp("A", "bad_role", False)],
            },
            "invalid_waypoint_role",
        ),
        (
            "duplicate_task_id_start",
            {
                "command_type": "start_route_task",
                "request_message_id": "duplicate_task_id_start",
                "task_session_id": "start_validation_session",
                "route_id": route_id,
                "route_waypoints": [wp("A", "task", False), wp("A", "task", False)],
            },
            "duplicate_waypoint_id",
        ),
        (
            "duplicate_task_transit_id_start",
            {
                "command_type": "start_route_task",
                "request_message_id": "duplicate_task_transit_id_start",
                "task_session_id": "start_validation_session",
                "route_id": route_id,
                "route_waypoints": [wp("A", "task", False), wp("A", "transit", False)],
            },
            "duplicate_waypoint_id",
        ),
        (
            "missing_broadcast_id_start",
            {
                "command_type": "start_route_task",
                "request_message_id": "missing_broadcast_id_start",
                "task_session_id": "start_validation_session",
                "route_id": route_id,
                "route_waypoints": [{
                    **wp("A", "task", True),
                    "broadcast_id": "",
                }],
            },
            "missing_broadcast_id",
        ),
        (
            "missing_task_waypoints_start",
            {
                "command_type": "start_route_task",
                "request_message_id": "missing_task_waypoints_start",
                "task_session_id": "start_validation_session",
                "route_id": route_id,
                "route_waypoints": [wp("T1", "transit", False)],
            },
            "missing_task_waypoints",
        ),
        (
            "missing_pose_start",
            {
                "command_type": "start_route_task",
                "request_message_id": "missing_pose_start",
                "task_session_id": "start_validation_session",
                "route_id": route_id,
                "route_waypoints": [{
                    "waypoint_id": "A",
                    "waypoint_role": "task",
                    "frame_id": "map",
                    "orientation": [0.0, 0.0, 0.0, 1.0],
                }],
            },
            "missing_waypoint_pose",
        ),
        (
            "zero_orientation_start",
            {
                "command_type": "start_route_task",
                "request_message_id": "zero_orientation_start",
                "task_session_id": "start_validation_session",
                "route_id": route_id,
                "route_waypoints": [{
                    **wp("A", "task", False),
                    "orientation": [0.0, 0.0, 0.0, 0.0],
                }],
            },
            "missing_waypoint_pose",
        ),
    ]

    for request_id, command, expected_error_code in bad_starts:
        node.command(command)
        result = wait_command_result(node, request_id)
        assert result.get("status") == "error", result
        assert result.get("error_code") == expected_error_code, result

    assert not any(e.get("event_type") == "broadcast_requested" for e in node.events), node.events
    assert not any(e.get("event_type") == "task_waypoint_completed" for e in node.events), node.events
    assert not any(e.get("event_type") == "route_task_completed" for e in node.events), node.events
    assert_no_navigation_failed(node)

    session = f"start_validation_session_{int(time.time() * 1000)}"
    node.command({
        "command_type": "start_route_task",
        "request_message_id": "valid_start_after_errors",
        "task_session_id": session,
        "route_id": route_id,
        "route_waypoints": [wp("A", "task", False)],
    })
    valid_start = wait_command_result(node, "valid_start_after_errors")
    assert valid_start.get("status") == "success", valid_start
    assert spin_until(node, lambda: any(e.get("event_type") == "route_task_completed" for e in node.events)), (
        "valid route did not complete after bad starts"
    )
    summary = [event_data(e) for e in node.events if e.get("event_type") == "route_task_completed"][-1]
    assert summary.get("completed_task_ids") == ["A"], summary
    assert summary.get("skipped_task_ids") == [], summary

    return {
        "command_results": [
            event_data(e)
            for e in node.events
            if e.get("event_type") == "navigation_command_result"
        ],
        "route_task_completed": summary,
        "event_count": len(node.events),
    }


def scenario_id_waypoint_start_revision(node: RouteTaskSemanticSim) -> Dict:
    """验证 route_waypoint_ids 启动、revision 校验和错误码。"""
    node.reset_events()
    revision = f"sim_revision_{int(time.time() * 1000)}"
    route_id = "id_waypoint_route"
    route = [
        wp("1", "task", False),
        wp("2", "transit", False),
        wp("3", "task", False),
    ]

    node.publish_waypoints_cache(route, revision)
    assert spin_until(node, lambda: False, 0.3) is False

    node.command({
        "command_type": "start_route_task",
        "request_message_id": "id_start_success",
        "task_session_id": "id_start_success_session",
        "route_id": route_id,
        "waypoints_revision": revision,
        "route_waypoint_ids": ["1", "2", "3"],
    })
    id_start_success = wait_command_result(node, "id_start_success")
    assert id_start_success.get("status") == "success", id_start_success
    assert spin_until(node, lambda: any(e.get("event_type") == "route_task_completed" for e in node.events)), (
        "ID waypoint route did not complete"
    )
    success_summary = [event_data(e) for e in node.events if e.get("event_type") == "route_task_completed"][-1]
    assert success_summary.get("completed_task_ids") == ["1", "3"], success_summary

    bad_commands = [
        (
            "id_start_missing_revision",
            {
                "command_type": "start_route_task",
                "request_message_id": "id_start_missing_revision",
                "task_session_id": "id_start_missing_revision_session",
                "route_id": route_id,
                "route_waypoint_ids": ["1", "2", "3"],
            },
            "missing_waypoints_revision",
        ),
        (
            "id_start_revision_mismatch",
            {
                "command_type": "start_route_task",
                "request_message_id": "id_start_revision_mismatch",
                "task_session_id": "id_start_revision_mismatch_session",
                "route_id": route_id,
                "waypoints_revision": "wrong_revision",
                "route_waypoint_ids": ["1", "2", "3"],
            },
            "waypoints_revision_mismatch",
        ),
        (
            "id_start_missing_id",
            {
                "command_type": "start_route_task",
                "request_message_id": "id_start_missing_id",
                "task_session_id": "id_start_missing_id_session",
                "route_id": route_id,
                "waypoints_revision": revision,
                "route_waypoint_ids": ["1", "404"],
            },
            "waypoint_id_not_found",
        ),
        (
            "id_start_empty_ids",
            {
                "command_type": "start_route_task",
                "request_message_id": "id_start_empty_ids",
                "task_session_id": "id_start_empty_ids_session",
                "route_id": route_id,
                "waypoints_revision": revision,
                "route_waypoint_ids": [],
            },
            "invalid_route_waypoint_ids",
        ),
        (
            "id_start_ambiguous_source",
            {
                "command_type": "start_route_task",
                "request_message_id": "id_start_ambiguous_source",
                "task_session_id": "id_start_ambiguous_source_session",
                "route_id": route_id,
                "waypoints_revision": revision,
                "route_waypoint_ids": ["1", "2", "3"],
                "route_waypoints": route,
            },
            "ambiguous_route_waypoint_source",
        ),
    ]

    for request_id, command, expected_error_code in bad_commands:
        node.command(command)
        result = wait_command_result(node, request_id)
        assert result.get("status") == "error", result
        assert result.get("error_code") == expected_error_code, result

    assert_no_navigation_failed(node)
    return {
        "id_start_success": id_start_success,
        "route_task_completed": success_summary,
        "command_results": [
            event_data(e)
            for e in node.events
            if e.get("event_type") == "navigation_command_result"
        ],
        "event_count": len(node.events),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scenario",
        choices=(
            "all",
            "bidirectional_jump",
            "jump_to_no_broadcast_shortcut",
            "transit_final_align_and_backward",
            "route_1_to_25_mixed_segments_and_jumps",
            "protocol_errors",
            "start_validation",
            "id_waypoint_start_revision",
        ),
        default="all",
    )
    args = parser.parse_args()

    rclpy.init()
    node = RouteTaskSemanticSim()
    try:
        assert spin_until(node, lambda: node.cmd_pub.get_subscription_count() > 0, 5.0), (
            "navigation_state_manager not subscribed to /navigation/requests"
        )
        # 先让 navigation_state_manager 消费几帧 TF/odom/localization_status。
        # 否则 route 起点虽然在当前位置，但 current_pose 还没建立，会误走 Nav2 action server 分支。
        spin_until(node, lambda: False, 0.5)
        scenarios = []
        if args.scenario in ("all", "bidirectional_jump"):
            scenarios.append(("bidirectional_jump", scenario_bidirectional_jump))
        if args.scenario in ("all", "jump_to_no_broadcast_shortcut"):
            scenarios.append(("jump_to_no_broadcast_shortcut", scenario_jump_to_no_broadcast_shortcut))
        if args.scenario in ("all", "transit_final_align_and_backward"):
            scenarios.append(("transit_final_align_and_backward", scenario_transit_final_align_and_backward))
        if args.scenario in ("all", "route_1_to_25_mixed_segments_and_jumps"):
            scenarios.append(("route_1_to_25_mixed_segments_and_jumps", scenario_route_1_to_25_mixed_segments_and_jumps))
        if args.scenario in ("all", "protocol_errors"):
            scenarios.append(("protocol_errors", scenario_protocol_errors))
        if args.scenario in ("all", "start_validation"):
            scenarios.append(("start_validation", scenario_start_validation))
        if args.scenario in ("all", "id_waypoint_start_revision"):
            scenarios.append(("id_waypoint_start_revision", scenario_id_waypoint_start_revision))

        results = {}
        for name, scenario in scenarios:
            results[name] = scenario(node)

        print(json.dumps({"status": "pass", "results": results}, ensure_ascii=False, indent=2))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
