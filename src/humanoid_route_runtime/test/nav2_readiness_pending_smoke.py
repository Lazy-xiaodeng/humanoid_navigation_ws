#!/usr/bin/env python3
"""验证 Nav2 就绪门控、缓存路线释放和最终对齐目标拒绝重试。"""

import json
import time

import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


class Nav2ReadinessPendingSmoke(Node):
    def __init__(self):
        super().__init__("nav2_readiness_pending_smoke")
        self.command_pub = self.create_publisher(String, "/navigation/requests", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.robot_pub = self.create_publisher(String, "/robot_status_raw", 10)
        self.localization_pub = self.create_publisher(String, "/localization/trust_status", 10)
        self.recovery_status_pub = self.create_publisher(String, "/localization/recovery_status", 10)
        self.map_pub = self.create_publisher(String, "/map/status", 10)
        self.create_subscription(String, "/navigation/status", self.on_status, 100)

        self.events: list[dict] = []
        self.lifecycle_active = False
        self.final_goal_attempts = 0
        self.final_goal_accepts = 0
        self.reject_final_attempts_remaining = 2
        self.drop_lifecycle_during_next_final = False
        self.lifecycle_drop_duration = 2.0
        self.restore_lifecycle_after_drop = False
        self.create_timer(0.05, self.publish_inputs)

        self.lifecycle_services = [
            self.create_service(GetState, f"/{name}/get_state", self.get_lifecycle_state)
            for name in ("planner_server", "controller_server", "behavior_server", "bt_navigator")
        ]
        self.through_server = ActionServer(
            self,
            NavigateThroughPoses,
            "navigate_through_poses",
            execute_callback=self.execute_through,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        self.final_server = ActionServer(
            self,
            NavigateToPose,
            "navigate_to_pose",
            execute_callback=self.execute_final,
            goal_callback=self.handle_final_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )

    def get_lifecycle_state(self, _, response):
        if self.lifecycle_active:
            response.current_state.id = State.PRIMARY_STATE_ACTIVE
            response.current_state.label = "active"
        else:
            response.current_state.id = State.PRIMARY_STATE_INACTIVE
            response.current_state.label = "inactive"
        return response

    def publish_inputs(self):
        odom = Odometry()
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)
        self.robot_pub.publish(String(data=json.dumps({
            "values": {
                "robot_status": "Walk",
                "motion_busy": False,
                "control_ready_for_navigation": True,
            }
        })))
        self.localization_pub.publish(String(data=json.dumps({
            "state": "trusted_ro",
            "reason": "ro_normal_match_frames=5",
            "pose_initialized": True,
            "pose_trusted": True,
            "can_start_navigation": True,
            "localization_recovery_required": False,
            "recovery_requires_global_relocalization": False,
            "ro_verification_epoch": 10,
            "integration_mode": "enforce",
        })))
        self.map_pub.publish(String(data=json.dumps({
            "data": {
                "current_map_id": "hall",
                "map_state": "ready",
                "localization_state": "stable",
            }
        })))

    def on_status(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if payload.get("event_type"):
            self.events.append(payload)

    def handle_final_goal(self, _):
        self.final_goal_attempts += 1
        if self.reject_final_attempts_remaining > 0:
            self.reject_final_attempts_remaining -= 1
            return GoalResponse.REJECT
        self.final_goal_accepts += 1
        return GoalResponse.ACCEPT

    def execute_final(self, goal_handle):
        if self.drop_lifecycle_during_next_final:
            self.drop_lifecycle_during_next_final = False
            self.lifecycle_active = False
            time.sleep(self.lifecycle_drop_duration)
            if self.restore_lifecycle_after_drop:
                self.lifecycle_active = True
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToPose.Result()
        goal_handle.succeed()
        return NavigateToPose.Result()

    def execute_through(self, goal_handle):
        goal_handle.succeed()
        return NavigateThroughPoses.Result()

    def start_route(self, suffix: str):
        waypoint = {
            "waypoint_id": f"task-{suffix}",
            "waypoint_role": "task",
            "frame_id": "map",
            "map_id": "hall",
            "position": [0.0, 0.0, 0.0],
            "orientation": [0.0, 0.0, 0.0, 1.0],
            "need_broadcast": False,
            "broadcast_blocking": False,
            "stop_and_align": True,
        }
        command = {
            "command_type": "start_route_task",
            "request_message_id": f"nav2-ready-start-{suffix}",
            "task_session_id": f"session-{suffix}",
            "route_id": f"route-{suffix}",
            "map_id": "hall",
            "route_waypoints": [waypoint],
        }
        self.publish_command(command)

    def start_two_task_route(self, suffix: str):
        def task(waypoint_id: str, x: float) -> dict:
            return {
                "waypoint_id": waypoint_id,
                "waypoint_role": "task",
                "frame_id": "map",
                "map_id": "hall",
                "position": [x, 0.0, 0.0],
                "orientation": [0.0, 0.0, 0.0, 1.0],
                "need_broadcast": False,
                "broadcast_blocking": False,
                "stop_and_align": True,
            }
        self.publish_command({
            "command_type": "start_route_task",
            "request_message_id": f"nav2-ready-start-{suffix}",
            "task_session_id": f"session-{suffix}",
            "route_id": f"route-{suffix}",
            "map_id": "hall",
            "route_waypoints": [task(f"task-{suffix}-1", 0.0), task(f"task-{suffix}-2", 2.0)],
        })

    def stop_route(self, suffix: str):
        self.publish_command({
            "command_type": "stop_route_task",
            "request_message_id": f"nav2-ready-stop-{suffix}",
            "task_session_id": f"session-{suffix}",
            "route_id": f"route-{suffix}",
            "reason": "upper_layer_cancelled_while_waiting_nav2",
        })

    def publish_command(self, command: dict):
        self.command_pub.publish(String(data=json.dumps({
            "request_type": "navigation_command",
            "command_data": command,
        })))


def count_event(events: list[dict], event_type: str) -> int:
    return sum(event.get("event_type") == event_type for event in events)


def spin_until(executor, predicate, timeout: float) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return False


def main():
    rclpy.init()
    node = Nav2ReadinessPendingSmoke()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        if not spin_until(executor, lambda: node.command_pub.get_subscription_count() > 0, 5.0):
            raise RuntimeError("路线运行层未订阅导航请求")
        spin_until(executor, lambda: False, 0.8)

        node.start_route("release")
        if not spin_until(executor, lambda: count_event(node.events, "navigation_pending") >= 1, 3.0):
            raise RuntimeError("Nav2 未就绪时路线没有进入 pending")
        spin_until(executor, lambda: False, 1.0)
        if node.final_goal_attempts != 0:
            raise RuntimeError("Nav2 未 ACTIVE 时错误地下发了目标")

        node.lifecycle_active = True
        completed = spin_until(
            executor,
            lambda: count_event(node.events, "route_task_completed") >= 1,
            8.0,
        )
        if not completed:
            raise RuntimeError("Nav2 就绪后缓存路线未完成")
        if node.final_goal_attempts != 3 or node.final_goal_accepts != 1:
            raise RuntimeError(
                f"最终对齐拒绝重试次数异常: attempts={node.final_goal_attempts}, "
                f"accepts={node.final_goal_accepts}"
            )
        if count_event(node.events, "navigation_failed") != 0:
            raise RuntimeError("短暂拒绝被错误地判定为导航失败")

        node.lifecycle_active = False
        if not spin_until(
            executor,
            lambda: any(
                not event.get("event_data", {}).get("nav2_ready", True)
                for event in node.events
                if event.get("event_type") == "navigation_status"
            ),
            2.0,
        ):
            spin_until(executor, lambda: False, 0.6)
        goals_before_cancel_test = node.final_goal_attempts
        pending_before = count_event(node.events, "navigation_pending")
        node.start_route("cancel")
        if not spin_until(
            executor,
            lambda: count_event(node.events, "navigation_pending") > pending_before,
            3.0,
        ):
            raise RuntimeError("第二条路线没有进入 Nav2 pending")
        node.recovery_status_pub.publish(String(data=json.dumps({
            "event_type": "localization_relocalize_failed",
            "result_code": "attempts_exhausted",
            "reason": "global_relocalization_timeout",
            "attempt_id": "test-exhausted-attempt",
            "attempt_number": 3,
        })))
        if not spin_until(
            executor,
            lambda: count_event(node.events, "navigation_relocalization_failed") >= 1,
            2.0,
        ):
            raise RuntimeError("搜索耗尽没有转发为导航恢复失败事件")
        exhausted_event = next(
            event for event in node.events
            if event.get("event_type") == "navigation_relocalization_failed"
        )
        exhausted_data = exhausted_event.get("event_data", {})
        if exhausted_data.get("task_session_id") != "session-cancel" or \
                exhausted_data.get("route_id") != "route-cancel":
            raise RuntimeError("搜索耗尽事件没有携带缓存路线关联字段")
        node.stop_route("cancel")
        if not spin_until(
            executor,
            lambda: count_event(node.events, "navigation_pending_cancelled") >= 1,
            3.0,
        ):
            raise RuntimeError("等待 Nav2 时 stop 未取消缓存路线")

        node.lifecycle_active = True
        spin_until(executor, lambda: False, 2.0)
        if node.final_goal_attempts != goals_before_cancel_test:
            raise RuntimeError("已终止的缓存路线在 Nav2 就绪后被错误释放")

        transient_completed_before = count_event(node.events, "route_task_completed")
        transient_deferred_before = count_event(node.events, "navigation_resume_deferred")
        transient_goals_before = node.final_goal_attempts
        node.lifecycle_drop_duration = 0.7
        node.restore_lifecycle_after_drop = True
        node.drop_lifecycle_during_next_final = True
        node.start_route("transient-nav2-drop")
        if not spin_until(
            executor,
            lambda: count_event(node.events, "route_task_completed") > transient_completed_before,
            4.0,
        ):
            raise RuntimeError("短暂 Nav2 readiness 抖动下路线没有正常完成")
        if count_event(node.events, "navigation_resume_deferred") != transient_deferred_before:
            raise RuntimeError("0.7s Nav2 readiness 抖动错误触发了停车缓存")
        if node.final_goal_attempts != transient_goals_before + 1:
            raise RuntimeError("短暂 Nav2 readiness 抖动造成目标重复下发")

        completed_before_dispatch_test = count_event(node.events, "route_task_completed")
        deferred_before = count_event(node.events, "navigation_resume_deferred")
        goals_before_dispatch_test = node.final_goal_attempts
        node.lifecycle_drop_duration = 2.0
        node.restore_lifecycle_after_drop = False
        node.drop_lifecycle_during_next_final = True
        node.start_two_task_route("active-dispatch")
        if not spin_until(
            executor,
            lambda: count_event(node.events, "navigation_resume_deferred") > deferred_before,
            5.0,
        ):
            raise RuntimeError("活动路线下一段没有在 Nav2 掉线后进入派发等待")
        spin_until(executor, lambda: False, 0.8)
        if node.final_goal_attempts != goals_before_dispatch_test + 1:
            raise RuntimeError("Nav2 未恢复时错误地下发了下一路线段")

        node.lifecycle_active = True
        if not spin_until(
            executor,
            lambda: count_event(node.events, "route_task_completed") > completed_before_dispatch_test,
            6.0,
        ):
            raise RuntimeError("Nav2 恢复后活动路线没有续发完成")
        if node.final_goal_attempts != goals_before_dispatch_test + 3:
            raise RuntimeError("活动路线恢复后没有只重发被取消段并正常推进下一段")

        print("PENDING_EVENTS", count_event(node.events, "navigation_pending"))
        print("FINAL_GOAL_ATTEMPTS", node.final_goal_attempts)
        print("FINAL_GOAL_ACCEPTS", node.final_goal_accepts)
        print("ROUTE_COMPLETED", count_event(node.events, "route_task_completed"))
        print("PENDING_CANCELLED", count_event(node.events, "navigation_pending_cancelled"))
        print("RELOCALIZATION_EXHAUSTED", count_event(node.events, "navigation_relocalization_failed"))
        print("ACTIVE_DISPATCH_DEFERRED", count_event(node.events, "navigation_resume_deferred"))
        print("NAV2_READINESS_PENDING_SMOKE_PASS")
    finally:
        executor.shutdown(timeout_sec=2.0)
        node.through_server.destroy()
        node.final_server.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
