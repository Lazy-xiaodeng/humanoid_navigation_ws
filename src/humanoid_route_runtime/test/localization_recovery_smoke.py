#!/usr/bin/env python3
"""验证冷启动定位门控，以及行走中定位丢失后的停车恢复闭环。"""

import argparse
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


def waypoint(waypoint_id: str, role: str, x: float) -> dict:
    return {
        "waypoint_id": waypoint_id,
        "waypoint_role": role,
        "frame_id": "map",
        "map_id": "hall",
        "position": [x, 0.0, 0.0],
        "orientation": [0.0, 0.0, 0.0, 1.0],
        "need_broadcast": False,
        "broadcast_blocking": False,
        "stop_and_align": role == "task",
    }


class LocalizationRecoverySmoke(Node):
    def __init__(self, all_transits_passed: bool, integration_mode: str = "enforce"):
        super().__init__("localization_recovery_smoke")
        self.all_transits_passed = all_transits_passed
        self.integration_mode = integration_mode
        self.command_pub = self.create_publisher(String, "/navigation/requests", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.robot_pub = self.create_publisher(String, "/robot_status_raw", 10)
        self.localization_pub = self.create_publisher(String, "/localization/trust_status", 10)
        self.map_pub = self.create_publisher(String, "/map/status", 10)
        self.create_subscription(String, "/navigation/status", self.on_status, 100)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 100)

        self.events: list[dict] = []
        self.phase = "startup_required"
        self.healthy_publish_count = 0
        self.healthy_count_at_resume = 0
        self.ro_verification_epoch = 0
        self.zero_cmd_count = 0
        self.through_goal_count = 0
        self.through_cancel_count = 0
        self.through_goal_xs: list[list[float]] = []
        self.final_goal_count = 0
        self.cold_start_premature_goal = False
        self.create_timer(0.05, self.publish_inputs)

        self.through_server = ActionServer(
            self,
            NavigateThroughPoses,
            "navigate_through_poses",
            execute_callback=self.execute_through,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=self.cancel_through,
        )
        self.final_server = ActionServer(
            self,
            NavigateToPose,
            "navigate_to_pose",
            execute_callback=self.execute_final,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )

    def localization_payload(self) -> dict:
        if self.phase == "transient_large_jump":
            return {
                "state": "bridge_holding_last_good_tf",
                "reason": "HOLD large_jump age=0.10s reason=test",
                "pose_initialized": True,
                "pose_trusted": False,
                "can_start_navigation": False,
                "localization_recovery_required": False,
                "recovery_requires_global_relocalization": False,
            }
        if self.phase == "transient_ro_lost":
            return {
                "state": "ro_verifying",
                "reason": "robosense_unhealthy_observing LOST",
                "pose_initialized": True,
                "pose_trusted": False,
                "can_start_navigation": False,
                "localization_recovery_required": False,
                "recovery_requires_global_relocalization": False,
            }
        if self.phase == "startup_required":
            return {
                "state": "startup_requires_global_relocalization",
                "reason": "origin_seed_without_global_confirmation",
                "pose_initialized": True,
                "pose_trusted": False,
                "can_start_navigation": False,
                "localization_recovery_required": True,
                "recovery_requires_global_relocalization": False,
            }
        if self.phase == "global_relocalizing":
            return {
                "state": "global_relocalizing",
                "reason": "global_relocalization_running",
                "pose_initialized": True,
                "pose_trusted": False,
                "can_start_navigation": False,
                "localization_recovery_required": True,
                "recovery_requires_global_relocalization": False,
            }
        if self.phase == "ro_verifying":
            return {
                "state": "ro_verifying",
                "reason": "ro_normal_match_frames=4/5",
                "pose_initialized": True,
                "pose_trusted": False,
                "can_start_navigation": False,
                "localization_recovery_required": False,
                "recovery_requires_global_relocalization": False,
            }
        if self.phase == "recovery":
            return {
                "state": "recovery_requires_global_relocalization",
                "reason": "large_jump_hold_seen_without_global_confirmation",
                "pose_initialized": True,
                "pose_trusted": False,
                "can_start_navigation": False,
                "localization_recovery_required": True,
                "recovery_requires_global_relocalization": True,
            }
        if self.phase == "recovery_healthy":
            self.healthy_publish_count += 1
        return {
            "state": "trusted_ro",
            "reason": "ro_normal_match_frames=5",
            "pose_initialized": True,
            "pose_trusted": True,
            "can_start_navigation": True,
            "localization_recovery_required": False,
            "recovery_requires_global_relocalization": False,
        }

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
        localization = self.localization_payload()
        if localization.get("pose_trusted") and localization.get("can_start_navigation"):
            self.ro_verification_epoch += 1
        localization["ro_verification_epoch"] = self.ro_verification_epoch
        localization["integration_mode"] = self.integration_mode
        self.localization_pub.publish(String(data=json.dumps(localization)))
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
            if payload.get("event_type") == "navigation_resumed" and self.healthy_count_at_resume == 0:
                self.healthy_count_at_resume = self.healthy_publish_count

    def on_cmd_vel(self, msg: Twist):
        values = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]
        if all(abs(value) < 1e-9 for value in values):
            self.zero_cmd_count += 1

    def execute_through(self, goal_handle):
        self.through_goal_count += 1
        self.through_goal_xs.append([
            pose.pose.position.x for pose in goal_handle.request.poses
        ])
        if self.through_goal_count >= 2:
            goal_handle.succeed()
            result = NavigateThroughPoses.Result()
            result.error_code = 0
            return result
        feedback = NavigateThroughPoses.Feedback()
        feedback.number_of_poses_remaining = 1 if self.all_transits_passed else 2
        goal_handle.publish_feedback(feedback)
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = NavigateThroughPoses.Result()
                result.error_code = 0
                return result
            if not goal_handle.is_active:
                break
            time.sleep(0.05)
        result = NavigateThroughPoses.Result()
        result.error_code = 0
        return result

    def cancel_through(self, _):
        self.through_cancel_count += 1
        return CancelResponse.ACCEPT

    def execute_final(self, goal_handle):
        self.final_goal_count += 1
        goal_handle.succeed()
        return NavigateToPose.Result()

    def start_route(self, request_suffix: str):
        data = {
            "command_type": "start_route_task",
            "request_message_id": f"localization-recovery-smoke-{request_suffix}",
            "task_session_id": "session-localization-recovery-smoke",
            "route_id": "route-localization-recovery-smoke",
            "map_id": "hall",
            "route_waypoints": [
                waypoint("a0", "transit", 0.2),
                waypoint("a1", "transit", 0.4),
                waypoint("a2", "transit", 0.6),
                waypoint("t1", "task", 0.8),
            ],
        }
        self.command_pub.publish(String(data=json.dumps({
            "request_type": "navigation_command",
            "command_data": data,
        })))

    def stop_pending_route(self):
        data = {
            "command_type": "stop_route_task",
            "request_message_id": "localization-recovery-smoke-stop-pending",
            "task_session_id": "session-localization-recovery-smoke",
            "route_id": "route-localization-recovery-smoke",
            "reason": "upper_layer_cancelled_while_waiting_localization",
        }
        self.command_pub.publish(String(data=json.dumps({
            "request_type": "navigation_command",
            "command_data": data,
        })))


def has_event(events: list[dict], event_type: str, field: str = "", value=None) -> bool:
    for event in events:
        if event.get("event_type") != event_type:
            continue
        if not field or event.get("event_data", {}).get(field) == value:
            return True
    return False


def event_count(events: list[dict], event_type: str) -> int:
    return sum(1 for event in events if event.get("event_type") == event_type)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--all-transits-passed", action="store_true")
    parser.add_argument("--stop-during-recovery", action="store_true")
    args = parser.parse_args()
    rclpy.init()
    node = LocalizationRecoverySmoke(args.all_transits_passed)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        deadline = time.time() + 18.0
        sent_cancelled_start = False
        cancelled_start_time = 0.0
        sent_stop = False
        sent_auto_start = False
        auto_start_time = 0.0
        pending_cancelled = False
        entered_recovery = False
        entered_healthy = False
        sent_recovery_stop = False
        recovery_stop_time = 0.0
        while time.time() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if not sent_cancelled_start and node.command_pub.get_subscription_count() > 0:
                settle = time.time() + 0.8
                while time.time() < settle:
                    executor.spin_once(timeout_sec=0.05)
                node.start_route("cancelled-pending")
                sent_cancelled_start = True
                cancelled_start_time = time.time()
            pending_seen = has_event(node.events, "navigation_pending")
            pending_cancelled = has_event(node.events, "navigation_pending_cancelled")
            if sent_cancelled_start and pending_seen and not sent_stop:
                if time.time() - cancelled_start_time >= 0.5:
                    node.stop_pending_route()
                    sent_stop = True
            if sent_stop and pending_cancelled and not sent_auto_start:
                node.start_route("automatic-after-trust")
                sent_auto_start = True
                auto_start_time = time.time()
            if (
                sent_auto_start
                and not entered_recovery
                and node.phase != "trusted"
                and node.through_goal_count > 0
            ):
                node.cold_start_premature_goal = True
            if sent_auto_start and node.through_goal_count == 0:
                elapsed = time.time() - auto_start_time
                if elapsed >= 0.5 and node.phase == "startup_required":
                    node.phase = "global_relocalizing"
                if elapsed >= 1.0 and node.phase == "global_relocalizing":
                    node.phase = "ro_verifying"
                if elapsed >= 1.5 and node.phase == "ro_verifying":
                    node.phase = "trusted"
            if (
                sent_auto_start
                and not entered_recovery
                and node.through_goal_count >= 1
                and event_count(node.events, "waypoint_passed") >= 2
            ):
                node.phase = "recovery"
                entered_recovery = True
            paused = has_event(node.events, "navigation_paused", "pause_source", "localization_recovery")
            if paused and node.through_cancel_count >= 1 and not entered_healthy:
                if args.stop_during_recovery:
                    node.stop_pending_route()
                    sent_recovery_stop = True
                    recovery_stop_time = time.time()
                node.phase = "recovery_healthy"
                entered_healthy = True
            resumed = has_event(node.events, "navigation_resumed", "resume_source", "localization_recovery")
            recovery_stopped = has_event(node.events, "navigation_stopped")
            if (
                args.stop_during_recovery
                and sent_recovery_stop
                and recovery_stopped
                and time.time() - recovery_stop_time >= 1.0
            ):
                break
            recovery_goal_started = (
                node.final_goal_count >= 1
                if args.all_transits_passed
                else node.through_goal_count >= 2
            )
            if resumed and recovery_goal_started:
                break

        settle = time.time() + 2.5
        while time.time() < settle:
            executor.spin_once(timeout_sec=0.05)

        paused = has_event(node.events, "navigation_paused", "pause_source", "localization_recovery")
        resumed = has_event(node.events, "navigation_resumed", "resume_source", "localization_recovery")
        recovery_stopped = has_event(node.events, "navigation_stopped")
        print("GOALS", node.through_goal_count)
        print("CANCELS", node.through_cancel_count)
        print("ZERO_CMDS", node.zero_cmd_count)
        print("THROUGH_GOAL_XS", node.through_goal_xs)
        print("FINAL_GOALS", node.final_goal_count)
        print("PENDING_CANCELLED", pending_cancelled)
        print("COLD_START_PREMATURE_GOAL", node.cold_start_premature_goal)
        print("PAUSED", paused)
        print("RESUMED", resumed)
        print("RECOVERY_STOPPED", recovery_stopped)
        print("HEALTHY_COUNT_AT_RESUME", node.healthy_count_at_resume)
        if not pending_cancelled or node.cold_start_premature_goal:
            raise SystemExit(1)
        if not paused or node.through_cancel_count < 1 or node.zero_cmd_count < 1:
            raise SystemExit(2)
        if args.stop_during_recovery:
            if not recovery_stopped or resumed or node.through_goal_count != 1 or node.final_goal_count != 0:
                raise SystemExit(3)
        else:
            if not resumed:
                raise SystemExit(3)
            if node.healthy_count_at_resume < 3:
                raise SystemExit(4)
        if node.through_goal_xs[0] != [0.2, 0.4, 0.6, 0.8]:
            raise SystemExit(6)
        if args.stop_during_recovery:
            pass
        elif args.all_transits_passed:
            if len(node.through_goal_xs) != 1 or node.final_goal_count < 1:
                raise SystemExit(7)
        else:
            if len(node.through_goal_xs) < 2 or node.through_goal_xs[1] != [0.6, 0.8]:
                raise SystemExit(8)
        print("LOCALIZATION_COLD_START_AND_RECOVERY_SMOKE_PASS")
    finally:
        executor.shutdown(timeout_sec=2.0)
        node.through_server.destroy()
        node.final_server.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
