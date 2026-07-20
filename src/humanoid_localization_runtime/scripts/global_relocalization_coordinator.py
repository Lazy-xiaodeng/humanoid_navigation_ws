#!/usr/bin/env python3
"""按需组织全局重定位，并守护“全局候选 -> RO 精修 -> bridge”交接。"""

import json
import math
import os
import time
import uuid
from typing import Any, Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def parse_object(data: str) -> Optional[Dict[str, Any]]:
    try:
        value = json.loads(data)
    except json.JSONDecodeError:
        return None
    return value if isinstance(value, dict) else None


def nested_data(payload: Dict[str, Any]) -> Dict[str, Any]:
    value = payload.get("data")
    return value if isinstance(value, dict) else payload


def stamp_seconds(msg: PoseStamped) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


VALID_INTEGRATION_MODES = ("off", "shadow", "enforce")


def normalize_integration_mode(value: Any) -> str:
    mode = str(value).strip().lower()
    return mode if mode in VALID_INTEGRATION_MODES else "off"


class GlobalRelocalizationCoordinator(Node):
    """独占管理一次恢复 attempt，只应用与当前地图和时间戳关联的结果。"""

    def __init__(self):
        super().__init__("global_relocalization_coordinator")
        self.trust_topic = self.declare_parameter(
            "trust_status_topic", "/localization/trust_status"
        ).value
        self.map_topic = self.declare_parameter("map_status_topic", "/map/status").value
        self.global_request_topic = self.declare_parameter(
            "global_request_topic", "/global_relocalization/request"
        ).value
        self.global_status_topic = self.declare_parameter(
            "global_status_topic", "/global_relocalization/recovery_status"
        ).value
        self.recovery_pose_topic = self.declare_parameter(
            "recovery_pose_topic", "/global_relocalization/recovery_pose"
        ).value
        self.recovery_map_odom_topic = self.declare_parameter(
            "recovery_map_odom_topic", "/global_relocalization/recovery_map_to_odom"
        ).value
        self.bridge_status_topic = self.declare_parameter(
            "bridge_status_topic", "/localization/prior_map_odom_bridge_status"
        ).value
        self.robosense_status_topic = self.declare_parameter(
            "robosense_status_topic", "/prior_localization/robosense_status"
        ).value
        self.recovery_status_topic = self.declare_parameter(
            "recovery_status_topic", "/localization/recovery_status"
        ).value
        self.navigation_status_topic = self.declare_parameter(
            "navigation_status_topic", "/navigation/status"
        ).value
        self.bridge_apply_topic = self.declare_parameter(
            "bridge_apply_topic", "/localization/global_recovery_map_to_odom"
        ).value
        self.robosense_apply_topic = self.declare_parameter(
            "robosense_apply_topic", "/prior_localization/global_relocalization_pose"
        ).value
        self.robosense_refined_map_odom_topic = self.declare_parameter(
            "robosense_refined_map_odom_topic",
            "/prior_localization/global_relocalization_refined_map_to_odom",
        ).value

        requested_mode = self.declare_parameter("integration_mode", "shadow").value
        self.integration_mode = normalize_integration_mode(requested_mode)
        if self.integration_mode != str(requested_mode).strip().lower():
            self.get_logger().error(
                f"invalid integration_mode={requested_mode!r}; falling back to off"
            )
        self.auto_trigger = bool(self.declare_parameter("auto_trigger", True).value)
        # Deprecated compatibility parameters. Side-effect authority is owned only
        # by integration_mode so conflicting boolean combinations cannot go live.
        self.declare_parameter("dry_run", True)
        self.declare_parameter("auto_apply", False)
        self.dry_run = self.integration_mode != "enforce"
        self.auto_apply = self.integration_mode == "enforce"
        self.attempt_timeout_sec = max(
            1.0, float(self.declare_parameter("attempt_timeout_sec", 45.0).value)
        )
        self.apply_timeout_sec = max(
            1.0, float(self.declare_parameter("apply_timeout_sec", 5.0).value)
        )
        self.max_attempts = max(1, int(self.declare_parameter("max_attempts", 3).value))
        self.retry_backoff_sec = max(
            0.0, float(self.declare_parameter("retry_backoff_sec", 2.0).value)
        )
        self.exhausted_retry_cooldown_sec = max(
            0.0,
            float(self.declare_parameter("exhausted_retry_cooldown_sec", 30.0).value),
        )
        self.candidate_stamp_tolerance_sec = max(
            0.001,
            float(self.declare_parameter("candidate_stamp_tolerance_sec", 0.03).value),
        )
        self.candidate_max_receipt_age_sec = max(
            0.1, float(self.declare_parameter("candidate_max_receipt_age_sec", 2.0).value)
        )
        self.map_assets_json = str(self.declare_parameter("map_assets_json", "{}").value)
        self.map_registry_path = str(
            self.declare_parameter(
                "map_registry_path",
                "/home/ubuntu/software/Todesk/Files/humanoid_ws/data/maps/map_registry.json",
            ).value
        )
        try:
            parsed_assets = json.loads(self.map_assets_json)
            self.map_assets = parsed_assets if isinstance(parsed_assets, dict) else {}
        except json.JSONDecodeError:
            self.map_assets = {}
            self.get_logger().error("map_assets_json is invalid; map-specific assets disabled")
        self.load_map_registry_assets()

        latched_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.request_pub = self.create_publisher(String, self.global_request_topic, 10)
        self.status_pub = self.create_publisher(String, self.recovery_status_topic, 10)
        self.bridge_apply_pub = self.create_publisher(PoseStamped, self.bridge_apply_topic, 10)
        self.ro_apply_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.robosense_apply_topic, 10
        )
        self.create_subscription(String, self.trust_topic, self.on_trust, 20)
        self.create_subscription(String, self.map_topic, self.on_map_status, 20)
        self.create_subscription(String, self.global_status_topic, self.on_global_status, latched_qos)
        self.create_subscription(PoseStamped, self.recovery_pose_topic, self.on_recovery_pose, latched_qos)
        self.create_subscription(
            PoseStamped, self.recovery_map_odom_topic, self.on_recovery_map_odom, latched_qos
        )
        self.create_subscription(String, self.bridge_status_topic, self.on_bridge_status, 20)
        self.create_subscription(String, self.robosense_status_topic, self.on_ro_status, 20)
        self.create_subscription(
            PoseStamped,
            self.robosense_refined_map_odom_topic,
            self.on_ro_refined_map_odom,
            20,
        )
        self.create_subscription(String, self.navigation_status_topic, self.on_navigation_status, 20)
        self.create_timer(0.2, self.on_timer)

        self.state = "idle"
        self.attempt_id = ""
        self.recovery_type = ""
        self.attempt_number = 0
        self.attempt_started = 0.0
        self.apply_started = 0.0
        self.retry_after = 0.0
        self.failed_at = 0.0
        self.current_map_id = ""
        self.map_ready = False
        self.latest_trust: Dict[str, Any] = {}
        self.pose_candidates: Dict[int, Tuple[PoseStamped, float]] = {}
        self.map_odom_candidates: Dict[int, Tuple[PoseStamped, float]] = {}
        self.pending_accepted_statuses: Dict[Tuple[str, int], Dict[str, Any]] = {}
        self.consumed_results = set()
        self.last_global_state = ""
        self.last_ro_status: Dict[str, Any] = {}
        self.auto_trigger_suppressed = False
        self.pending_apply_pose: Optional[PoseStamped] = None
        self.pending_apply_map_odom: Optional[PoseStamped] = None
        self.pending_apply_stamp = math.nan
        self.ro_refined_map_odom_candidates: Dict[int, Tuple[PoseStamped, float]] = {}
        self.pending_ro_commit: Optional[Dict[str, Any]] = None

        self.publish_event("localization_recovery_coordinator_ready", "coordinator_ready")

    def mode_allows_search(self) -> bool:
        return self.integration_mode in ("shadow", "enforce")

    def mode_allows_side_effects(self) -> bool:
        return self.integration_mode == "enforce"

    def automatic_trigger_enabled(self) -> bool:
        return self.auto_trigger and self.mode_allows_search()

    def now_wall(self) -> float:
        return time.monotonic()

    def publish_json(self, publisher, payload: Dict[str, Any]):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        publisher.publish(msg)

    def publish_event(self, event_type: str, reason: str, **extra):
        payload = {
            "protocol_version": 1,
            "event_type": event_type,
            "stamp_sec": self.get_clock().now().nanoseconds * 1e-9,
            "attempt_id": self.attempt_id,
            "attempt_number": self.attempt_number,
            "map_id": self.current_map_id,
            "recovery_type": self.recovery_type,
            "state": self.state,
            "reason": reason,
            "integration_mode": self.integration_mode,
            "side_effects_enabled": self.mode_allows_side_effects(),
            "dry_run": self.dry_run,
            "auto_apply": self.auto_apply,
        }
        payload.update(extra)
        self.publish_json(self.status_pub, payload)

    def send_request(self, command: str, reason: str):
        request = {
            "protocol_version": 1,
            "command": command,
            "attempt_id": self.attempt_id,
            "map_id": self.current_map_id,
            "recovery_type": self.recovery_type,
            "reason": reason,
        }
        assets = self.map_assets.get(self.current_map_id, {})
        if isinstance(assets, dict):
            request.update({key: value for key, value in assets.items() if isinstance(value, str)})
        self.publish_json(self.request_pub, request)

    def load_map_registry_assets(self):
        if not self.map_registry_path or not os.path.isfile(self.map_registry_path):
            return
        try:
            with open(self.map_registry_path, "r", encoding="utf-8") as stream:
                registry = json.load(stream)
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().error(f"failed to load map registry assets: {exc}")
            return
        maps = registry.get("maps", []) if isinstance(registry, dict) else []
        for item in maps:
            if not isinstance(item, dict):
                continue
            map_id = str(item.get("map_id", ""))
            if not map_id:
                continue
            assets = {
                "map_path": str(item.get("open3d_prior_map_file", "")),
                "scan_context_database_path": str(
                    item.get("global_relocalization_scan_context_database_file", "")
                ),
                "solid_database_path": str(
                    item.get("global_relocalization_solid_database_file", "")
                ),
            }
            self.map_assets.setdefault(map_id, {}).update(assets)

    def validate_map_assets(self) -> Tuple[bool, str]:
        assets = self.map_assets.get(self.current_map_id)
        if not isinstance(assets, dict):
            return False, "map_assets_not_registered"
        map_path = str(assets.get("map_path", ""))
        if not map_path or not os.path.isfile(map_path):
            return False, "globalmap_not_ready"
        return True, "ready"

    def recovery_required(self) -> bool:
        state = str(self.latest_trust.get("state", ""))
        return bool(
            self.latest_trust.get("startup_requires_global_relocalization", False)
            or self.latest_trust.get("recovery_requires_global_relocalization", False)
            or state in (
                "startup_requires_global_relocalization",
                "recovery_requires_global_relocalization",
            )
        )

    def requested_recovery_type(self) -> str:
        if bool(self.latest_trust.get("recovery_requires_global_relocalization", False)):
            return "online_recovery"
        return "cold_start"

    def start_attempt(self, retry: bool = False):
        if not self.mode_allows_search():
            self.state = "off"
            self.publish_event(
                "localization_relocalize_attempt_deferred", "integration_mode_off",
                result_code="integration_mode_off",
            )
            return
        if not self.map_ready or not self.current_map_id:
            self.state = "waiting_map"
            self.publish_event(
                "localization_relocalize_attempt_deferred", "globalmap_not_ready",
                result_code="globalmap_not_ready"
            )
            return
        assets_ready, assets_reason = self.validate_map_assets()
        if not assets_ready:
            self.state = "waiting_map"
            self.publish_event(
                "localization_relocalize_attempt_deferred", assets_reason,
                result_code=assets_reason
            )
            return
        if not retry:
            self.attempt_number = 0
        self.failed_at = 0.0
        self.attempt_number += 1
        self.attempt_id = f"reloc-{uuid.uuid4().hex}"
        self.recovery_type = self.requested_recovery_type()
        self.state = "searching"
        self.attempt_started = self.now_wall()
        self.apply_started = 0.0
        self.pose_candidates.clear()
        self.map_odom_candidates.clear()
        self.pending_accepted_statuses.clear()
        self.consumed_results.clear()
        self.pending_apply_pose = None
        self.pending_apply_map_odom = None
        self.pending_apply_stamp = math.nan
        self.ro_refined_map_odom_candidates.clear()
        self.pending_ro_commit = None
        self.send_request("start", "localization_trust_requires_global")
        self.publish_event("localization_recovery_started", "global_relocalization_started")
        self.publish_event("localization_relocalize_requested", "request_published")

    def cancel_attempt(self, reason: str, terminal_state: str = "idle"):
        if self.attempt_id:
            self.send_request("cancel", reason)
        self.state = terminal_state
        self.pose_candidates.clear()
        self.map_odom_candidates.clear()
        self.pending_accepted_statuses.clear()
        self.pending_apply_pose = None
        self.pending_apply_map_odom = None
        self.pending_apply_stamp = math.nan
        self.ro_refined_map_odom_candidates.clear()
        self.pending_ro_commit = None

    def on_trust(self, msg: String):
        payload = parse_object(msg.data)
        if payload is None:
            return
        self.latest_trust = payload
        trusted = bool(payload.get("pose_trusted", False) and payload.get("can_start_navigation", False))
        if self.state == "verifying" and trusted:
            self.publish_event("localization_recovered", "ro_handoff_verified")
            self.cancel_attempt("recovery_complete", "idle")
            self.attempt_id = ""
            self.attempt_number = 0
            return
        if self.state == "shadow_complete" and not self.recovery_required():
            self.state = "idle"
            self.attempt_id = ""
            self.attempt_number = 0
        if (
            self.automatic_trigger_enabled() and not self.auto_trigger_suppressed
            and self.recovery_required() and self.state in ("idle", "waiting_map")
        ):
            self.start_attempt()

    def on_map_status(self, msg: String):
        payload = parse_object(msg.data)
        if payload is None:
            return
        data = nested_data(payload)
        map_id = str(data.get("current_map_id", data.get("map_id", "")))
        ready = str(data.get("map_state", "")).lower() == "ready"
        changed = bool(self.current_map_id and map_id and map_id != self.current_map_id)
        if changed and self.state not in ("idle", "waiting_map"):
            self.publish_event("localization_relocalize_failed", "map_changed_during_attempt")
            self.cancel_attempt("map_changed", "waiting_map")
            self.attempt_id = ""
            self.attempt_number = 0
        self.current_map_id = map_id
        self.map_ready = ready
        if (
            self.automatic_trigger_enabled() and not self.auto_trigger_suppressed
            and ready and self.recovery_required() and self.state in ("idle", "waiting_map")
        ):
            self.start_attempt()

    def candidate_key(self, stamp: float) -> int:
        return int(round(stamp * 1e9))

    def on_recovery_pose(self, msg: PoseStamped):
        self.pose_candidates[self.candidate_key(stamp_seconds(msg))] = (msg, self.now_wall())
        self.try_pending_results()

    def on_recovery_map_odom(self, msg: PoseStamped):
        self.map_odom_candidates[self.candidate_key(stamp_seconds(msg))] = (msg, self.now_wall())
        self.try_pending_results()

    def nearest_candidate(self, candidates, stamp: float):
        if not candidates:
            return None
        target = self.candidate_key(stamp)
        key = min(candidates, key=lambda item: abs(item - target))
        if abs(key - target) * 1e-9 > self.candidate_stamp_tolerance_sec:
            return None
        return candidates[key]

    def validate_result(self, payload: Dict[str, Any]):
        if self.state != "searching":
            return None, "attempt_not_searching"
        if str(payload.get("attempt_id", "")) != self.attempt_id:
            return None, "attempt_id_mismatch"
        if str(payload.get("map_id", "")) != self.current_map_id:
            return None, "map_id_mismatch"
        stamp = payload.get("stamp_sec")
        if not isinstance(stamp, (int, float)) or not math.isfinite(float(stamp)):
            return None, "invalid_result_stamp"
        result_key = (self.attempt_id, self.candidate_key(float(stamp)))
        if result_key in self.consumed_results:
            return None, "duplicate_result"
        pose_item = self.nearest_candidate(self.pose_candidates, float(stamp))
        map_odom_item = self.nearest_candidate(self.map_odom_candidates, float(stamp))
        if pose_item is None or map_odom_item is None:
            return None, "pose_status_not_correlated"
        now = self.now_wall()
        if now - pose_item[1] > self.candidate_max_receipt_age_sec or now - map_odom_item[1] > self.candidate_max_receipt_age_sec:
            return None, "candidate_expired"
        if pose_item[0].header.frame_id != "map" or map_odom_item[0].header.frame_id != "map":
            return None, "candidate_wrong_frame"
        self.consumed_results.add(result_key)
        return (pose_item[0], map_odom_item[0]), "accepted"

    def on_global_status(self, msg: String):
        payload = parse_object(msg.data)
        if payload is None:
            self.publish_event("localization_relocalize_attempt_deferred", "invalid_global_status_json")
            return
        state = str(payload.get("state", "")).lower()
        self.last_global_state = state
        if not state.startswith("accepted"):
            if state.startswith("need_active_view") or state.startswith("reject"):
                self.publish_event(
                    "localization_relocalize_attempt_deferred",
                    str(payload.get("reason", state)),
                    result_code=state,
                    global_result=payload,
                )
            return
        self.process_accepted_status(payload)

    def process_accepted_status(self, payload: Dict[str, Any]):
        # 接受状态、PoseStamped 和 map->odom 是三条异步话题；
        # 在这里必须完成 attempt/map/stamp 合同关联，禁止串用旧候选。
        state = str(payload.get("state", "")).lower()
        result, reason = self.validate_result(payload)
        if result is None:
            if reason == "pose_status_not_correlated" and self.state == "searching":
                stamp = payload.get("stamp_sec")
                if isinstance(stamp, (int, float)) and math.isfinite(float(stamp)):
                    key = (str(payload.get("attempt_id", "")), self.candidate_key(float(stamp)))
                    self.pending_accepted_statuses[key] = payload
            self.publish_event(
                "localization_relocalize_attempt_deferred", reason,
                result_code=reason, global_result=payload
            )
            return
        stamp = float(payload["stamp_sec"])
        self.pending_accepted_statuses.pop(
            (str(payload.get("attempt_id", "")), self.candidate_key(stamp)), None
        )
        pose, map_odom = result
        self.state = "candidate_validated"
        self.publish_event(
            "localization_relocalize_accepted", "candidate_contract_validated",
            result_code=state, global_result=payload
        )
        self.send_request("cancel", "candidate_validated")
        if self.integration_mode == "shadow":
            self.state = "shadow_complete"
            self.publish_event(
                "localization_relocalize_shadow_result", "shadow_candidate_validated_no_side_effects",
                result_code="shadow_observed", global_result=payload,
            )
            return
        if not self.mode_allows_side_effects():
            self.state = "off"
            self.publish_event(
                "localization_relocalize_attempt_deferred", "integration_mode_off",
                result_code="integration_mode_off",
            )
            return
        self.apply_candidate(pose, map_odom)

    def try_pending_results(self):
        if self.state != "searching":
            return
        for payload in list(self.pending_accepted_statuses.values()):
            self.process_accepted_status(payload)
            if self.state != "searching":
                break

    def apply_candidate(self, pose: PoseStamped, map_odom: PoseStamped):
        # 全局 pose 只作为 RO 局部精匹配初值。此时不向 bridge 发布
        # 全局算法计算的 map->odom，避免绕过 RO 独立收敛证明。
        ro_pose = PoseWithCovarianceStamped()
        ro_pose.header = pose.header
        ro_pose.pose.pose = pose.pose
        ro_pose.pose.covariance[0] = 0.01
        ro_pose.pose.covariance[7] = 0.01
        ro_pose.pose.covariance[14] = 0.04
        ro_pose.pose.covariance[21] = 0.03
        ro_pose.pose.covariance[28] = 0.03
        ro_pose.pose.covariance[35] = 0.01
        self.pending_apply_pose = pose
        self.pending_apply_map_odom = map_odom
        self.pending_apply_stamp = stamp_seconds(pose)
        self.ro_apply_pub.publish(ro_pose)
        self.state = "waiting_ro_commit"
        self.apply_started = self.now_wall()
        self.publish_event(
            "localization_initialpose_published", "ro_pose_published_waiting_trusted_commit",
            candidate_stamp_sec=self.pending_apply_stamp,
        )

    def on_bridge_status(self, msg: String):
        if self.state != "applying":
            return
        if msg.data.startswith("ACCEPTED global_recovery"):
            self.state = "verifying"
            self.publish_event("localization_relocalize_accepted", "bridge_applied_waiting_ro")

    def on_ro_refined_map_odom(self, msg: PoseStamped):
        key = self.candidate_key(stamp_seconds(msg))
        self.ro_refined_map_odom_candidates[key] = (msg, self.now_wall())
        if self.pending_ro_commit is not None:
            self.try_apply_ro_commit(self.pending_ro_commit)

    def on_ro_status(self, msg: String):
        payload = parse_object(msg.data)
        if payload is not None:
            self.last_ro_status = payload
        if self.state != "waiting_ro_commit" or payload is None:
            return
        if (
            str(payload.get("status", "")) != "NORMAL"
            or str(payload.get("source", "")) != "ro_trusted_global_anchor_commit"
        ):
            return
        anchor_stamp = payload.get("anchor_stamp")
        if (
            not isinstance(anchor_stamp, (int, float))
            or not math.isfinite(float(anchor_stamp))
            or abs(float(anchor_stamp) - self.pending_apply_stamp)
            > self.candidate_stamp_tolerance_sec
        ):
            self.publish_event(
                "localization_relocalize_attempt_deferred",
                "ro_commit_anchor_stamp_mismatch",
                result_code="ro_commit_anchor_stamp_mismatch",
                ro_status=payload,
            )
            return
        self.pending_ro_commit = payload
        self.try_apply_ro_commit(payload)

    def try_apply_ro_commit(self, payload: Dict[str, Any]):
        # RO commit 必须回指本次全局 anchor，并与同时间戳 refined map->odom 配对。
        # 只有该 refined 结果拥有 bridge 自动恢复授权。
        if self.state != "waiting_ro_commit":
            return
        stamp = payload.get("stamp")
        if not isinstance(stamp, (int, float)) or not math.isfinite(float(stamp)):
            return
        item = self.nearest_candidate(self.ro_refined_map_odom_candidates, float(stamp))
        if item is None:
            return
        refined_map_odom, receipt_time = item
        if self.now_wall() - receipt_time > self.candidate_max_receipt_age_sec:
            self.publish_event(
                "localization_relocalize_attempt_deferred",
                "ro_refined_map_odom_expired",
                result_code="ro_refined_map_odom_expired",
            )
            return
        if refined_map_odom.header.frame_id != "map":
            self.publish_event(
                "localization_relocalize_attempt_deferred",
                "ro_refined_map_odom_wrong_frame",
                result_code="ro_refined_map_odom_wrong_frame",
            )
            return
        self.bridge_apply_pub.publish(refined_map_odom)
        self.state = "applying"
        self.apply_started = self.now_wall()
        self.pending_ro_commit = None
        self.publish_event(
            "localization_ro_commit_verified", "trusted_ro_commit_bridge_apply_published",
            candidate_stamp_sec=self.pending_apply_stamp,
            refined_stamp_sec=float(stamp),
        )

    def on_navigation_status(self, msg: String):
        payload = parse_object(msg.data)
        if payload is None:
            return
        event_type = str(payload.get("event_type", payload.get("message_type", "")))
        if event_type in ("navigation_stopped", "navigation_pending_cancelled"):
            self.auto_trigger_suppressed = True
            if self.state not in ("idle", "failed"):
                self.publish_event("localization_relocalize_failed", "navigation_stopped_by_upper_layer")
                self.cancel_attempt("navigation_stopped_by_upper_layer", "idle")
                self.attempt_id = ""
                self.attempt_number = 0
            return
        if event_type in ("navigation_pending", "waypoint_started"):
            self.auto_trigger_suppressed = False
            if self.state == "failed":
                self.state = "idle"
                self.attempt_number = 0
            if self.automatic_trigger_enabled() and self.recovery_required() and self.state in ("idle", "waiting_map"):
                self.start_attempt()

    def schedule_retry_or_fail(self, reason: str):
        self.send_request("cancel", reason)
        if self.attempt_number < self.max_attempts and self.recovery_required():
            self.state = "retry_wait"
            self.retry_after = self.now_wall() + self.retry_backoff_sec
            self.publish_event(
                "localization_relocalize_attempt_deferred", reason,
                result_code="retry_scheduled"
            )
            return
        self.state = "failed"
        self.failed_at = self.now_wall()
        self.publish_event(
            "localization_relocalize_failed",
            reason,
            result_code="attempts_exhausted",
            route_remains_stopped=True,
            automatic_retry=True,
            retry_cooldown_sec=self.exhausted_retry_cooldown_sec,
        )

    def prune_candidates(self):
        cutoff = self.now_wall() - self.candidate_max_receipt_age_sec
        self.pose_candidates = {key: value for key, value in self.pose_candidates.items() if value[1] >= cutoff}
        self.map_odom_candidates = {key: value for key, value in self.map_odom_candidates.items() if value[1] >= cutoff}
        active_cutoff = self.get_clock().now().nanoseconds * 1e-9 - self.candidate_max_receipt_age_sec
        self.pending_accepted_statuses = {
            key: value for key, value in self.pending_accepted_statuses.items()
            if isinstance(value.get("stamp_sec"), (int, float))
            and float(value["stamp_sec"]) >= active_cutoff
        }
        self.ro_refined_map_odom_candidates = {
            key: value
            for key, value in self.ro_refined_map_odom_candidates.items()
            if value[1] >= cutoff
        }

    def on_timer(self):
        self.prune_candidates()
        now = self.now_wall()
        if self.state == "searching" and now - self.attempt_started > self.attempt_timeout_sec:
            self.schedule_retry_or_fail("global_relocalization_timeout")
        elif self.state in ("waiting_ro_commit", "applying") and now - self.apply_started > self.apply_timeout_sec:
            reason = "ro_trusted_commit_timeout" if self.state == "waiting_ro_commit" else "bridge_apply_timeout"
            self.schedule_retry_or_fail(reason)
        elif self.state == "retry_wait" and now >= self.retry_after:
            self.start_attempt(retry=True)
        elif (
            self.state == "failed"
            and self.exhausted_retry_cooldown_sec > 0.0
            and self.failed_at > 0.0
            and now - self.failed_at >= self.exhausted_retry_cooldown_sec
            and self.automatic_trigger_enabled()
            and not self.auto_trigger_suppressed
            and self.recovery_required()
        ):
            self.publish_event(
                "localization_relocalize_cycle_restarting",
                "exhausted_retry_cooldown_elapsed",
                previous_attempts=self.attempt_number,
            )
            self.state = "idle"
            self.attempt_number = 0
            self.start_attempt()


def main(args=None):
    rclpy.init(args=args)
    node = GlobalRelocalizationCoordinator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
