#!/usr/bin/env python3
"""定位可信度监督层。

该节点不发布 TF，也不做配准。它只把 bridge 的 map->odom 接收状态、
RoboSense 定位器的匹配健康状态和后续全局重定位状态合成为一个清晰的
“导航是否可以相信当前定位”的结论。
"""

import json
import math
import re
import time
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


_KEY_VALUE_RE = re.compile(r"([A-Za-z0-9_]+)=([^ ]+)")
_VALID_INTEGRATION_MODES = ("off", "shadow", "enforce")


def _parse_key_values(text: str) -> Dict[str, str]:
    return {match.group(1): match.group(2) for match in _KEY_VALUE_RE.finditer(text)}


def _as_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _json_or_text(data: str) -> Dict[str, Any]:
    try:
        payload = json.loads(data)
        return payload if isinstance(payload, dict) else {"raw": data}
    except json.JSONDecodeError:
        fields = _parse_key_values(data)
        return {**fields, "raw": data}


class LocalizationTrustSupervisor(Node):
    """合成定位可信度状态，给导航任务层做启动/恢复门控。"""

    def __init__(self):
        super().__init__("localization_trust_supervisor")

        self.bridge_status_topic = self.declare_parameter(
            "bridge_status_topic", "/localization/prior_map_odom_bridge_status"
        ).value
        self.robosense_status_topic = self.declare_parameter(
            "robosense_status_topic", "/prior_localization/robosense_status"
        ).value
        self.global_status_topic = self.declare_parameter(
            "global_status_topic", "/global_relocalization/recovery_status"
        ).value
        self.trust_status_topic = self.declare_parameter(
            "trust_status_topic", "/localization/trust_status"
        ).value
        requested_mode = str(
            self.declare_parameter("integration_mode", "shadow").value
        ).strip().lower()
        self.integration_mode = (
            requested_mode if requested_mode in _VALID_INTEGRATION_MODES else "off"
        )
        if self.integration_mode != requested_mode:
            self.get_logger().error(
                f"invalid integration_mode={requested_mode!r}; falling back to off"
            )

        self.bridge_status_timeout_sec = float(
            self.declare_parameter("bridge_status_timeout_sec", 3.0).value
        )
        self.robosense_status_timeout_sec = float(
            self.declare_parameter("robosense_status_timeout_sec", 3.0).value
        )
        self.origin_seed_radius = float(
            self.declare_parameter("origin_seed_radius", 0.30).value
        )
        self.ro_only_startup_max_map_odom_norm = float(
            self.declare_parameter("ro_only_startup_max_map_odom_norm", 0.35).value
        )
        self.require_global_after_large_jump_hold = bool(
            self.declare_parameter("require_global_after_large_jump_hold", True).value
        )
        self.large_jump_hold_min_duration_sec = max(
            0.0,
            float(self.declare_parameter("large_jump_hold_min_duration_sec", 3.0).value),
        )
        self.large_jump_hold_min_updates = max(
            1, int(self.declare_parameter("large_jump_hold_min_updates", 3).value)
        )
        self.require_global_after_ro_unhealthy = bool(
            self.declare_parameter("require_global_after_ro_unhealthy", True).value
        )
        self.ro_unhealthy_min_duration_sec = max(
            0.0,
            float(self.declare_parameter("ro_unhealthy_min_duration_sec", 3.0).value),
        )
        self.ro_unhealthy_min_updates = max(
            1, int(self.declare_parameter("ro_unhealthy_min_updates", 3).value)
        )
        self.ro_stale_recovery_sec = max(
            self.robosense_status_timeout_sec,
            float(self.declare_parameter("ro_stale_recovery_sec", 3.0).value),
        )
        self.ro_verified_required_frames = max(
            1, int(self.declare_parameter("ro_verified_required_frames", 5).value)
        )
        self.publish_rate = max(
            0.2, float(self.declare_parameter("publish_rate", 2.0).value)
        )

        self.bridge_status: Dict[str, Any] = {}
        self.robosense_status: Dict[str, Any] = {}
        self.global_status: Dict[str, Any] = {}
        self.last_bridge_time = 0.0
        self.last_ro_time = 0.0
        self.last_global_time = 0.0

        self.pose_initialized = False
        self.origin_seeded = False
        self.startup_requires_global_relocalization = False
        self.recovery_requires_global_relocalization = False
        self.recovery_trigger = ""
        self.bridge_accept_kind = ""
        self.ro_verified_count = 0
        self.ro_verified_stamp = 0.0
        self.ro_verification_epoch = 0
        self.ever_trusted = False
        self.large_jump_hold_started = 0.0
        self.large_jump_hold_updates = 0
        self.ro_unhealthy_started = 0.0
        self.ro_unhealthy_updates = 0
        self.ro_unhealthy_last_stamp = 0.0
        self.ro_unhealthy_reason = ""
        self.state = "waiting_initial_pose"
        self.reason = "waiting_for_bridge_accept"
        self.pose_trusted = False
        self.can_start_navigation = False

        self.status_pub = self.create_publisher(String, self.trust_status_topic, 10)
        self.create_subscription(String, self.bridge_status_topic, self.on_bridge_status, 10)
        self.create_subscription(String, self.robosense_status_topic, self.on_robosense_status, 10)
        self.create_subscription(String, self.global_status_topic, self.on_global_status, 10)
        self.create_timer(1.0 / self.publish_rate, self.publish_status)

        self.get_logger().info(
            "localization_trust_supervisor started: "
            f"bridge={self.bridge_status_topic}, ro={self.robosense_status_topic}, "
            f"out={self.trust_status_topic}, integration_mode={self.integration_mode}"
        )

    def recovery_control_enabled(self) -> bool:
        return self.integration_mode == "enforce"

    def clear_ro_unhealthy_observation(self):
        self.ro_unhealthy_started = 0.0
        self.ro_unhealthy_updates = 0
        self.ro_unhealthy_last_stamp = 0.0
        self.ro_unhealthy_reason = ""

    def latch_online_recovery(self, trigger: str):
        self.recovery_requires_global_relocalization = True
        self.recovery_trigger = trigger
        # 故障前的 RO 健康帧不能用于恢复后的重新放行。
        self.ro_verified_count = 0
        self.ro_verified_stamp = 0.0

    def observe_ro_health(self, now: float, status: str, source: str, ro_stamp: float):
        # 在线 LOST/LOW_ACCURACY 防抖只在曾经真正可信后启用，排除开机和候选交接期。
        if not self.pose_initialized or not self.ever_trusted:
            self.clear_ro_unhealthy_observation()
            return
        if status == "NORMAL" and source == "ro_normal_match":
            self.clear_ro_unhealthy_observation()
            return
        if self._global_relocalization_active(now):
            return
        if self.ro_unhealthy_started <= 0.0:
            self.ro_unhealthy_started = now
            self.ro_unhealthy_reason = f"{status}:{source}"
        if math.isfinite(ro_stamp):
            if ro_stamp > self.ro_unhealthy_last_stamp + 1e-6:
                self.ro_unhealthy_updates += 1
                self.ro_unhealthy_last_stamp = ro_stamp
        else:
            self.ro_unhealthy_updates += 1
        unhealthy_age = max(0.0, now - self.ro_unhealthy_started)
        if (
            self.require_global_after_ro_unhealthy
            and unhealthy_age >= self.ro_unhealthy_min_duration_sec
            and self.ro_unhealthy_updates >= self.ro_unhealthy_min_updates
        ):
            self.latch_online_recovery(
                f"robosense_unhealthy age={unhealthy_age:.2f}s "
                f"updates={self.ro_unhealthy_updates} reason={self.ro_unhealthy_reason}"
            )

    def on_bridge_status(self, msg: String):
        now = time.time()
        text = msg.data.strip()
        kind = text.split(" ", 1)[0] if text else "UNKNOWN"
        fields = _parse_key_values(text)
        xy_norm = _as_float(fields.get("map_odom_xy_norm"), math.inf)

        self.last_bridge_time = now
        self.bridge_status = {
            "topic": self.bridge_status_topic,
            "state": kind,
            "text": text,
            "fields": fields,
            "stamp_sec": now,
        }

        large_jump_hold = kind in ("HOLD", "DEGRADED") and "large_jump" in text
        if large_jump_hold:
            if self.large_jump_hold_started <= 0.0:
                self.large_jump_hold_started = now
                self.large_jump_hold_updates = 0
            self.large_jump_hold_updates += 1
            reported_age = _as_float(fields.get("age", "").rstrip("s"), -1.0)
            observed_age = max(0.0, now - self.large_jump_hold_started)
            hold_age = max(observed_age, reported_age)
            confirmed = (
                kind == "DEGRADED"
                or (
                    hold_age >= self.large_jump_hold_min_duration_sec
                    and self.large_jump_hold_updates >= self.large_jump_hold_min_updates
                )
            )
            if self.require_global_after_large_jump_hold and confirmed:
                self.latch_online_recovery(text)
        else:
            # 短时 HOLD 自行恢复时只清除未成熟观察，不清除已确认的恢复锁存。
            self.large_jump_hold_started = 0.0
            self.large_jump_hold_updates = 0

        if kind == "ACCEPTED":
            self.pose_initialized = True
            self.bridge_accept_kind = self._accepted_kind(text)
            if self.bridge_accept_kind == "global_recovery":
                self.startup_requires_global_relocalization = False
                self.recovery_requires_global_relocalization = False
                self.recovery_trigger = ""
                self.clear_ro_unhealthy_observation()
                self.ro_verified_count = 0
            if self.bridge_accept_kind == "initial_pose":
                self.origin_seeded = (
                    fields.get("origin_seeded", "").lower() == "true"
                    or xy_norm <= self.origin_seed_radius
                )
                if (
                    (self.origin_seeded or xy_norm > self.ro_only_startup_max_map_odom_norm)
                    and not self._latest_source_is_global_or_manual()
                ):
                    self.startup_requires_global_relocalization = True
            elif self.bridge_accept_kind.startswith("confirmed") or self.bridge_accept_kind == "small_correction":
                if self.ro_verified_count >= self.ro_verified_required_frames:
                    self.origin_seeded = False

        self.evaluate()

    def on_robosense_status(self, msg: String):
        now = time.time()
        payload = _json_or_text(msg.data)
        status = str(payload.get("status", "UNKNOWN"))
        source = str(payload.get("source", ""))

        self.last_ro_time = now
        self.robosense_status = {
            "topic": self.robosense_status_topic,
            "state": status,
            "source": source,
            "payload": payload,
            "stamp_sec": now,
        }

        ro_stamp = _as_float(payload.get("stamp"), math.nan)
        self.observe_ro_health(now, status, source, ro_stamp)
        if (
            status == "NORMAL"
            and source == "ro_normal_match"
            and math.isfinite(ro_stamp)
            and ro_stamp > self.ro_verified_stamp + 1e-6
        ):
            self.ro_verified_count += 1
            self.ro_verified_stamp = ro_stamp
            self.ro_verification_epoch += 1
        elif status == "NORMAL" and source == "ro_normal_match":
            # 重复状态不能伪装成新的雷达验证帧。
            pass
        else:
            self.ro_verified_count = 0
            self.ro_verified_stamp = 0.0

        # 人工定位是显式授权；自动全局候选只是 RO 初值，不能提前解除恢复锁。
        if "manual" in source.lower():
            self.startup_requires_global_relocalization = False
            self.recovery_requires_global_relocalization = False
            self.recovery_trigger = ""
            self.large_jump_hold_started = 0.0
            self.large_jump_hold_updates = 0
            self.clear_ro_unhealthy_observation()

        self.evaluate()

    def on_global_status(self, msg: String):
        now = time.time()
        payload = _json_or_text(msg.data)
        self.last_global_time = now
        self.global_status = {
            "topic": self.global_status_topic,
            "payload": payload,
            "stamp_sec": now,
        }
        state = str(payload.get("state", payload.get("status", ""))).lower()
        # 算法 accepted 只是恢复候选，不等于定位已经被 bridge/RO 接管。
        # 只有 bridge 明确报告 ACCEPTED global_recovery 后才清除恢复要求。
        self.evaluate()

    def _accepted_kind(self, text: str) -> str:
        if "global_recovery" in text:
            return "global_recovery"
        if "initial_pose" in text:
            return "initial_pose"
        if "small_correction" in text:
            return "small_correction"
        if "confirmed_" in text:
            return "confirmed_correction"
        return "accepted"

    def _global_relocalization_active(self, now: float) -> bool:
        if self.last_global_time <= 0.0 or now - self.last_global_time > 5.0:
            return False
        payload = self.global_status.get("payload", {})
        state = str(payload.get("state", payload.get("status", ""))).lower()
        return state in ("running", "searching", "relocalizing", "recovering")

    def _source_is_global_or_manual(self, source: str) -> bool:
        lowered = source.lower()
        return (
            "manual" in lowered
            or "global" in lowered
            or "scancontext" in lowered
            or "scan_context" in lowered
        )

    def _latest_source_is_global_or_manual(self) -> bool:
        return self._source_is_global_or_manual(str(self.robosense_status.get("source", "")))

    def evaluate(self):
        now = time.time()
        bridge_age = now - self.last_bridge_time if self.last_bridge_time > 0.0 else math.inf
        ro_age = now - self.last_ro_time if self.last_ro_time > 0.0 else math.inf

        self.pose_trusted = False
        self.can_start_navigation = False

        if bridge_age > self.bridge_status_timeout_sec:
            self.state = "waiting_bridge_status" if not self.pose_initialized else "bridge_status_stale"
            self.reason = (
                "waiting_for_bridge_status"
                if not self.pose_initialized
                else f"bridge_status_stale_{bridge_age:.1f}s"
            )
            return

        bridge_state = str(self.bridge_status.get("state", "UNKNOWN"))
        if not self.pose_initialized:
            self.state = "waiting_initial_pose"
            self.reason = "bridge_has_not_accepted_map_to_odom"
            return

        if self.recovery_control_enabled() and self._global_relocalization_active(now):
            self.state = "global_relocalizing"
            self.reason = "global_relocalization_active"
            return

        if self.recovery_control_enabled() and self.startup_requires_global_relocalization:
            self.state = "startup_requires_global_relocalization"
            self.reason = (
                "ro_only_initial_map_odom_norm_exceeds_"
                f"{self.ro_only_startup_max_map_odom_norm:.2f}m"
            )
            return

        if self.recovery_control_enabled() and self.recovery_requires_global_relocalization:
            self.state = "recovery_requires_global_relocalization"
            self.reason = "large_jump_hold_seen_without_global_confirmation"
            return

        if bridge_state in ("HOLD", "DEGRADED"):
            self.state = "bridge_holding_last_good_tf"
            self.reason = str(self.bridge_status.get("text", bridge_state))
            return

        if ro_age > self.robosense_status_timeout_sec:
            if (
                self.ever_trusted
                and self.require_global_after_ro_unhealthy
                and ro_age >= self.ro_stale_recovery_sec
                and not self._global_relocalization_active(now)
            ):
                self.latch_online_recovery(
                    f"robosense_status_stale age={ro_age:.2f}s"
                )
                self.state = "recovery_requires_global_relocalization"
                self.reason = "robosense_status_stale_confirmed"
                return
            self.state = "robosense_status_stale"
            self.reason = f"robosense_status_stale_{ro_age:.1f}s"
            return

        if self.ro_verified_count >= self.ro_verified_required_frames:
            self.pose_trusted = True
            self.can_start_navigation = True
            self.ever_trusted = True
            self.origin_seeded = False
            self.state = "trusted_ro"
            self.reason = f"ro_normal_match_frames={self.ro_verified_count}"
            return

        if self.origin_seeded:
            self.state = "initialized_untrusted"
            self.reason = "origin_seeded_unverified"
            return

        ro_state = str(self.robosense_status.get("state", "UNKNOWN"))
        ro_source = str(self.robosense_status.get("source", ""))
        self.state = "ro_verifying"
        self.reason = (
            f"need_ro_normal_match_frames "
            f"{self.ro_verified_count}/{self.ro_verified_required_frames} "
            f"current={ro_state}:{ro_source}"
        )

    def publish_status(self):
        self.evaluate()
        now = time.time()
        msg = String()
        msg.data = json.dumps(
            {
                "stamp_sec": now,
                "integration_mode": self.integration_mode,
                "global_relocalization_side_effects_enabled": self.recovery_control_enabled(),
                "state": self.state,
                "reason": self.reason,
                "pose_initialized": self.pose_initialized,
                "pose_trusted": self.pose_trusted,
                "can_start_navigation": self.can_start_navigation,
                "origin_seeded": self.origin_seeded,
                "startup_requires_global_relocalization": self.startup_requires_global_relocalization,
                "recovery_requires_global_relocalization": self.recovery_requires_global_relocalization,
                "localization_recovery_required": (
                    self.recovery_control_enabled()
                    and self.recovery_requires_global_relocalization
                ),
                "recovery_trigger": self.recovery_trigger,
                "ro_only_startup_max_map_odom_norm": self.ro_only_startup_max_map_odom_norm,
                "require_global_after_large_jump_hold": self.require_global_after_large_jump_hold,
                "bridge_accept_kind": self.bridge_accept_kind,
                "ro_verified_count": self.ro_verified_count,
                "ro_verified_required_frames": self.ro_verified_required_frames,
                "ro_verified_stamp": self.ro_verified_stamp,
                "ro_verification_epoch": self.ro_verification_epoch,
                "ever_trusted": self.ever_trusted,
                "large_jump_hold_observed_age_sec": (
                    max(0.0, now - self.large_jump_hold_started)
                    if self.large_jump_hold_started > 0.0
                    else 0.0
                ),
                "large_jump_hold_updates": self.large_jump_hold_updates,
                "ro_unhealthy_observed_age_sec": (
                    max(0.0, now - self.ro_unhealthy_started)
                    if self.ro_unhealthy_started > 0.0
                    else 0.0
                ),
                "ro_unhealthy_updates": self.ro_unhealthy_updates,
                "ro_unhealthy_reason": self.ro_unhealthy_reason,
                "ro_unhealthy_min_duration_sec": self.ro_unhealthy_min_duration_sec,
                "ro_unhealthy_min_updates": self.ro_unhealthy_min_updates,
                "ro_stale_recovery_sec": self.ro_stale_recovery_sec,
                "bridge_status_age_sec": (
                    now - self.last_bridge_time if self.last_bridge_time > 0.0 else None
                ),
                "robosense_status_age_sec": (
                    now - self.last_ro_time if self.last_ro_time > 0.0 else None
                ),
                "bridge_status": self.bridge_status,
                "robosense_status": self.robosense_status,
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationTrustSupervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
