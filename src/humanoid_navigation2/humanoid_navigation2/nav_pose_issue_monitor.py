#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import os
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PolygonStamped, PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_list_to_yaw(q: List[float]) -> Optional[float]:
    if len(q) < 4:
        return None
    x, y, z, w = [float(v) for v in q[:4]]
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def dist2d(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def point_in_poly(point: Tuple[float, float], poly: List[Tuple[float, float]]) -> bool:
    x, y = point
    inside = False
    n = len(poly)
    if n < 3:
        return False

    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        intersects = (yi > y) != (yj > y)
        if intersects:
            x_at_y = (xj - xi) * (y - yi) / ((yj - yi) + 1e-12) + xi
            if x < x_at_y:
                inside = not inside
        j = i

    return inside


def point_segment_distance(
    point: Tuple[float, float],
    a: Tuple[float, float],
    b: Tuple[float, float],
) -> float:
    px, py = point
    ax, ay = a
    bx, by = b
    dx = bx - ax
    dy = by - ay
    denom = dx * dx + dy * dy
    if denom <= 1e-12:
        return dist2d(point, a)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    closest = (ax + t * dx, ay + t * dy)
    return dist2d(point, closest)


def point_poly_distance(point: Tuple[float, float], poly: List[Tuple[float, float]]) -> float:
    if len(poly) < 3:
        return float("inf")
    if point_in_poly(point, poly):
        return 0.0
    return min(point_segment_distance(point, poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly)))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float
    frame_id: str
    stamp_wall: float


@dataclass
class CostmapAnalysis:
    available: bool
    reason: str = ""
    frame_id: str = ""
    footprint_frame_id: str = ""
    footprint_cells_checked: int = 0
    max_cost_inside_footprint: int = -1
    lethal_cells_inside_footprint: int = 0
    inflated_cells_inside_footprint: int = 0
    nearest_lethal_dist_m: Optional[float] = None
    nearest_lethal_cost: Optional[int] = None
    nearest_lethal_point: Optional[Tuple[float, float]] = None
    nearest_high_dist_m: Optional[float] = None
    nearest_high_cost: Optional[int] = None
    nearest_high_point: Optional[Tuple[float, float]] = None
    nearest_high_point_base: Optional[Tuple[float, float]] = None
    nearest_high_sector: str = "unknown"


class NavPoseIssueMonitor(Node):
    def __init__(self):
        super().__init__("nav_pose_issue_monitor")

        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("robot_pose_topic", "/robot_realpose")
        self.declare_parameter("navigation_status_topic", "/navigation/status")
        self.declare_parameter("waypoints_topic", "/navigation/waypoints_data")
        self.declare_parameter("navigation_requests_topic", "/navigation/requests")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("plan_topic", "/plan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("costmap_topic", "/local_costmap/costmap")
        self.declare_parameter("footprint_topic", "/local_costmap/published_footprint")
        self.declare_parameter("rosout_topic", "/rosout")
        self.declare_parameter("active_report_period_sec", 2.0)
        self.declare_parameter("idle_report_period_sec", 10.0)
        self.declare_parameter("lethal_cost_threshold", 90)
        self.declare_parameter("inflated_cost_threshold", 50)
        self.declare_parameter("near_obstacle_warn_dist", 0.35)
        self.declare_parameter("output_jsonl", "/tmp/nav_pose_issue_monitor.jsonl")

        self.global_frame = self.get_parameter("global_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.active_report_period = float(self.get_parameter("active_report_period_sec").value)
        self.idle_report_period = float(self.get_parameter("idle_report_period_sec").value)
        self.lethal_threshold = int(self.get_parameter("lethal_cost_threshold").value)
        self.inflated_threshold = int(self.get_parameter("inflated_cost_threshold").value)
        self.near_obstacle_warn_dist = float(self.get_parameter("near_obstacle_warn_dist").value)
        self.output_jsonl = str(self.get_parameter("output_jsonl").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.robot_pose: Optional[Pose2D] = None
        self.status_pose: Optional[Pose2D] = None
        self.current_goal: Optional[Dict[str, Any]] = None
        self.last_goal_key: Optional[str] = None
        self.current_state: str = ""
        self.current_event_type: str = ""
        self.waypoints_by_id: Dict[str, Dict[str, Any]] = {}
        self.last_request: Optional[Dict[str, Any]] = None
        self.last_cmd_vel: Optional[Tuple[float, float, float]] = None
        self.latest_costmap: Optional[OccupancyGrid] = None
        self.latest_footprint: Optional[PolygonStamped] = None
        self.latest_plan: Optional[Path] = None
        self.latest_goal_pose: Optional[PoseStamped] = None
        self.last_rosout_collision: Optional[Tuple[float, str]] = None
        self.last_rosout_controller_error: Optional[Tuple[float, str]] = None
        self.last_snapshot_wall = 0.0
        self.last_costmap_alert_wall = 0.0
        self.last_costmap_analysis: Optional[CostmapAnalysis] = None
        self.start_wall = time.time()

        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("robot_pose_topic").value,
            self.robot_pose_cb,
            20,
        )
        self.create_subscription(
            String,
            self.get_parameter("navigation_status_topic").value,
            self.navigation_status_cb,
            20,
        )
        self.create_subscription(
            String,
            self.get_parameter("waypoints_topic").value,
            self.waypoints_cb,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter("navigation_requests_topic").value,
            self.navigation_request_cb,
            10,
        )
        self.create_subscription(PoseStamped, self.get_parameter("goal_topic").value, self.goal_pose_cb, 10)
        self.create_subscription(Path, self.get_parameter("plan_topic").value, self.plan_cb, 10)
        self.create_subscription(Twist, self.get_parameter("cmd_vel_topic").value, self.cmd_vel_cb, 20)
        self.create_subscription(OccupancyGrid, self.get_parameter("costmap_topic").value, self.costmap_cb, 10)
        self.create_subscription(PolygonStamped, self.get_parameter("footprint_topic").value, self.footprint_cb, 10)
        self.create_subscription(Log, self.get_parameter("rosout_topic").value, self.rosout_cb, 50)

        self.create_timer(0.5, self.timer_cb)

        if self.output_jsonl:
            os.makedirs(os.path.dirname(self.output_jsonl) or ".", exist_ok=True)

        self.get_logger().info("nav_pose_issue_monitor started")
        self.get_logger().info(f"robot_pose_topic       : {self.get_parameter('robot_pose_topic').value}")
        self.get_logger().info(f"navigation_status_topic: {self.get_parameter('navigation_status_topic').value}")
        self.get_logger().info(f"costmap_topic          : {self.get_parameter('costmap_topic').value}")
        self.get_logger().info(f"footprint_topic        : {self.get_parameter('footprint_topic').value}")
        self.get_logger().info(f"output_jsonl           : {self.output_jsonl}")

    def robot_pose_cb(self, msg: PoseWithCovarianceStamped):
        pose = msg.pose.pose
        self.robot_pose = Pose2D(
            x=float(pose.position.x),
            y=float(pose.position.y),
            yaw=yaw_from_quat(pose.orientation),
            frame_id=msg.header.frame_id or self.global_frame,
            stamp_wall=time.time(),
        )

    def navigation_status_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f"failed to parse navigation status JSON: {exc}")
            return

        event_type = data.get("event_type", "")
        if event_type:
            self.current_event_type = event_type
            event_data = data.get("event_data", {})
            if event_type == "waypoint_started":
                self.current_goal = {
                    "waypoint_id": event_data.get("waypoint_id", ""),
                    "waypoint_name": event_data.get("waypoint_name", ""),
                    "position": event_data.get("position", []),
                    "frame_id": "map",
                }
                self.emit_snapshot(f"waypoint_started:{self.current_goal.get('waypoint_name', '')}", force=True)
            elif event_type in ("navigation_progress_update", "navigation_obstacle_blocked"):
                current_pose = event_data.get("current_pose", {})
                self.update_status_pose(current_pose)
                if event_type == "navigation_obstacle_blocked":
                    self.emit_snapshot("navigation_obstacle_blocked_status", force=True)
            return

        self.current_state = str(data.get("current_state", self.current_state))
        self.update_status_pose(data.get("current_pose", {}))
        goal = data.get("current_goal")
        if goal:
            self.current_goal = goal
            goal_key = json.dumps(goal, sort_keys=True, ensure_ascii=True)
            if goal_key != self.last_goal_key:
                self.last_goal_key = goal_key
                self.emit_snapshot(f"current_goal:{goal.get('waypoint_name', '')}", force=True)

        if data.get("obstacle_blocked") is True:
            self.emit_snapshot("navigation_status_obstacle_blocked", force=True)

    def update_status_pose(self, pose_data: Dict[str, Any]):
        position = pose_data.get("position", {}) if isinstance(pose_data, dict) else {}
        orientation = pose_data.get("orientation", {}) if isinstance(pose_data, dict) else {}
        if not position:
            return
        q = type("Q", (), {})()
        q.x = float(orientation.get("x", 0.0))
        q.y = float(orientation.get("y", 0.0))
        q.z = float(orientation.get("z", 0.0))
        q.w = float(orientation.get("w", 1.0))
        self.status_pose = Pose2D(
            x=float(position.get("x", 0.0)),
            y=float(position.get("y", 0.0)),
            yaw=yaw_from_quat(q),
            frame_id="navigation_status_current_pose_source_is_/odom",
            stamp_wall=time.time(),
        )

    def waypoints_cb(self, msg: String):
        try:
            root = json.loads(msg.data)
        except Exception:
            return

        payload = root.get("data", root)
        if isinstance(payload, dict) and "data" in payload:
            payload = payload["data"]
        waypoints = payload.get("waypoints", {}) if isinstance(payload, dict) else {}
        if not isinstance(waypoints, dict):
            return

        parsed = {}
        for type_map in waypoints.values():
            if not isinstance(type_map, dict):
                continue
            for wp_id, wp in type_map.items():
                if isinstance(wp, dict):
                    parsed[str(wp_id)] = wp
        self.waypoints_by_id = parsed

    def navigation_request_cb(self, msg: String):
        try:
            self.last_request = json.loads(msg.data)
        except Exception:
            self.last_request = {"raw": msg.data}
        self.emit_snapshot("navigation_request", force=True)

    def goal_pose_cb(self, msg: PoseStamped):
        self.latest_goal_pose = msg
        self.emit_snapshot("goal_pose_topic", force=True)

    def plan_cb(self, msg: Path):
        self.latest_plan = msg

    def cmd_vel_cb(self, msg: Twist):
        self.last_cmd_vel = (float(msg.linear.x), float(msg.angular.z), time.time())

    def costmap_cb(self, msg: OccupancyGrid):
        self.latest_costmap = msg

    def footprint_cb(self, msg: PolygonStamped):
        self.latest_footprint = msg

    def rosout_cb(self, msg: Log):
        text = msg.msg or ""
        if "RegulatedPurePursuitController detected collision ahead" in text:
            self.last_rosout_collision = (time.time(), text)
            self.emit_snapshot("rpp_collision_ahead", force=True)
        elif "Controller patience exceeded" in text:
            self.last_rosout_controller_error = (time.time(), text)
            self.emit_snapshot("controller_patience_exceeded", force=True)

    def timer_cb(self):
        self.last_costmap_analysis = self.analyze_costmap()

        active = self.current_state in ("planning", "executing", "paused") or self.current_goal is not None
        period = self.active_report_period if active else self.idle_report_period
        if time.time() - self.last_snapshot_wall >= period:
            self.emit_snapshot("periodic_active" if active else "periodic_idle", force=False)

        analysis = self.last_costmap_analysis
        if analysis and analysis.available:
            near = analysis.nearest_high_dist_m is not None and analysis.nearest_high_dist_m <= self.near_obstacle_warn_dist
            overlapping = analysis.lethal_cells_inside_footprint > 0 or analysis.inflated_cells_inside_footprint > 0
            if (near or overlapping) and time.time() - self.last_costmap_alert_wall >= 2.0:
                self.last_costmap_alert_wall = time.time()
                self.emit_snapshot("local_costmap_near_or_overlapping_footprint", force=True)

    def analyze_costmap(self) -> CostmapAnalysis:
        if self.latest_costmap is None:
            return CostmapAnalysis(False, reason="no local costmap received")
        if self.latest_footprint is None:
            return CostmapAnalysis(False, reason="no published footprint received")

        costmap = self.latest_costmap
        footprint = self.latest_footprint
        costmap_frame = costmap.header.frame_id
        footprint_frame = footprint.header.frame_id

        poly = [(float(p.x), float(p.y)) for p in footprint.polygon.points]
        if len(poly) < 3:
            return CostmapAnalysis(False, reason="published footprint has fewer than 3 points")

        if footprint_frame and footprint_frame != costmap_frame:
            transform = self.lookup_transform(costmap_frame, footprint_frame)
            if transform is None:
                return CostmapAnalysis(
                    False,
                    reason=f"costmap frame {costmap_frame} != footprint frame {footprint_frame}, no TF",
                    frame_id=costmap_frame,
                    footprint_frame_id=footprint_frame,
                )
            poly = [self.transform_point_2d(transform, x, y) for x, y in poly]

        width = costmap.info.width
        height = costmap.info.height
        res = costmap.info.resolution
        if width == 0 or height == 0 or res <= 0.0:
            return CostmapAnalysis(False, reason="invalid costmap metadata")

        origin = costmap.info.origin
        origin_yaw = yaw_from_quat(origin.orientation)
        cos_yaw = math.cos(origin_yaw)
        sin_yaw = math.sin(origin_yaw)

        def world_to_grid(x: float, y: float) -> Tuple[float, float]:
            dx = x - origin.position.x
            dy = y - origin.position.y
            gx = (cos_yaw * dx + sin_yaw * dy) / res
            gy = (-sin_yaw * dx + cos_yaw * dy) / res
            return gx, gy

        def grid_to_world(i: int, j: int) -> Tuple[float, float]:
            lx = (i + 0.5) * res
            ly = (j + 0.5) * res
            wx = origin.position.x + cos_yaw * lx - sin_yaw * ly
            wy = origin.position.y + sin_yaw * lx + cos_yaw * ly
            return wx, wy

        grid_poly = [world_to_grid(x, y) for x, y in poly]
        min_i = max(0, int(math.floor(min(p[0] for p in grid_poly))) - 2)
        max_i = min(width - 1, int(math.ceil(max(p[0] for p in grid_poly))) + 2)
        min_j = max(0, int(math.floor(min(p[1] for p in grid_poly))) - 2)
        max_j = min(height - 1, int(math.ceil(max(p[1] for p in grid_poly))) + 2)

        max_inside = -1
        lethal_inside = 0
        inflated_inside = 0
        checked = 0
        for j in range(min_j, max_j + 1):
            for i in range(min_i, max_i + 1):
                wx, wy = grid_to_world(i, j)
                if not point_in_poly((wx, wy), poly):
                    continue
                checked += 1
                cost = int(costmap.data[j * width + i])
                if cost < 0:
                    continue
                max_inside = max(max_inside, cost)
                if cost >= self.lethal_threshold:
                    lethal_inside += 1
                if cost >= self.inflated_threshold:
                    inflated_inside += 1

        nearest_lethal = (float("inf"), None, None)
        nearest_high = (float("inf"), None, None)
        for j in range(height):
            row = j * width
            for i in range(width):
                cost = int(costmap.data[row + i])
                if cost < self.inflated_threshold:
                    continue
                wx, wy = grid_to_world(i, j)
                d = point_poly_distance((wx, wy), poly)
                if cost >= self.lethal_threshold and d < nearest_lethal[0]:
                    nearest_lethal = (d, cost, (wx, wy))
                if d < nearest_high[0]:
                    nearest_high = (d, cost, (wx, wy))

        analysis = CostmapAnalysis(
            True,
            frame_id=costmap_frame,
            footprint_frame_id=footprint_frame,
            footprint_cells_checked=checked,
            max_cost_inside_footprint=max_inside,
            lethal_cells_inside_footprint=lethal_inside,
            inflated_cells_inside_footprint=inflated_inside,
        )

        if nearest_lethal[1] is not None:
            analysis.nearest_lethal_dist_m = nearest_lethal[0]
            analysis.nearest_lethal_cost = nearest_lethal[1]
            analysis.nearest_lethal_point = nearest_lethal[2]
        if nearest_high[1] is not None:
            analysis.nearest_high_dist_m = nearest_high[0]
            analysis.nearest_high_cost = nearest_high[1]
            analysis.nearest_high_point = nearest_high[2]
            base_tf = self.lookup_transform(self.base_frame, costmap_frame)
            if base_tf is not None and nearest_high[2] is not None:
                analysis.nearest_high_point_base = self.transform_point_2d(base_tf, nearest_high[2][0], nearest_high[2][1])
                analysis.nearest_high_sector = self.classify_base_point(analysis.nearest_high_point_base)

        return analysis

    def lookup_transform(self, target_frame: str, source_frame: str):
        try:
            return self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        except TransformException:
            return None

    @staticmethod
    def transform_point_2d(transform, x: float, y: float) -> Tuple[float, float]:
        yaw = yaw_from_quat(transform.transform.rotation)
        c = math.cos(yaw)
        s = math.sin(yaw)
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        return (c * x - s * y + tx, s * x + c * y + ty)

    @staticmethod
    def classify_base_point(point: Optional[Tuple[float, float]]) -> str:
        if point is None:
            return "unknown"
        x, y = point
        front_back = "front" if x > 0.20 else "rear" if x < -0.20 else "middle"
        left_right = "left" if y > 0.15 else "right" if y < -0.15 else "center"
        return f"{front_back}-{left_right}"

    def current_goal_info(self) -> Dict[str, Any]:
        goal = self.current_goal or {}
        waypoint_id = str(goal.get("waypoint_id", goal.get("id", "")))
        waypoint = self.waypoints_by_id.get(waypoint_id, {})
        position = goal.get("position") or waypoint.get("position") or []
        orientation = goal.get("orientation") or waypoint.get("orientation") or []
        name = goal.get("waypoint_name") or goal.get("name") or waypoint.get("name", "")
        frame_id = goal.get("frame_id") or waypoint.get("frame_id", "map")
        yaw = quat_list_to_yaw(orientation) if orientation else None
        return {
            "id": waypoint_id,
            "name": name,
            "position": position,
            "orientation_yaw": yaw,
            "frame_id": frame_id,
        }

    def emit_snapshot(self, reason: str, force: bool):
        now = time.time()
        if not force and now - self.last_snapshot_wall < 0.25:
            return
        self.last_snapshot_wall = now

        goal = self.current_goal_info()
        analysis = self.last_costmap_analysis or self.analyze_costmap()
        self.last_costmap_analysis = analysis

        lines = []
        lines.append("")
        lines.append("=" * 88)
        lines.append(f"NAV POSE ISSUE SNAPSHOT reason={reason} t={now:.3f} uptime={now - self.start_wall:.1f}s")
        lines.append(f"state={self.current_state or 'unknown'} event={self.current_event_type or 'none'}")

        if goal.get("position"):
            pos = goal["position"]
            yaw = goal.get("orientation_yaw")
            yaw_text = "unknown" if yaw is None else f"{yaw:.3f} rad"
            lines.append(
                f"goal[{goal.get('frame_id')}]: name={goal.get('name')} id={goal.get('id')} "
                f"pos=({float(pos[0]):.3f}, {float(pos[1]):.3f}, {float(pos[2]) if len(pos) > 2 else 0.0:.3f}) yaw={yaw_text}"
            )
        else:
            lines.append("goal: none observed yet")

        if self.robot_pose:
            age = now - self.robot_pose.stamp_wall
            lines.append(
                f"robot_realpose[{self.robot_pose.frame_id}]: "
                f"x={self.robot_pose.x:.3f} y={self.robot_pose.y:.3f} yaw={self.robot_pose.yaw:.3f} age={age:.2f}s"
            )
        else:
            lines.append("robot_realpose: missing")

        if self.status_pose:
            age = now - self.status_pose.stamp_wall
            lines.append(
                "navigation_status.current_pose[/odom source, not map]: "
                f"x={self.status_pose.x:.3f} y={self.status_pose.y:.3f} yaw={self.status_pose.yaw:.3f} age={age:.2f}s"
            )
        else:
            lines.append("navigation_status.current_pose: missing")

        self.append_distance_lines(lines, goal)
        self.append_motion_lines(lines)
        self.append_costmap_lines(lines, analysis)
        self.append_rosout_lines(lines, now)
        self.append_diagnosis_lines(lines, goal, analysis)
        lines.append("=" * 88)

        text = "\n".join(lines)
        self.get_logger().info(text)
        self.write_jsonl(reason, goal, analysis)

    def append_distance_lines(self, lines: List[str], goal: Dict[str, Any]):
        pos = goal.get("position") or []
        if len(pos) >= 2 and self.robot_pose and goal.get("frame_id", "map") == self.robot_pose.frame_id:
            d = math.hypot(self.robot_pose.x - float(pos[0]), self.robot_pose.y - float(pos[1]))
            lines.append(f"robot_realpose_to_goal_distance: {d:.3f} m")
        elif len(pos) >= 2 and self.robot_pose:
            lines.append(
                "robot_realpose_to_goal_distance: skipped because goal frame "
                f"{goal.get('frame_id')} != robot frame {self.robot_pose.frame_id}"
            )

        if len(pos) >= 2 and self.status_pose:
            d = math.hypot(self.status_pose.x - float(pos[0]), self.status_pose.y - float(pos[1]))
            lines.append(f"navigation_status_pose_to_goal_distance[/odom mixed with map goal]: {d:.3f} m")

        if self.robot_pose and self.status_pose:
            d = math.hypot(self.robot_pose.x - self.status_pose.x, self.robot_pose.y - self.status_pose.y)
            dyaw = normalize_angle(self.robot_pose.yaw - self.status_pose.yaw)
            lines.append(
                "robot_realpose_vs_navigation_status_pose_delta: "
                f"dxy={d:.3f} m dyaw={dyaw:.3f} rad"
            )

    def append_motion_lines(self, lines: List[str]):
        if self.last_cmd_vel:
            vx, wz, stamp = self.last_cmd_vel
            lines.append(f"cmd_vel: vx={vx:.3f} wz={wz:.3f} age={time.time() - stamp:.2f}s")
        if self.latest_plan:
            count = len(self.latest_plan.poses)
            if count > 0:
                first = self.latest_plan.poses[0].pose.position
                last = self.latest_plan.poses[-1].pose.position
                lines.append(
                    f"latest_plan[{self.latest_plan.header.frame_id}]: poses={count} "
                    f"start=({first.x:.3f},{first.y:.3f}) end=({last.x:.3f},{last.y:.3f})"
                )

    def append_costmap_lines(self, lines: List[str], analysis: CostmapAnalysis):
        if not analysis.available:
            lines.append(f"local_costmap_analysis: unavailable reason={analysis.reason}")
            return

        lines.append(
            f"local_costmap_analysis[{analysis.frame_id}]: footprint_frame={analysis.footprint_frame_id} "
            f"cells_inside={analysis.footprint_cells_checked} max_inside_cost={analysis.max_cost_inside_footprint} "
            f"inside_inflated={analysis.inflated_cells_inside_footprint} inside_lethal={analysis.lethal_cells_inside_footprint}"
        )
        if analysis.nearest_high_dist_m is not None:
            base_text = ""
            if analysis.nearest_high_point_base:
                bx, by = analysis.nearest_high_point_base
                base_text = f" base_point=(x={bx:.3f}, y={by:.3f}) sector={analysis.nearest_high_sector}"
            lines.append(
                f"nearest_high_cost_obstacle: dist_to_footprint={analysis.nearest_high_dist_m:.3f}m "
                f"cost={analysis.nearest_high_cost}{base_text}"
            )
        else:
            lines.append("nearest_high_cost_obstacle: none above threshold")

        if analysis.nearest_lethal_dist_m is not None:
            lines.append(
                f"nearest_lethal_obstacle: dist_to_footprint={analysis.nearest_lethal_dist_m:.3f}m "
                f"cost={analysis.nearest_lethal_cost}"
            )

    def append_rosout_lines(self, lines: List[str], now: float):
        if self.last_rosout_collision:
            stamp, text = self.last_rosout_collision
            lines.append(f"last_rpp_collision_log: age={now - stamp:.2f}s msg={text}")
        if self.last_rosout_controller_error:
            stamp, text = self.last_rosout_controller_error
            lines.append(f"last_controller_error_log: age={now - stamp:.2f}s msg={text}")

    def append_diagnosis_lines(self, lines: List[str], goal: Dict[str, Any], analysis: CostmapAnalysis):
        lines.append("diagnosis:")
        if goal.get("position"):
            lines.append("- Goal chain observed: waypoint target is available and is sent as a map-frame Nav2 goal.")
        else:
            lines.append("- Goal chain not yet observed; wait for waypoint_started/current_goal.")

        lines.append(
            "- navigation_status.current_pose is risky for map-distance checks because "
            "navigation_state_manager subscribes /odom, while goals are in map."
        )

        if analysis.available:
            if analysis.lethal_cells_inside_footprint > 0:
                lines.append("- CRITICAL: local costmap has lethal cells inside the published footprint.")
            elif analysis.inflated_cells_inside_footprint > 0:
                lines.append("- WARNING: local costmap has inflated/high-cost cells inside the published footprint.")
            elif analysis.nearest_high_dist_m is not None and analysis.nearest_high_dist_m <= self.near_obstacle_warn_dist:
                lines.append("- WARNING: local costmap obstacle is very close to the footprint.")

            if analysis.nearest_high_point_base:
                lines.append(
                    f"- Nearest high-cost obstacle is in robot base sector: {analysis.nearest_high_sector}. "
                    "front-left matches a left door-frame / front-wall stop pattern."
                )

        if self.last_rosout_collision:
            lines.append("- RPP collision log observed: Nav2 stopped because the local costmap predicted collision.")

        lines.append(
            "- If RViz/robot_realpose says the base is centered but the physical robot is left of that pose, "
            "the remaining cause is localization drift or base_footprint/extrinsic offset, not an App goal coordinate error."
        )

    def write_jsonl(self, reason: str, goal: Dict[str, Any], analysis: CostmapAnalysis):
        if not self.output_jsonl:
            return
        item = {
            "time": time.time(),
            "reason": reason,
            "state": self.current_state,
            "event": self.current_event_type,
            "goal": goal,
            "robot_realpose": self.robot_pose.__dict__ if self.robot_pose else None,
            "navigation_status_pose": self.status_pose.__dict__ if self.status_pose else None,
            "cmd_vel": self.last_cmd_vel,
            "costmap_analysis": analysis.__dict__ if analysis else None,
            "last_rosout_collision": self.last_rosout_collision,
            "last_rosout_controller_error": self.last_rosout_controller_error,
        }
        try:
            with open(self.output_jsonl, "a", encoding="utf-8") as f:
                f.write(json.dumps(item, ensure_ascii=False) + "\n")
        except Exception as exc:
            self.get_logger().warn(f"failed to write {self.output_jsonl}: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = NavPoseIssueMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
