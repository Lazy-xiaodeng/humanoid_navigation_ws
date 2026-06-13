#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""路线任务改造静态校验脚本。

这个脚本不依赖 ROS2 runtime，也不 import 项目节点类；
它只读取源码文本，检查路线任务首版的几个关键不变量是否还存在。
用途是给开工前/提交前多一道轻量护栏，防止后续修改把 through 主路径、
业务 ack 包装、旧命令拦截或 launch 入口无意删掉。
"""

from pathlib import Path
import sys


def resolve_workspace_root() -> Path:
    """定位源码工作区根目录。

    源码路径运行时，脚本位于 `src/humanoid_navigation2/scripts`；
    install 路径运行时，脚本位于 `install/.../share/humanoid_navigation2/scripts`。
    后者没有固定的 `parents[3] -> workspace` 关系，因此优先使用当前工作目录，
    再沿脚本路径向上寻找包含关键源码包的目录。
    """
    candidates = [Path.cwd()]
    candidates.extend(Path(__file__).resolve().parents)
    for candidate in candidates:
        if (
            (candidate / "src/humanoid_navigation/humanoid_navigation/navigation_state_manager.py").exists()
            and (candidate / "src/humanoid_navigation2/scripts/validate_route_task_static.py").exists()
        ):
            return candidate
    raise RuntimeError(
        "无法定位源码工作区根目录。请在 humanoid_ws 根目录运行，"
        "或使用源码路径 src/humanoid_navigation2/scripts/validate_route_task_static.py。"
    )


WORKSPACE_ROOT = resolve_workspace_root()


def read_text(relative_path: str) -> str:
    """按仓库相对路径读取 UTF-8 文本。"""
    return (WORKSPACE_ROOT / relative_path).read_text(encoding="utf-8")


def require_contains(text: str, needle: str, description: str, failures: list[str]) -> None:
    """检查某段关键文本是否存在，并在失败列表里记录中文原因。"""
    if needle not in text:
        failures.append(f"缺失: {description} -> {needle}")


def require_order(text: str, first: str, second: str, description: str, failures: list[str]) -> None:
    """检查两个关键调用的先后顺序，避免完成事件在清状态之后才发。"""
    first_index = text.find(first)
    second_index = text.find(second)
    if first_index < 0 or second_index < 0 or first_index >= second_index:
        failures.append(f"顺序错误: {description}")


def require_not_contains(text: str, needle: str, description: str, failures: list[str]) -> None:
    """检查某段风险文本不应存在。"""
    if needle in text:
        failures.append(f"不应存在: {description} -> {needle}")


def extract_function_body(text: str, function_name: str) -> str:
    """按函数名截取源码片段，用于做更精确的局部不变量检查。

    这里不解析 Python AST，是因为本脚本只做轻量文本护栏；
    按下一段同级 `    def ` 截断已经足够覆盖当前节点源码风格。
    """
    start = text.find(f"    def {function_name}(")
    if start < 0:
        return ""
    next_def = text.find("\n    def ", start + 1)
    if next_def < 0:
        return text[start:]
    return text[start:next_def]


def extract_between(text: str, start_marker: str, end_marker: str) -> str:
    """截取两个标记之间的源码片段，用于检查分支内部是否出现风险逻辑。"""
    start = text.find(start_marker)
    if start < 0:
        return ""
    end = text.find(end_marker, start + len(start_marker))
    if end < 0:
        return text[start:]
    return text[start:end]


def main() -> int:
    failures: list[str] = []

    navigation_state = read_text("src/humanoid_navigation/humanoid_navigation/navigation_state_manager.py")
    websocket_server = read_text("src/humanoid_websocket/humanoid_websocket/websocket_server.py")
    dynamic_waypoints = read_text("src/humanoid_navigation/humanoid_navigation/dynamic_waypoints_manager.py")
    data_integration = read_text("src/humanoid_websocket/humanoid_websocket/data_integration_node_recoverable.py")
    navigation_setup = read_text("src/humanoid_navigation/setup.py")
    navigation_package_xml = read_text("src/humanoid_navigation/package.xml")
    robot_real_launch = read_text("src/humanoid_bringup/launch/robot_real.launch.py")
    navigation_launch = read_text("src/humanoid_navigation/launch/navigation.launch.py")
    fusion_sc_launch = read_text("src/humanoid_navigation/launch/navigation_fusion_sc.launch.py")
    nav2_param_files = sorted((WORKSPACE_ROOT / "src/humanoid_navigation2/config").glob("nav2_params*.yaml"))
    nav2_param_texts = {str(path.relative_to(WORKSPACE_ROOT)): path.read_text(encoding="utf-8") for path in nav2_param_files}
    navigation2_launch = read_text("src/humanoid_navigation2/launch/navigation2.launch.py")
    navigation2_robosense_launch = read_text("src/humanoid_navigation2/launch/navigation2_robosense_lidar.launch.py")
    through_bt_xml = read_text("src/humanoid_navigation2/config/behavior_tree/navigate_through_poses_no_backup.xml")
    implementation_checklist = read_text("src/humanoid_navigation2/docs/路线任务改造函数级实施清单.md")
    data_integration_doc_section = extract_between(
        implementation_checklist,
        "### 6.9 `data_integration_node_recoverable.py`",
        "### 6.10",
    )
    bridge_route_task_validation_body = extract_between(
        dynamic_waypoints,
        'if command_type == "start_route_task":',
        "return True",
    )
    publish_event_body = extract_function_body(navigation_state, "publish_route_task_event")
    send_ack_body = extract_function_body(navigation_state, "send_route_task_ack")
    mark_transit_body = extract_function_body(navigation_state, "mark_transit_passed")
    handle_target_arrived_body = extract_function_body(navigation_state, "handle_target_task_arrived")
    finalize_task_body = extract_function_body(navigation_state, "finalize_task_waypoint_completion")
    complete_route_body = extract_function_body(navigation_state, "complete_route_task")
    route_task_failure_body = extract_function_body(navigation_state, "handle_route_task_navigation_failed")
    route_task_status_body = extract_function_body(navigation_state, "build_route_task_status_summary")
    reset_route_task_body = extract_function_body(navigation_state, "reset_route_task_state")
    normalize_route_waypoints_body = extract_function_body(navigation_state, "normalize_route_task_waypoints")
    validate_waypoint_source_body = extract_function_body(navigation_state, "validate_route_waypoint_source")
    validate_waypoints_revision_body = extract_function_body(navigation_state, "validate_waypoints_revision_for_id_mode")
    build_route_waypoints_from_ids_body = extract_function_body(navigation_state, "build_route_waypoints_from_ids")
    normalize_orientation_body = extract_function_body(navigation_state, "normalize_route_task_orientation")
    route_waypoint_pose_body = extract_function_body(navigation_state, "route_waypoint_to_pose_stamped")
    start_active_segment_body = extract_function_body(navigation_state, "start_active_segment_navigation")
    status_summary_body = extract_function_body(navigation_state, "get_current_status_summary")
    navigation_request_body = extract_function_body(navigation_state, "navigation_request_callback")
    navigation_command_body = extract_function_body(navigation_state, "handle_navigation_command")
    start_route_task_body = extract_function_body(navigation_state, "handle_start_route_task")
    pause_route_task_body = extract_function_body(navigation_state, "handle_pause_route_task")
    resume_route_task_body = extract_function_body(navigation_state, "handle_resume_route_task")
    stop_route_task_body = extract_function_body(navigation_state, "handle_stop_route_task")
    navigation_status_callback_body = extract_function_body(data_integration, "navigation_status_callback")
    maybe_navigation_exception_body = extract_function_body(data_integration, "maybe_publish_navigation_exception")
    jump_body = extract_function_body(navigation_state, "handle_jump_to_waypoint")
    broadcast_finished_body = extract_function_body(navigation_state, "handle_broadcast_finished")
    route_task_id_body = extract_function_body(navigation_state, "route_task_id")
    route_task_bool_body = extract_function_body(navigation_state, "route_task_bool")
    through_goal_response_body = extract_function_body(navigation_state, "route_task_through_goal_response_callback")
    through_feedback_body = extract_function_body(navigation_state, "route_task_through_feedback_callback")
    through_result_body = extract_function_body(navigation_state, "route_task_through_result_callback")
    resolve_feedback_body = extract_function_body(navigation_state, "resolve_transit_progress_from_feedback")
    resolve_pose_body = extract_function_body(navigation_state, "resolve_transit_progress_from_pose")
    obstacle_wait_body = extract_function_body(navigation_state, "enter_obstacle_wait_state")

    runtime_path_texts = {
        "src/humanoid_navigation2/launch/navigation2.launch.py": navigation2_launch,
        "src/humanoid_navigation2/launch/navigation2_robosense_lidar.launch.py": navigation2_robosense_launch,
        "src/humanoid_navigation2/config/behavior_tree/navigate_through_poses_no_backup.xml": through_bt_xml,
        "src/humanoid_navigation/humanoid_navigation/navigation_state_manager.py": navigation_state,
        "src/humanoid_navigation/humanoid_navigation/dynamic_waypoints_manager.py": dynamic_waypoints,
        "src/humanoid_websocket/humanoid_websocket/websocket_server.py": websocket_server,
        "src/humanoid_websocket/humanoid_websocket/data_integration_node_recoverable.py": data_integration,
        "src/humanoid_navigation/launch/navigation.launch.py": navigation_launch,
        "src/humanoid_navigation/launch/navigation_fusion_sc.launch.py": fusion_sc_launch,
        "src/humanoid_bringup/launch/robot_real.launch.py": robot_real_launch,
    }
    runtime_path_texts.update(nav2_param_texts)
    for runtime_path, runtime_text in runtime_path_texts.items():
        require_not_contains(
            runtime_text,
            "/home/ubuntu/humanoid_ws",
            f"{runtime_path} 不应引用主工作区绝对路径，Todesk 验证必须路径隔离",
            failures,
        )

    # route task 运动策略：transit 段用 NavigateThroughPoses，最终 task 用 NavigateToPose 收尾对齐。
    require_contains(navigation_state, "NavigateThroughPoses", "导入/使用 NavigateThroughPoses", failures)
    require_contains(navigation_state, "NavigateToPose", "导入/使用 NavigateToPose 做最终 task 对齐", failures)
    require_contains(navigation_state, "self.nav_through_poses_client", "创建 through action client", failures)
    require_contains(navigation_state, "self.nav_to_pose_client", "创建最终 task NavigateToPose action client", failures)
    require_contains(navigation_state, "NavigateThroughPoses.Goal()", "构造 through goal", failures)
    require_contains(navigation_state, "NavigateToPose.Goal()", "构造最终 task NavigateToPose goal", failures)
    require_contains(navigation_state, "goal_msg.poses = poses", "一次性下发当前段 poses", failures)
    for through_launch_name, through_launch_text in (
        ("navigation2.launch.py", navigation2_launch),
        ("navigation2_robosense_lidar.launch.py", navigation2_robosense_launch),
    ):
        require_contains(
            through_launch_text,
            "default_through_bt_xml_file",
            f"{through_launch_name} 声明 through 专用 BT 默认文件",
            failures,
        )
        require_contains(
            through_launch_text,
            "through_bt_xml_file = LaunchConfiguration('through_bt_xml_file'",
            f"{through_launch_name} 暴露 through_bt_xml_file 参数",
            failures,
        )
        require_contains(
            through_launch_text,
            "'default_nav_through_poses_bt_xml': through_bt_xml_file",
            f"{through_launch_name} through action 使用 through 专用 BT",
            failures,
        )
        require_not_contains(
            through_launch_text,
            "'default_nav_through_poses_bt_xml': bt_xml_file",
            f"{through_launch_name} through action 不得复用单点 bt_xml_file",
            failures,
        )
    for needle, description in (
        ('goals="{goals}"', "through BT 使用 goals 黑板"),
        ("ComputePathThroughPoses", "through BT 使用 ComputePathThroughPoses"),
        ("RemovePassedGoals", "through BT 自动移除已通过目标"),
        ('goal_checker_id="xy_goal_checker"', "through BT FollowPath 显式使用 xy_goal_checker"),
    ):
        require_contains(through_bt_xml, needle, description, failures)
    require_not_contains(through_bt_xml, "ComputePathToPose", "through BT 不得使用单点 ComputePathToPose", failures)
    require_not_contains(through_bt_xml, 'goal="{goal}"', "through BT 不得使用单点 goal 黑板", failures)
    if not nav2_param_texts:
        failures.append("缺失: Nav2 参数文件 -> src/humanoid_navigation2/config/nav2_params*.yaml")
    for nav2_param_path, nav2_param_text in nav2_param_texts.items():
        require_contains(
            nav2_param_text,
            'navigators: ["navigate_to_pose", "navigate_through_poses"]',
            f"{nav2_param_path} 注册 NavigateThroughPoses navigator",
            failures,
        )
        require_contains(
            nav2_param_text,
            'plugin: "nav2_bt_navigator::NavigateToPoseNavigator"',
            f"{nav2_param_path} 注册 NavigateToPoseNavigator plugin",
            failures,
        )
        require_contains(
            nav2_param_text,
            'plugin: "nav2_bt_navigator::NavigateThroughPosesNavigator"',
            f"{nav2_param_path} 注册 NavigateThroughPosesNavigator plugin",
            failures,
        )
    require_contains(navigation_package_xml, "<depend>nav2_msgs</depend>", "package.xml 声明 nav2_msgs 依赖", failures)
    require_contains(navigation_package_xml, "<depend>action_msgs</depend>", "package.xml 声明 action_msgs 依赖", failures)
    for command_type, handler in (
        ('"start_route_task"', "self.handle_start_route_task(command_data, request_data)"),
        ('"pause_route_task"', "self.handle_pause_route_task(command_data, request_data)"),
        ('"resume_route_task"', "self.handle_resume_route_task(command_data, request_data)"),
        ('"stop_route_task"', "self.handle_stop_route_task(command_data, request_data)"),
        ('"jump_to_waypoint"', "self.handle_jump_to_waypoint(command_data, request_data)"),
        ('"broadcast_finished"', "self.handle_broadcast_finished(command_data, request_data)"),
    ):
        require_contains(navigation_command_body, command_type, f"状态机分发 route task 命令 {command_type}", failures)
        require_contains(navigation_command_body, handler, f"状态机调用 route task handler {handler}", failures)
    require_contains(
        navigation_command_body,
        "route_task_command_types = {",
        "状态机定义 route task 命令集合用于异常兜底",
        failures,
    )
    for route_task_control in (
        '"pause_route_task"',
        '"resume_route_task"',
        '"stop_route_task"',
    ):
        require_contains(
            navigation_command_body,
            route_task_control,
            f"route task 控制命令进入异常兜底集合 {route_task_control}",
            failures,
        )
    require_contains(
        navigation_command_body,
        "if command_type in route_task_command_types:",
        "route task 命令异常时进入专属业务 ack 兜底",
        failures,
    )
    require_contains(
        navigation_command_body,
        'self.send_route_task_ack(command_type, "error",',
        "route task 命令异常兜底发送 navigation_command_result",
        failures,
    )
    require_contains(
        navigation_command_body,
        'error_code="internal_error"',
        "route task 命令异常兜底使用 internal_error",
        failures,
    )
    require_contains(
        navigation_command_body,
        '"unknown_navigation_command"',
        "未知导航命令必须返回标准 status=error 的旧 ack",
        failures,
    )
    require_order(
        navigation_command_body,
        '"unknown_navigation_command"',
        '"error_code": "unknown_navigation_command"',
        "未知导航命令旧 ack 必须先标识 ack_type 再携带 error_code",
        failures,
    )
    require_contains(
        navigation_command_body,
        '"error_code": "unknown_navigation_command"',
        "未知导航命令旧 ack 必须携带 error_code 方便 APP 展示",
        failures,
    )
    require_contains(
        navigation_command_body,
        'command_type or "navigation_command"',
        "非 route task 未预期异常必须返回标准 status=error 的旧 ack",
        failures,
    )
    require_contains(
        navigation_command_body,
        '"error_code": "internal_error"',
        "非 route task 未预期异常旧 ack 必须携带 internal_error",
        failures,
    )
    require_contains(
        navigation_request_body,
        "request_data: Dict[str, Any] = {}",
        "navigation_request_callback 必须先初始化 request_data，避免 JSON 解析失败后 except 二次崩溃",
        failures,
    )
    require_contains(
        navigation_request_body,
        "request_type = \"\"",
        "navigation_request_callback 必须先初始化 request_type，确保错误 ack 可带稳定 request_type",
        failures,
    )
    require_contains(
        navigation_request_body,
        "if not isinstance(request_data, dict):",
        "navigation_request_callback 必须识别非对象 JSON payload，避免把 APP 坏包误报 internal_error",
        failures,
    )
    require_contains(
        navigation_request_body,
        '"error_code": "invalid_request_payload"',
        "非对象 JSON payload 必须返回 invalid_request_payload",
        failures,
    )
    require_not_contains(
        navigation_state,
        'send_acknowledgment("error"',
        "send_acknowledgment 第二个参数才是 status，禁止再用 ack_type=error 的旧错误写法",
        failures,
    )
    require_contains(
        start_route_task_body,
        'task_session_id = self.route_task_id(command_data.get("task_session_id"))',
        "start_route_task 使用 route_task_id 归一化 task_session_id",
        failures,
    )
    require_contains(
        start_route_task_body,
        'route_id = self.route_task_id(command_data.get("route_id"))',
        "start_route_task 使用 route_task_id 归一化 route_id",
        failures,
    )
    require_contains(
        start_route_task_body,
        'request_message_id = self.route_task_id(command_data.get("request_message_id"))',
        "start_route_task 使用 route_task_id 归一化 request_message_id",
        failures,
    )
    require_contains(
        start_route_task_body,
        '"task_session_id": task_session_id',
        "active_route_task 保存归一化后的 task_session_id",
        failures,
    )
    require_contains(
        start_route_task_body,
        '"route_id": route_id',
        "active_route_task 保存归一化后的 route_id",
        failures,
    )
    require_contains(
        start_route_task_body,
        '"request_message_id": request_message_id',
        "active_route_task 保存归一化后的 request_message_id",
        failures,
    )
    for route_id_mode_needle, route_id_mode_description in (
        ('"route_waypoint_ids": command_data.get("route_waypoint_ids", [])', "websocket 入口透传 route_waypoint_ids"),
        ('"waypoints_revision": command_data.get("waypoints_revision", "")', "websocket 入口透传 waypoints_revision"),
        ('"map_id": command_data.get("map_id", "")', "websocket 入口透传 map_id"),
        ("route_waypoint_ids = normalized.get(\"route_waypoint_ids\")", "桥接层归一化 route_waypoint_ids"),
        ("\"waypoints_revision\",", "桥接层归一化 waypoints_revision"),
        ("self.current_waypoints_revision = \"\"", "状态管理器缓存点位库 revision"),
        ("self.waypoints_data_by_map = {}", "状态管理器缓存多地图点位库"),
        ("self.current_waypoints_revisions_by_map = {}", "状态管理器缓存多地图 revision"),
        ("self.extract_waypoints_revision(message_data, legacy_data)", "点位数据回调提取 waypoints_revision"),
        ("self.extract_waypoints_revisions_by_map(message_data, legacy_data)", "点位数据回调提取多地图 revision"),
        ("def validate_route_waypoint_source", "状态机新增完整快照/ID 列表来源互斥校验"),
        ("def validate_waypoints_revision_for_id_mode", "状态机新增 ID 模式 revision 校验"),
        ("def build_route_waypoints_from_ids", "状态机新增 ID 列表补全完整点位函数"),
    ):
        target_text = "\n".join((websocket_server, dynamic_waypoints, navigation_state))
        require_contains(target_text, route_id_mode_needle, route_id_mode_description, failures)
    for source_needle, source_description in (
        ("ambiguous_route_waypoint_source", "同时传完整快照和 ID 列表必须返回 ambiguous_route_waypoint_source"),
        ("stored_waypoint_ids", "ID 列表模式来源标记 stored_waypoint_ids"),
        ("inline_route_waypoints", "完整快照模式来源标记 inline_route_waypoints"),
        ("invalid_route_waypoint_ids", "空 ID 列表必须返回 invalid_route_waypoint_ids"),
    ):
        require_contains(validate_waypoint_source_body, source_needle, source_description, failures)
    for revision_needle, revision_description in (
        ("missing_map_id", "ID 模式缺 map_id 必须返回 missing_map_id"),
        ("missing_waypoints_revision", "ID 模式缺 revision 必须返回 missing_waypoints_revision"),
        ("waypoints_cache_not_ready", "点位缓存未就绪必须返回 waypoints_cache_not_ready"),
        ("waypoints_revision_mismatch", "revision 不一致必须返回 waypoints_revision_mismatch"),
        ("self.current_waypoints_revisions_by_map.get(normalized_map_id", "revision 必须按 map_id 校验"),
    ):
        require_contains(validate_waypoints_revision_body, revision_needle, revision_description, failures)
    for id_build_needle, id_build_description in (
        ("self.find_waypoint_data_by_id(waypoint_id, normalized_map_id)", "ID 模式必须按 map_id 从状态机点位缓存查完整点位"),
        ("waypoint_id_not_found", "ID 不存在必须返回 waypoint_id_not_found"),
        ("raw_stored_waypoint", "ID 补全后保留原始点位快照用于排查"),
    ):
        require_contains(build_route_waypoints_from_ids_body, id_build_needle, id_build_description, failures)
    require_contains(
        start_route_task_body,
        "self.validate_route_waypoint_source(command_data)",
        "start_route_task 必须先判断完整快照/ID 列表来源",
        failures,
    )
    require_contains(
        start_route_task_body,
        "self.validate_waypoints_revision_for_id_mode(command_data, map_id)",
        "start_route_task 的 ID 模式必须按 map_id 校验 waypoints_revision",
        failures,
    )
    require_contains(
        start_route_task_body,
        "self.build_route_waypoints_from_ids(",
        "start_route_task 的 ID 模式必须补全为完整 route_waypoints",
        failures,
    )
    require_contains(
        start_route_task_body,
        '"route_waypoint_source": route_waypoint_source',
        "active_route_task 必须记录路线点来源",
        failures,
    )
    require_contains(
        start_route_task_body,
        '"map_id": map_id',
        "active_route_task 必须记录启动时的 map_id",
        failures,
    )
    require_contains(
        start_route_task_body,
        '"waypoints_revision": self.route_task_id(command_data.get("waypoints_revision"))',
        "active_route_task 必须记录启动时的 waypoints_revision",
        failures,
    )
    require_contains(
        publish_event_body,
        "APP 应按 data.event_type 消费",
        "route task 事件发布函数必须用中文注释说明 APP 的消费入口",
        failures,
    )
    require_contains(
        publish_event_body,
        "event_id 用于 APP 去重、埋点和问题复盘",
        "route task 事件发布函数必须用中文注释说明 event_id 的 APP 用途",
        failures,
    )
    # 中文注释：外层 publish_status_update() 虽然有 timestamp，但 APP 侧常会只消费 event_data。
    # route task 离散事件的数据体也必须自带 timestamp，便于前端按事件体排序、去重和复盘。
    require_contains(
        publish_event_body,
        'payload.setdefault("timestamp", time.time())',
        "route task 事件 event_data 必须统一补 timestamp",
        failures,
    )
    require_contains(
        send_ack_body,
        "APP 收到 status=success",
        "route task 业务 ack 函数必须说明 APP 收到成功 ack 后的动作",
        failures,
    )
    require_contains(
        send_ack_body,
        "APP 收到 status=error",
        "route task 业务 ack 函数必须说明 APP 收到错误 ack 后的动作",
        failures,
    )
    require_contains(
        complete_route_body,
        "APP 收到 route_task_completed 后",
        "route task 完成函数必须说明 APP 收到完成事件后的动作",
        failures,
    )
    require_contains(
        handle_target_arrived_body,
        "APP 收到 broadcast_requested 后",
        "broadcast_requested 发布处必须说明 APP 收到播报请求后的动作",
        failures,
    )
    require_contains(
        mark_transit_body,
        "APP 收到 waypoint_passed 后",
        "waypoint_passed 发布处必须说明 APP 收到 transit 进度后的动作",
        failures,
    )
    require_contains(
        finalize_task_body,
        "APP 收到 task_waypoint_completed 后",
        "task_waypoint_completed 发布处必须说明 APP 收到 task 完成后的动作",
        failures,
    )
    require_contains(
        jump_body,
        "APP 收到 jump_updated 后",
        "jump_updated 发布处必须说明 APP 收到跳点更新后的动作",
        failures,
    )
    require_contains(
        broadcast_finished_body,
        "APP 收到 broadcast_finished success ack 后",
        "broadcast_finished 成功 ack 处必须说明 APP 后续等待的异步事件",
        failures,
    )
    require_contains(
        route_task_failure_body,
        "APP 收到 navigation_failed(route_task=true) 后",
        "route task 失败事件发布处必须说明 APP 展示和复盘动作",
        failures,
    )
    require_contains(
        route_task_status_body,
        "APP 读取周期 route_task 摘要时",
        "周期 route task 摘要必须说明 APP 如何消费当前态字段",
        failures,
    )
    require_contains(
        route_task_id_body,
        "缺失 ID 返回空字符串",
        "route_task_id helper 必须说明缺失 ID 的处理语义",
        failures,
    )
    require_contains(
        route_task_bool_body,
        "无法识别的布尔值回退默认值",
        "route_task_bool helper 必须说明无法识别输入的默认值语义",
        failures,
    )

    # 回调隔离：旧 session / 旧 goal 回调必须同时被 version 和 generation 拦住。
    require_contains(navigation_state, "route_task_version != self.route_task_version", "through 回调绑定 route_task_version", failures)
    require_contains(navigation_state, "generation != self.current_route_task_goal_generation", "through 回调绑定 generation", failures)
    for callback_name, callback_body in (
        ("goal response", through_goal_response_body),
        ("feedback", through_feedback_body),
        ("result", through_result_body),
    ):
        require_contains(callback_body, "route_task_version != self.route_task_version", f"through {callback_name} 回调检查 route_task_version", failures)
        require_contains(callback_body, "generation != self.current_route_task_goal_generation", f"through {callback_name} 回调检查 generation", failures)
    require_contains(
        through_goal_response_body,
        "self.route_task_last_feedback_time = time.time()",
        "through goal accepted 后刷新 feedback 超时计时",
        failures,
    )
    require_contains(
        start_active_segment_body,
        "try:",
        "through send_goal_async 必须捕获发送异常",
        failures,
    )
    require_contains(
        start_active_segment_body,
        "send_goal_future = self.nav_through_poses_client.send_goal_async",
        "through 段启动调用 send_goal_async",
        failures,
    )
    require_contains(
        start_active_segment_body,
        'f"NavigateThroughPoses send goal failed: {exc}"',
        "through send_goal_async 异常带失败原因",
        failures,
    )
    require_contains(
        start_active_segment_body,
        'failure_code="send_goal_failed"',
        "through send_goal_async 异常进入 route task 失败流程并带 failure_code",
        failures,
    )
    require_contains(
        start_active_segment_body,
        "if send_failure_ack:",
        "命令触发的 through send_goal_async 异常必须先返回 navigation_command_result(error)，避免 APP 业务 ack 等待超时",
        failures,
    )
    require_contains(
        start_active_segment_body,
        "self.send_route_task_ack(",
        "命令触发的 through send_goal_async 异常调用 send_route_task_ack",
        failures,
    )
    require_order(
        start_active_segment_body,
        "self.send_route_task_ack(",
        "self.handle_route_task_navigation_failed(",
        "through send_goal_async 异常应先发业务 ack，再发 navigation_failed 复盘",
        failures,
    )
    require_contains(
        start_active_segment_body,
        'error_code="send_goal_failed"',
        "命令触发的 through send_goal_async 异常业务 ack 使用 send_goal_failed",
        failures,
    )
    require_contains(
        start_active_segment_body,
        "return False",
        "through send_goal_async 异常后返回 False",
        failures,
    )
    require_contains(
        through_result_body,
        "self.clear_route_task_goal_after_terminal_result()",
        "through terminal result 后清理 route task goal handle",
        failures,
    )
    require_contains(
        navigation_state,
        "def clear_route_task_goal_after_terminal_result",
        "route task terminal result 清理 goal handle helper",
        failures,
    )

    # 当前段 transit：passed_transit_waypoint_ids 必须保存在 active_segment 内，而不是全局去重表。
    require_contains(navigation_state, "passed_transit_waypoint_ids", "当前段 transit passed 记录", failures)
    require_contains(navigation_state, "resolve_transit_progress_from_pose", "pose + 阈值 fallback", failures)
    require_contains(navigation_state, "def resolve_current_progress_anchor_task_id", "jump 当前进度锚点 task 解析", failures)
    require_contains(
        navigation_state,
        "progress_anchor_task_id = self.resolve_current_progress_anchor_task_id()",
        "jump 重建段时使用当前进度锚点，而不是盲目使用旧目标 task",
        failures,
    )
    require_contains(
        jump_body,
        'self.current_anchor_task_id = self.active_segment.get("segment_start_task_id", "")',
        "jump 成功后 current_anchor_task_id 来自新段起点快照",
        failures,
    )
    require_contains(
        jump_body,
        "will_complete_without_navigation = self.should_complete_active_segment_without_navigation()",
        "jump 必须识别无需 Nav2 的快捷完成分支，避免同步完成后 active_segment 被 reset",
        failures,
    )
    require_contains(
        jump_body,
        "if will_complete_without_navigation:",
        "jump 快捷完成分支必须单独处理 jump_updated/ack 时序",
        failures,
    )
    require_contains(
        jump_body,
        "甚至可能一路推进到 route_task_completed 并 reset active_segment",
        "jump 快捷完成分支注释必须说明为什么要先发 jump_updated 和 ack",
        failures,
    )
    require_contains(
        jump_body,
        "if will_complete_without_navigation:\n            return",
        "jump 快捷完成分支启动后必须直接返回，避免 route reset 后再次读取 active_segment",
        failures,
    )
    require_contains(
        jump_body,
        "interrupted_waypoint_id = self.waiting_broadcast_waypoint_id",
        "jump 打断等待播报时必须记录被中断 task",
        failures,
    )
    require_contains(
        jump_body,
        "self.skipped_task_ids.append(interrupted_waypoint_id)",
        "jump 打断等待播报 task 后必须计入 skipped，保证最终统计闭合",
        failures,
    )
    require_contains(
        jump_body,
        "if target_waypoint_id in self.skipped_task_ids:",
        "jump 目标点重新成为 active target 时必须从 skipped_task_ids 移除",
        failures,
    )
    require_contains(
        jump_body,
        "if skipped_task_id != target_waypoint_id",
        "jump 目标点移出 skipped 时只能移除当前目标",
        failures,
    )
    require_contains(
        jump_body,
        "if self.active_route_task is not None:\n                self.handle_route_task_navigation_failed(",
        "jump 新段启动失败时只在内部尚未清理 route task 的情况下补发失败，避免 send_goal 异常导致双份 navigation_failed",
        failures,
    )
    require_contains(resolve_feedback_body, "self.mark_transit_passed", "through feedback 只标记 transit passed", failures)
    require_contains(resolve_pose_body, "self.mark_transit_passed", "pose fallback 只标记 transit passed", failures)
    require_not_contains(resolve_feedback_body, "self.handle_target_task_arrived", "through feedback 不直接完成 task", failures)
    require_not_contains(resolve_pose_body, "self.handle_target_task_arrived", "pose fallback 不直接完成 task", failures)
    require_not_contains(resolve_feedback_body, "self.finalize_task_waypoint_completion", "through feedback 不推进 task completed", failures)
    require_not_contains(resolve_pose_body, "self.finalize_task_waypoint_completion", "pose fallback 不推进 task completed", failures)
    require_contains(navigation_state, '"interrupted_broadcast": interrupted_broadcast', "jump_updated 打断播报上下文字段", failures)
    require_contains(navigation_state, "def route_task_bool", "route task 布尔字段安全解析", failures)
    require_contains(navigation_state, "def route_task_id", "route task ID 字段安全归一化 helper", failures)
    require_contains(navigation_state, "def normalize_route_task_frame_id", "route task frame_id 归一化 helper", failures)
    require_contains(navigation_state, "def normalize_route_task_position", "route task position 归一化 helper", failures)
    require_contains(navigation_state, "def normalize_route_task_orientation", "route task orientation 归一化 helper", failures)
    require_contains(navigation_state, "def current_navigation_mode_value", "route task 状态发布 navigation_mode 统一解析", failures)
    require_contains(navigation_state, "def build_route_task_status_summary", "周期状态 route task 上下文摘要", failures)
    require_contains(
        broadcast_finished_body,
        'raw_broadcast_result = command_data.get("broadcast_result", "completed")',
        "broadcast_finished.broadcast_result 先读取原始值以区分缺失和空字符串",
        failures,
    )
    require_contains(
        broadcast_finished_body,
        "if raw_broadcast_result is None:",
        "broadcast_finished.broadcast_result 仅 None 按默认 completed 处理",
        failures,
    )
    require_contains(
        broadcast_finished_body,
        'broadcast_result = str(raw_broadcast_result).strip().lower()',
        "broadcast_finished.broadcast_result 字符串化并做 strip/lower 归一化",
        failures,
    )
    require_contains(
        status_summary_body,
        'status_summary["route_task"] = self.build_route_task_status_summary()',
        "周期状态摘要挂载 route_task 上下文",
        failures,
    )
    for field in (
        '"task_session_id"',
        '"route_id"',
        '"current_anchor_task_id"',
        '"current_target_task_id"',
        '"current_target_task_index"',
        '"master_route_task_ids"',
        '"completed_task_ids"',
        '"skipped_task_ids"',
        '"awaiting_broadcast"',
        '"waiting_broadcast_waypoint_id"',
        '"waiting_broadcast_id"',
        '"active_segment"',
    ):
        require_contains(route_task_status_body, field, f"route_task 周期状态字段 {field}", failures)
    require_contains(route_task_status_body, '"master_route_task_ids": list(self.master_route_task_ids)', "route_task 周期状态拷贝 master_route_task_ids", failures)
    require_contains(route_task_status_body, '"completed_task_ids": list(self.completed_task_ids)', "route_task 周期状态拷贝 completed_task_ids", failures)
    require_contains(route_task_status_body, '"skipped_task_ids": list(self.skipped_task_ids)', "route_task 周期状态拷贝 skipped_task_ids", failures)
    require_contains(route_task_status_body, "dict(self.active_segment) if self.active_segment else None", "route_task 周期状态 active_segment 快照", failures)
    require_contains(
        navigation_state,
        'return "route_task"',
        "active route task 状态发布 navigation_mode=route_task",
        failures,
    )
    require_contains(
        navigation_state,
        '"navigation_mode": self.current_navigation_mode_value()',
        "状态 payload 使用 current_navigation_mode_value()",
        failures,
    )
    require_contains(navigation_state, "interrupt_broadcast = self.route_task_bool", "jump.interrupt_broadcast 安全布尔解析", failures)
    require_contains(navigation_state, "self.route_task_last_feedback_time", "route task through feedback 时间戳", failures)
    require_contains(navigation_state, "def check_route_task_feedback_timeout", "route task through feedback 超时检查", failures)
    require_contains(
        navigation_state,
        "self.check_route_task_feedback_timeout()",
        "周期检查中调用 route task feedback 超时检查",
        failures,
    )
    require_contains(
        navigation_state,
        "time.time() - self.route_task_last_feedback_time",
        "route task feedback 超时按最后 feedback 时间计算",
        failures,
    )
    require_contains(
        navigation_state,
        '"NavigateThroughPoses feedback timeout"',
        "route task feedback 超时带失败原因",
        failures,
    )
    require_contains(
        navigation_state,
        'failure_code="feedback_timeout"',
        "route task feedback 超时进入 route task 失败流程并带 failure_code",
        failures,
    )
    require_not_contains(
        navigation_state,
        'interrupt_broadcast = bool(command_data.get("interrupt_broadcast"',
        "jump.interrupt_broadcast 不应使用 bool('false') 误判逻辑",
        failures,
    )
    require_contains(navigation_state, 'properties.get("need_broadcast", False)', "task.need_broadcast properties fallback", failures)
    require_contains(navigation_state, 'properties.get("broadcast_id", "")', "task.broadcast_id properties fallback", failures)
    require_contains(navigation_state, 'properties.get("broadcast_blocking", True)', "task.broadcast_blocking 默认 true", failures)
    require_contains(navigation_state, 'properties.get("stop_and_align", True)', "task.stop_and_align 默认 true", failures)
    require_contains(
        normalize_route_waypoints_body,
        'waypoint_id = self.route_task_id(waypoint.get("waypoint_id"))',
        "route_waypoints[].waypoint_id 使用 route_task_id 归一化",
        failures,
    )
    require_contains(
        normalize_route_waypoints_body,
        "seen_waypoint_ids = set()",
        "route_waypoints 归一化必须维护 seen_waypoint_ids，拒绝重复 waypoint_id",
        failures,
    )
    require_contains(
        normalize_route_waypoints_body,
        "if waypoint_id in seen_waypoint_ids:",
        "route_waypoints 归一化必须检测重复 waypoint_id",
        failures,
    )
    require_contains(
        normalize_route_waypoints_body,
        '"duplicate_waypoint_id"',
        "route_waypoints 重复 waypoint_id 必须返回 duplicate_waypoint_id",
        failures,
    )
    require_contains(
        normalize_route_waypoints_body,
        "seen_waypoint_ids.add(waypoint_id)",
        "route_waypoints 归一化必须记录已出现 waypoint_id",
        failures,
    )
    require_contains(
        normalize_route_waypoints_body,
        'normalized["waypoint_id"] = waypoint_id',
        "route_waypoints[] 保存归一化后的 waypoint_id",
        failures,
    )
    require_contains(
        normalize_route_waypoints_body,
        'normalized["broadcast_id"] = self.route_task_id(task_broadcast_id)',
        "task.broadcast_id 使用 route_task_id 归一化",
        failures,
    )
    require_contains(
        normalize_route_waypoints_body,
        'if normalized["need_broadcast"] and not normalized["broadcast_id"]:',
        "task need_broadcast=true 时必须拒绝空 broadcast_id",
        failures,
    )
    require_contains(
        normalize_route_waypoints_body,
        '"missing_broadcast_id"',
        "task need_broadcast=true 且 broadcast_id 为空必须返回 missing_broadcast_id",
        failures,
    )
    require_order(
        normalize_route_waypoints_body,
        'normalized["broadcast_id"] = self.route_task_id(task_broadcast_id)',
        'if normalized["need_broadcast"] and not normalized["broadcast_id"]:',
        "broadcast_id 必须先归一化再做必填判断",
        failures,
    )
    require_contains(
        normalize_route_waypoints_body,
        'frame_id = self.normalize_route_task_frame_id(waypoint.get("frame_id", properties.get("frame_id", "")))',
        "route_waypoints[].frame_id 在归一化阶段处理",
        failures,
    )
    require_contains(
        normalize_route_waypoints_body,
        'position, position_error = self.normalize_route_task_position(waypoint.get("position"))',
        "route_waypoints[].position 在归一化阶段校验",
        failures,
    )
    require_contains(
        normalize_route_waypoints_body,
        'orientation, orientation_error = self.normalize_route_task_orientation(waypoint.get("orientation"))',
        "route_waypoints[].orientation 在归一化阶段校验",
        failures,
    )
    require_contains(
        normalize_orientation_body,
        "norm = math.sqrt",
        "orientation 归一化必须计算四元数范数",
        failures,
    )
    require_contains(
        normalize_orientation_body,
        "if norm <= 1e-6:",
        "orientation 归一化必须拒绝零长度四元数",
        failures,
    )
    require_contains(
        normalize_orientation_body,
        "orientation quaternion norm must be greater than zero",
        "orientation 零长度四元数必须返回明确错误",
        failures,
    )
    require_contains(
        normalize_orientation_body,
        "orientation = [component / norm for component in orientation]",
        "orientation 归一化必须输出单位四元数",
        failures,
    )
    require_contains(
        normalize_route_waypoints_body,
        'normalized["frame_id"] = frame_id',
        "route_waypoints[] 保存归一化后的 frame_id",
        failures,
    )
    require_contains(
        normalize_route_waypoints_body,
        'normalized["position"] = position',
        "route_waypoints[] 保存归一化后的 position",
        failures,
    )
    require_contains(
        normalize_route_waypoints_body,
        'normalized["orientation"] = orientation',
        "route_waypoints[] 保存归一化后的 orientation",
        failures,
    )
    require_not_contains(
        route_waypoint_pose_body,
        "self.waypoint_to_pose_stamped",
        "route task 专用 PoseStamped 转换不应复用旧普通导航默认转换",
        failures,
    )
    for pose_assignment in (
        "pose = PoseStamped()",
        'pose.header.frame_id = waypoint.get("frame_id", self.default_frame_id)',
        'position = waypoint.get("position", [])',
        'orientation = waypoint.get("orientation", [])',
        "pose.pose.position.x = float(position[0])",
        "pose.pose.orientation.w = float(orientation[3])",
    ):
        require_contains(
            route_waypoint_pose_body,
            pose_assignment,
            f"route task PoseStamped 转换使用归一化字段 {pose_assignment}",
            failures,
        )
    require_contains(
        navigation_state,
        'str(raw_waypoint_role or "").strip().lower()',
        "waypoint_role 字符串化并做 strip/lower 归一化",
        failures,
    )
    require_contains(navigation_state, 'normalized["raw_payload"] = dict(waypoint)', "保留 route waypoint 原始 payload", failures)
    require_contains(
        reset_route_task_body,
        "self.current_route_task_goal_generation = -1",
        "reset_route_task_state 必须让旧 through goal response/feedback/result generation 失效",
        failures,
    )
    require_contains(
        reset_route_task_body,
        "self.active_goal_generation = 0",
        "reset_route_task_state 必须清零 route task goal generation 计数，避免状态摘要残留旧段计数",
        failures,
    )
    require_contains(navigation_state, "send_failure_ack: bool = True", "段启动失败 ack 开关", failures)
    require_contains(navigation_state, "def reject_active_segment_start", "段启动失败统一处理", failures)
    require_contains(navigation_state, "def cleanup_route_task_segment_start_failure", "命令触发的段启动失败状态清理 helper", failures)
    final_pose_body = extract_function_body(navigation_state, "start_active_segment_final_pose_navigation")
    through_result_body = extract_function_body(navigation_state, "route_task_through_result_callback")
    complete_without_navigation_body = extract_function_body(navigation_state, "complete_active_segment_without_navigation")
    require_contains(
        navigation_state,
        "def start_active_segment_final_pose_navigation",
        "route task 最终 task NavigateToPose 收尾 helper",
        failures,
    )
    require_contains(
        start_active_segment_body,
        'if not self.active_segment.get("transit_waypoint_ids", []):',
        "无 transit 段必须直接走最终 task NavigateToPose",
        failures,
    )
    require_contains(
        start_active_segment_body,
        "self.start_active_segment_final_pose_navigation(",
        "无 transit 段调用最终 task 对齐 helper",
        failures,
    )
    require_contains(
        through_result_body,
        "self.start_active_segment_final_pose_navigation(",
        "through 成功后必须进入最终 task 对齐，而不是直接播报",
        failures,
    )
    require_contains(
        complete_without_navigation_body,
        "self.start_active_segment_final_pose_navigation(",
        "近距离快捷完成也必须进入最终 task 对齐",
        failures,
    )
    require_contains(
        final_pose_body,
        "goal_msg.behavior_tree = self.reverse_navigation_bt_xml",
        "倒走 task 必须设置 reverse BT",
        failures,
    )
    require_contains(final_pose_body, '"final_align_started"', "最终对齐开始事件", failures)
    require_contains(navigation_state, '"final_align_completed"', "最终对齐完成事件", failures)
    cleanup_segment_start_failure_body = extract_function_body(navigation_state, "cleanup_route_task_segment_start_failure")
    for cleanup_field in (
        "self.current_route_task_goal_generation = -1",
        "self.route_task_last_feedback_time = 0.0",
        "self.current_goal_handle = None",
        "self.route_task_goal_handle = None",
        "self.reset_route_task_state()",
        "self.reset_navigation_state()",
    ):
        require_contains(
            cleanup_segment_start_failure_body,
            cleanup_field,
            f"段启动失败清理 {cleanup_field}",
            failures,
        )
    require_contains(
        start_route_task_body,
        "self.cleanup_route_task_segment_start_failure()",
        "start_route_task 首段启动失败后清理 route task 和导航大状态",
        failures,
    )
    require_contains(
        start_route_task_body,
        'self.current_detailed_state = "ROUTE_TASK_FIRST_TASK_ALREADY_REACHED"',
        "start_route_task 首 task 已在附近时进入专属 detailed_state",
        failures,
    )
    require_contains(
        start_route_task_body,
        "self.current_navigation_mode = NavigationMode.MULTI_POINT",
        "start_route_task 首 task 快速路径也要设置导航模式，保持周期状态与 through 路径一致",
        failures,
    )
    require_contains(
        start_route_task_body,
        "self.navigation_start_time = time.time()",
        "start_route_task 首 task 快速路径也要设置 navigation_start_time，保证 navigation_duration 正常",
        failures,
    )
    require_order(
        start_route_task_body,
        'result_reason="first_task_already_reached"',
        "self.complete_active_segment_without_navigation()",
        "首 task 快速路径必须先返回 start_route_task success ack，再进入最终对齐/播报流程",
        failures,
    )
    require_contains(navigation_state, "send_failure_ack=False", "自动下一段失败不伪造命令 ack", failures)
    require_contains(navigation_state, '"next route segment start failed"', "自动下一段失败带失败原因", failures)
    require_contains(navigation_state, 'failure_code="next_segment_start_failed"', "自动下一段失败进入 route task 失败流程并带 failure_code", failures)
    require_order(
        broadcast_finished_body,
        'error_code="invalid_task_session"',
        'error_code="invalid_route_id"',
        "broadcast_finished 应先返回 invalid_task_session，再判断 route_id",
        failures,
    )
    require_order(
        broadcast_finished_body,
        'error_code="invalid_route_id"',
        'error_code="broadcast_context_mismatch"',
        "broadcast_finished 应先返回 invalid_route_id，再判断播报上下文",
        failures,
    )
    require_not_contains(
        navigation_state,
        "self.handle_navigation_failed(message)",
        "route task 失败不应进入旧普通导航失败策略",
        failures,
    )
    require_not_contains(
        route_task_failure_body,
        "self.send_acknowledgment(",
        "route task navigation_failed 是状态事件，不应再走旧 /navigation/acknowledgments 形成重复 navigation_command_result",
        failures,
    )
    require_contains(navigation_state, "def cancel_current_route_goal_safely", "route task 专用安全取消旧 goal", failures)
    require_contains(
        route_task_failure_body,
        "self.cancel_current_route_goal_safely",
        "route task 失败路径应使用专用安全取消，避免旧 cancel 回调清空新 goal",
        failures,
    )
    require_not_contains(
        route_task_failure_body,
        "self.cancel_navigation()",
        "route task 失败路径不应使用通用 cancel_navigation",
        failures,
    )
    require_contains(
        obstacle_wait_body,
        'self.cancel_current_route_goal_safely("obstacle_wait")',
        "route task 障碍等待暂停应使用专用安全取消，避免旧 cancel 回调清空恢复后的新 goal",
        failures,
    )
    require_order(
        obstacle_wait_body,
        "if self.active_route_task:",
        'self.cancel_current_route_goal_safely("obstacle_wait")',
        "障碍等待暂停应先判断 active_route_task 再使用 route task 专用安全取消",
        failures,
    )
    for field in (
        '"current_target_task_id"',
        '"failure_code"',
        '"segment_id"',
        '"segment_direction"',
        '"execution_waypoint_ids"',
        '"completed_task_ids"',
        '"skipped_task_ids"',
        '"failed_at"',
    ):
        require_contains(route_task_failure_body, field, f"route task navigation_failed 上下文字段 {field}", failures)
    require_contains(
        route_task_failure_body,
        '"execution_waypoint_ids": list(active_segment.get("execution_waypoint_ids", []))',
        "route task navigation_failed 必须发布 execution_waypoint_ids 快照拷贝",
        failures,
    )
    require_contains(
        route_task_failure_body,
        '"passed_transit_waypoint_ids": list(active_segment.get("passed_transit_waypoint_ids", []))',
        "route task navigation_failed 必须发布 passed_transit_waypoint_ids 快照拷贝",
        failures,
    )
    for failure_call in (
        'failure_code="send_goal_failed"',
        'failure_code="goal_rejected"',
        'failure_code="goal_canceled"',
        'failure_code="goal_failed"',
        'failure_code="feedback_timeout"',
        'failure_code="target_task_missing"',
        'failure_code="next_segment_build_failed"',
        'failure_code="next_segment_start_failed"',
        'failure_code="jump_segment_start_failed"',
    ):
        require_contains(
            navigation_state,
            failure_call,
            f"route task 失败调用带分类 {failure_call}",
            failures,
        )

    # APP 对外导航入口只保留 route task 新命令，旧单点/多点/展台/旧暂停继续终止不再分发。
    for legacy_command in (
        'command_type == "start_single_navigation"',
        'command_type == "start_multi_point_navigation"',
        'command_type == "start_exhibition_navigation"',
        'command_type == "stop_navigation"',
        'command_type == "pause_navigation"',
        'command_type == "resume_navigation"',
    ):
        require_not_contains(
            navigation_command_body,
            legacy_command,
            f"状态机不应继续分发旧 APP 导航命令 {legacy_command}",
            failures,
        )
    require_not_contains(
        navigation_state,
        "reject_legacy_navigation_command_during_route_task",
        "不再保留 route_task_active 旧命令拦截函数，旧命令应直接下线",
        failures,
    )

    # 完成事件必须先发摘要再清状态，APP 才能收到 completed_task_ids/skipped_task_ids。
    require_order(
        complete_route_body,
        'self.publish_route_task_event("route_task_completed", summary)',
        "self.reset_route_task_state()",
        "route_task_completed 必须先于 reset_route_task_state",
        failures,
    )

    # 业务 ack：navigation_command_result 必须作为 navigation_status 事件推送，而不是顶层 data_type。
    require_contains(data_integration, 'data_type="navigation_status"', "navigation_command_result 顶层 data_type", failures)
    require_contains(data_integration, '"event_type": "navigation_command_result"', "navigation_command_result 事件类型", failures)
    require_contains(data_integration, "business_error_code = event_data.get(\"error_code\")", "navigation ack 元数据透传业务 error_code", failures)
    require_not_contains(
        data_integration,
        'push_msg["metadata"]["error_code"] = "nav_error"',
        "navigation ack metadata 不应固定写 nav_error",
        failures,
    )
    require_contains(data_integration, "转发导航确认", "navigation ack 日志无乱码", failures)
    require_not_contains(data_integration, "����", "data_integration 不应残留乱码占位符", failures)
    require_contains(data_integration, "路线任务导航失败", "route task navigation_failed APP 标题", failures)
    require_contains(data_integration, "current_target_task_id", "route task navigation_failed APP 文案包含目标 task", failures)
    require_contains(
        maybe_navigation_exception_body,
        'event_data = status_data.get("event_data", {})',
        "navigation_failed 系统异常处理必须解包 event_data",
        failures,
    )
    require_contains(
        maybe_navigation_exception_body,
        "event_payload = event_data if isinstance(event_data, dict) else status_data",
        "navigation_failed 系统异常处理使用 event_payload 兼容嵌套/旧扁平 payload",
        failures,
    )
    require_contains(
        maybe_navigation_exception_body,
        'event_payload.get("failure_code")',
        "route task navigation_failed system_exception code 使用 event_data.failure_code",
        failures,
    )
    require_contains(
        maybe_navigation_exception_body,
        'event_payload.get("route_task")',
        "route task navigation_failed system_exception 从 event_data 判断 route_task",
        failures,
    )
    require_contains(data_integration, '"navigation_failed"', "navigation_failed 立即推送白名单", failures)
    require_contains(data_integration, '"broadcast_requested"', "broadcast_requested 立即推送白名单", failures)
    require_contains(data_integration, '"waypoint_passed"', "waypoint_passed 立即推送白名单", failures)
    require_contains(data_integration, '"jump_updated"', "jump_updated 立即推送白名单", failures)
    require_contains(data_integration, '"final_align_started"', "final_align_started 立即推送白名单", failures)
    require_contains(data_integration, '"final_align_completed"', "final_align_completed 立即推送白名单", failures)
    require_contains(data_integration, '"task_waypoint_completed"', "task_waypoint_completed 立即推送白名单", failures)
    require_contains(data_integration, '"route_task_completed"', "route_task_completed 立即推送白名单", failures)
    require_contains(
        navigation_status_callback_body,
        'self.should_push_navigation_status_immediately(basic_status.get("event_type", ""))',
        "navigation_status_callback 按 event_type 判断立即推送",
        failures,
    )
    require_contains(
        navigation_status_callback_body,
        "self.publish_navigation_status_event(enhanced_status)",
        "navigation_status_callback 对白名单事件立即推送 enhanced_status",
        failures,
    )

    # 事件 payload 字段：每个事件都要在对应函数内携带清单规定的字段，不能只靠全文搜索“碰巧存在”。
    # publish_route_task_event() 会统一补 event_id/task_session_id/route_id；这里检查各事件自己的业务字段。
    for field in (
        '"request_message_id"',
        '"ack_type"',
        '"command_type"',
        '"status"',
        '"result_reason"',
        '"error_code"',
        '"message"',
    ):
        require_contains(send_ack_body, field, f"navigation_command_result 字段 {field}", failures)
    require_contains(
        navigation_state,
        "def build_route_task_event_id(self, event_type: str, session_id: str = \"\")",
        "event_id 生成支持从 event_data/command_data 传入 session_id",
        failures,
    )
    build_event_id_body = extract_function_body(navigation_state, "build_route_task_event_id")
    require_contains(
        build_event_id_body,
        "session_id = self.route_task_id(session_id)",
        "build_route_task_event_id 必须使用 route_task_id 归一化 session_id，避免数字 0 被当成缺失",
        failures,
    )
    require_contains(
        send_ack_body,
        'event_data["event_id"] = self.build_route_task_event_id',
        "navigation_command_result event_id 使用命令中的 task_session_id",
        failures,
    )
    require_contains(
        send_ack_body,
        'event_data.get("task_session_id", "")',
        "navigation_command_result event_id 显式传入 task_session_id",
        failures,
    )
    require_contains(
        send_ack_body,
        'self.route_task_id(command_data.get("request_message_id"))',
        "navigation_command_result 使用 route_task_id 归一化 request_message_id",
        failures,
    )
    require_contains(
        send_ack_body,
        'self.route_task_id(command_data.get("task_session_id"))',
        "navigation_command_result 使用 route_task_id 归一化 task_session_id",
        failures,
    )
    require_contains(
        send_ack_body,
        'self.route_task_id(command_data.get("route_id"))',
        "navigation_command_result 使用 route_task_id 归一化 route_id",
        failures,
    )
    require_not_contains(
        send_ack_body,
        'event_data["task_session_id"] = event_data["task_session_id"] or self.active_route_task.get("task_session_id", "")',
        "navigation_command_result 不应使用 active_route_task 回填缺失 task_session_id",
        failures,
    )
    require_not_contains(
        send_ack_body,
        'event_data["route_id"] = event_data["route_id"] or self.active_route_task.get("route_id", "")',
        "navigation_command_result 不应使用 active_route_task 回填缺失 route_id",
        failures,
    )
    require_contains(
        jump_body,
        'task_session_id = self.route_task_id(command_data.get("task_session_id"))',
        "jump_to_waypoint 使用 route_task_id 归一化 task_session_id",
        failures,
    )
    require_contains(
        jump_body,
        'route_id = self.route_task_id(command_data.get("route_id"))',
        "jump_to_waypoint 使用 route_task_id 归一化 route_id",
        failures,
    )
    require_contains(
        jump_body,
        'error_code="invalid_route_id"',
        "jump_to_waypoint 校验 route_id 与当前任务一致",
        failures,
    )
    require_order(
        jump_body,
        'error_code="invalid_task_session"',
        'error_code="invalid_route_id"',
        "jump_to_waypoint 应先返回 invalid_task_session，再判断 route_id",
        failures,
    )
    require_contains(
        jump_body,
        'target_waypoint_id = self.route_task_id(command_data.get("target_waypoint_id"))',
        "jump_to_waypoint 使用 route_task_id 归一化 target_waypoint_id",
        failures,
    )
    for field_name in ("task_session_id", "route_id", "waypoint_id", "broadcast_id"):
        require_contains(
            broadcast_finished_body,
            f'{field_name} = self.route_task_id(command_data.get("{field_name}"))',
            f"broadcast_finished 使用 route_task_id 归一化 {field_name}",
            failures,
        )
    require_contains(
        broadcast_finished_body,
        'error_code="invalid_route_id"',
        "broadcast_finished 校验 route_id 与当前任务一致",
        failures,
    )

    for field in (
        '"segment_id"',
        '"waypoint_id"',
        '"waypoint_role"',
        '"passed_transit_waypoint_ids"',
        '"current_target_task_id"',
    ):
        require_contains(mark_transit_body, field, f"waypoint_passed 字段 {field}", failures)
    require_contains(
        mark_transit_body,
        '"passed_transit_waypoint_ids": list(passed_ids)',
        "waypoint_passed 必须发布 passed_transit_waypoint_ids 快照拷贝",
        failures,
    )

    for field in (
        '"segment_id"',
        '"waypoint_id"',
        '"broadcast_id"',
        '"current_target_task_id"',
    ):
        require_contains(handle_target_arrived_body, field, f"broadcast_requested 字段 {field}", failures)

    for field in (
        '"segment_id"',
        '"waypoint_id"',
        '"completed_task_ids"',
        '"next_target_task_id"',
    ):
        require_contains(finalize_task_body, field, f"task_waypoint_completed 字段 {field}", failures)
    require_contains(
        finalize_task_body,
        '"completed_task_ids": list(self.completed_task_ids)',
        "task_waypoint_completed 必须发布 completed_task_ids 快照拷贝",
        failures,
    )
    require_contains(
        finalize_task_body,
        "if waypoint_id in self.skipped_task_ids:",
        "实际完成 task 时必须从 skipped_task_ids 移除，避免完成/跳过重叠",
        failures,
    )
    require_contains(
        finalize_task_body,
        "if skipped_task_id != waypoint_id",
        "实际完成 task 时只移除当前 task 的 skipped 记录",
        failures,
    )
    require_contains(
        finalize_task_body,
        '"skipped_task_ids": list(self.skipped_task_ids)',
        "task_waypoint_completed 必须发布 skipped_task_ids 快照拷贝",
        failures,
    )

    for field in (
        '"completed_waypoint_id"',
        '"completed_task_ids"',
        '"skipped_task_ids"',
        '"completed_at"',
        '"result"',
        '"summary"',
    ):
        require_contains(complete_route_body, field, f"route_task_completed 字段 {field}", failures)
    require_contains(complete_route_body, '"completed_task_ids": list(self.completed_task_ids)', "完成摘要拷贝 completed_task_ids", failures)
    require_contains(complete_route_body, '"skipped_task_ids": list(self.skipped_task_ids)', "完成摘要拷贝 skipped_task_ids", failures)

    for field in (
        '"segment_id"',
        '"target_waypoint_id"',
        '"segment_direction"',
        '"execution_waypoint_ids"',
        '"skipped_task_ids"',
        '"interrupt_broadcast"',
        '"interrupted_broadcast"',
    ):
        require_contains(jump_body, field, f"jump_updated 字段 {field}", failures)
    require_contains(
        jump_body,
        '"execution_waypoint_ids": list(self.active_segment.get("execution_waypoint_ids", []))',
        "jump_updated 必须发布 execution_waypoint_ids 快照拷贝",
        failures,
    )
    require_contains(
        jump_body,
        '"skipped_task_ids": list(self.skipped_task_ids)',
        "jump_updated 必须发布 skipped_task_ids 快照拷贝",
        failures,
    )

    # websocket/桥接：APP 外层 message_id 要透传成 request_message_id，路线数组和上下文 ID 要保留。
    require_contains(websocket_server, '"request_message_id"', "websocket 透传 request_message_id", failures)
    require_contains(
        websocket_server,
        'command_data.get("request_message_id") or request_message_id',
        "websocket request_message_id 为空时回退外层 message_id",
        failures,
    )
    require_contains(websocket_server, '"route_waypoints"', "websocket 透传 route_waypoints", failures)
    require_contains(
        websocket_server,
        '"broadcast_result": command_data.get("broadcast_result", "completed")',
        "websocket 缺省 broadcast_result 时按 completed 透传",
        failures,
    )
    require_contains(dynamic_waypoints, 'command_data.get("task_session_id")', "桥接层校验 task_session_id", failures)
    require_contains(dynamic_waypoints, 'command_data.get("route_id")', "桥接层校验 route_id", failures)
    require_contains(
        bridge_route_task_validation_body,
        'required_fields = ("task_session_id", "route_id", "waypoint_id", "broadcast_id")',
        "桥接层 broadcast_finished 缺字段提示包含 route_id",
        failures,
    )
    require_contains(dynamic_waypoints, "normalized_waypoint[\"waypoint_id\"] = str", "桥接层 waypoint_id 字符串化", failures)
    require_contains(dynamic_waypoints, "继续转发给状态机返回业务 ack", "桥接层 route task 缺字段继续转发说明", failures)
    require_not_contains(
        bridge_route_task_validation_body,
        "return False",
        "桥接层 route task 分支不应硬拒绝，避免 APP 收不到 navigation_command_result",
        failures,
    )

    # launch：默认和 fusion 启动都必须进入已改造的新入口，并显式带 route task 参数。
    for launch_name, launch_text in (
        ("navigation.launch.py", navigation_launch),
        ("navigation_fusion_sc.launch.py", fusion_sc_launch),
    ):
        require_contains(launch_text, "executable='navigation_state_manager'", f"{launch_name} 启动新状态管理器", failures)
        require_contains(launch_text, "'route_task.first_task_reached_tolerance_m'", f"{launch_name} 首 task 参数", failures)
        require_contains(launch_text, "'route_task.transit_passed_tolerance_m'", f"{launch_name} transit 阈值参数", failures)
        require_contains(launch_text, "'route_task.default_interrupt_broadcast'", f"{launch_name} 播报打断参数", failures)
    require_contains(
        navigation_setup,
        "'navigation_state_manager = humanoid_navigation.navigation_state_manager:main'",
        "setup.py 注册 navigation_state_manager console script",
        failures,
    )

    # robot_real 是整机 bringup 间接入口：它不直接创建状态机，而是 include APP 层 launch。
    # 这样既能保证 start_navigation.sh 主链路进入新状态机，也避免同名 navigation_state_manager 重复启动。
    require_contains(robot_real_launch, "navigation_fusion_sc.launch.py", "robot_real include SC APP 层 launch", failures)
    require_contains(robot_real_launch, "navigation2_robosense_lidar.launch.py", "robot_real include ro 定位链路", failures)
    require_contains(robot_real_launch, "navigation2.launch.py", "robot_real include Open3D prior 定位链路", failures)
    require_not_contains(robot_real_launch, "navigation_fusion.launch.py", "robot_real 不再暴露 HDL APP 旧入口", failures)
    require_not_contains(robot_real_launch, "navigation2_fusion", "robot_real 不再暴露 SC/HDL 旧导航栈入口", failures)
    require_contains(robot_real_launch, "route task 启动链路说明", "robot_real 说明 route task 间接入口", failures)
    require_contains(
        implementation_checklist,
        "`robot_real.launch.py` 是整机 bringup 入口，不直接创建 `navigation_state_manager`",
        "清单说明 robot_real 是间接入口",
        failures,
    )
    # 文档注释标准：路线任务跨 APP、桥接层、状态机和 Nav2，文档里的代码片段也必须解释业务意图。
    # 这组检查用于防止后续评审版把“中文注释/字段用途/APP 消费动作”的要求误删。
    require_contains(implementation_checklist, "### 1.1 中文注释与说明标准", "清单包含中文注释与说明标准", failures)
    require_contains(
        implementation_checklist,
        "新增函数必须在函数入口附近说明“这个函数负责什么业务步骤”",
        "清单要求新增函数补中文业务职责注释",
        failures,
    )
    require_contains(
        implementation_checklist,
        "新增事件 payload 组装处必须说明“APP 收到后应该如何消费”",
        "清单要求事件 payload 组装处说明 APP 消费动作",
        failures,
    )
    require_contains(
        implementation_checklist,
        "新增错误码返回处必须说明“触发条件”和“APP 建议展示文案”",
        "清单要求错误码返回处说明触发条件和用户文案",
        failures,
    )
    require_contains(
        implementation_checklist,
        "每个 Python/JSON 结构示例后必须有“中文注释”或“字段说明”",
        "清单要求文档代码片段附中文解释",
        failures,
    )
    require_contains(
        implementation_checklist,
        "这个结构是状态机内部使用的“已归一化路线点”",
        "RouteWaypoint 示例后有中文运行时用途说明",
        failures,
    )
    require_contains(
        implementation_checklist,
        "这个结构表示“一次正在执行的路线任务会话”",
        "RouteTaskState 示例后有中文运行时用途说明",
        failures,
    )
    require_contains(
        implementation_checklist,
        "这个结构表示当前 through 段，而不是整条路线",
        "ActiveSegment 示例后有中文运行时用途说明",
        failures,
    )
    require_contains(
        implementation_checklist,
        "这段伪代码的核心不是“从旧段里删几个点”",
        "jump 重建伪代码后有中文算法意图说明",
        failures,
    )

    # 文档口径：首版验收不再要求同步 recoverable 旧入口，避免实施者误改旧文件。
    require_contains(implementation_checklist, "recoverable 旧入口不作为首版验收路径", "清单明确 recoverable 非首版验收路径", failures)
    require_not_contains(
        implementation_checklist,
        "recoverable 版本按主链路口径同步",
        "清单不应继续把 recoverable 同步作为首版完成条件",
        failures,
    )
    # 状态机口径：首版只用 current_detailed_state 表达 route task 子状态，不新增 NavigationState 主枚举。
    # 这条检查避免文档把 ROUTE_TASK_SEGMENT_NAVIGATING 等子状态误写成“主状态”，导致实现和验收口径分裂。
    require_contains(implementation_checklist, "首版不新增 `NavigationState` 枚举值", "清单明确不新增 NavigationState 枚举", failures)
    require_contains(implementation_checklist, "route task 的细分阶段统一放在 `current_detailed_state`", "清单明确 route task 子状态承载字段", failures)
    require_contains(navigation_state, 'self.current_detailed_state = "ROUTE_TASK_SEGMENT_NAVIGATING"', "代码使用 detailed_state 表达 through 段导航", failures)
    require_contains(
        implementation_checklist,
        "route task 失败清理时必须使用 route task 专用安全取消",
        "清单说明 route task 失败路径安全取消",
        failures,
    )
    require_contains(
        implementation_checklist,
        "route task `navigation_failed` 复盘字段",
        "清单说明 route task navigation_failed 复盘字段",
        failures,
    )
    require_contains(
        implementation_checklist,
        "首版 `failure_code` 建议文案",
        "清单说明 route task failure_code 文案建议",
        failures,
    )
    require_contains(
        implementation_checklist,
        "`failure_code` 是机器可读分类",
        "清单说明 failure_code 与 reason 的分工",
        failures,
    )
    for documented_failure_code in (
        "`target_task_missing`",
        "`next_segment_build_failed`",
        "`next_segment_start_failed`",
        "`jump_segment_start_failed`",
    ):
        require_contains(
            implementation_checklist,
            documented_failure_code,
            f"清单说明 {documented_failure_code} 文案建议",
            failures,
        )
    require_contains(implementation_checklist, "首版不新增独立停车/对齐动作", "broadcast_requested 触发口径不依赖独立停车对齐", failures)
    require_contains(implementation_checklist, "### 12.2 APP -> ROS 命令字段", "清单包含 APP->ROS 字段字典", failures)
    require_contains(
        implementation_checklist,
        "不硬拒绝缺字段；业务错误统一交给状态机返回 `navigation_command_result`",
        "清单说明桥接层不硬拒绝 route task 坏包",
        failures,
    )
    require_contains(implementation_checklist, "`interrupt_broadcast`:", "清单解释 interrupt_broadcast 字段", failures)
    require_contains(implementation_checklist, "状态机使用 `route_task_bool()` 解析", "清单说明 interrupt_broadcast 安全布尔解析", failures)
    require_contains(implementation_checklist, "`broadcast_result`:", "清单解释 broadcast_result 字段", failures)
    require_contains(implementation_checklist, "首版只接受 `completed`", "清单说明 broadcast_result 首版枚举", failures)
    require_contains(
        implementation_checklist,
        "状态机必须先转字符串，再执行 `strip().lower()`",
        "清单说明 broadcast_result 归一化规则",
        failures,
    )
    require_contains(
        implementation_checklist,
        "APP 显式传入 `\"\"` 不等同于缺失字段",
        "清单说明 broadcast_result 空字符串不能按默认完成处理",
        failures,
    )
    require_contains(
        implementation_checklist,
        "`None` 转为空字符串，其它值执行 `str(value).strip()`",
        "清单说明 route task ID 统一归一化规则",
        failures,
    )
    require_contains(
        implementation_checklist,
        "不得直接使用 `str(value)` 处理可为空的 ID",
        "清单说明不能把 None 转成伪 ID",
        failures,
    )
    require_contains(
        implementation_checklist,
        "不得直接使用 `value or \"\"` 处理 ID",
        "清单说明不能用 or 空串吞掉数字 ID",
        failures,
    )
    require_contains(
        implementation_checklist,
        "初始化 route task 运行态，且只保存归一化后的 ID",
        "清单说明 start_route_task 初始化只保存归一化 ID",
        failures,
    )
    require_contains(
        implementation_checklist,
        "`route_waypoints[].waypoint_id` 是后续 jump 查找、through 段构建、transit passed 和完成事件的主键",
        "清单说明 route waypoint id 归一化必要性",
        failures,
    )
    require_contains(
        implementation_checklist,
        "`broadcast_id` 是 `broadcast_requested` 和 `broadcast_finished` 闭环匹配字段",
        "清单说明 broadcast_id 归一化必要性",
        failures,
    )
    require_contains(
        implementation_checklist,
        "`frame_id` 决定 route waypoint 转成 `PoseStamped` 后挂在哪个坐标系下",
        "清单说明 frame_id 归一化必要性",
        failures,
    )
    require_contains(
        implementation_checklist,
        "`start_route_task` 归一化阶段返回 `missing_waypoint_pose`",
        "清单说明 pose 坏值在启动归一化阶段失败",
        failures,
    )
    require_contains(
        implementation_checklist,
        "`orientation` 不允许静默默认成 `[0, 0, 0, 1]`",
        "清单说明 orientation 不允许静默默认",
        failures,
    )
    require_contains(
        implementation_checklist,
        "route task 专用 `PoseStamped` 转换不得复用旧普通导航 `waypoint_to_pose_stamped()` 的默认字段填充逻辑",
        "清单说明 route task PoseStamped 转换不复用旧默认逻辑",
        failures,
    )
    require_contains(
        implementation_checklist,
        "`start_route_task` 首段启动失败时，必须先返回 `navigation_command_result(error)`，再清理 route task 专属状态和导航大状态",
        "清单说明首段启动失败 ack 后清理状态",
        failures,
    )
    require_contains(
        implementation_checklist,
        "命令触发的 `send_goal_async()` 或 goal response callback 注册阶段抛异常时，必须先返回 `navigation_command_result(error, error_code=send_goal_failed)`",
        "清单说明命令触发 send_goal_async 异常先返回业务 ack",
        failures,
    )
    require_contains(
        implementation_checklist,
        "自动下一段触发的 `send_goal_async()` 或 goal response callback 注册阶段抛异常时，不得伪造新的命令 ack",
        "清单说明自动下一段 send_goal_async 异常不伪造命令 ack",
        failures,
    )
    require_contains(
        implementation_checklist,
        "`reset_route_task_state()` 必须把 `current_route_task_goal_generation` 置为 `-1`",
        "清单说明 route task reset 必须让旧 through generation 失效",
        failures,
    )
    require_contains(
        implementation_checklist,
        "首 task 快速路径虽然不下发 through goal，也必须设置 `current_navigation_mode` 和 `navigation_start_time`",
        "清单说明首 task 快速路径补齐 navigation_mode/navigation_start_time",
        failures,
    )
    require_contains(
        implementation_checklist,
        "首 task 快速路径必须先返回 `navigation_command_result(success, result_reason=first_task_already_reached)`",
        "清单说明首 task 快速路径先业务 ack 再发后续事件",
        failures,
    )
    require_contains(
        implementation_checklist,
        "`maybe_publish_navigation_exception()` 处理 `navigation_failed` 时，必须先解包 `event_data`",
        "清单说明 navigation_failed 系统异常必须解包 event_data",
        failures,
    )
    require_contains(
        implementation_checklist,
        "route task 失败只通过 `/navigation/status` 的 `navigation_failed` 事件通知 APP",
        "清单说明 route task 失败不得再走旧 acknowledgments 双通道",
        failures,
    )
    require_contains(
        implementation_checklist,
        "`unknown_navigation_command`：导航命令类型不支持",
        "清单必须给未知导航命令错误码提供 APP 展示文案",
        failures,
    )
    require_contains(
        implementation_checklist,
        "`internal_error`：导航命令处理异常",
        "清单必须给旧命令 internal_error 提供 APP 展示文案",
        failures,
    )
    for legacy_error_doc in (
        "`unknown_request_type`：导航请求类型不支持",
        "`invalid_request_payload`：导航请求格式异常",
        "`waypoint_not_found`：目标点位不存在",
        "`empty_waypoint_list`：点位列表为空",
        "`empty_exhibition_points`：没有可用的展台点位",
    ):
        require_contains(
            implementation_checklist,
            legacy_error_doc,
            f"清单必须给旧 ack 错误码提供 APP 展示文案 {legacy_error_doc}",
            failures,
        )
    require_contains(
        implementation_checklist,
        "自动下一段启动失败时，不得伪造新的命令 ack",
        "清单说明自动下一段失败不伪造 ack",
        failures,
    )
    require_contains(
        data_integration_doc_section,
        "`navigation_failed`",
        "清单说明 navigation_failed route task 事件也要即时推送",
        failures,
    )
    require_contains(
        implementation_checklist,
        "当前 PowerShell 同样未识别 `colcon`",
        "清单必须记录当前终端无法直接运行 colcon 构建",
        failures,
    )
    require_contains(
        implementation_checklist,
        "工作区顶层未发现 `install/setup.*`",
        "清单必须记录当前工作区缺少可 source 的 install/setup 脚本",
        failures,
    )
    require_contains(
        implementation_checklist,
        "顶层未发现 `build / install / log`",
        "清单必须记录当前工作区没有已构建产物目录",
        failures,
    )
    # 中文注释：验证入口必须集中写在清单正文里，避免后续开发者只知道改代码，
    # 却不知道每次提交前至少要跑哪些本地护栏，以及哪些检查必须去真实 ROS2 环境完成。
    require_contains(
        implementation_checklist,
        "开工前/提交前验证总入口",
        "清单必须提供集中式开工前/提交前验证入口",
        failures,
    )
    require_contains(
        implementation_checklist,
        "python src\\humanoid_navigation2\\scripts\\validate_route_task_static.py",
        "清单必须写明路线任务静态校验脚本运行命令",
        failures,
    )
    require_contains(
        implementation_checklist,
        "python -m py_compile src\\humanoid_navigation\\humanoid_navigation\\navigation_state_manager.py",
        "清单必须写明改造相关 Python 文件的语法检查命令",
        failures,
    )
    require_contains(
        implementation_checklist,
        "XML and UTF-8 read ok",
        "清单必须写明文档 UTF-8 和 package.xml 读取检查命令",
        failures,
    )
    require_contains(
        implementation_checklist,
        "ros2 action info /navigate_through_poses",
        "清单必须写明真实 ROS2 action info 验证命令",
        failures,
    )
    require_contains(
        implementation_checklist,
        "colcon build",
        "清单必须写明真实 ROS2 工作空间构建验证命令",
        failures,
    )
    require_contains(
        implementation_checklist,
        "以下事件除前文已列字段外，必须补齐 `event_id`、`timestamp`、`task_session_id`、`route_id`。",
        "清单必须把 timestamp 写入 route task 事件 event_data 必带字段",
        failures,
    )
    require_contains(implementation_checklist, "### 12.4 ROS -> APP ack 字段", "清单包含 ROS->APP ack 字段字典", failures)
    require_not_contains(
        implementation_checklist,
        "2. 停车/对齐完成。",
        "broadcast_requested 不应继续写成必须等待独立停车/对齐完成",
        failures,
    )
    require_not_contains(
        implementation_checklist,
        "2. `ROUTE_TASK_STARTING`",
        "清单不应把 ROUTE_TASK_STARTING 写成 NavigationState 主状态",
        failures,
    )

    if failures:
        print("路线任务静态校验失败：")
        for failure in failures:
            print(f"- {failure}")
        return 1

    print("路线任务静态校验通过：关键 through、协议、入口和事件不变量均存在。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
