from types import SimpleNamespace
from unittest.mock import Mock

from humanoid_navigation.navigation_state_manager import (
    NavigationState,
    NavigationStateManager,
)


def make_manager(state=NavigationState.EXECUTING, pause_source=""):
    waypoint = {"id": "p1", "name": "展台", "position": [1.0, 2.0, 0.0]}
    return SimpleNamespace(
        current_state=state,
        current_detailed_state=state.value,
        current_waypoint=waypoint,
        current_waypoint_index=0,
        total_waypoints=1,
        current_goal_handle=object() if state == NavigationState.EXECUTING else None,
        current_pause_source=pause_source,
        current_pause_reason="",
        current_resume_mode="",
        pause_time=0.0,
        pause_duration_limit=0,
        robot_action_interlock_active=False,
        robot_action_interlock_paused_navigation=False,
        robot_action_resume_pending=False,
        robot_action_interlock_started_at=0.0,
        cancel_navigation=Mock(),
        publish_zero_cmd_vel=Mock(),
        clear_obstacle_wait_state=Mock(),
        reset_block_detection=Mock(),
        build_pause_event_data=Mock(return_value={"pause_source": "robot_action"}),
        publish_status_update=Mock(),
        send_acknowledgment=Mock(),
        navigate_to_waypoint=Mock(),
        get_logger=lambda: Mock(),
    )


def call_interlock(manager, enabled):
    request = SimpleNamespace(data=enabled)
    response = SimpleNamespace(success=False, message="")
    return NavigationStateManager.robot_action_interlock_callback(
        manager, request, response
    )


def test_active_navigation_is_paused_before_action():
    manager = make_manager()

    response = call_interlock(manager, True)

    assert response.success is True
    assert manager.robot_action_interlock_active is True
    assert manager.robot_action_interlock_paused_navigation is True
    assert manager.current_state == NavigationState.PAUSED
    assert manager.current_pause_source == "robot_action"
    manager.cancel_navigation.assert_called_once()
    assert manager.publish_zero_cmd_vel.call_count == 3


def test_release_registers_resume_and_timer_restores_same_waypoint():
    manager = make_manager()
    call_interlock(manager, True)

    response = call_interlock(manager, False)
    assert response.success is True
    assert manager.robot_action_resume_pending is True

    NavigationStateManager.try_resume_after_robot_action(manager)

    assert manager.current_state == NavigationState.EXECUTING
    assert manager.current_pause_source == ""
    manager.navigate_to_waypoint.assert_called_once_with(manager.current_waypoint)


def test_action_does_not_take_resume_ownership_of_user_paused_navigation():
    manager = make_manager(state=NavigationState.PAUSED, pause_source="user_request")

    call_interlock(manager, True)
    call_interlock(manager, False)
    NavigationStateManager.try_resume_after_robot_action(manager)

    assert manager.current_state == NavigationState.PAUSED
    assert manager.current_pause_source == "user_request"
    manager.navigate_to_waypoint.assert_not_called()


def test_interlock_blocks_and_defers_new_navigation_start():
    manager = SimpleNamespace(
        robot_action_interlock_active=True,
        require_walk_mode_for_navigation=True,
    )

    reason = NavigationStateManager.get_navigation_start_block_reason(manager)

    assert "动作互锁" in reason
