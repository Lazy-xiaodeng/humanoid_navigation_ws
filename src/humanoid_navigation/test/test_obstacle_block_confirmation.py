from types import SimpleNamespace
from unittest.mock import Mock

from humanoid_navigation.navigation_state_manager import NavigationStateManager


def make_manager():
    manager = NavigationStateManager.__new__(NavigationStateManager)
    manager.obstacle_block_require_sensor_confirmation = True
    manager.obstacle_block_sensor_timeout_sec = 1.0
    manager.obstacle_resume_use_roi = True
    manager.obstacle_roi_has_obstacle_topic = "/front_obstacle/has_obstacle"
    manager.obstacle_roi_timeout_sec = 1.0
    manager.obstacle_roi_required_clear_frames = 3
    manager.roi_obstacle_clear_confirm_count = 3
    manager.latest_roi_obstacle_has_obstacle = False
    manager.latest_roi_obstacle_stamp = 100.0
    manager.local_costmap_topic = "/local_costmap/costmap"
    manager.latest_front_obstacle_blocked = False
    manager.latest_front_obstacle_stats = {"occupied_cells": 0}
    manager.latest_local_costmap_stamp = 100.0
    return manager


def test_low_speed_with_clear_sensors_is_not_obstacle():
    manager = make_manager()

    confirmed, stats = manager.get_obstacle_block_confirmation(now=100.5)

    assert confirmed is False
    assert stats["decision"] == "low_speed_unconfirmed"
    assert stats["confirmed_by"] == []


def test_fresh_roi_obstacle_confirms_blockage():
    manager = make_manager()
    manager.latest_roi_obstacle_has_obstacle = True
    manager.roi_obstacle_clear_confirm_count = 0

    confirmed, stats = manager.get_obstacle_block_confirmation(now=100.5)

    assert confirmed is True
    assert stats["confirmed_by"] == ["roi"]


def test_fresh_costmap_obstacle_confirms_blockage():
    manager = make_manager()
    manager.latest_front_obstacle_blocked = True

    confirmed, stats = manager.get_obstacle_block_confirmation(now=100.5)

    assert confirmed is True
    assert stats["confirmed_by"] == ["costmap"]


def test_stale_obstacle_samples_do_not_confirm_blockage():
    manager = make_manager()
    manager.latest_roi_obstacle_has_obstacle = True
    manager.latest_front_obstacle_blocked = True

    confirmed, stats = manager.get_obstacle_block_confirmation(now=102.0)

    assert confirmed is False
    assert stats["roi"]["fresh"] is False
    assert stats["costmap"]["fresh"] is False


def test_confirmation_gate_can_be_disabled_for_rollback():
    manager = make_manager()
    manager.obstacle_block_require_sensor_confirmation = False

    confirmed, stats = manager.get_obstacle_block_confirmation(now=100.5)

    assert confirmed is True
    assert stats["decision"] == "legacy_speed_only"


def test_reset_clears_unconfirmed_stall_state():
    manager = make_manager()
    manager.current_detailed_state = "STALLED_UNCONFIRMED"
    manager.is_blocked_by_obstacle = True
    manager.block_start_time = 99.0
    manager.block_reported = False
    manager.clear_block_recovery_candidate = lambda: None

    manager.reset_block_detection()

    assert manager.current_detailed_state == "EXECUTING"
    assert manager.is_blocked_by_obstacle is False
    assert manager.block_start_time is None


def test_unconfirmed_stall_does_not_enter_obstacle_wait():
    enter_obstacle_wait_state = Mock()
    manager = SimpleNamespace(
        block_reported=False,
        obstacle_wait_enable=True,
        current_detailed_state="EXECUTING",
        get_obstacle_block_confirmation=lambda: (False, {"decision": "low_speed_unconfirmed"}),
        get_logger=lambda: Mock(),
        enter_obstacle_wait_state=enter_obstacle_wait_state,
    )

    NavigationStateManager.handle_obstacle_block_timeout(manager, 4.0)

    assert manager.current_detailed_state == "STALLED_UNCONFIRMED"
    enter_obstacle_wait_state.assert_not_called()


def test_confirmed_obstacle_enters_obstacle_wait():
    enter_obstacle_wait_state = Mock()
    manager = SimpleNamespace(
        block_reported=False,
        obstacle_wait_enable=True,
        current_detailed_state="EXECUTING",
        get_obstacle_block_confirmation=lambda: (True, {"confirmed_by": ["roi"]}),
        get_logger=lambda: Mock(),
        enter_obstacle_wait_state=enter_obstacle_wait_state,
    )

    NavigationStateManager.handle_obstacle_block_timeout(manager, 4.0)

    enter_obstacle_wait_state.assert_called_once_with(4.0)


def test_pose_delta_uses_point_one_as_stall_entry_threshold():
    manager = SimpleNamespace(
        velocity_threshold=0.10,
        blockage_recovery_velocity_threshold=0.10,
    )

    assert NavigationStateManager.is_stopped_for_blockage(manager, 0.099, "pose_delta") is True
    assert NavigationStateManager.is_stopped_for_blockage(manager, 0.116, "pose_delta") is False


def test_odom_twist_uses_same_stall_entry_threshold():
    manager = SimpleNamespace(
        velocity_threshold=0.10,
        blockage_recovery_velocity_threshold=0.10,
    )

    assert NavigationStateManager.is_stopped_for_blockage(manager, 0.099, "odom_twist") is True
    assert NavigationStateManager.is_stopped_for_blockage(manager, 0.116, "odom_twist") is False
