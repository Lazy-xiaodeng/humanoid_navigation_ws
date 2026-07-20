import json
import threading
from types import MethodType, SimpleNamespace
from unittest.mock import Mock, patch

from std_msgs.msg import String

from humanoid_websocket.websocket_client import HumanoidWebSocketClient, RobotState


def make_client(available=("wave_greet_bye",)):
    client = SimpleNamespace(
        available_motion_names=frozenset(available),
        robot_state=RobotState.WALK,
        is_executing_motion=False,
        motion_request_lock=threading.Lock(),
        publish_action_result=Mock(),
        get_logger=lambda: Mock(),
    )
    client.validate_motion_name = MethodType(
        HumanoidWebSocketClient.validate_motion_name, client
    )
    return client


def test_null_and_placeholder_gesture_ids_are_rejected():
    client = make_client()

    for value in (None, "", "  ", "null", "None", "undefined"):
        motion_name, code, message = HumanoidWebSocketClient.validate_motion_name(
            client, value
        )
        assert code == "missing_gesture_id"
        assert message


def test_unknown_gesture_is_rejected_against_ota_catalog():
    client = make_client()

    motion_name, code, message = HumanoidWebSocketClient.validate_motion_name(
        client, "fixed_legacy_action"
    )

    assert motion_name == "fixed_legacy_action"
    assert code == "invalid_gesture_id"
    assert "OTA" in message


def test_valid_ota_gesture_is_accepted():
    client = make_client()

    assert HumanoidWebSocketClient.validate_motion_name(
        client, " wave_greet_bye "
    ) == ("wave_greet_bye", "", "")


def test_catalog_unavailable_fails_closed():
    client = make_client(available=())

    _, code, _ = HumanoidWebSocketClient.validate_motion_name(client, "wave_greet_bye")

    assert code == "motion_catalog_unavailable"


def test_invalid_command_publishes_error_without_starting_motion_thread():
    client = make_client()
    payload = {
        "command_type": "execute_gesture",
        "parameters": {"gesture_id": "null"},
        "client_id": "app-test",
        "timestamp": 123.0,
    }
    msg = String(data=json.dumps(payload))

    with patch("humanoid_websocket.websocket_client.threading.Thread") as thread_cls:
        HumanoidWebSocketClient.robot_control_callback(client, msg)

    thread_cls.assert_not_called()
    client.publish_action_result.assert_called_once()
    result = client.publish_action_result.call_args.kwargs
    assert result["status"] == "rejected"
    assert result["result_code"] == "missing_gesture_id"
    assert result["walk_ready"] is True


def test_interlock_acquire_failure_rolls_back_without_scheduling_motion():
    client = make_client()
    client.get_motion_completion_timeout = Mock(return_value=10.0)
    client.set_navigation_action_interlock = Mock(
        side_effect=[(False, "response timeout"), (True, "released")]
    )

    with patch(
        "humanoid_websocket.websocket_client.asyncio.run_coroutine_threadsafe"
    ) as schedule:
        HumanoidWebSocketClient._run_motion_task(client, "wave_greet_bye")

    assert client.set_navigation_action_interlock.call_args_list[0].args == (True,)
    assert client.set_navigation_action_interlock.call_args_list[1].args == (False,)
    schedule.assert_not_called()
    result = client.publish_action_result.call_args.kwargs
    assert result["result_code"] == "navigation_interlock_failed"
    assert client.is_executing_motion is False
