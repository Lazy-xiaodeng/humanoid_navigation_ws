#!/usr/bin/env python3
"""Verify shadow relocalization observations cannot pause an active route."""

import importlib.util
import pathlib
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor


def load_smoke_module():
    source = pathlib.Path(__file__).with_name("localization_recovery_smoke.py")
    spec = importlib.util.spec_from_file_location("localization_recovery_smoke_shared", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    module = load_smoke_module()
    rclpy.init()
    node = module.LocalizationRecoverySmoke(False, integration_mode="shadow")
    node.phase = "trusted"
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        discovery_deadline = time.monotonic() + 3.0
        while time.monotonic() < discovery_deadline:
            executor.spin_once(timeout_sec=0.05)
            if node.command_pub.get_subscription_count() > 0:
                break
        if node.command_pub.get_subscription_count() == 0:
            raise RuntimeError("route runtime did not subscribe to navigation requests")

        node.start_route("shadow-no-pause")
        goal_deadline = time.monotonic() + 5.0
        while time.monotonic() < goal_deadline and node.through_goal_count == 0:
            executor.spin_once(timeout_sec=0.05)
        if node.through_goal_count == 0:
            raise RuntimeError("route did not start under healthy shadow status")

        node.phase = "recovery"
        observation_deadline = time.monotonic() + 1.5
        while time.monotonic() < observation_deadline:
            executor.spin_once(timeout_sec=0.05)

        paused = module.has_event(
            node.events, "navigation_paused", "pause_source", "localization_recovery"
        )
        if paused or node.through_cancel_count or node.zero_cmd_count:
            raise RuntimeError(
                "shadow mode affected live navigation: "
                f"paused={paused} cancels={node.through_cancel_count} zeros={node.zero_cmd_count}"
            )
        print("LOCALIZATION_SHADOW_NO_PAUSE_OK")
        return 0
    finally:
        executor.shutdown(timeout_sec=2.0)
        node.through_server.destroy()
        node.final_server.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
