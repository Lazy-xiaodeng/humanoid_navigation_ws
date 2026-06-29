#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

tail -F \
  "$SCRIPT_DIR"/debug_logs/start_ros_control_plane_*.log \
  "$SCRIPT_DIR"/debug_logs/start_navigation_stack_current_*.log
