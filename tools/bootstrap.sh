#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

usage() {
  cat <<USAGE
Usage: $0 [options]

Auto-detect Ubuntu/ROS target and run the matching humanoid_ws bootstrap script.

Default mapping:
  Ubuntu 22.04 -> ROS 2 Humble
  Ubuntu 24.04 -> ROS 2 Jazzy

Override examples:
  ROS_DISTRO=humble bash tools/bootstrap.sh
  ROS_DISTRO=jazzy bash tools/bootstrap.sh --clean-build

All options are forwarded to the selected bootstrap script.
USAGE
}

normalized_arch() {
  case "$(uname -m)" in
    aarch64|arm64|ARM64)
      printf '%s\n' aarch64
      ;;
    x86_64|amd64|AMD64)
      printf '%s\n' x86_64
      ;;
    *)
      uname -m
      ;;
  esac
}

detect_ros_distro() {
  if [ -n "${ROS_DISTRO:-}" ]; then
    printf '%s\n' "$ROS_DISTRO"
    return 0
  fi

  case "${VERSION_ID:-}" in
    22.04)
      if [ -f /opt/ros/humble/setup.bash ]; then
        printf '%s\n' humble
      else
        printf '%s\n' humble
      fi
      ;;
    24.04)
      if [ -f /opt/ros/jazzy/setup.bash ]; then
        printf '%s\n' jazzy
      else
        printf '%s\n' jazzy
      fi
      ;;
    *)
      die "Unsupported Ubuntu version: ${VERSION_ID:-unknown}. Supported: 22.04/Humble and 24.04/Jazzy."
      ;;
  esac
}

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
  usage
  exit 0
fi

[ -r /etc/os-release ] || die "/etc/os-release not found."
# shellcheck disable=SC1091
. /etc/os-release

[ "${ID:-}" = "ubuntu" ] || die "Unsupported OS ID: ${ID:-unknown}. This script targets Ubuntu."

ROS_DISTRO="$(detect_ros_distro)"
export ROS_DISTRO

case "$ROS_DISTRO" in
  humble)
    [ "${VERSION_ID:-}" = "22.04" ] ||
      die "ROS_DISTRO=humble is supported by this bootstrap on Ubuntu 22.04 only; current Ubuntu is ${VERSION_ID:-unknown}."
    target="$SCRIPT_DIR/bootstrap_ubuntu22_humble.sh"
    ;;
  jazzy)
    [ "${VERSION_ID:-}" = "24.04" ] ||
      die "ROS_DISTRO=jazzy is supported by this bootstrap on Ubuntu 24.04 only; current Ubuntu is ${VERSION_ID:-unknown}."
    target="$SCRIPT_DIR/bootstrap_ubuntu24_jazzy.sh"
    ;;
  *)
    die "Unsupported ROS_DISTRO=$ROS_DISTRO. Supported: humble, jazzy."
    ;;
esac

arch="$(normalized_arch)"
case "$arch" in
  aarch64|x86_64)
    ;;
  *)
    die "Unsupported CPU architecture: $(uname -m). Supported: aarch64 and x86_64."
    ;;
esac

printf 'Detected Ubuntu %s, ROS_DISTRO=%s, arch=%s\n' "${VERSION_ID:-unknown}" "$ROS_DISTRO" "$arch"
exec "$target" "$@"
