#!/usr/bin/env bash
set -u

TARGET_USER="${TARGET_USER:-ubuntu}"
TARGET_UID="${TARGET_UID:-1000}"
TARGET_SINK="${TARGET_SINK:-alsa_output.usb-GeneralPlus_USB_Audio_Device-00.analog-stereo}"
TARGET_SOURCE="${TARGET_SOURCE:-alsa_input.usb-BAB_usb123_20170726905926-00.analog-stereo}"
SINK_USB_ID="${SINK_USB_ID:-1b3f:2008}"
SOURCE_USB_ID="${SOURCE_USB_ID:-0031:0022}"
INTERVAL_SECONDS="${INTERVAL_SECONDS:-5}"

log() {
  printf '%s %s\n' "$(date '+%F %T')" "$*"
}

pulse_env() {
  XDG_RUNTIME_DIR="/run/user/${TARGET_UID}" \
  PULSE_SERVER="unix:/run/user/${TARGET_UID}/pulse/native" \
  "$@"
}

pactl_user() {
  pulse_env pactl "$@" 2>/dev/null
}

has_todesk_session() {
  pgrep -f '/opt/todesk/bin/ToDesk_Session' >/dev/null 2>&1
}

kill_extra_pulseaudio() {
  pgrep -u "${TARGET_USER}" -f '^pulseaudio --start' | while read -r pid; do
    [ -n "${pid}" ] || continue
    log "stopping extra pulseaudio pid=${pid}"
    kill "${pid}" 2>/dev/null || true
  done
}

find_card_by_usb_id() {
  local target_usb_id="$1"
  local card usbid
  for usbid in /proc/asound/card*/usbid; do
    [ -r "${usbid}" ] || continue
    if [ "$(cat "${usbid}" 2>/dev/null)" = "${target_usb_id}" ]; then
      card="${usbid#/proc/asound/card}"
      card="${card%/usbid}"
      printf '%s\n' "${card}"
      return 0
    fi
  done
  return 1
}

reset_usb_card() {
  local usb_id="$1"
  local label="$2"
  local card sysdev usbdev
  card="$(find_card_by_usb_id "${usb_id}" || true)"
  [ -n "${card}" ] || return 1

  sysdev="$(readlink -f "/sys/class/sound/card${card}/device" 2>/dev/null || true)"
  [ -n "${sysdev}" ] || return 1

  usbdev="${sysdev%:*}"
  if [ -w "${usbdev}/authorized" ]; then
    log "resetting ${label} USB audio at ${usbdev}"
    echo 0 > "${usbdev}/authorized"
    sleep 2
    echo 1 > "${usbdev}/authorized"
    sleep 3
    return 0
  fi

  return 1
}

pcm_has_dead_owner() {
  local usb_id="$1"
  local pcm_path="$2"
  local label="$3"
  local card status owner
  card="$(find_card_by_usb_id "${usb_id}" || true)"
  [ -n "${card}" ] || return 1

  status="/proc/asound/card${card}/${pcm_path}/sub0/status"
  [ -r "${status}" ] || return 1

  owner="$(awk '/owner_pid/ {print $3}' "${status}" 2>/dev/null || true)"
  [ -n "${owner}" ] || return 1
  [ -d "/proc/${owner}" ] && return 1

  log "${label} PCM is stuck on dead owner_pid=${owner}"
  return 0
}

target_sink_exists() {
  pactl_user list short sinks | awk '{print $2}' | grep -Fx "${TARGET_SINK}" >/dev/null 2>&1
}

target_source_exists() {
  pactl_user list short sources | awk '{print $2}' | grep -Fx "${TARGET_SOURCE}" >/dev/null 2>&1
}

ensure_default_sink() {
  target_sink_exists || return 1

  local current
  current="$(pactl_user get-default-sink || true)"
  if [ "${current}" != "${TARGET_SINK}" ]; then
    log "setting default sink to ${TARGET_SINK}"
    pactl_user set-default-sink "${TARGET_SINK}" || true
  fi

  pactl_user set-sink-mute "${TARGET_SINK}" 0 || true
}

ensure_default_source() {
  target_source_exists || return 1

  local current
  current="$(pactl_user get-default-source || true)"
  if [ "${current}" != "${TARGET_SOURCE}" ]; then
    log "setting default source to ${TARGET_SOURCE}"
    pactl_user set-default-source "${TARGET_SOURCE}" || true
  fi

  pactl_user set-source-mute "${TARGET_SOURCE}" 0 || true
  pactl_user set-source-volume "${TARGET_SOURCE}" 100% || true
}

repair_once() {
  kill_extra_pulseaudio

  if ! target_sink_exists || ! target_source_exists; then
    log "target audio node missing; restarting wireplumber"
    systemctl --user -M "${TARGET_USER}@" restart wireplumber.service 2>/dev/null || \
      XDG_RUNTIME_DIR="/run/user/${TARGET_UID}" runuser -u "${TARGET_USER}" -- \
        systemctl --user restart wireplumber.service 2>/dev/null || true
    sleep 2
  fi

  if ! target_sink_exists || pcm_has_dead_owner "${SINK_USB_ID}" "pcm0p" "GeneralPlus playback"; then
    reset_usb_card "${SINK_USB_ID}" "GeneralPlus" || true
  fi

  if ! target_source_exists || pcm_has_dead_owner "${SOURCE_USB_ID}" "pcm0c" "usb123 capture"; then
    reset_usb_card "${SOURCE_USB_ID}" "usb123" || true
  fi

  ensure_default_sink || true
  ensure_default_source || true
}

daemon() {
  log "started; guarding ToDesk sink=${TARGET_SINK} source=${TARGET_SOURCE}"
  while true; do
    if has_todesk_session; then
      repair_once
    fi
    sleep "${INTERVAL_SECONDS}"
  done
}

case "${1:-}" in
  --daemon)
    daemon
    ;;
  --once|"")
    repair_once
    ;;
  *)
    echo "usage: $0 [--once|--daemon]" >&2
    exit 2
    ;;
esac
