#!/usr/bin/env bash
set -euo pipefail

TEXT="${XIAORUI_BROADCAST_TEXT:-}"
BROADCAST_ID="${XIAORUI_BROADCAST_ID:-manual}"
VOICE="${XIAORUI_EDGE_TTS_VOICE:-zh-CN-XiaoxiaoNeural}"
EDGE_TTS_BIN="${XIAORUI_EDGE_TTS_BIN:-/home/ubuntu/.local/bin/edge-tts}"
FFPLAY_BIN="${XIAORUI_FFPLAY_BIN:-/usr/bin/ffplay}"
TMP_DIR="${XIAORUI_BROADCAST_TMP_DIR:-/tmp}"

if [[ -z "${TEXT}" ]]; then
  echo "XIAORUI_BROADCAST_TEXT is empty" >&2
  exit 2
fi

if [[ ! -x "${EDGE_TTS_BIN}" ]]; then
  echo "edge-tts not found or not executable: ${EDGE_TTS_BIN}" >&2
  exit 3
fi

if [[ ! -x "${FFPLAY_BIN}" ]]; then
  echo "ffplay not found or not executable: ${FFPLAY_BIN}" >&2
  exit 4
fi

SAFE_ID="$(printf '%s' "${BROADCAST_ID}" | tr -c 'A-Za-z0-9_.-' '_')"
MEDIA_FILE="${TMP_DIR}/xiaorui_broadcast_${SAFE_ID}_$$.mp3"

cleanup() {
  rm -f "${MEDIA_FILE}"
}
trap cleanup EXIT

"${EDGE_TTS_BIN}" \
  --voice "${VOICE}" \
  --text "${TEXT}" \
  --write-media "${MEDIA_FILE}" >/dev/null

"${FFPLAY_BIN}" -nodisp -autoexit -loglevel error "${MEDIA_FILE}"
