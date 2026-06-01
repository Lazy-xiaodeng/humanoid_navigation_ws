#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/humanoid_ws}"
cd "$WORKSPACE"

MON_DIR="${1:-}"
if [ -z "$MON_DIR" ]; then
  MON_DIR="$(find "$WORKSPACE/debug_monitor" -maxdepth 1 -type d -name 'live_*' 2>/dev/null | sort | tail -1 || true)"
fi

if [ -z "$MON_DIR" ] || [ ! -f "$MON_DIR/pids.tsv" ]; then
  echo "No live monitor pid file found."
  exit 1
fi

echo "stop_time=$(date '+%F %T %z')" >> "$MON_DIR/meta.txt"

mapfile -t pids < <(awk '{print $1}' "$MON_DIR/pids.tsv" | rg '^[0-9]+$' || true)

if [ "${#pids[@]}" -eq 0 ]; then
  echo "No monitor processes to stop."
  echo "$MON_DIR"
  exit 0
fi

for pid in "${pids[@]}"; do
  kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
done

sleep 2

for pid in "${pids[@]}"; do
  kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
done

sleep 1

{
  echo "remaining_processes:"
  for pid in "${pids[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      ps -o pid,ppid,stat,etime,cmd -p "$pid" || true
    fi
  done
} >> "$MON_DIR/meta.txt"

echo "$MON_DIR"
