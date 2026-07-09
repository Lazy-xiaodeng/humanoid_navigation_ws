#!/bin/bash
set -eo pipefail
set +u

WORKSPACE="${WORKSPACE:-$HOME}"
HUMANOID_WS="${HUMANOID_WS:-$HOME/humanoid_ws}"
LOG_DIR="$WORKSPACE/service_logs"
MAIN_LOG="$LOG_DIR/stop_main.log"
ASR_DIR="${ASR_DIR:-$HOME/asr-client}"

mkdir -p "$LOG_DIR"

get_timestamp() {
    date '+%Y-%m-%d %H:%M:%S'
}

log_info() {
    local msg="$1"
    local timestamp
    timestamp="$(get_timestamp)"
    echo "[$timestamp] $msg"
    echo "[$timestamp] $msg" >> "$MAIN_LOG"
}

run_sudo() {
    if [ "$(id -u)" -eq 0 ]; then
        "$@"
        return $?
    fi
    if [ -t 0 ]; then
        sudo "$@"
    else
        sudo -n "$@"
    fi
}

wait_for_processes_to_exit() {
    local timeout_sec="$1"
    shift
    local pids=("$@")
    local elapsed=0
    local pid
    local any_alive

    while [ "$elapsed" -lt "$timeout_sec" ]; do
        any_alive=0
        for pid in "${pids[@]}"; do
            if kill -0 "$pid" > /dev/null 2>&1; then
                any_alive=1
                break
            fi
        done

        [ "$any_alive" -eq 1 ] || return 0
        sleep 1
        elapsed=$((elapsed + 1))
    done

    return 1
}

collect_pids_by_patterns() {
    local patterns=("$@")
    local seen=""
    local pattern
    local pid

    for pattern in "${patterns[@]}"; do
        while IFS= read -r pid; do
            [ -n "$pid" ] || continue
            [ "$pid" != "$$" ] || continue
            case " $seen " in
                *" $pid "*) continue ;;
            esac
            seen="$seen $pid"
            echo "$pid"
        done < <(pgrep -f "$pattern" || true)
    done
}

stop_pids() {
    local name="$1"
    shift
    local pids=("$@")

    if [ "${#pids[@]}" -eq 0 ]; then
        log_info "No $name processes detected."
        return 0
    fi

    log_info "Detected $name processes: ${pids[*]}"
    ps -o pid,ppid,stat,cmd -p "$(IFS=,; echo "${pids[*]}")" || true

    log_info "Stopping $name with SIGINT..."
    kill -INT "${pids[@]}" > /dev/null 2>&1 || true
    if wait_for_processes_to_exit 8 "${pids[@]}"; then
        log_info "$name stopped cleanly."
        return 0
    fi

    log_info "$name still running, sending SIGTERM..."
    kill -TERM "${pids[@]}" > /dev/null 2>&1 || true
    if wait_for_processes_to_exit 5 "${pids[@]}"; then
        log_info "$name stopped after SIGTERM."
        return 0
    fi

    log_info "$name still running, sending SIGKILL..."
    kill -KILL "${pids[@]}" > /dev/null 2>&1 || true
    sleep 1
}

stop_navigation() {
    log_info "Stopping navigation module..."
    if [ -x "$HUMANOID_WS/stop_navigation.sh" ]; then
        "$HUMANOID_WS/stop_navigation.sh" >> "$MAIN_LOG" 2>&1 || true
    else
        log_info "Navigation stop script not found: $HUMANOID_WS/stop_navigation.sh"
    fi
}

stop_speech() {
    log_info "Stopping speech module..."

    local patterns=(
        "$ASR_DIR/listening_sound.py"
        "asr-client-master.1.0.jar"
        "arecord -d 0 -r 48000 -f S16_LE -c 1 -t raw"
    )
    mapfile -t speech_pids < <(collect_pids_by_patterns "${patterns[@]}")
    stop_pids "speech" "${speech_pids[@]}"
}

stop_backend() {
    log_info "Stopping backend module..."

    if command -v systemctl > /dev/null 2>&1; then
        if run_sudo systemctl stop backend.service >> "$MAIN_LOG" 2>&1; then
            log_info "backend.service stopped."
        else
            log_info "Could not stop backend.service through systemd; falling back to process cleanup."
        fi
    fi

    local patterns=(
        "/data/dev-ops/server/guided-tour-robot-app.jar"
        "guided-tour-robot-app.jar --spring.config.location=/data/dev-ops/server/prod.yml"
    )
    mapfile -t backend_pids < <(collect_pids_by_patterns "${patterns[@]}")

    if [ "${#backend_pids[@]}" -eq 0 ]; then
        log_info "No backend Java processes detected."
        return 0
    fi

    log_info "Detected backend processes: ${backend_pids[*]}"
    ps -o pid,ppid,stat,cmd -p "$(IFS=,; echo "${backend_pids[*]}")" || true

    if kill -TERM "${backend_pids[@]}" > /dev/null 2>&1; then
        wait_for_processes_to_exit 8 "${backend_pids[@]}" || true
    elif command -v sudo > /dev/null 2>&1; then
        run_sudo kill -TERM "${backend_pids[@]}" > /dev/null 2>&1 || true
        wait_for_processes_to_exit 8 "${backend_pids[@]}" || true
    fi

    mapfile -t backend_pids < <(collect_pids_by_patterns "${patterns[@]}")
    if [ "${#backend_pids[@]}" -gt 0 ]; then
        log_info "Backend still running, sending SIGKILL..."
        kill -KILL "${backend_pids[@]}" > /dev/null 2>&1 || run_sudo kill -KILL "${backend_pids[@]}" > /dev/null 2>&1 || true
    fi
}

main() {
    echo "========================================" | tee -a "$MAIN_LOG"
    echo "Stop all services at $(get_timestamp)" | tee -a "$MAIN_LOG"
    echo "========================================" | tee -a "$MAIN_LOG"

    stop_navigation
    stop_speech
    stop_backend

    echo "========================================" | tee -a "$MAIN_LOG"
    echo "Stop all services finished at $(get_timestamp)" | tee -a "$MAIN_LOG"
    echo "Log: $MAIN_LOG" | tee -a "$MAIN_LOG"
    echo "========================================" | tee -a "$MAIN_LOG"
}

main
