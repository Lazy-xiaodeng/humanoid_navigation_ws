#!/bin/bash
# 一键启动脚本：start_all_services.sh
# 兼容从旧路径调用；真实脚本位置以解析后的文件路径为准。

set -eo pipefail

resolve_script_path() {
    local source="${BASH_SOURCE[0]}"
    while [ -L "$source" ]; do
        local dir
        dir="$(cd -P "$(dirname "$source")" && pwd)"
        source="$(readlink "$source")"
        [[ "$source" != /* ]] && source="$dir/$source"
    done
    cd -P "$(dirname "$source")" && pwd
}

SCRIPT_DIR="$(resolve_script_path)"

# --- 配置区 ---
WORKSPACE="${WORKSPACE:-$HOME}"
HUMANOID_WS="${HUMANOID_WS:-$SCRIPT_DIR}"
LOG_DIR="${LOG_DIR:-$WORKSPACE/service_logs}"
MAIN_LOG="$LOG_DIR/startup_main.log"
START_NAVIGATION_SCRIPT="${START_NAVIGATION_SCRIPT:-$HUMANOID_WS/start_navigation.sh}"
TARGET_USER="${TARGET_USER:-$(id -un 2>/dev/null || printf '%s' "${USER:-}")}"
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

log_error() {
    local msg="$1"
    local timestamp
    timestamp="$(get_timestamp)"
    echo "[$timestamp] [错误] $msg"
    echo "[$timestamp] [错误] $msg" >> "$MAIN_LOG"
}

append_log() {
    local file="$1"
    local msg="$2"
    echo "[$(get_timestamp)] $msg" >> "$file"
}

wait_for_path() {
    local path="$1"
    local wait_seconds="$2"
    local i
    for ((i=1; i<=wait_seconds; i++)); do
        if [ -e "$path" ]; then
            return 0
        fi
        sleep 1
    done
    return 1
}

ensure_runtime_environment() {
    local user_id
    user_id="$(id -u "$TARGET_USER" 2>/dev/null || id -u)"

    export HOME="${HOME:-$WORKSPACE}"
    export USER="${USER:-$TARGET_USER}"
    export LOGNAME="${LOGNAME:-$TARGET_USER}"
    export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$user_id}"
    export DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-unix:path=$XDG_RUNTIME_DIR/bus}"
    export DISPLAY="${DISPLAY:-:0}"
    export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"

    if [ "${SERVICE_AUTO_START:-0}" = "1" ]; then
        export PULSE_SERVER="${PULSE_SERVER:-unix:$XDG_RUNTIME_DIR/pulse/native}"
    fi
}

cleanup_old_speech_processes() {
    local asr_log="$1"

    append_log "$asr_log" "清理旧语音进程"
    if command -v timeout >/dev/null 2>&1; then
        timeout 5s pkill -f "$ASR_DIR/listening_sound.py" 2>/dev/null || true
        timeout 5s pkill -f "asr-client-master.1.0.jar" 2>/dev/null || true
    else
        pkill -f "$ASR_DIR/listening_sound.py" 2>/dev/null || true
        pkill -f "asr-client-master.1.0.jar" 2>/dev/null || true
    fi
    sleep 2
}

wait_for_speech_prerequisites() {
    local asr_log="$1"
    local pulse_socket="$XDG_RUNTIME_DIR/pulse/native"

    append_log "$asr_log" "检查语音服务启动前置条件"

    if wait_for_path "$XDG_RUNTIME_DIR" 30; then
        append_log "$asr_log" "检测到 XDG_RUNTIME_DIR: $XDG_RUNTIME_DIR"
    else
        append_log "$asr_log" "未等待到 XDG_RUNTIME_DIR: $XDG_RUNTIME_DIR"
        append_log "$asr_log" "这通常表示开机时 ubuntu 用户会话尚未创建，PulseAudio 用户态环境可能还不能用"
    fi

    if wait_for_path "$pulse_socket" 45; then
        append_log "$asr_log" "检测到 PulseAudio socket: $pulse_socket"
    else
        append_log "$asr_log" "未检测到 PulseAudio socket，尝试启动 pulseaudio"
        pulseaudio --start >> "$asr_log" 2>&1 || pulseaudio -D --fail=false >> "$asr_log" 2>&1 || true
        sleep 3
        if wait_for_path "$pulse_socket" 10; then
            append_log "$asr_log" "pulseaudio 启动后检测到 socket: $pulse_socket"
        else
            append_log "$asr_log" "pulseaudio 尝试启动后仍未检测到 socket: $pulse_socket"
        fi
    fi

    if command -v pactl >/dev/null 2>&1; then
        if pactl info >> "$asr_log" 2>&1; then
            append_log "$asr_log" "PulseAudio 连接正常"
        else
            append_log "$asr_log" "PulseAudio 仍不可用，语音 Python 可能依赖的录音/播放环境尚未就绪"
        fi
    else
        append_log "$asr_log" "未找到 pactl，跳过 PulseAudio 连通性检查"
    fi
}

start_navigation() {
    log_info "开始启动导航模块..."
    local nav_log="$LOG_DIR/navigation_$(date '+%Y%m%d_%H%M%S').txt"

    if [ ! -x "$START_NAVIGATION_SCRIPT" ]; then
        log_error "导航模块启动失败：找不到可执行脚本 $START_NAVIGATION_SCRIPT"
        return 1
    fi

    if "$START_NAVIGATION_SCRIPT" >> "$nav_log" 2>&1; then
        log_info "导航模块启动指令已发送 (后台运行)"
        return 0
    fi

    log_error "导航模块启动失败，请检查日志: $nav_log"
    return 1
}

start_speech() {
    log_info "开始启动语音模块..."
    local asr_log="$LOG_DIR/speech_$(date '+%Y%m%d_%H%M%S').txt"
    local asr_dir="$ASR_DIR"
    local py_exe="$asr_dir/myenv/bin/python"
    local py_ver_dir=""
    local lib_path=""

    append_log "$asr_log" "进入 start_speech"

    if [ -d "$asr_dir/myenv/lib" ]; then
        py_ver_dir="$(find "$asr_dir/myenv/lib" -maxdepth 1 -type d -name 'python3*' | head -n1 || true)"
    fi

    if [ -n "$py_ver_dir" ]; then
        lib_path="$py_ver_dir/site-packages"
    fi

    if [ ! -f "$py_exe" ]; then
        log_error "语音模块启动失败：找不到 Python 解释器 $py_exe"
        append_log "$asr_log" "错误：找不到 Python 解释器 $py_exe"
        return 1
    fi

    if [ ! -f "$asr_dir/listening_sound.py" ]; then
        log_error "语音模块启动失败：找不到脚本 $asr_dir/listening_sound.py"
        append_log "$asr_log" "错误：找不到脚本 $asr_dir/listening_sound.py"
        return 1
    fi

    if [ "${SERVICE_AUTO_START:-0}" = "1" ]; then
        ensure_runtime_environment
        append_log "$asr_log" "开机自启模式环境: HOME=$HOME XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR PULSE_SERVER=$PULSE_SERVER DISPLAY=$DISPLAY XAUTHORITY=$XAUTHORITY"
        if [ "${CLEAN_OLD_SPEECH:-0}" = "1" ]; then
            cleanup_old_speech_processes "$asr_log"
        else
            append_log "$asr_log" "开机自启模式：跳过旧语音进程清理"
        fi
        wait_for_speech_prerequisites "$asr_log"

        append_log "$asr_log" "准备以开机自启模式启动 Python，工作目录: $asr_dir"
        (
            cd "$asr_dir" || exit 1
            nohup env \
                HOME="$HOME" \
                USER="$USER" \
                LOGNAME="$LOGNAME" \
                DISPLAY="$DISPLAY" \
                XAUTHORITY="$XAUTHORITY" \
                XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
                DBUS_SESSION_BUS_ADDRESS="$DBUS_SESSION_BUS_ADDRESS" \
                PULSE_SERVER="$PULSE_SERVER" \
                MPLBACKEND=Agg \
                PYTHONUNBUFFERED=1 \
                PYTHONPATH="$lib_path" \
                "$py_exe" "$asr_dir/listening_sound.py" >> "$asr_log" 2>&1 &
        )
        sleep 5

        if pgrep -f "$asr_dir/listening_sound.py" >/dev/null 2>&1; then
            append_log "$asr_log" "Python 语音进程启动成功"
        else
            log_error "语音 Python 进程未能存活，请检查日志: $asr_log"
            append_log "$asr_log" "Python 语音进程未存活，通常是缺少会话音频环境、依赖导入失败或脚本内部异常"
            return 1
        fi
    else
        append_log "$asr_log" "手动启动模式：使用原始 Python 启动方式"
        nohup stdbuf -oL env PYTHONPATH="$lib_path" "$py_exe" "$asr_dir/listening_sound.py" >> "$asr_log" 2>&1 &
        append_log "$asr_log" "Python 启动指令已发送"
    fi

    append_log "$asr_log" "准备启动 Java"
    (nohup java -jar "$asr_dir/asr-client-master.1.0.jar" >> "$asr_log" 2>&1 &)

    log_info "语音模块启动完成，日志记录在 $asr_log"
}

start_backend() {
    log_info "开始启动中控模块..."

    local ops_log="$LOG_DIR/backend_$(date '+%Y%m%d_%H%M%S').txt"

    if (
        cd /data/dev-ops/server || { log_error "无法进入中控目录"; exit 1; }
        ./restart-backend.sh >> "$ops_log" 2>&1
    ); then
        log_info "中控模块启动完成，日志记录在 $ops_log"
        return 0
    fi

    log_error "中控模块启动失败，请检查日志: $ops_log"
    return 1
}

run_step() {
    local step_name="$1"
    shift

    if "$@"; then
        return 0
    fi

    log_error "$step_name 执行失败，继续尝试启动后续模块"
    return 1
}

main() {
    local failed=0

    echo "========================================" | tee -a "$MAIN_LOG"
    echo "开始一键启动服务流程" | tee -a "$MAIN_LOG"
    echo "启动时间: $(get_timestamp)" | tee -a "$MAIN_LOG"
    echo "工作区路径: $HUMANOID_WS" | tee -a "$MAIN_LOG"
    echo "========================================" | tee -a "$MAIN_LOG"

    run_step "导航模块" start_navigation || failed=1
    sleep 5

    run_step "语音模块" start_speech || failed=1
    sleep 3

    run_step "中控模块" start_backend || failed=1

    echo "========================================" | tee -a "$MAIN_LOG"
    if [ "$failed" -eq 0 ]; then
        echo "所有服务启动流程结束！" | tee -a "$MAIN_LOG"
    else
        echo "服务启动流程结束，但存在失败项，请检查上方日志。" | tee -a "$MAIN_LOG"
    fi
    echo "========================================" | tee -a "$MAIN_LOG"

    return "$failed"
}

main
