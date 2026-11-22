#!/bin/bash
# ==============================================================================
# ARGO Quadrotor SITL Split-Dashboard Runner (tmux 기반 2분할 대시보드)
# ==============================================================================
# 기능:
#   1. DDS Agent(ROS 2 브리지) 자동 기동
#   2. 상하 2분할 (상단: PX4 실시간 콘솔, 하단: Pegasus 파이썬 로그)
#   3. 마우스 휠 스크롤 지원 + 마우스 드래그 시 윈도우/리눅스 클립보드 자동 복사
#   4. 상단 또는 하단 창에서 Ctrl+C 입력 시 세션 전체(tmux) 즉시 폭파 및 단일 터미널 복귀
# ==============================================================================

# [ARGO 변경] 설치 위치와 DDS 환경은 공통 설정에서 읽습니다. Ubuntu에 Windows PATH를 넣지 않습니다.
_config="${ARGO_ENV_FILE:-${XDG_CONFIG_HOME:-$HOME/.config}/argo/simulation.env}"
[ ! -f "$_config" ] || source "$_config"
source "$ARGO_WS_DIR/simulator/scripts/vehicle_args.sh"
[[ $# = 0 ]] || { echo "Unknown arguments: $*" >&2; exit 2; }
python3 "$ARGO_WS_DIR/simulator/vehicle_profiles.py" check || exit $?
export PEGASUS_FASTLIVO_CONFIG_DIR="$(python3 "$ARGO_WS_DIR/simulator/vehicle_profiles.py" calibration-dir)"
source "$ARGO_WS_DIR/simulator/tools/rviz/ros_env.sh" || exit 1
source "$ARGO_WS_DIR/install/local_setup.bash" || exit 1
TASKKILL_BIN="/mnt/c/Windows/System32/taskkill.exe"
CLIP_BIN="/mnt/c/Windows/System32/clip.exe"

# tmux needs terminal capabilities even in IDE/automation PTYs exporting dumb.
if [ -z "${TERM:-}" ] || [ "$TERM" = dumb ]; then
    export TERM=xterm-256color
fi
SESSION_NAME="argo_sim"
# One launcher owns the PX4/Kit lifecycle. Re-running attaches to it.
exec 8>"/tmp/argo-sim-runner-$UID.lock"
if ! flock -n 8; then
    python3 "$ARGO_WS_DIR/simulator/tools/runtime/sim_session.py" check-vehicle || exit $?
    echo "ARGO simulator is already running; attaching to its dashboard."
    exec tmux attach-session -t "$SESSION_NAME" 8>&-
fi
ISAAC_LOG="/tmp/argo_isaac_sim.log"
rm -f "$ISAAC_LOG"

# 1. 이전 잔여 프로세스 및 기존 tmux 세션 정리
python3 "$ARGO_WS_DIR/simulator/tools/runtime/cleanup_stale_argo.py"
tmux kill-session -t $SESSION_NAME 2>/dev/null || true
pkill -9 -f '(^|/)MicroXRCEAgent( |$)' 2>/dev/null || true
pkill -9 -f '(^|/)px4( |$)' 2>/dev/null || true
if [ "$ARGO_WINDOWS_HOST" = 1 ] && [ -f "$TASKKILL_BIN" ]; then
    "$TASKKILL_BIN" /F /IM kit.exe 2>/dev/null || true
fi
sleep 0.5

# 2. 세션 종료 시 전체 프로세스 강제 회수(Trap) 및 비정상 종료 에러 출력
cleanup() {
    python3 "$ARGO_WS_DIR/simulator/tools/runtime/sim_session.py" clear --pid "$$"
    if [ -n "${PX4_GATEWAY_PID:-}" ]; then
        kill -TERM "$PX4_GATEWAY_PID" 2>/dev/null || true
        for _gateway_wait in {1..30}; do
            kill -0 "$PX4_GATEWAY_PID" 2>/dev/null || break
            sleep 0.1
        done
        # A blocked DDS shutdown must never leave an old gateway on port 17449.
        kill -KILL -- "-$PX4_GATEWAY_PID" 2>/dev/null || true
        wait "$PX4_GATEWAY_PID" 2>/dev/null || true
    fi
    echo ""
    echo "=========================================================="
    echo " [ARGO Simulator] 대시보드 종료: 모든 프로세스를 정리합니다..."
    echo "=========================================================="
    tmux kill-session -t $SESSION_NAME 2>/dev/null || true
    # [ARGO 변경] 파일명 전체 검색 대신 이 프로젝트의 실제 argv/자식 프로세스만 정리합니다.
    python3 "$ARGO_WS_DIR/simulator/tools/runtime/cleanup_stale_argo.py"
    pkill -9 -f '(^|/)MicroXRCEAgent( |$)' 2>/dev/null || true
        pkill -9 -f '(^|/)px4( |$)' 2>/dev/null || true
    if [ "$ARGO_WINDOWS_HOST" = 1 ] && [ -f "$TASKKILL_BIN" ]; then
        "$TASKKILL_BIN" /F /IM kit.exe 2>/dev/null || true
    fi
    if [ -f "$ISAAC_LOG" ] && grep -qE "Traceback|ImportError|KeyError|FileNotFoundError|ModuleNotFoundError" "$ISAAC_LOG" 2>/dev/null; then
        echo "----------------------------------------------------------"
        echo " ⚠️ [진단 로그] Isaac Sim (argo_sim.py) 실행 중 발생한 에러:"
        tail -n 25 "$ISAAC_LOG"
        echo "----------------------------------------------------------"
    fi
    echo " [ARGO Simulator] 모든 프로세스 및 포트 정리 완료!"
}
trap cleanup EXIT
trap 'exit 130' SIGINT SIGTERM

# Record the domain before starting children. Navigation in another terminal
# joins this session instead of silently waiting on a different ROS domain.
python3 "$ARGO_WS_DIR/simulator/tools/runtime/sim_session.py" write --pid "$$"
echo " [ROS] aircraft=$ARGO_VEHICLE domain=$ROS_DOMAIN_ID scene=${ARGO_SUPER_SCENE:-warehouse}"

# 3. Micro-XRCE-DDS Agent 백그라운드 기동
echo " [1/3] Micro-XRCE-DDS Agent 기동 [UDP 8888]..."
MicroXRCEAgent udp4 -p 8888 > /tmp/micro_xrce_agent.log 2>&1 8>&- &
# [ARGO 변경] Ubuntu는 PX4와 Fast DDS로 직접 통신합니다. WSL 우회 게이트웨이만 조건부 실행합니다.
if [ "$ARGO_ROS_TRANSPORT" = cyclone_tcp ]; then
    setsid python3 "$ARGO_WS_DIR/simulator/tools/runtime/px4_dds_gateway.py" > /tmp/argo-px4-gateway.log 2>&1 8>&- &
    PX4_GATEWAY_PID=$!
    python3 "$ARGO_WS_DIR/simulator/tools/rviz/ensure_ros_daemon.py" --restart
fi
sleep 1

# Spawn the pane commands directly. An interactive shell would re-source
# ~/.bashrc and can overwrite ROS_DOMAIN_ID, DISPLAY, or transport settings.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
_pane_env=(env)
for _name in ARGO_VEHICLE ARGO_SUPER_SCENE ARGO_SUPER_TERRAIN_USD ARGO_SUPER_MISSION_FILE ARGO_SCENE_VIEW ARGO_HEADLESS PATH LD_LIBRARY_PATH ARGO_ENV_FILE ARGO_DIAGNOSTIC_STATE ARGO_CONTACT_REPORT_PATH ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION ARGO_ROS_TRANSPORT ARGO_WS_DIR PX4_DIR ISAAC_PYTHON ISAAC_SIM_PATH ISAAC_WINDOWS_PYTHON_BAT ARGO_SIM_PLATFORM PEGASUS_FASTLIVO_CONFIG_DIR PEGASUS_SLAM_SCENE PEGASUS_BIND_HOST PX4_SIM_HOSTNAME DISPLAY WAYLAND_DISPLAY XAUTHORITY XDG_RUNTIME_DIR; do
    if [[ -v $_name ]]; then
        _pane_env+=("$_name=${!_name}")
    fi
done
printf -v _px4_command '%q ' "${_pane_env[@]}" bash "$SCRIPT_DIR/run_px4_sim.sh"
printf -v _isaac_command '%q ' "${_pane_env[@]}" bash "$SCRIPT_DIR/run_isaac_sim.sh"
echo " [2/3] tmux 분할 대시보드 생성 중..."
tmux new-session -d -s "$SESSION_NAME" -n "ARGO_SITL" "$_px4_command"
tmux set -t $SESSION_NAME -g mouse on
tmux set -t $SESSION_NAME -g pane-border-status top
tmux set -t $SESSION_NAME -g pane-border-format " #{pane_title} "

# 클립보드 자동 복사 바인딩: 드래그 종료 시 윈도우 clip.exe로 즉시 복사
if [ "$ARGO_WINDOWS_HOST" = 1 ] && [ -f "$CLIP_BIN" ]; then
    tmux bind-key -t $SESSION_NAME -T copy-mode MouseDragEnd1Pane send-keys -X copy-pipe-and-cancel "$CLIP_BIN" 2>/dev/null || true
    tmux bind-key -t $SESSION_NAME -T copy-mode-vi MouseDragEnd1Pane send-keys -X copy-pipe-and-cancel "$CLIP_BIN" 2>/dev/null || true
fi

# Both panes own real PTYs, including PX4's interactive console.
tmux select-pane -t "$SESSION_NAME:0.0" -T "🔴 PX4 비행 콘솔 (종료: Ctrl+C)"
echo " [3/3] Pegasus / Isaac Sim 3D 뷰어 연결 중..."
tmux split-window -v -p 40 -t "$SESSION_NAME:0.0" "$_isaac_command"
tmux select-pane -t "$SESSION_NAME:0.1" -T "🟢 Pegasus / Isaac Sim (종료: Ctrl+C)"

# 7. 상단 콘솔로 포커스 맞추고 대시보드 화면으로 전환
tmux select-pane -t $SESSION_NAME:0.0
tmux attach-session -t $SESSION_NAME
