#!/usr/bin/env bash
# A pane's Ctrl+C closes the complete simulator dashboard.
set -eo pipefail
_config="${ARGO_ENV_FILE:-${XDG_CONFIG_HOME:-$HOME/.config}/argo/simulation.env}"
source "$_config"
source "$ARGO_WS_DIR/simulator/scripts/vehicle_args.sh"
[[ $# = 0 ]] || { echo "Unknown arguments: $*" >&2; exit 2; }
trap 'tmux kill-session -t argo_sim 2>/dev/null || true' EXIT
trap 'exit 130' INT TERM
cd "$PX4_DIR"
# On WSL a connect() issued before the Windows listener exists can stay in
# SYN-SENT for minutes. Inspect the listener without consuming MAVLink's socket.
if [ "${ARGO_SIM_PLATFORM:-}" = wsl ]; then
    echo '[PX4] Waiting for Windows Isaac MAVLink listener on TCP 4560...'
    _ready=0
    for _attempt in {1..180}; do
        if /mnt/c/Windows/System32/cmd.exe /c 'netstat -ano -p tcp' 2>/dev/null | grep -E ':4560[[:space:]].*LISTENING' >/dev/null; then
            _ready=1
            break
        fi
        sleep 1
    done
    [ "$_ready" = 1 ] || { echo 'Isaac MAVLink listener did not start within 180 seconds.' >&2; exit 1; }
fi
_argo_target=isaac_argo
if [ "$ARGO_VEHICLE" = argo_v2 ]; then
    python3 "$ARGO_WS_DIR/simulator/tools/runtime/generate_argo_v2_airframe.py"
    cp "$ARGO_WS_DIR/simulator/px4/10021_isaac_argo_v2" "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/10021_isaac_argo_v2"
    _argo_target=isaac_argo_v2
fi
PX4_SIM_HOSTNAME="${PX4_SIM_HOSTNAME:-localhost}" make px4_sitl "$_argo_target"
