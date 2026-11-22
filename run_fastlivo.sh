#!/usr/bin/env bash
export ARGO_ENV_FILE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/simulator/environment.sh"
source "$ARGO_ENV_FILE"
exec bash "$ARGO_WS_DIR/simulator/tools/rviz/run_fastlivo.sh" rviz:=False camera:=True "$@"
