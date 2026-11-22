#!/usr/bin/env bash
export ARGO_ENV_FILE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/simulator/environment.sh"
source "$ARGO_ENV_FILE"
exec bash "$ARGO_WS_DIR/simulator/scripts/run_isaac_sim.sh" "$@"
