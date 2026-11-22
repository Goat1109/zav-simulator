#!/usr/bin/env bash
export ARGO_ENV_FILE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/environment.sh"
source "$ARGO_ENV_FILE"
source "$ARGO_WS_DIR/simulator/tools/rviz/ros_env.sh" || return 1
source "$ARGO_WS_DIR/install/local_setup.bash"
