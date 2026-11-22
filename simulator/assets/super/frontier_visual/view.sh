#!/usr/bin/env bash
set -euo pipefail
_asset_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
_workspace="$(cd -- "$_asset_dir/../../../.." && pwd)"
python3 "$_workspace/simulator/tools/super/restore_frontier_assets.py"
source "$_workspace/.argo.env"
exec env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u LD_LIBRARY_PATH \
  PYTHONNOUSERSITE=1 ISAAC_PATH="$ISAAC_SIM_PATH" \
  "$ISAAC_PYTHON" "$_asset_dir/viewer.py" "$@"
