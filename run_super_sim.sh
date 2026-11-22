#!/usr/bin/env bash
set -eo pipefail
_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$_workspace/simulator/scripts/vehicle_args.sh"
export ARGO_SUPER_SCENE="${1:-high_speed}"
case "$ARGO_SUPER_SCENE" in warehouse|high_speed|dense|mixed|dasan|frontier) ;; *) echo 'Usage: ./run_super_sim.sh warehouse|high_speed|dense|mixed|dasan|frontier [--vehicle argo_v1|argo_v2] [--overview]' >&2; exit 2 ;; esac
case "${2:-}" in
    '') export ARGO_SCENE_VIEW=chase ;;
    --overview) export ARGO_SCENE_VIEW=overview ;;
    *) echo 'Optional second argument: --overview' >&2; exit 2 ;;
esac
if [ "$ARGO_SUPER_SCENE" = frontier ]; then
    python3 "$_workspace/simulator/tools/super/restore_frontier_assets.py" || exit 1
    export ARGO_SUPER_TERRAIN_USD="${ARGO_SUPER_TERRAIN_USD:-$_workspace/simulator/assets/super/frontier_visual/frontier_indoor.usda}"
    if [ ! -f "$ARGO_SUPER_TERRAIN_USD" ]; then
        echo "Frontier terrain asset not found: $ARGO_SUPER_TERRAIN_USD" >&2
        echo 'Restore the photographic Frontier assets described in docs/frontier-navigation.md.' >&2
        exit 1
    fi
fi
exec "$_workspace/run_argo_sim.sh"
