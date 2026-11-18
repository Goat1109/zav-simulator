#!/usr/bin/env bash
set -eo pipefail
_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export ZAV_SUPER_SCENE="${1:-high_speed}"
case "$ZAV_SUPER_SCENE" in high_speed|dense|mixed|dasan|frontier) ;; *) echo 'Usage: ./run_super_sim.sh high_speed|dense|mixed|dasan|frontier' >&2; exit 2 ;; esac
case "${2:-}" in
    '') export ZAV_SCENE_VIEW=chase ;;
    --overview) export ZAV_SCENE_VIEW=overview ;;
    *) echo 'Optional second argument: --overview' >&2; exit 2 ;;
esac
exec "$_workspace/run_zav_sim.sh"
