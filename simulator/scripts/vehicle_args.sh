#!/usr/bin/env bash
# Source from a launcher to consume only aircraft selection options.
_argo_args=()
while (($#)); do
    case "$1" in
        --vehicle)
            [[ $# -ge 2 ]] || { echo '--vehicle requires argo_v1 or argo_v2' >&2; exit 2; }
            export ARGO_VEHICLE="$2"; shift 2 ;;
        --vehicle=*) export ARGO_VEHICLE="${1#*=}"; shift ;;
        *) _argo_args+=("$1"); shift ;;
    esac
done
export ARGO_VEHICLE="${ARGO_VEHICLE:-argo_v1}"
case "$ARGO_VEHICLE" in
    argo_v1|argo_v2) ;;
    *) echo "Unknown aircraft: $ARGO_VEHICLE (choose argo_v1 or argo_v2)" >&2; exit 2 ;;
esac
set -- "${_argo_args[@]}"
unset _argo_args
