#!/usr/bin/env bash
# Qt/GTK from the host must not load VS Code Snap's private glibc libraries.
unset GTK_PATH GTK_EXE_PREFIX GTK_DATA_PREFIX GIO_MODULE_DIR LD_PRELOAD
if [ -n "${LD_LIBRARY_PATH:-}" ]; then
    _argo_gui_libs=""
    IFS=: read -ra _argo_gui_entries <<< "$LD_LIBRARY_PATH"
    for entry in "${_argo_gui_entries[@]}"; do
        case "$entry" in /snap/*|'') continue ;; esac
        _argo_gui_libs="${_argo_gui_libs:+$_argo_gui_libs:}$entry"
    done
    export LD_LIBRARY_PATH="$_argo_gui_libs"
fi
exec "$@"
