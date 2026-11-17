#!/bin/sh
set -eu
if [ -d /mnt/wslg ]; then
    export DISPLAY="${DISPLAY:-:0}"
    export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
fi
remote_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
exec python3 "$remote_dir/remote.py"
