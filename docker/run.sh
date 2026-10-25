#!/bin/bash
# Run the ROS 2 Jazzy container with this repo mounted at /ws.
# Works from Git Bash on Windows (Docker Desktop + WSLg), from WSL, or on Linux.
#   docker/run.sh                       -> interactive shell
#   docker/run.sh make sim              -> any command inside the container
set -e
export MSYS_NO_PATHCONV=1   # stop Git Bash rewriting /container/paths
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${IMAGE:-visual-slam-nav:jazzy}"
NAME="${NAME:-vslam-nav}"

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    docker build -t "$IMAGE" "$REPO_DIR/docker"
fi

case "$(uname -s)" in
    MINGW*|MSYS*|CYGWIN*)
        # Docker Desktop on Windows: WSLg is exposed inside its VM here
        REPO_MOUNT="$(cd "$REPO_DIR" && pwd -W)"
        WSLG=/run/desktop/mnt/host/wslg
        HAS_DXG=1 ;;
    *)
        REPO_MOUNT="$REPO_DIR"
        WSLG=/mnt/wslg
        [ -e /dev/dxg ] && HAS_DXG=1 || HAS_DXG=0 ;;
esac

GUI_ARGS=(-e DISPLAY="${DISPLAY:-:0}")
if [ -d "$WSLG" ] || [ "$WSLG" = /run/desktop/mnt/host/wslg ]; then
    GUI_ARGS+=(-v "$WSLG/.X11-unix":/tmp/.X11-unix -v "$WSLG":/mnt/wslg
               -e WAYLAND_DISPLAY=wayland-0
               -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir
               -e PULSE_SERVER=/mnt/wslg/PulseServer)
else
    GUI_ARGS+=(-v /tmp/.X11-unix:/tmp/.X11-unix)
fi
if [ "$HAS_DXG" = 1 ] && [ "${LIBGL_ALWAYS_SOFTWARE:-0}" != 1 ]; then
    # GPU-accelerated OpenGL through Mesa's D3D12 driver (WSL2)
    GUI_ARGS+=(--device /dev/dxg -v /usr/lib/wsl:/usr/lib/wsl:ro
               -e LD_LIBRARY_PATH=/usr/lib/wsl/lib
               -e GALLIUM_DRIVER=d3d12
               -e MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}")
fi

TTY_ARGS=(-i)
[ -t 0 ] && [ -t 1 ] && TTY_ARGS=(-it)

exec docker run --rm "${TTY_ARGS[@]}" --name "$NAME" \
    --ipc host \
    -e TURTLEBOT3_MODEL=waffle \
    -e QT_X11_NO_MITSHM=1 \
    -e LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-0}" \
    "${GUI_ARGS[@]}" \
    -v "$REPO_MOUNT":/ws \
    "$IMAGE" "${@:-bash}"
