#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
#
# Start phase for exterior_camera_rbnx.
#
# Source ROS humble (for rclpy / sensor_msgs.Image — needed both by
# the spawned camera_node and by the sentinel wait in on_activate),
# then exec the Primitive driver module. The driver itself is a thin
# subprocess manager; the real publishing work happens inside the
# camera_node it spawns.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u

# Codegen outputs needed at runtime (atlas_pb2 + lifecycle servicer).
CODEGEN_PROTO="$PKG/rbnx-build/codegen/proto_gen"
if [[ ! -d "$CODEGEN_PROTO" ]]; then
    echo "[exterior_camera/start] ERR: codegen output missing at $CODEGEN_PROTO" >&2
    echo "[exterior_camera/start]      Run \`bash scripts/build.sh\` first." >&2
    exit 2
fi
export PYTHONPATH="$CODEGEN_PROTO:$PKG:${PYTHONPATH:-}"

# robonix_api lives in the robonix source tree — same lookup as
# vla_client_rbnx / piper_ctl_rbnx.
if ROBONIX_API="$(rbnx path robonix-api 2>/dev/null)"; then
    export PYTHONPATH="$ROBONIX_API:$PYTHONPATH"
fi

exec python3 -u -m exterior_camera.main
