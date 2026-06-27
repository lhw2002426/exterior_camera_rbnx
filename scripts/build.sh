#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
#
# Build phase for exterior_camera_rbnx.
#
# Pure Python — no colcon, no vendored ROS msgs. We only need
# `rbnx codegen` to materialize the atlas_pb2 / atlas_pb2_grpc stubs
# so the start script can RegisterPrimitive on atlas. ROS imports
# (rclpy, sensor_msgs, cv_bridge) come from the system /opt/ros
# overlay sourced in scripts/start.sh.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
CLEAN="${RBNX_BUILD_CLEAN:-}"

if [[ "$CLEAN" == "1" ]]; then
    echo "[exterior_camera/build] clean: removing rbnx-build/"
    rm -rf rbnx-build
fi
mkdir -p rbnx-build

# Codegen — only proto stubs are needed (no MCP).
if command -v rbnx &>/dev/null; then
    FLAGS=(--out-dir "$PKG/rbnx-build/codegen")
    [[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
    echo "[exterior_camera/build] rbnx codegen ${FLAGS[*]}"
    rbnx codegen -p "$PKG" "${FLAGS[@]}" || true
else
    echo "[exterior_camera/build] WARN: rbnx CLI not found, skipping codegen"
fi

# Soft-check runtime deps (Python side; ROS deps are checked in start.sh).
python3 -c "import cv2, numpy" 2>/dev/null || \
    echo "[WARN] Missing: pip install opencv-python numpy"

touch "$PKG/rbnx-build/.rbnx-built"
echo "[exterior_camera/build] done."
