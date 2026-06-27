#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
#
# Build phase for exterior_camera_rbnx.
#
# Pure Python — no colcon, no vendored ROS msgs. We only need
# `rbnx codegen` to materialize:
#   - the atlas_pb2 / atlas_pb2_grpc stubs (robonix_api uses these)
#   - the lifecycle Driver Servicer (auto-bound via @exterior_camera.on_*)
# ROS imports (rclpy, sensor_msgs) come from the system /opt/ros overlay
# sourced in scripts/start.sh.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
CLEAN="${RBNX_BUILD_CLEAN:-}"

if [[ "$CLEAN" == "1" ]]; then
    echo "[exterior_camera/build] clean: removing rbnx-build/"
    rm -rf rbnx-build
fi
mkdir -p rbnx-build

# Codegen — proto stubs + lifecycle Driver servicer.
if command -v rbnx &>/dev/null; then
    FLAGS=(--out-dir "$PKG/rbnx-build/codegen")
    [[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
    echo "[exterior_camera/build] rbnx codegen ${FLAGS[*]}"
    rbnx codegen -p "$PKG" "${FLAGS[@]}" || true
else
    echo "[exterior_camera/build] WARN: rbnx CLI not found, skipping codegen"
fi

# Soft-check runtime deps (Python side; ROS deps are checked at start time).
python3 -c "import cv2, numpy" 2>/dev/null || \
    echo "[WARN] Missing: pip install opencv-python numpy"

touch "$PKG/rbnx-build/.rbnx-built"
echo "[exterior_camera/build] done."
