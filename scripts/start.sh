#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
#
# Start phase. Two steps in a single process (atlas_register_and_launch.py):
#   1. RegisterPrimitive on atlas — so rbnx boot's wait_for_registration
#      loop unblocks and `rbnx caps` shows the package.
#   2. Spawn `python3 -m exterior_camera.camera_node` which opens the
#      V4L2 device and publishes /exterior_camera/color/image_raw.
#
# No Driver(CMD_INIT) handler — see file header in
# scripts/atlas_register_and_launch.py.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u

# Codegen output (atlas_pb2 + atlas_pb2_grpc) on PYTHONPATH.
CODEGEN="$PKG/rbnx-build/codegen/proto_gen"
if [[ ! -d "$CODEGEN" ]]; then
    echo "[exterior_camera/start] ERR: codegen output missing at $CODEGEN" >&2
    echo "[exterior_camera/start]      Run \`bash scripts/build.sh\` first." >&2
    exit 2
fi
export PYTHONPATH="$CODEGEN:$PKG:${PYTHONPATH:-}"

exec python3 -u "$PKG/scripts/atlas_register_and_launch.py"
