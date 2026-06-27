#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""exterior_camera_rbnx — atlas register + ROS2 node wrapper.

Same pattern as piper_description_rbnx (intentionally — both are
primitives with NO atlas-routed contracts, just "I'm alive" providers
so rbnx boot proceeds). The script:

    1. RegisterPrimitive(id=exterior_camera,
                         namespace=robonix/primitive/camera_exterior)
       on atlas. This unblocks rbnx boot's wait_for_registration loop
       and makes the package show up under `rbnx caps`.
    2. Spawn `python3 -m exterior_camera.camera_node` as a child
       process group. The node opens the V4L2 device and publishes
       sensor_msgs/Image on /exterior_camera/color/image_raw.
    3. Heartbeat every 30 s so atlas doesn't evict us at the 90 s
       default timeout.
    4. Forward SIGTERM/SIGINT to the child process group so rbnx boot's
       teardown is clean.

What this script intentionally does NOT do:
    - Declare any capability over gRPC/ROS/MCP. ROS topics are a
      global side-channel; atlas-routing them would only add
      indirection.
    - Bind a Driver(CMD_INIT) Servicer. With no `*/driver` capability
      registered, rbnx boot sees `driver_contract=None` and skips the
      CMD_INIT/CMD_ACTIVATE handshake entirely (system providers
      auto-promote to ACTIVE).
"""
from __future__ import annotations

import os
import signal
import subprocess
import sys
import threading
import time

import grpc                       # type: ignore
import atlas_pb2 as pb            # type: ignore
import atlas_pb2_grpc as pb_grpc  # type: ignore


PROVIDER_ID    = "exterior_camera"
NAMESPACE      = "robonix/primitive/camera_exterior"
HEARTBEAT_PERIOD_S = 30.0


def _log(msg: str) -> None:
    print(f"[exterior_camera] {msg}", flush=True)


def _register_with_atlas(stub: pb_grpc.AtlasStub) -> None:
    try:
        req = pb.RegisterRequest(
            id=PROVIDER_ID,
            namespace=NAMESPACE,
            capability_md_path="",
        )
        stub.RegisterPrimitive(req, timeout=5.0)
    except grpc.RpcError as e:
        _log(f"RegisterPrimitive failed: {e.code().name} {e.details()}")
        sys.exit(2)
    _log(f"registered with atlas (id={PROVIDER_ID}, namespace={NAMESPACE})")


def _heartbeat_forever(stub: pb_grpc.AtlasStub) -> None:
    while True:
        time.sleep(HEARTBEAT_PERIOD_S)
        try:
            stub.Heartbeat(pb.HeartbeatRequest(id=PROVIDER_ID), timeout=5.0)
        except grpc.RpcError:
            pass


def _spawn_node(pkg_root: str) -> subprocess.Popen:
    """Spawn the camera publisher node as a child of its own process group
    so we can SIGTERM the whole tree on shutdown.

    All config flows through env (see package_manifest.yaml header).
    """
    _log("spawning exterior_camera.camera_node")
    for key in (
        "EXTERIOR_CAMERA_DEVICE",
        "EXTERIOR_CAMERA_TOPIC",
        "EXTERIOR_CAMERA_FRAME_ID",
        "EXTERIOR_CAMERA_FPS",
        "EXTERIOR_CAMERA_WIDTH",
        "EXTERIOR_CAMERA_HEIGHT",
    ):
        if key in os.environ:
            _log(f"  {key}={os.environ[key]}")

    env = os.environ.copy()
    # Make sure the package root is on PYTHONPATH so `import exterior_camera`
    # works regardless of where rbnx-boot exec'd us from.
    pp = env.get("PYTHONPATH", "")
    if pkg_root not in pp.split(os.pathsep):
        env["PYTHONPATH"] = pkg_root + (os.pathsep + pp if pp else "")

    return subprocess.Popen(
        ["python3", "-u", "-m", "exterior_camera.camera_node"],
        env=env,
        start_new_session=True,
    )


def main() -> int:
    pkg_root = os.environ.get(
        "RBNX_PACKAGE_ROOT",
        os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
    )

    atlas_endpoint = os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051")
    _log(f"connecting to atlas at {atlas_endpoint}")
    channel = grpc.insecure_channel(atlas_endpoint)
    stub = pb_grpc.AtlasStub(channel)

    _register_with_atlas(stub)

    threading.Thread(
        target=_heartbeat_forever,
        args=(stub,),
        name="exterior_camera-heartbeat",
        daemon=True,
    ).start()

    proc = _spawn_node(pkg_root)

    def _forward(sig, _frame):
        _log(f"got signal {sig}; forwarding to camera_node pid={proc.pid}")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
    signal.signal(signal.SIGTERM, _forward)
    signal.signal(signal.SIGINT,  _forward)

    rc = proc.wait()
    _log(f"camera_node exited rc={rc}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
