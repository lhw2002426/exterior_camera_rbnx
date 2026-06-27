#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""exterior_camera_rbnx — V4L2 USB camera primitive.

Owns `robonix/primitive/camera_exterior/*` for the piper_grasp deploy.
Wraps an rclpy publisher node (`exterior_camera.camera_node`) that
opens a V4L2 device and publishes sensor_msgs/Image.

Lifecycle (per Robonix developer guide §5):
    on_init       — light: validate cfg, cache for activate.
    on_activate   — heavy: spawn the camera publisher subprocess, wait
                    for the first sensor_msgs/Image on the configured
                    topic as proof the V4L2 device is alive, then
                    atlas-declare the `image` topic. `driver` is
                    auto-declared by the framework via the generated
                    lifecycle Servicer.
    on_deactivate — symmetric: kill publisher subprocess.
    on_shutdown   — last-chance kill (idempotent w/ on_deactivate).

Config (from manifest's primitive[].config block, delivered via
Driver(CMD_INIT, config_json)):
    device              default "/dev/video11"  — V4L2 path
    topic               default "/exterior_camera/color/image_raw"
    frame_id            default "exterior_camera"
    fps                 default 30.0            — publish rate target
    width               default 0               — 0 = device default
    height              default 0
    encoding            default "bgr8"          — bgr8 | rgb8
    buffer_size         default 1               — V4L2 internal buffers
    sentinel_timeout_s  default 10.0
"""
from __future__ import annotations

import logging
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Optional

from robonix_api import Primitive, Ok, Err

logging.basicConfig(
    level=os.environ.get("EXTERIOR_CAMERA_LOG_LEVEL", "INFO"),
    format="[exterior_camera] %(message)s",
)
log = logging.getLogger("exterior_camera")

# Provider id MUST match the deploy manifest's `primitive: - name:`
# entry for this package.
exterior_camera = Primitive(
    id="exterior_camera",
    namespace="robonix/primitive/camera_exterior",
)

_pkg_root: Path = Path(__file__).resolve().parent.parent

# Subprocess + cached cfg. Allocated in on_activate, released in
# on_deactivate / on_shutdown.
_cam_proc: Optional[subprocess.Popen] = None
_resolved_cfg: Optional[dict[str, Any]] = None


# ── helpers ──────────────────────────────────────────────────────────────

def _kill_camera() -> None:
    """Tear down the publisher subprocess. Idempotent — safe to call
    from on_deactivate followed by on_shutdown without raising on
    the second call."""
    global _cam_proc
    proc = _cam_proc
    _cam_proc = None
    if proc is None:
        return
    if proc.poll() is not None:
        return  # already exited
    log.info("killing camera subprocess pid=%d", proc.pid)
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        log.warning("camera subprocess did not exit on SIGTERM, sending SIGKILL")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


def _spawn_camera(cfg: dict[str, Any]) -> None:
    """Spawn `python3 -m exterior_camera.camera_node` in its own process
    group so we can SIGTERM the whole tree on shutdown.

    All cfg values are converted to --cli-flags. Keep this list in
    sync with camera_node._build_parser().
    """
    global _cam_proc

    cli = [
        sys.executable, "-u", "-m", "exterior_camera.camera_node",
        "--device",      str(cfg.get("device",   "/dev/video11")),
        "--topic",       str(cfg.get("topic",    "/exterior_camera/color/image_raw")),
        "--frame-id",    str(cfg.get("frame_id", "exterior_camera")),
        "--fps",         str(cfg.get("fps",      30.0)),
        "--width",       str(int(cfg.get("width",  0))),
        "--height",      str(int(cfg.get("height", 0))),
        "--encoding",    str(cfg.get("encoding",   "bgr8")),
        "--buffer-size", str(int(cfg.get("buffer_size", 1))),
    ]
    log.info("spawning: %s", " ".join(cli))

    env = os.environ.copy()
    # Make sure `python3 -m exterior_camera.camera_node` resolves
    # regardless of where rbnx-boot exec'd us from.
    pp = env.get("PYTHONPATH", "")
    pkg_root_str = str(_pkg_root)
    if pkg_root_str not in pp.split(os.pathsep):
        env["PYTHONPATH"] = pkg_root_str + (os.pathsep + pp if pp else "")

    _cam_proc = subprocess.Popen(
        cli,
        env=env,
        start_new_session=True,
    )


def _wait_for_first_image(topic: str, timeout_s: float) -> bool:
    """Block until at least one sensor_msgs/Image lands on `topic`, or
    `timeout_s` elapses. Runs in a temporary rclpy node — we don't
    keep a long-lived ROS node in this driver process because the
    actual publishing happens in the spawned camera_node subprocess.

    Returns True on success. False on timeout. Logs the wait so that
    rbnx boot's spinner output explains what's happening.
    """
    log.info("waiting for first sensor_msgs/Image on %s (timeout %.1fs)",
             topic, timeout_s)

    import rclpy
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from sensor_msgs.msg import Image

    rclpy_was_initialized = rclpy.ok()
    if not rclpy_was_initialized:
        rclpy.init()

    node = rclpy.create_node("exterior_camera_sentinel")
    got_frame = threading.Event()

    def _cb(_msg):
        got_frame.set()

    # Subscribe with BEST_EFFORT so we're compatible with whatever
    # the publisher uses; QoS matching here is permissive on our side.
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    sub = node.create_subscription(Image, topic, _cb, qos)

    deadline = time.monotonic() + timeout_s
    ok = False
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if got_frame.is_set():
            ok = True
            break

    node.destroy_subscription(sub)
    node.destroy_node()
    if not rclpy_was_initialized:
        rclpy.shutdown()

    return ok


# ── lifecycle handlers ───────────────────────────────────────────────────

@exterior_camera.on_init
def init(cfg: dict):
    """REGISTERED → INACTIVE. Validate cfg + cache for activate.

    Light only — DO NOT touch the V4L2 device, DO NOT spawn anything,
    DO NOT declare on atlas. Heavy work belongs in on_activate so a
    CMD_DEACTIVATE → CMD_ACTIVATE re-cycle works without a half-baked
    init side effect."""
    global _resolved_cfg
    cfg = cfg or {}

    # Light validation: numeric fields must parse, encoding must be known.
    try:
        fps = float(cfg.get("fps", 30.0))
        if fps <= 0:
            return Err(f"fps must be > 0, got {fps}")
    except (TypeError, ValueError) as e:
        return Err(f"fps not numeric: {e}")

    try:
        timeout = float(cfg.get("sentinel_timeout_s", 10.0))
        if timeout <= 0:
            return Err(f"sentinel_timeout_s must be > 0, got {timeout}")
    except (TypeError, ValueError) as e:
        return Err(f"sentinel_timeout_s not numeric: {e}")

    for k in ("width", "height", "buffer_size"):
        try:
            v = int(cfg.get(k, 0))
            if v < 0:
                return Err(f"{k} must be >= 0, got {v}")
        except (TypeError, ValueError) as e:
            return Err(f"{k} not integer: {e}")

    encoding = str(cfg.get("encoding", "bgr8")).lower()
    if encoding not in ("bgr8", "rgb8"):
        return Err(f"encoding must be bgr8 or rgb8, got {encoding!r}")

    _resolved_cfg = dict(cfg)
    log.info(
        "CMD_INIT ok (device=%s, topic=%s, fps=%.1f, encoding=%s)",
        cfg.get("device",   "/dev/video11"),
        cfg.get("topic",    "/exterior_camera/color/image_raw"),
        fps,
        encoding,
    )
    return Ok()


@exterior_camera.on_activate
def activate():
    """INACTIVE → ACTIVE. Spawn the publisher subprocess, wait for the
    first sensor_msgs/Image, declare the topic on atlas.

    On any failure between spawn and declare, the camera subprocess is
    torn down before returning Err so the next CMD_ACTIVATE starts
    from a clean state."""
    cfg = _resolved_cfg or {}

    topic = str(cfg.get("topic", "/exterior_camera/color/image_raw"))
    sentinel_timeout = float(cfg.get("sentinel_timeout_s", 10.0))

    try:
        _spawn_camera(cfg)
    except Exception as e:  # noqa: BLE001
        return Err(f"spawn camera_node failed: {e}")

    if not _wait_for_first_image(topic, sentinel_timeout):
        _kill_camera()
        return Err(
            f"no sensor_msgs/Image on {topic} within "
            f"{sentinel_timeout:.1f}s — is the V4L2 device "
            f"{cfg.get('device', '/dev/video11')!r} attached and not "
            f"held by another process? `v4l2-ctl --list-devices`"
        )

    try:
        exterior_camera.declare_ros2_topic(
            "robonix/primitive/camera_exterior/image",
            topic=topic,
            qos="reliable",
            description=(
                f"Exterior USB-camera RGB stream (sensor_msgs/Image, "
                f"{cfg.get('encoding', 'bgr8')}). Source: "
                f"{cfg.get('device', '/dev/video11')} via V4L2."
            ),
        )
    except Exception as e:  # noqa: BLE001
        _kill_camera()
        return Err(f"declare_ros2_topic failed: {e}")

    log.info("CMD_ACTIVATE ok: image=%s", topic)
    return Ok()


@exterior_camera.on_deactivate
def deactivate():
    """ACTIVE → INACTIVE. Kill the camera subprocess. Idempotent."""
    _kill_camera()
    log.info("CMD_DEACTIVATE ok")
    return Ok()


@exterior_camera.on_shutdown
def shutdown():
    """any → TERMINATED. Last-chance kill. Idempotent w/ on_deactivate."""
    _kill_camera()
    return Ok()


if __name__ == "__main__":
    exterior_camera.run()
