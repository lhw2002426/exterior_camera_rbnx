#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""exterior_camera.camera_node — V4L2 USB camera → ROS2 sensor_msgs/Image.

Standalone rclpy node (NOT a robonix Skill). Lifecycle is controlled
by atlas_register_and_launch.py: it spawns this script as a child
process group and forwards SIGTERM on shutdown.

Config is provided via env (start.sh / atlas_register_and_launch.py
forward whatever the operator set on the rbnx boot shell):

    EXTERIOR_CAMERA_DEVICE      default /dev/video11
    EXTERIOR_CAMERA_TOPIC       default /exterior_camera/color/image_raw
    EXTERIOR_CAMERA_FRAME_ID    default exterior_camera
    EXTERIOR_CAMERA_FPS         default 30.0
    EXTERIOR_CAMERA_WIDTH       default 0      (0 = use device default)
    EXTERIOR_CAMERA_HEIGHT      default 0
    EXTERIOR_CAMERA_ENCODING    default bgr8   (matches OpenCV's default
                                                and what most consumers
                                                are happy with; vla_client
                                                decodes both rgb8/bgr8)

QoS:
    RELIABLE + KEEP_LAST(10) + VOLATILE. Matches what was used during
    bring-up — strict consumers (RELIABLE subscribers) are happy, and
    BEST_EFFORT subscribers (vla_client uses that) are still able to
    receive (the compatibility rule is "publisher must be at least as
    strict as the subscriber").
"""
from __future__ import annotations

import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image


def _env(name: str, default: str) -> str:
    v = os.environ.get(name, "").strip()
    return v if v else default


def _env_float(name: str, default: float) -> float:
    try:
        return float(_env(name, str(default)))
    except ValueError:
        return default


def _env_int(name: str, default: int) -> int:
    try:
        return int(_env(name, str(default)))
    except ValueError:
        return default


class ExteriorCameraPublisher(Node):
    def __init__(self):
        super().__init__("exterior_camera_publisher")

        self.device     = _env("EXTERIOR_CAMERA_DEVICE",   "/dev/video11")
        self.topic_name = _env("EXTERIOR_CAMERA_TOPIC",    "/exterior_camera/color/image_raw")
        self.frame_id   = _env("EXTERIOR_CAMERA_FRAME_ID", "exterior_camera")
        self.fps        = _env_float("EXTERIOR_CAMERA_FPS",    30.0)
        self.width      = _env_int("EXTERIOR_CAMERA_WIDTH",     0)
        self.height     = _env_int("EXTERIOR_CAMERA_HEIGHT",    0)
        self.encoding   = _env("EXTERIOR_CAMERA_ENCODING", "bgr8").lower()
        if self.encoding not in ("bgr8", "rgb8"):
            self.get_logger().warning(
                f"unsupported EXTERIOR_CAMERA_ENCODING={self.encoding!r}, "
                f"falling back to bgr8"
            )
            self.encoding = "bgr8"

        # RELIABLE: strict subscribers are happy; BEST_EFFORT subscribers
        # are also compatible. KEEP_LAST(10) bounds the queue.
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(Image, self.topic_name, image_qos)

        self.capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            raise RuntimeError(f"无法打开摄像头：{self.device}")

        # Reduce internal buffering latency.
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if self.fps > 0:
            self.capture.set(cv2.CAP_PROP_FPS, self.fps)
        if self.width > 0:
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        if self.height > 0:
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        actual_w   = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h   = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.capture.get(cv2.CAP_PROP_FPS)

        self.last_read_warning_time = 0.0
        self.timer = self.create_timer(
            1.0 / max(self.fps, 0.001),
            self.publish_frame,
        )

        self.get_logger().info(
            f"Opened {self.device}: {actual_w}x{actual_h}, FPS={actual_fps:.1f}"
        )
        self.get_logger().info(
            f"Publishing {self.encoding} frames to {self.topic_name} "
            f"with RELIABLE QoS at target {self.fps:.1f} Hz "
            f"(frame_id={self.frame_id})"
        )

    def publish_frame(self) -> None:
        success, frame = self.capture.read()
        if not success or frame is None:
            # Throttle the warning to once every 2s when the device misbehaves.
            now = time.monotonic()
            if now - self.last_read_warning_time >= 2.0:
                self.get_logger().warning(
                    f"Failed to read frame from {self.device}"
                )
                self.last_read_warning_time = now
            return

        # OpenCV returns BGR. Convert if the operator asked for rgb8.
        if self.encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        h, w = frame.shape[:2]
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = h
        msg.width  = w
        msg.encoding = self.encoding
        msg.is_bigendian = False
        msg.step = w * 3
        msg.data = np.ascontiguousarray(frame).tobytes()
        self.publisher.publish(msg)

    def destroy_node(self) -> bool:
        if hasattr(self, "capture") and self.capture.isOpened():
            self.capture.release()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ExteriorCameraPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f"Error: {error}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
