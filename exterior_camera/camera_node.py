#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""exterior_camera.camera_node — V4L2 USB camera → ROS2 sensor_msgs/Image.

Standalone rclpy node, spawned as a child by `exterior_camera.main`.
Config is provided via command-line arguments (NOT env, NOT robonix
config). The robonix lifecycle layer lives in `main.py`; this file
is just the publisher.

Usage (standalone, no robonix):
    source /opt/ros/humble/setup.bash
    python3 -u -m exterior_camera.camera_node \\
        --device /dev/video11 \\
        --topic /exterior_camera/color/image_raw \\
        --frame-id exterior_camera \\
        --fps 30 \\
        --encoding bgr8

QoS:
    RELIABLE + KEEP_LAST(10) + VOLATILE. Strict consumers (RELIABLE
    subscribers) and lax ones (BEST_EFFORT, used by vla_client) are
    both compatible — DDS QoS matching rule: publisher must be at
    least as strict as the subscriber.
"""
from __future__ import annotations

import argparse
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


class ExteriorCameraPublisher(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("exterior_camera_publisher")

        self.device     = args.device
        self.topic_name = args.topic
        self.frame_id   = args.frame_id
        self.fps        = float(args.fps)
        self.width      = int(args.width)
        self.height     = int(args.height)
        self.encoding   = args.encoding.lower()
        if self.encoding not in ("bgr8", "rgb8"):
            self.get_logger().warning(
                f"unsupported --encoding {self.encoding!r}, falling back to bgr8"
            )
            self.encoding = "bgr8"

        # RELIABLE: strict subscribers happy; BEST_EFFORT also compatible.
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
        if args.buffer_size > 0:
            self.capture.set(cv2.CAP_PROP_BUFFERSIZE, int(args.buffer_size))
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

        # OpenCV returns BGR. Convert if rgb8 was requested.
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


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Exterior camera publisher")
    p.add_argument("--device",      default="/dev/video11")
    p.add_argument("--topic",       default="/exterior_camera/color/image_raw")
    p.add_argument("--frame-id",    default="exterior_camera")
    p.add_argument("--fps",         type=float, default=30.0)
    p.add_argument("--width",       type=int,   default=0,  help="0 = device default")
    p.add_argument("--height",      type=int,   default=0,  help="0 = device default")
    p.add_argument("--encoding",    default="bgr8", choices=["bgr8", "rgb8"])
    p.add_argument("--buffer-size", type=int,   default=1)
    return p


def main(argv=None):
    args = _build_parser().parse_args(argv)
    rclpy.init()
    node = None
    try:
        node = ExteriorCameraPublisher(args)
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
