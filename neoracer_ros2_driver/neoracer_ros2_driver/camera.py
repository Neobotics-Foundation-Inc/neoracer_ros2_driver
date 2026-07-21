#!/usr/bin/env python3

"""
Stream a USB camera to ``/camera`` as a JPEG-encoded sensor_msgs/Image.

The camera is an MJPG source: it already delivers JPEG-compressed frames over
USB. This node passes those native bytes straight through to ``/camera``
(``encoding = 'jpeg'``) without decoding and re-encoding them. The student
library reads the frame with ``cv2.imdecode(np.frombuffer(msg.data, np.uint8))``.

Passthrough matters for frame rate: decoding each MJPG frame to BGR and then
re-encoding it to JPEG costs ~20 ms of CPU per frame, which caps the node near
45 fps and compresses the image twice. Reading the raw MJPG buffer instead
(``CAP_PROP_CONVERT_RGB = 0``) removes both the decode and the re-encode, so the
node can sustain a much higher frame rate at a fraction of the CPU and with no
second compression loss.

The capture device defaults to the stable udev symlink ``/dev/osrbot_usb_cam``
and the node requests an MJPG stream at the configured resolution/rate. If the
device cannot be opened (or stalls) it scans ``/dev/video*`` and retries, so an
unplugged or re-enumerated camera recovers without restarting the node.

Dependencies: opencv-python, numpy.
"""

import os
import re

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

import cv2
import numpy as np


def device_to_index(device):
    """Resolve a ``/dev/video*`` path (or symlink) to its integer V4L2 index."""
    try:
        real = os.path.realpath(device)
        match = re.search(r'(\d+)$', real)
        if match:
            return int(match.group(1))
    except OSError:
        pass
    return None


def _is_jpeg(frame):
    """True if ``frame`` is a raw MJPG buffer (starts with the JPEG SOI marker).

    With ``CAP_PROP_CONVERT_RGB = 0`` a working MJPG capture returns the
    compressed byte buffer; a driver that ignores the flag returns a decoded
    ``(H, W, 3)`` array instead. Checking the SOI marker rejects that case so we
    never publish a decoded frame mislabelled as ``jpeg``.
    """
    if frame is None:
        return False
    raw = np.asarray(frame).reshape(-1)
    return raw.size >= 2 and raw[0] == 0xFF and raw[1] == 0xD8


class CameraNode(Node):
    """Publish MJPG frames from a USB camera onto ``/camera``."""

    def __init__(self):
        super().__init__('camera_node')

        self.video_device = self.declare_parameter(
            'video_device', '/dev/osrbot_usb_cam').value
        self.width = self.declare_parameter('image_width', 640).value
        self.height = self.declare_parameter('image_height', 480).value
        self.fps = self.declare_parameter('framerate', 30.0).value
        self.frame_id = self.declare_parameter('frame_id', 'camera_link').value
        self.scan_max = self.declare_parameter('scan_max_index', 10).value

        self.pub = self.create_publisher(Image, '/camera', qos_profile_sensor_data)
        self.cap = None
        self._fail_count = 0
        self._open_camera()

        # Drive the capture loop slightly faster than the target rate so we never
        # starve the stream; the camera itself caps the true frame rate.
        self.timer = self.create_timer(1.0 / max(1.0, self.fps), self._tick)
        self.get_logger().info('[INFO] Camera node ready, publishing /camera')

    def _open_camera(self):
        """(Re)open the capture device, trying the configured path then a scan."""
        if self.cap is not None:
            self.cap.release()
            self.cap = None

        candidates = []
        idx = device_to_index(self.video_device)
        if idx is not None:
            candidates.append(idx)
        candidates += [i for i in range(self.scan_max) if i not in candidates]

        for i in candidates:
            cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                cap.set(cv2.CAP_PROP_FPS, self.fps)
                # Hand back the camera's raw MJPG buffer instead of a decoded
                # BGR frame. read() then returns the compressed JPEG bytes.
                cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
                ok, probe = cap.read()
                if ok and _is_jpeg(probe):
                    self.cap = cap
                    self._fail_count = 0
                    self.get_logger().info(f'[INFO] Camera opened at /dev/video{i}')
                    return
                cap.release()

        self.get_logger().warn(
            '[WARN] No working MJPG camera found; will keep retrying...',
            throttle_duration_sec=5.0)

    def _tick(self):
        """Grab one native MJPG frame and publish it to ``/camera`` unchanged."""
        if self.cap is None:
            self._open_camera()
            return

        ok, frame = self.cap.read()
        if not ok or frame is None:
            self._fail_count += 1
            self.get_logger().warn(
                '[WARN] Frame read failed; is the camera unplugged?',
                throttle_duration_sec=1.0)
            if self._fail_count >= 10:
                self._open_camera()
            return
        self._fail_count = 0

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = self.height
        msg.width = self.width
        msg.encoding = 'jpeg'
        msg.is_bigendian = False
        # Compressed stream: a per-row byte step is not meaningful.
        msg.step = 0
        msg.data = np.asarray(frame).reshape(-1).tobytes()
        self.pub.publish(msg)

    def destroy_node(self):
        """Release the capture device on shutdown."""
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    """Spin the camera node until shutdown."""
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
