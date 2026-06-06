#!/usr/bin/env python3

"""
Stream a USB camera to ``/camera`` as a JPEG-encoded sensor_msgs/Image.

The published ``Image.data`` carries the raw JPEG byte stream
(``encoding = 'jpeg'``). The student library decodes it with
``cv2.imdecode(np.frombuffer(msg.data, np.uint8))``, so the frame is published
JPEG-compressed rather than as a raw RGB buffer.

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


class CameraNode(Node):
    """Publish MJPG frames from a USB camera onto ``/camera``."""

    def __init__(self):
        super().__init__('camera_node')

        self.video_device = self.declare_parameter(
            'video_device', '/dev/osrbot_usb_cam').value
        self.width = self.declare_parameter('image_width', 640).value
        self.height = self.declare_parameter('image_height', 480).value
        self.fps = self.declare_parameter('framerate', 30.0).value
        self.jpeg_quality = self.declare_parameter('jpeg_quality', 80).value
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
                ok, _ = cap.read()
                if ok:
                    self.cap = cap
                    self._fail_count = 0
                    self.get_logger().info(f'[INFO] Camera opened at /dev/video{i}')
                    return
                cap.release()

        self.get_logger().warn(
            '[WARN] No working camera found; will keep retrying...',
            throttle_duration_sec=5.0)

    def _tick(self):
        """Grab one frame, JPEG-encode it, and publish it to ``/camera``."""
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

        ok, jpeg = cv2.imencode(
            '.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)])
        if not ok:
            return

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = 'jpeg'
        msg.is_bigendian = False
        msg.step = frame.shape[1] * 3
        msg.data = np.asarray(jpeg).tobytes()
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
