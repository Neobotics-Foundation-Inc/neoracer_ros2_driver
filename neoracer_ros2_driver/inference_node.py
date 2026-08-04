#!/usr/bin/env python3

"""
Run camera frames through a YOLO model and publish the detections.

Subscribes to ``/camera`` (the driver's native MJPG passthrough,
``encoding = 'jpeg'``) and publishes ``vision_msgs/Detection2DArray`` on
``/detections``, so student code and any downstream perception node read boxes
from the graph instead of each re-running a detector on the same frames.

The model runs through Ultralytics on the Orin's integrated GPU. Both weight
formats it loads are useful here: a ``.pt`` file runs anywhere and is what a
freshly trained model comes out as, while a TensorRT ``.engine`` exported by
``yolo export format=engine`` runs several times faster. Engines are built for
one board and one TensorRT version, so ``models/`` is a per-car drop point
rather than something the repository carries.

Inference is slower than the 60 fps camera, so the subscription keeps a depth-1
best-effort queue: while a frame is in the model the middleware holds only the
newest one behind it and drops the rest. Detections therefore track the present
rather than falling further behind a backlog. ``max_rate_hz`` caps the work
below that, leaving GPU headroom for whatever else the car is running.

Health goes to ``/diagnostics`` (``DiagnosticArray``) on a timer: inference
latency, detection counts, and whether frames are still arriving.

Dependencies: ultralytics, torch (setup phase 6, `racecar setup ml`), opencv,
numpy, vision_msgs.
"""

import os
import time

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Point2D,
    Pose2D,
)

import cv2
import numpy as np

from neoracer_ros2_driver import inference_lib as il


def load_yolo(model_path, task='detect'):
    """
    Import Ultralytics and load ``model_path``, returning the model.

    The import is deferred to here rather than done at module scope because it
    pulls torch and CUDA: several seconds and a few hundred MB that a car
    running teleop without this node should not pay, and an ImportError worth
    reporting as "run the ML setup phase" rather than as a bare traceback.
    """
    os.environ.setdefault('YOLO_VERBOSE', 'False')
    from ultralytics import YOLO
    return YOLO(model_path, task=task)


class InferenceNode(Node):
    """Publish YOLO detections for the frames arriving on the camera topic."""

    def __init__(self):
        super().__init__('inference_node')

        model_path = self.declare_parameter('model_path', 'yolo11n.pt').value
        self.device = str(self.declare_parameter('device', '0').value)
        self.imgsz = self.declare_parameter('imgsz', 640).value
        self.half = self.declare_parameter('half', False).value
        self.score_threshold = self.declare_parameter('score_threshold', 0.5).value
        self.iou_threshold = self.declare_parameter('iou_threshold', 0.45).value
        self.max_detections = self.declare_parameter('max_detections', 10).value
        self.declare_parameter('class_filter', rclpy.Parameter.Type.INTEGER_ARRAY)
        self.class_filter = self.get_parameter_or('class_filter').value or None

        image_topic = self.declare_parameter('image_topic', '/camera').value
        det_topic = self.declare_parameter('detections_topic', '/detections').value
        max_rate = self.declare_parameter('max_rate_hz', 15.0).value
        self.publish_annotated = self.declare_parameter('publish_annotated', False).value
        annotated_topic = self.declare_parameter(
            'annotated_topic', '/detections/image').value
        self.annotated_quality = self.declare_parameter('annotated_quality', 80).value
        diag_period = self.declare_parameter('diagnostics_period_sec', 1.0).value
        self.image_timeout = self.declare_parameter('image_timeout_sec', 5.0).value
        warmup = self.declare_parameter('warmup', True).value

        self.min_period = 1.0 / max_rate if max_rate and max_rate > 0 else 0.0

        self.model = self._load_model(model_path)
        if warmup:
            self._warmup()

        self.inference_count = 0
        self.detection_count = 0
        self.last_detections = 0
        self.last_inference_ms = 0.0
        self.avg_inference_ms = 0.0
        self.last_image_time = None
        self.last_infer_t = 0.0
        self.model_ok = True

        # Depth 1 + best effort: the newest frame wins and a slow model sheds the
        # rest in the middleware instead of queueing stale work.
        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.det_pub = self.create_publisher(Detection2DArray, det_topic, 10)
        self.annot_pub = (
            self.create_publisher(Image, annotated_topic, qos)
            if self.publish_annotated else None)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(Image, image_topic, self._image_cb, qos)
        self.create_timer(diag_period, self._publish_diagnostics)

        self.get_logger().info(
            f'[INFO] Inference node ready: {image_topic} -> {det_topic}')

    def _load_model(self, model_path):
        """Resolve and load the weights, or exit with a one-line reason."""
        search_dirs = [
            os.path.join(get_package_share_directory('neoracer_ros2_driver'), 'models'),
            os.getcwd(),
        ]
        resolved = il.resolve_model_path(model_path, search_dirs)

        try:
            model = load_yolo(resolved)
        except ImportError as exc:
            self.get_logger().fatal(
                f'[FATAL] ultralytics is not importable ({exc}); '
                'run `racecar setup ml` to install the GPU stack.')
            raise SystemExit(1)
        except Exception as exc:  # noqa: BLE001 - ultralytics raises bare Exception
            self.get_logger().fatal(
                f'[FATAL] Could not load model {resolved}: {exc}')
            raise SystemExit(1)

        self.get_logger().info(
            f'[INFO] Model {os.path.basename(resolved)} on device {self.device} '
            f'(imgsz {self.imgsz}, threshold {self.score_threshold})')
        return model

    def _warmup(self):
        """
        Run one throwaway frame so the first real frame is not the slow one.

        A cold model pays for CUDA context creation and, for a .pt file, cuDNN
        kernel autotuning: the better part of a second on the first call, which
        would otherwise land as a stall partway into a lap.
        """
        blank = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
        t0 = time.monotonic()
        try:
            self._predict(blank)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().fatal(f'[FATAL] Warmup inference failed: {exc}')
            raise SystemExit(1)
        self.get_logger().info(
            f'[INFO] Warmup done in {(time.monotonic() - t0) * 1000:.0f} ms')

    def _predict(self, frame):
        """Run one BGR frame through the model and return the single result."""
        return self.model.predict(
            frame,
            imgsz=self.imgsz,
            conf=self.score_threshold,
            iou=self.iou_threshold,
            max_det=max(1, self.max_detections),
            classes=self.class_filter,
            device=self.device,
            half=self.half,
            verbose=False,
        )[0]

    def _image_cb(self, msg):
        """Decode one frame, run the model, publish what it found."""
        self.last_image_time = self.get_clock().now()

        now = time.monotonic()
        if self.min_period and (now - self.last_infer_t) < self.min_period:
            return
        self.last_infer_t = now

        try:
            frame = il.decode_image(msg.data, msg.encoding, msg.height, msg.width)
        except ValueError as exc:
            self.get_logger().warn(
                f'[WARN] Frame decode failed: {exc}', throttle_duration_sec=5.0)
            return

        try:
            t0 = time.monotonic()
            result = self._predict(frame)
            elapsed_ms = (time.monotonic() - t0) * 1000.0
        except Exception as exc:  # noqa: BLE001 - torch/TensorRT raise broadly
            self.model_ok = False
            self.get_logger().error(
                f'[ERROR] Inference failed: {exc}', throttle_duration_sec=5.0)
            return

        self.model_ok = True
        self.last_inference_ms = elapsed_ms
        self.inference_count += 1
        # Exponential average; the raw per-frame figure is next to it in
        # /diagnostics for anyone chasing a stall rather than a trend.
        self.avg_inference_ms = (
            elapsed_ms if self.avg_inference_ms == 0.0
            else 0.9 * self.avg_inference_ms + 0.1 * elapsed_ms)

        height, width = frame.shape[:2]
        boxes = result.boxes
        detections = il.build_detections(
            boxes.xyxy.cpu().numpy(),
            boxes.conf.cpu().numpy(),
            boxes.cls.cpu().numpy().astype(int),
            result.names,
            width,
            height,
            self.score_threshold,
            self.max_detections,
        )

        self.det_pub.publish(self._to_msg(detections, msg.header))
        self.last_detections = len(detections)
        self.detection_count += len(detections)

        if self.annot_pub is not None and self.annot_pub.get_subscription_count() > 0:
            self._publish_annotated(result, msg.header, width, height)

    @staticmethod
    def _to_msg(detections, header):
        """Pack detection dicts into a Detection2DArray stamped like the frame."""
        array = Detection2DArray()
        array.header = header

        for entry in detections:
            det = Detection2D()
            det.header = header

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis = ObjectHypothesis()
            hypothesis.hypothesis.class_id = entry['class_id']
            hypothesis.hypothesis.score = entry['score']
            det.results.append(hypothesis)

            det.bbox = BoundingBox2D()
            det.bbox.center = Pose2D()
            det.bbox.center.position = Point2D(x=entry['cx'], y=entry['cy'])
            det.bbox.center.theta = 0.0
            det.bbox.size_x = entry['w']
            det.bbox.size_y = entry['h']

            array.detections.append(det)

        return array

    def _publish_annotated(self, result, header, width, height):
        """Publish the drawn overlay as JPEG, matching the camera topic's format."""
        ok, buf = cv2.imencode(
            '.jpg', result.plot(),
            [int(cv2.IMWRITE_JPEG_QUALITY), int(self.annotated_quality)])
        if not ok:
            return

        msg = Image()
        msg.header = header
        msg.height = height
        msg.width = width
        msg.encoding = 'jpeg'
        msg.is_bigendian = False
        msg.step = 0
        msg.data = buf.tobytes()
        self.annot_pub.publish(msg)

    def _publish_diagnostics(self):
        """Report inference health on /diagnostics for the dashboard."""
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = 'YOLO Inference'
        status.hardware_id = f'jetson_gpu:{self.device}'

        if not self.model_ok:
            status.level = DiagnosticStatus.ERROR
            status.message = 'Inference failed'
        elif self.last_image_time is None:
            status.level = DiagnosticStatus.WARN
            status.message = 'No images received yet'
        else:
            age = (self.get_clock().now() - self.last_image_time).nanoseconds / 1e9
            if age > self.image_timeout:
                status.level = DiagnosticStatus.WARN
                status.message = f'No image for {age:.1f}s'
            else:
                status.level = DiagnosticStatus.OK
                status.message = f'Running ({self.avg_inference_ms:.1f} ms avg)'

        status.values = [
            KeyValue(key='inference_count', value=str(self.inference_count)),
            KeyValue(key='detection_count', value=str(self.detection_count)),
            KeyValue(key='last_detections', value=str(self.last_detections)),
            KeyValue(key='last_inference_ms', value=f'{self.last_inference_ms:.1f}'),
            KeyValue(key='avg_inference_ms', value=f'{self.avg_inference_ms:.1f}'),
            KeyValue(key='imgsz', value=str(self.imgsz)),
            KeyValue(key='device', value=self.device),
            KeyValue(key='score_threshold', value=str(self.score_threshold)),
        ]
        msg.status.append(status)
        self.diag_pub.publish(msg)


def main(args=None):
    """Spin the inference node until shutdown."""
    rclpy.init(args=args)
    node = None
    try:
        # Constructed inside the try: a fatal model load raises SystemExit, and
        # shutdown still has to run so the context is not left initialized.
        node = InferenceNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
