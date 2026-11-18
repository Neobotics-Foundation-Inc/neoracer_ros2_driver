#!/usr/bin/env python3
"""
NeoRacer pure pursuit service: chase YOLO detections, tuning dashboard.

Same pattern as the wallfollow dashboard (stdlib HTTP + rclpy) on port 8083.
Subscribes to the driver inference node's detections
(vision_msgs/Detection2DArray on /edgetpu/inference): the highest-scoring
detections (up to max_detections) above score_threshold are candidates and
the largest box is the target. PD steering on the target's horizontal
offset; throttle runs at `speed` and brakes as the box area grows toward
`safety_area` (full stop at or beyond it, so it never rams the target).

If no object is found the car KEEPS DOING THE LAST ACTUATED COMMAND, speed
and angle both, republished continuously so the mux never zeroes it. The
handset's mode switch is the physical gate; the shipped yaml has speed 0.0.
Steering is negated on publish: wire positive turns this car left
(REP-103, physically verified).

The dashboard shows the model the inference node is serving (read from the
driver's inference.yaml with the car profile's override) and the camera's
live frame rate. Saved tuning goes to the car profile
(dashboards/pursuit.yaml) over the shipped pursuit.yaml.
"""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import signal
import sys
import threading
import time

from ackermann_msgs.msg import AckermannDriveStamped
import cv2
from neoracer_ros2_driver import car_profile
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
import yaml

PORT = 8083
BASE = Path(__file__).resolve().parent
# The shared styles, fonts and logos are served from scripts/web/ (webui.py).
sys.path.insert(0, str(BASE.parents[1]))
import webui  # noqa: E402, I100

# Shipped defaults stay untouched; saved tuning goes to the per-car file,
# which loads on top of them, in the car profile (dashboards/<name>.yaml).
DEFAULTS_PATH = BASE / 'pursuit.yaml'
YAML_PATH = car_profile.dashboard_settings('pursuit')

PUBLISH_RATE_HZ = 15.0  # keep-last republish rate toward the mux
PREVIEW_RATE_HZ = 15.0

_lock = threading.Lock()
_params: dict = {}
_state: dict = {
    'detections': [],
    'target': None,
    'error': None,
    'area': None,
    'steer': 0.0,
    'speed_cmd': 0.0,
    'res': [0, 0],
    'model': '?',
    'cam_fps': 0.0,
}
_preview = b''


def load_params():
    with _lock:
        _params.update(yaml.safe_load(DEFAULTS_PATH.read_text()))
        if YAML_PATH.exists():
            _params.update(yaml.safe_load(YAML_PATH.read_text()) or {})
        _params['speed'] = max(0.0, min(1.0, float(_params['speed'])))
        _params.setdefault('max_detections', 5)  # older yaml files lack this key


def save_params():
    """Rewrite values in place so the yaml comments survive."""
    if not YAML_PATH.exists():
        car_profile.write_text(YAML_PATH, DEFAULTS_PATH.read_text())
    text = YAML_PATH.read_text()
    with _lock:
        for k, v in _params.items():
            if isinstance(v, (int, float)):
                text = re.sub(rf'(?m)^{k}:\s*[-\d.eE+]+', f'{k}: {v}', text)
    car_profile.write_text(YAML_PATH, text)


def read_model_name(config_dir=None):
    """
    Return the model file the driver's inference node is serving.

    It is inference.yaml's model_path with the car profile's
    params/inference.yaml on top, as the node reads it at launch. config_dir
    None reads the installed config.
    """
    try:
        if config_dir is None:
            from ament_index_python.packages import get_package_share_directory

            config_dir = Path(get_package_share_directory('neoracer_ros2_driver')) / 'config'
        params = car_profile.node_params('inference', 'inference_node', config_dir)
        return Path(params['model_path']).name
    except (OSError, KeyError, TypeError, yaml.YAMLError):
        return 'unknown'


class PursuitNode(Node):
    def __init__(self):
        super().__init__('pursuit')
        with _lock:
            cam_topic = _params['camera_topic']
            det_topic = _params['detections_topic']
        self._pub = self.create_publisher(AckermannDriveStamped, '/drive', 1)
        self.create_subscription(Image, cam_topic, self._image_cb, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, det_topic, self._det_cb, 10)
        # Keep-last actuation: whatever was last commanded goes out at a
        # fixed rate whether or not detections are arriving.
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._actuate)
        self._last_preview = 0.0
        self._last_error = 0.0
        self._speed_cmd = 0.0
        self._steer_cmd = 0.0
        self._res = (640, 480)
        self._frame_stamps = []
        with _lock:
            _state['model'] = read_model_name()

    # ---- camera: preview + fps only ----

    def _image_cb(self, msg):
        global _preview
        now = time.monotonic()
        self._frame_stamps.append(now)
        self._frame_stamps = [t for t in self._frame_stamps if now - t < 2.0]
        with _lock:
            _state['cam_fps'] = round(len(self._frame_stamps) / 2.0, 1)
        if now - self._last_preview < 1.0 / PREVIEW_RATE_HZ:
            return
        self._last_preview = now
        if msg.encoding == 'jpeg':
            arr = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        else:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            if msg.encoding == 'rgb8':
                arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        if arr is None:
            return
        h, w = arr.shape[:2]
        self._res = (w, h)
        with _lock:
            p = dict(_params)
        pw = int(p['preview_width'])
        small = cv2.resize(arr, (pw, max(1, round(pw * h / w))))
        quality = [cv2.IMWRITE_JPEG_QUALITY, int(p['preview_quality'])]
        with _lock:
            _preview = cv2.imencode('.jpg', small, quality)[1].tobytes()

    # ---- detections -> control ----

    def _det_cb(self, msg):
        with _lock:
            p = dict(_params)
        dets = []
        for d in msg.detections:
            score, label = 1.0, ''
            if d.results:
                score = float(d.results[0].hypothesis.score)
                label = str(d.results[0].hypothesis.class_id)
            dets.append(
                {
                    'cx': d.bbox.center.position.x,
                    'cy': d.bbox.center.position.y,
                    'w': d.bbox.size_x,
                    'h': d.bbox.size_y,
                    'score': round(score, 3),
                    'label': label,
                }
            )
        # Candidates: above threshold, best scores first, capped at max_detections.
        dets = [d for d in dets if d['score'] >= p['score_threshold']]
        dets.sort(key=lambda d: d['score'], reverse=True)
        dets = dets[: int(p['max_detections'])]
        target = max(dets, key=lambda d: d['w'] * d['h'], default=None)

        if target is None:
            # Keep doing the last command. Only the display updates.
            self._update_state(dets, None, None, None)
            return

        W, H = self._res
        error = (target['cx'] - W / 2) / (W / 2)  # -1 left .. +1 right
        cmd = p['angle_kp'] * error + p['angle_kd'] * (error - self._last_error)
        self._last_error = error
        self._steer_cmd = max(-1.0, min(1.0, cmd))

        # Brake as the box area approaches safety_area; stop at or past it.
        area = (target['w'] * target['h']) / (W * H)
        target_speed = p['speed'] * max(0.0, 1.0 - area / max(p['safety_area'], 1e-3))
        self._speed_cmd += p['speed_kp'] * (target_speed - self._speed_cmd)
        self._speed_cmd = max(0.0, min(1.0, self._speed_cmd))
        self._update_state(dets, target, error, area)

    def _actuate(self):
        out = AckermannDriveStamped()
        out.drive.speed = float(self._speed_cmd)
        out.drive.steering_angle = float(-self._steer_cmd)  # wire positive = left
        self._pub.publish(out)

    def _update_state(self, dets, target, error, area):
        with _lock:
            _state.update(
                {
                    'detections': dets,
                    'target': target,
                    'error': None if error is None else round(error, 3),
                    'area': None if area is None else round(area, 4),
                    'steer': round(self._steer_cmd, 3),
                    'speed_cmd': round(self._speed_cmd, 3),
                    'res': list(self._res),
                }
            )


class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def do_GET(self):
        asset = webui.static_file(self.path)
        if asset is not None:
            self.send_response(200)
            self.send_header('Content-Type', asset[1])
            self.send_header('Content-Length', str(len(asset[0])))
            self.send_header('Cache-Control', webui.CACHE_CONTROL)
            self.end_headers()
            self.wfile.write(asset[0])
            return
        # The page also answers with a query string. theme.js reads ?theme=,
        # which a link from another port can carry: each port keeps its own storage.
        if self.path.split('?', 1)[0] == '/':
            self._send((BASE / 'pursuit.html').read_bytes(), 'text/html; charset=utf-8')
        elif self.path.startswith('/frame'):
            with _lock:
                data = _preview
            if data:
                self._send(data, 'image/jpeg', cache='no-store')
            else:
                self.send_error(503)
        elif self.path == '/state':
            with _lock:
                body = json.dumps({**_state, 'params': _params})
            self._send(body.encode(), 'application/json')
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path == '/params':
            data = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            with _lock:
                for k in (
                    'speed',
                    'speed_kp',
                    'safety_area',
                    'angle_kp',
                    'angle_kd',
                    'score_threshold',
                    'max_detections',
                ):
                    if k in data:
                        _params[k] = float(data[k])
                _params['speed'] = max(0.0, min(1.0, _params['speed']))
                _params['max_detections'] = max(1, int(_params['max_detections']))
        elif self.path == '/save':
            save_params()
        elif self.path == '/load':
            load_params()
        else:
            self.send_error(404)
            return
        self._send(b'ok', 'text/plain')

    def _send(self, body, ctype, cache=None):
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        if cache:
            self.send_header('Cache-Control', cache)
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        pass


def _spin(node):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


def main():
    load_params()
    rclpy.init()
    node = PursuitNode()
    spin = threading.Thread(target=_spin, args=(node,), daemon=True)
    spin.start()
    server = ThreadingHTTPServer(('0.0.0.0', PORT), Handler)

    # rclpy.init() installs SIGINT/SIGTERM handlers that shut the ROS context
    # down but leave serve_forever() blocked, so the process outlives the
    # signal until systemd's TimeoutStopSec expires and SIGKILLs it. Take the
    # signals back. shutdown() has to run off the serving thread or it
    # deadlocks waiting for the loop it is called from.
    def stop(*_):
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    print(f'Pure pursuit dashboard on http://0.0.0.0:{PORT}')
    server.serve_forever()

    # Unwind the ROS wait before the interpreter tears the context down;
    # otherwise the spin thread aborts the process from C++.
    rclpy.shutdown()
    spin.join(timeout=2)


if __name__ == '__main__':
    main()
