#!/usr/bin/env python3
"""
NeoRacer smart-follow service: wall following and car following in one.

Same pattern as the wallfollow and pursuit dashboards (stdlib HTTP + rclpy)
on port 8085, and the two controllers are those dashboards' code unchanged:

  wall following: the wallfollow dynamic gap follower on /scan. Steer toward
      the most open heading in a front arc, with obstacles inflated by the
      car's half-width. The slider is the throttle: with speed_kp and
      speed_kd at zero the car holds that constant command; nonzero gains
      bend it toward a target derived from how much road the chosen line has.
  car following: the pursuit controller on the driver inference node's
      detections (vision_msgs/Detection2DArray on /edgetpu/inference). PD
      steering on the target box's horizontal offset; throttle brakes as the
      box area grows toward safety_area.

Both run all the time and each keeps its own state; an arbiter picks which
one reaches /drive. A target seen within target_hold seconds hands the car
to the pursuit controller, otherwise the wall follower drives. The hold
bridges dropped detection frames so the state does not flicker.

CAUTION: the NeoRacer mux forwards /drive with no ROS deadman. The handset's
mode switch is the physical gate: with the handset on, the ESP32 follows
/drive only with the switch down. With the handset off or out of range it
follows /drive too, so keep the handset on and flip the switch up to stop.
The shipped yaml has both speeds at 0.0 so the car cannot drive until a
slider is raised.

Angles use the student convention (0 = nose, positive = right). On the
NeoRacer the student angle is minus the raw scan angle (physically verified).
/drive steering is negated on publish: positive on the wire turns this car
left (REP-103, physically verified). Speed feedback comes from /odom.

Saved tuning goes to the car profile (dashboards/smartfollow.yaml) over the
shipped smartfollow.yaml, and logs go to dashboards/smartfollow/ under the
data root.
"""

from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import re
import signal
import sys
import threading
import time

from ackermann_msgs.msg import AckermannDriveStamped
import cv2
from nav_msgs.msg import Odometry
from neoracer_ros2_driver import car_profile, data_root
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from vision_msgs.msg import Detection2DArray
import yaml

PORT = 8085
BASE = Path(__file__).resolve().parent
# The shared styles, fonts and logos are served from scripts/web/ (webui.py).
sys.path.insert(0, str(BASE.parents[1]))
import webui  # noqa: E402, I100

# Shipped defaults stay untouched; saved tuning goes to the per-car file,
# which loads on top of them, in the car profile (dashboards/<name>.yaml).
DEFAULTS_PATH = BASE / 'smartfollow.yaml'
YAML_PATH = car_profile.dashboard_settings('smartfollow')
LOG_DIR = data_root.current()[0] / 'dashboards' / 'smartfollow'

MAX_SPEED = 1.0  # hard throttle cap, same idea as drive_real.set_max_speed
SEARCH_STEP = 2  # degrees between candidate headings (wall following)
HALF_CLEAR = 0.21  # meters, car half-width plus margin, inflates obstacles
CENTER_BIAS = 0.006  # score penalty per degree off-center, stops ping-pong
# The LakiBeam's live span is 135 degrees either side of the nose. Its rear
# rays are blank and read as open space, so no heading is taken from there.
LIVE_HALF_DEG = 135.0
PUBLISH_RATE_HZ = 15.0  # arbiter and /drive rate
PREVIEW_RATE_HZ = 15.0
# The live chart is 420 px wide, so sending all 1200 rows of history on every
# poll draws several samples into each pixel and costs more than the camera
# stream does. Thin it to this many points; the log csv keeps every row.
HIST_POINTS = 300

# Tunable from the dashboard, in the order each panel shows them.
WALL_KEYS = (
    'speed',
    'kp',
    'kd',
    'speed_kp',
    'speed_kd',
    'window',
    'lookahead',
    'width',
    'side_weight',
)
CAR_KEYS = (
    'speed',
    'speed_kp',
    'safety_area',
    'angle_kp',
    'angle_kd',
    'score_threshold',
    'max_detections',
)

_lock = threading.Lock()
_params: dict = {}
_state: dict = {
    'scan': [],
    'hist': [],
    'overlay': {},
    'enc_speed': 0.0,
    'marks': [],
    'state': 'wall',
    'since': None,
    'steer': 0.0,
    'speed_cmd': 0.0,
    'error': 0.0,
    'detections': [],
    'target': None,
    'car_error': None,
    'area': None,
    'car_steer': 0.0,
    'car_speed_cmd': 0.0,
    'res': [0, 0],
    'model': '?',
    'cam_fps': 0.0,
}
_preview = b''
_hist: deque = deque(maxlen=1200)
# Full rows behind the live chart. POST /logs/save snapshots this to a csv,
# so a saved log is exactly what was on the screen.
_rows: deque = deque(maxlen=1200)
# Parameter-change markers [t, label] shown on the charts, so a run can be
# judged before vs after a tuning change. Saved into logs as "# mark" lines.
_marks: deque = deque(maxlen=40)
_T0 = time.monotonic()


def _now():
    return round(time.monotonic() - _T0, 2)


def _add_mark(label):
    # Coalesce rapid updates (the speed sliders fire while dragging).
    if (
        _marks
        and _now() - _marks[-1][0] < 1.0
        and _marks[-1][1].split(' ')[0] == label.split(' ')[0]
    ):
        _marks[-1] = [_now(), label]
    else:
        _marks.append([_now(), label])


def set_max_speed(speed):
    """
    Clamp throttle to [0, MAX_SPEED].

    Every speed that leaves this program passes through here, so /drive can
    never carry more than 1.0.
    """
    return max(0.0, min(MAX_SPEED, float(speed)))


def load_params():
    with _lock:
        before = json.dumps(_params, sort_keys=True)
        _params.update(yaml.safe_load(DEFAULTS_PATH.read_text()))
        if YAML_PATH.exists():
            for k, v in (yaml.safe_load(YAML_PATH.read_text()) or {}).items():
                # The wall and car blocks merge key by key, so a car file
                # that names one value keeps the shipped others.
                if isinstance(_params.get(k), dict):
                    # A block left empty in the car file keeps the shipped one.
                    _params[k] = {**_params[k], **(v if isinstance(v, dict) else {})}
                else:
                    _params[k] = v
        _params.setdefault('target_hold', 0.7)
        for group in ('wall', 'car'):
            _params[group]['speed'] = set_max_speed(_params[group]['speed'])
        _params['car']['max_detections'] = max(1, int(_params['car']['max_detections']))
        if before != '{}' and before != json.dumps(_params, sort_keys=True):
            _add_mark('yaml load')


def save_params():
    """
    Rewrite the numeric values in place so the yaml comments survive.

    Walks the file line by line tracking which block it is inside, so a key
    that exists in both blocks (speed, speed_kp) is written back to the one
    it came from.
    """
    if not YAML_PATH.exists():
        car_profile.write_text(YAML_PATH, DEFAULTS_PATH.read_text())
    header = re.compile(r'^([A-Za-z_]\w*):[ \t]*(#.*)?$')
    scalar = re.compile(r'^([ \t]*)([A-Za-z_]\w*):([ \t]*)[-\d.eE+]+(.*)$')
    out, group = [], None
    with _lock:
        for raw in YAML_PATH.read_text().splitlines(keepends=True):
            line = raw.rstrip('\n')
            h = header.match(line)
            if h:
                group = h.group(1)
                out.append(raw)
                continue
            m = scalar.match(line)
            if m:
                indent, key, gap, tail = m.groups()
                src = _params.get(group) if indent else _params
                if not indent:
                    group = None
                v = src.get(key) if isinstance(src, dict) else None
                if isinstance(v, (int, float)):
                    out.append(f'{indent}{key}:{gap}{v}{tail}\n')
                    continue
            out.append(raw)
    car_profile.write_text(YAML_PATH, ''.join(out))


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


class SmartFollowNode(Node):
    def __init__(self):
        super().__init__('smartfollow')
        with _lock:
            cam_topic = _params['camera_topic']
            det_topic = _params['detections_topic']
        self._pub = self.create_publisher(AckermannDriveStamped, '/drive', 1)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(Image, cam_topic, self._image_cb, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, det_topic, self._det_cb, 10)
        # The arbiter actuates at a fixed rate whether or not either sensor
        # is producing, so the mux never sees a gap.
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._actuate)

        self._enc = 0.0
        self._enc_stamp = 0.0
        # wall follower
        self._last_error = 0.0
        self._last_speed_error = 0.0
        self._trim = 0.0
        self._wall_speed_cmd = 0.0
        self._wall_steer_cmd = 0.0
        self._wall_error = 0.0
        # car follower
        self._last_car_error = 0.0
        self._car_speed_cmd = 0.0
        self._car_steer_cmd = 0.0
        self._car_error = 0.0
        self._res = (640, 480)
        self._last_target_stamp = -1e9
        self._last_preview = 0.0
        self._frame_stamps = []
        with _lock:
            _state['model'] = read_model_name()

    def _odom_cb(self, msg):
        self._enc = msg.twist.twist.linear.x
        self._enc_stamp = time.monotonic()

    # ---- wall following: /scan -> steering and throttle ----

    def _mean_at(self, msg, deg, window):
        """Average range (m) in a `window`-deg cone at `deg` (0 = nose, + = right)."""
        n = len(msg.ranges)
        center = round((math.radians(-deg) - msg.angle_min) / msg.angle_increment)
        half = max(1, round(math.radians(window / 2) / msg.angle_increment))
        vals = [msg.ranges[(center + k) % n] for k in range(-half, half + 1)]
        vals = [r for r in vals if msg.range_min < r < msg.range_max]
        # No return means open space, not zero. Zero would pull toward gaps.
        return sum(vals) / len(vals) if vals else msg.range_max

    def _scan_cb(self, msg):
        with _lock:
            p = dict(_params['wall'])
        # Headings stay inside the lidar's live span, and a width of 0 or less
        # cannot stop the callback.
        p['width'] = max(SEARCH_STEP, min(float(p['width']), LIVE_HALF_DEG))

        la = max(p['lookahead'], 0.1)
        degs = list(range(-int(p['width']), int(p['width']) + 1, SEARCH_STEP))
        dists = [min(self._mean_at(msg, d, p['window']), la) for d in degs]
        # An obstacle at d blocks every heading within atan(HALF_CLEAR/d)
        # of it. The car body would sideswipe it there.
        corridor = list(dists)
        for j, d in enumerate(dists):
            if d >= la:
                continue
            block = math.degrees(math.atan2(HALF_CLEAR, d))
            span = int(block // SEARCH_STEP) + 1
            for i in range(max(0, j - span), min(len(degs), j + span + 1)):
                if abs(degs[i] - degs[j]) <= block and d < corridor[i]:
                    corridor[i] = d
        best = max(
            range(len(degs)),
            key=lambda i: corridor[i] - CENTER_BIAS * abs(degs[i]) + p['side_weight'] * degs[i],
        )
        error = degs[best] / p['width']

        cmd = p['kp'] * error + p['kd'] * (error - self._last_error)
        self._last_error = error
        cmd = max(-1.0, min(1.0, cmd))

        # Speed, kept simple: the slider IS the throttle. With speed_kp
        # and speed_kd at zero the car holds that constant command, exactly
        # like rc.drive.set_speed_angle. Nonzero gains bend the throttle
        # toward the road-shaped target using measured speed: slower into
        # corners, corrected for battery and surface. Stale odometry just
        # falls back to the constant command.
        road = corridor[best]  # the chosen heading's furthest distance
        target = p['speed'] * p['max_mps'] * min(road / la, 1.0)
        serr = target - self._enc
        gains_off = p['speed_kp'] == 0 and p['speed_kd'] == 0
        odom_stale = time.monotonic() - self._enc_stamp > 0.5
        if p['speed'] <= 1e-3:
            self._trim = 0.0
            self._wall_speed_cmd = 0.0
        elif gains_off or odom_stale:
            self._trim = 0.0
            self._wall_speed_cmd = p['speed']
        else:
            # Never integrate up while commanded but not moving (gated).
            blocked = self._enc < 0.05 and self._wall_speed_cmd > 0.3
            if serr < 0 or not blocked:
                self._trim += p['speed_kp'] * serr + p['speed_kd'] * (
                    serr - self._last_speed_error
                )
            self._trim = max(-1.0, min(0.3, self._trim))
            self._wall_speed_cmd = max(0.0, min(1.0, p['speed'] + self._trim))
        self._last_speed_error = serr
        self._wall_steer_cmd = cmd
        self._wall_error = error

        overlay = {
            'width': p['width'],
            'target_deg': degs[best],
            'la': la,
            'error': round(error, 4),
            'steer': round(cmd, 3),
            'speed_cmd': round(self._wall_speed_cmd, 3),
        }
        scan = [
            [
                round(-math.degrees(msg.angle_min + i * msg.angle_increment), 1),
                round(msg.ranges[i], 3),
            ]
            for i in range(0, len(msg.ranges), 3)
            if msg.range_min < msg.ranges[i] < msg.range_max
        ]
        with _lock:
            _state.update({'scan': scan, 'overlay': overlay})

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

    # ---- car following: detections -> steering and throttle ----

    def _det_cb(self, msg):
        with _lock:
            p = dict(_params['car'])
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
            # Hold the last pursuit command. Once target_hold expires the
            # arbiter hands the car to the wall follower instead.
            self._update_car_state(dets, None, None, None)
            return

        W, H = self._res
        error = (target['cx'] - W / 2) / (W / 2)  # -1 left .. +1 right
        cmd = p['angle_kp'] * error + p['angle_kd'] * (error - self._last_car_error)
        self._last_car_error = error
        self._car_steer_cmd = max(-1.0, min(1.0, cmd))

        # Brake as the box area approaches safety_area; stop at or past it.
        area = (target['w'] * target['h']) / (W * H)
        target_speed = p['speed'] * max(0.0, 1.0 - area / max(p['safety_area'], 1e-3))
        self._car_speed_cmd += p['speed_kp'] * (target_speed - self._car_speed_cmd)
        self._car_speed_cmd = max(0.0, min(1.0, self._car_speed_cmd))
        self._car_error = error
        self._last_target_stamp = time.monotonic()
        self._update_car_state(dets, target, error, area)

    def _update_car_state(self, dets, target, error, area):
        with _lock:
            _state.update(
                {
                    'detections': dets,
                    'target': target,
                    'car_error': None if error is None else round(error, 3),
                    'area': None if area is None else round(area, 4),
                    'car_steer': round(self._car_steer_cmd, 3),
                    'car_speed_cmd': round(self._car_speed_cmd, 3),
                    'res': list(self._res),
                }
            )

    # ---- arbiter: pick a controller and drive ----

    def _actuate(self):
        now = time.monotonic()
        with _lock:
            hold = float(_params['target_hold'])
            caps = {'car': _params['car']['speed'], 'wall': _params['wall']['speed']}
        age = now - self._last_target_stamp
        car = age <= hold

        if car:
            speed, steer, error = self._car_speed_cmd, self._car_steer_cmd, self._car_error
        else:
            speed, steer, error = self._wall_speed_cmd, self._wall_steer_cmd, self._wall_error
        # A slider at 0 stops the car even while its controller holds the last
        # command, as the car follower does between detections. The wall
        # follower's speed loop may run above its slider, so only 0 is applied.
        if set_max_speed(caps['car' if car else 'wall']) <= 1e-3:
            speed = 0.0

        out = AckermannDriveStamped()
        out.drive.speed = set_max_speed(speed)
        out.drive.steering_angle = float(-steer)  # wire positive = left on this car
        self._pub.publish(out)

        t, flag = _now(), 1 if car else 0
        _rows.append(f'{t},{error:.4f},{steer:.4f},{speed:.3f},{self._enc:.3f},{flag}\n')
        _hist.append([t, round(error, 4), flag])
        with _lock:
            _state.update(
                {
                    'state': 'car' if car else 'wall',
                    'since': round(age, 1) if age < 1e6 else None,
                    'error': round(error, 4),
                    'steer': round(steer, 3),
                    'speed_cmd': round(speed, 3),
                    'hist': _thin(_hist),
                    'marks': list(_marks),
                    'enc_speed': round(self._enc, 3),
                }
            )


def _thin(hist):
    """
    Even sample of `hist` down to HIST_POINTS, newest row always kept.

    The newest row carries the state the chart's band is drawn from, so it is
    appended when the stride would have skipped it. A state flip shorter than
    the stride can be missed; at 15 Hz that is under 0.3 s, and target_hold
    holds a handover for longer than that.
    """
    rows = list(hist)
    stride = max(1, len(rows) // HIST_POINTS)
    if stride == 1:
        return rows
    sent = rows[::stride]
    if sent[-1] is not rows[-1]:
        sent.append(rows[-1])
    return sent


def _log_number(name):
    """Log7.csv -> 7, so lists sort numerically. Anything else sorts first."""
    m = re.match(r'Log(\d+)\.csv$', name)
    return int(m.group(1)) if m else 0


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
            self._send((BASE / 'smartfollow.html').read_bytes(), 'text/html; charset=utf-8')
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
        elif self.path == '/logs':
            names = sorted((f.name for f in LOG_DIR.glob('*.csv')), key=_log_number)
            self._send(json.dumps(names).encode(), 'application/json')
        elif self.path.startswith('/logs/'):
            f = LOG_DIR / Path(self.path).name
            if f.is_file():
                self._send(f.read_bytes(), 'text/csv')
            else:
                self.send_error(404)
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path == '/params':
            data = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            with _lock:
                changed = []
                if 'target_hold' in data:
                    v = max(0.0, float(data['target_hold']))
                    if _params.get('target_hold') != v:
                        _params['target_hold'] = v
                        changed.append(f'target_hold {v:g}')
                changed += _update_group('wall', WALL_KEYS, data.get('wall', {}))
                changed += _update_group('car', CAR_KEYS, data.get('car', {}))
                _params['car']['max_detections'] = max(1, int(_params['car']['max_detections']))
                if changed:
                    _add_mark(', '.join(changed))
        elif self.path == '/logs/save':
            LOG_DIR.mkdir(parents=True, exist_ok=True)
            latest = max((_log_number(f.name) for f in LOG_DIR.glob('Log*.csv')), default=0)
            name = f'Log{latest + 1}.csv'
            rows = list(_rows)
            tmin = float(rows[0].split(',')[0]) if rows else 0.0
            with open(LOG_DIR / name, 'w') as f:
                f.write('t,error,steer,speed_cmd,speed_ms,state\n')
                f.writelines(f'# mark,{t},{label}\n' for t, label in list(_marks) if t >= tmin)
                f.writelines(rows)
            self._send(name.encode(), 'text/plain')
            return
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


def _update_group(group, keys, data):
    """Apply a posted parameter block. Caller holds _lock."""
    changed = []
    for k in keys:
        if k in data:
            v = float(data[k])
            if k == 'speed':
                v = set_max_speed(v)
            if _params[group].get(k) != v:
                _params[group][k] = v
                changed.append(f'{group}.{k} {v:g}')
    return changed


def _spin(node):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


def main():
    load_params()
    rclpy.init()
    node = SmartFollowNode()
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

    print(f'Smart follow dashboard on http://0.0.0.0:{PORT}')
    server.serve_forever()

    # Unwind the ROS wait before the interpreter tears the context down;
    # otherwise the spin thread aborts the process from C++.
    rclpy.shutdown()
    spin.join(timeout=2)


if __name__ == '__main__':
    main()
