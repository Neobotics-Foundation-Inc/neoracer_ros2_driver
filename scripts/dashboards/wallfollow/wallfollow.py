#!/usr/bin/env python3
"""
NeoRacer wall-follow service: /scan -> PD steering -> /drive + web dashboard.

Same pattern as the driver's dashboard.py (stdlib HTTP + rclpy) on port 8081.
CAUTION: the NeoRacer mux forwards /drive with no ROS deadman. The handset's
mode switch is the physical gate: the ESP32 follows /drive only with it down.
The shipped yaml has speed 0.0 so the car cannot drive until the slider is
raised.

Two forms, selected by `mode` in wallfollow.yaml (file edit only, no UI switch):
  static:  mirrored side windows at look_angle. Error = right - left meters.
           Setpoint 0 keeps the car centered between the walls.
  dynamic: steer toward the most open heading in a front arc, with obstacles
           inflated by the car's half-width. side_weight biases one side.

Angles use the student convention (0 = nose, positive = right). The mapping
from a raw scan angle to that convention is LIDAR_MOUNT_YAW_DEG, read from the
LakiBeam's mount in frames.yaml; see the constant. /drive steering is negated
on publish: positive on the wire turns this car left (REP-103, physically
verified; the car steered mirrored without the negation). Speed feedback comes
from /odom.
"""

from collections import deque
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import math
from pathlib import Path
import re
import signal
import sys
import threading
import time

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from neoracer_ros2_driver import car_profile, data_root
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import yaml

PORT = 8081
BASE = Path(__file__).resolve().parent
# The shared styles, fonts and logos are served from scripts/web/ (webui.py).
# overview.py reads the lidar's mount from frames.yaml for the 8080 page.
sys.path.insert(0, str(BASE.parents[1]))
import overview  # noqa: E402, I100
import webui  # noqa: E402

# Shipped defaults stay untouched; saved tuning goes to the per-car file,
# which loads on top of them, in the car profile (dashboards/<name>.yaml).
DEFAULTS_PATH = BASE / 'wallfollow.yaml'
YAML_PATH = car_profile.dashboard_settings('wallfollow')
LOG_DIR = data_root.current()[0] / 'dashboards' / 'wallfollow'

MAX_SPEED = 1.0  # hard throttle cap, same idea as drive_real.set_max_speed
SEARCH_STEP = 2  # degrees between candidate headings (dynamic)
HALF_CLEAR = 0.21  # meters, car half-width plus margin, inflates obstacles
CENTER_BIAS = 0.006  # score penalty per degree off-center, stops ping-pong


def lidar_mount_yaw_deg(config_dir=None):
    """
    Return the raw scan angle that points at the car's nose, in degrees.

    The NeoRacer's LakiBeam faces forward: scan zero is the nose, over 360
    degrees at 1440 points, with the rear 90 degrees blank. frames.yaml holds
    its measured mount yaw (laser_yaw_deg, positive turns left), with the car
    profile's value on top, which is where the lidar calibration writes it.
    A ray at raw angle r points at r + laser_yaw in base_link, so the nose is
    at raw -laser_yaw. config_dir None reads the installed config, as the
    8080 overview does; a config that cannot be read gives 0.
    """
    return -math.degrees(overview.laser_mount_from_config(config_dir)[2])


# Raw scan angle that points at the car's nose, in degrees:
#
#     student_deg = LIDAR_MOUNT_YAW_DEG - raw_deg
#
# Upstream's RPLIDAR faces aft and uses 180 here. Nothing else assumes a
# sweep width: indices come from the message's own angle_min and
# angle_increment and wrap modulo the point count, which is right for the
# LakiBeam's full turn; its blank rear rays read as open space.
LIDAR_MOUNT_YAW_DEG = lidar_mount_yaw_deg()

_lock = threading.Lock()
_params: dict = {}
_state: dict = {'scan': [], 'hist': [], 'overlay': {}, 'enc_speed': 0.0}
_hist: deque = deque(maxlen=600)
# Full rows behind the live chart. POST /logs/save snapshots this to a csv,
# so a saved log is exactly what was on the screen.
_rows: deque = deque(maxlen=600)
# Parameter-change markers [t, label] shown on the charts, so a run can be
# judged before vs after a tuning change. Saved into logs as "# mark" lines.
_marks: deque = deque(maxlen=40)
_T0 = time.monotonic()


def _now():
    return round(time.monotonic() - _T0, 2)


def _add_mark(label):
    # Coalesce rapid updates (the speed slider fires while dragging).
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
        before = dict(_params)
        _params.update(yaml.safe_load(DEFAULTS_PATH.read_text()))
        if YAML_PATH.exists():
            _params.update(yaml.safe_load(YAML_PATH.read_text()) or {})
        _params['speed'] = set_max_speed(_params['speed'])
        _params.setdefault('lookahead', 3.0)  # older yaml files lack these keys
        _params.setdefault('max_mps', 4.0)
        _params.setdefault('speed_kp', 0.18)
        _params.setdefault('speed_kd', 0.05)
        # 0 = static, 1 = dynamic. Older yamls spell the mode out.
        _params['mode'] = 1 if str(_params['mode']).strip() in ('1', 'dynamic') else 0
        if before and before != _params:
            _add_mark('yaml load')


def save_params():
    """Rewrite the numeric values in place so the yaml comments survive."""
    if not YAML_PATH.exists():
        car_profile.write_text(YAML_PATH, DEFAULTS_PATH.read_text())
    text = YAML_PATH.read_text()
    with _lock:
        for k, v in _params.items():
            if k == 'mode':
                continue
            if re.search(rf'(?m)^{k}:', text):
                text = re.sub(rf'(?m)^{k}:\s*[-\d.eE+]+', f'{k}: {v}', text)
            else:
                text += f'{k}: {v}\n'
    car_profile.write_text(YAML_PATH, text)


class WallFollowNode(Node):
    def __init__(self):
        super().__init__('wallfollow')
        self._pub = self.create_publisher(AckermannDriveStamped, '/drive', 1)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos_profile_sensor_data)
        self._last_error = 0.0
        self._enc = 0.0
        self._enc_stamp = 0.0
        self._speed_cmd = 0.0
        self._trim = 0.0
        self._last_speed_error = 0.0

    def _odom_cb(self, msg):
        self._enc = msg.twist.twist.linear.x
        self._enc_stamp = time.monotonic()

    @staticmethod
    def _index_at(msg, deg):
        """Ray index for a student angle (0 = nose, positive = right)."""
        raw = math.radians(LIDAR_MOUNT_YAW_DEG - deg)
        return round((raw - msg.angle_min) / msg.angle_increment) % len(msg.ranges)

    @staticmethod
    def _student_deg(msg, i):
        """Student angle of ray `i`, wrapped to (-180, 180]."""
        raw = math.degrees(msg.angle_min + i * msg.angle_increment)
        return (LIDAR_MOUNT_YAW_DEG - raw + 180.0) % 360.0 - 180.0

    def _mean_at(self, msg, deg, window):
        """Average range (m) in a `window`-deg cone at `deg` (0 = nose, + = right)."""
        n = len(msg.ranges)
        center = self._index_at(msg, deg)
        half = max(1, round(math.radians(window / 2) / msg.angle_increment))
        vals = [msg.ranges[(center + k) % n] for k in range(-half, half + 1)]
        vals = [r for r in vals if msg.range_min < r < msg.range_max]
        # No return means open space, not zero. Zero would pull toward gaps.
        return sum(vals) / len(vals) if vals else msg.range_max

    def _scan_cb(self, msg):
        with _lock:
            p = dict(_params)

        la = max(p['lookahead'], 0.1)
        if int(p['mode']) == 1:
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
                key=lambda i: corridor[i]
                - CENTER_BIAS * abs(degs[i])
                + p['side_weight'] * degs[i],
            )
            error = degs[best] / p['width']
            overlay = {'mode': 'dynamic', 'width': p['width'], 'target_deg': degs[best], 'la': la}
        else:
            left = min(self._mean_at(msg, -p['look_angle'], p['window']), la)
            right = min(self._mean_at(msg, p['look_angle'], p['window']), la)
            error = right - left  # >0: drifted left -> steer right
            overlay = {
                'mode': 'static',
                'look_angle': p['look_angle'],
                'window': p['window'],
                'left': round(left, 2),
                'right': round(right, 2),
                'la': la,
            }

        cmd = p['kp'] * error + p['kd'] * (error - self._last_error)
        self._last_error = error
        cmd = max(-1.0, min(1.0, cmd))

        # Speed shaping, race_fast style: the slider is the throttle cap.
        # Slow when the wall ahead is inside lookahead or when steering hard,
        # and PD-track the target so the ramp is smooth.
        front = min(self._mean_at(msg, 0, p['window']), la)
        # Target speed in m/s from how much road the car actually has:
        # the shorter of the corridor straight ahead and the corridor along
        # the chosen line. Long corridor = straight = fast; corners shorten
        # it and slow the car with no separate corner term. speed_kp/kd then
        # regulate the MEASURED speed from odometry onto that target, so
        # battery sag and surface changes are corrected instead of baked in.
        if int(p['mode']) == 1:
            road = corridor[best]  # the chosen heading's furthest distance
        else:
            road = front
        # Speed, kept simple: the slider IS the throttle. With speed_kp
        # and speed_kd at zero the car holds that constant command, exactly
        # like rc.drive.set_speed_angle. Nonzero gains bend the throttle
        # toward the road-shaped target using measured speed: slower into
        # corners, corrected for battery and surface. Stale odometry just
        # falls back to the constant command.
        target = p['speed'] * p['max_mps'] * min(road / la, 1.0)
        serr = target - self._enc
        gains_off = p['speed_kp'] == 0 and p['speed_kd'] == 0
        odom_stale = time.monotonic() - self._enc_stamp > 0.5
        if p['speed'] <= 1e-3:
            self._trim = 0.0
            self._speed_cmd = 0.0
        elif gains_off or odom_stale:
            self._trim = 0.0
            self._speed_cmd = p['speed']
        else:
            # Never integrate up while commanded but not moving (gated).
            blocked = self._enc < 0.05 and self._speed_cmd > 0.3
            if serr < 0 or not blocked:
                self._trim += p['speed_kp'] * serr + p['speed_kd'] * (
                    serr - self._last_speed_error
                )
            self._trim = max(-1.0, min(0.3, self._trim))
            self._speed_cmd = max(0.0, min(1.0, p['speed'] + self._trim))
        self._last_speed_error = serr

        out = AckermannDriveStamped()
        out.drive.speed = set_max_speed(self._speed_cmd)
        out.drive.steering_angle = float(-cmd)  # wire positive = left on this car
        self._pub.publish(out)

        t = _now()
        _rows.append(f'{t},{error:.4f},{cmd:.4f},{self._speed_cmd:.3f},{self._enc:.3f}\n')
        _hist.append([t, round(error, 4)])
        overlay.update(
            {
                'error': round(error, 4),
                'steer': round(cmd, 3),
                'speed_cmd': round(self._speed_cmd, 3),
            }
        )
        scan = [
            [round(self._student_deg(msg, i), 1), round(msg.ranges[i], 3)]
            for i in range(0, len(msg.ranges), 3)
            if msg.range_min < msg.ranges[i] < msg.range_max
        ]
        with _lock:
            _state.update(
                {
                    'scan': scan,
                    'overlay': overlay,
                    'hist': list(_hist),
                    'marks': list(_marks),
                    'enc_speed': round(self._enc, 3),
                }
            )


def _log_number(name):
    """Log7.csv -> 7, so lists sort numerically. Anything else sorts first."""
    m = re.match(r'Log(\d+)\.csv$', name)
    return int(m.group(1)) if m else 0


class Handler(BaseHTTPRequestHandler):
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
        if self.path == '/':
            self._send((BASE / 'wallfollow.html').read_bytes(), 'text/html; charset=utf-8')
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
                for k in (
                    'speed',
                    'kp',
                    'kd',
                    'speed_kp',
                    'speed_kd',
                    'window',
                    'lookahead',
                    'look_angle',
                    'width',
                    'side_weight',
                ):
                    if k in data:
                        v = float(data[k])
                        if k == 'speed':
                            v = set_max_speed(v)
                        if _params.get(k) != v:
                            _params[k] = v
                            changed.append(f'{k} {v:g}')
                if changed:
                    _add_mark(', '.join(changed))
        elif self.path == '/logs/save':
            LOG_DIR.mkdir(parents=True, exist_ok=True)
            latest = max((_log_number(f.name) for f in LOG_DIR.glob('Log*.csv')), default=0)
            name = f'Log{latest + 1}.csv'
            rows = list(_rows)
            tmin = float(rows[0].split(',')[0]) if rows else 0.0
            with open(LOG_DIR / name, 'w') as f:
                f.write('t,error,steer,speed_cmd,speed_ms\n')
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

    def _send(self, body, ctype):
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
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
    node = WallFollowNode()
    spin = threading.Thread(target=_spin, args=(node,), daemon=True)
    spin.start()
    server = HTTPServer(('0.0.0.0', PORT), Handler)

    # rclpy.init() installs SIGINT/SIGTERM handlers that shut the ROS context
    # down but leave serve_forever() blocked, so the process outlives the
    # signal until systemd's TimeoutStopSec expires and SIGKILLs it. Take the
    # signals back. shutdown() has to run off the serving thread or it
    # deadlocks waiting for the loop it is called from.
    def stop(*_):
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    print(f'Wall-follow dashboard on http://0.0.0.0:{PORT} (mode={_params["mode"]})')
    server.serve_forever()

    # Unwind the ROS wait before the interpreter tears the context down;
    # otherwise the spin thread aborts the process from C++.
    rclpy.shutdown()
    spin.join(timeout=2)


if __name__ == '__main__':
    main()
