#!/usr/bin/env python3
"""
NeoRacer camera label dashboard: collect frames, then label them, YOLO out.

Two-stage workflow on port 8082 (stdlib HTTP + rclpy, wallfollow pattern):
  Collect: Snap saves the current camera frame to unlabeled/.
  Label:   pick an unlabeled image, drag boxes, save. The file moves to
           images/ with a YOLO label file in labels/ (class_id cx cy w h,
           normalized) and classes.txt stays in sync.

Everything is written under datasets/camlabel in the data root, created at
startup when absent.

camlabel.yaml sets the live-preview resolution sent to the browser, with the
car profile's dashboards/camlabel.yaml on top. Saved images are always the
full camera frame. Thumbnails, immutable file names with long browser
caching, and one combined /info endpoint keep the many-cars-one-room WiFi
load low.
"""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import signal
import sys
import threading

import cv2
from neoracer_ros2_driver import car_profile, data_root
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import yaml

PORT = 8082
BASE = Path(__file__).resolve().parent
# The shared styles, fonts and logos are served from scripts/web/ (webui.py).
sys.path.insert(0, str(BASE.parents[1]))
import webui  # noqa: E402, I100

# Dataset lives under the data root (data_root.py), outside the code tree, so
# reinstalling or re-cloning never touches collected frames or labels.
DATASET = data_root.current()[0] / 'datasets' / 'camlabel'
UNLABELED = DATASET / 'unlabeled'
IMAGES = DATASET / 'images'
LABELS = DATASET / 'labels'

CACHE_FOREVER = 'max-age=31536000, immutable'

_lock = threading.Lock()  # latest frame
_fs_lock = threading.Lock()  # file numbering and moves
_live = b''
_preview = b''
_cam_res = [0, 0]
_thumbs: dict[str, bytes] = {}


def load_cfg():
    """Return the preview settings: camlabel.yaml, then the car profile's on top."""
    cfg = {'preview_width': 320, 'preview_quality': 45}
    try:
        cfg.update(yaml.safe_load((BASE / 'camlabel.yaml').read_text()) or {})
    except OSError:
        pass
    # A car's own preview settings live in the car profile, so a driver
    # update leaves them alone.
    cfg.update(car_profile.read_yaml(car_profile.dashboard_settings('camlabel')))
    return cfg


_cfg = load_cfg()


class CameraNode(Node):
    def __init__(self):
        super().__init__('camlabel')
        self.create_subscription(Image, '/camera/color', self._cb, qos_profile_sensor_data)

    def _cb(self, msg):
        global _live, _preview, _cam_res
        if msg.encoding == 'jpeg':
            data = bytes(msg.data)
            arr = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        else:
            # Raw frame fallback for other camera drivers.
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
            if msg.encoding == 'rgb8':
                arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            data = cv2.imencode('.jpg', arr)[1].tobytes()
        prev, res = data, [msg.width, msg.height]
        if arr is not None:
            h, w = arr.shape[:2]
            res = [w, h]
            pw = int(_cfg['preview_width'])
            small = cv2.resize(arr, (pw, max(1, round(pw * h / w))))
            quality = [cv2.IMWRITE_JPEG_QUALITY, int(_cfg['preview_quality'])]
            prev = cv2.imencode('.jpg', small, quality)[1].tobytes()
        with _lock:
            _live = data
            _preview = prev
            _cam_res = res


def _num(name):
    """u7.jpg or img7.jpg -> 7."""
    m = re.search(r'(\d+)\.jpg$', name)
    return int(m.group(1)) if m else 0


def _nums(folder, pattern):
    return sorted(_num(f.name) for f in folder.glob(pattern)) if folder.exists() else []


def _entries(folder, pattern):
    """
    Return [[number, mtime_ns], ...] for the files matching pattern.

    The browser can cache files forever yet never show a stale image if a
    dataset is wiped and numbers restart.
    """
    if not folder.exists():
        return []
    return sorted([_num(f.name), f.stat().st_mtime_ns] for f in folder.glob(pattern))


def _next_unlabeled_number():
    """
    Return the next unlabeled number; names are never reused.

    Unique names let the browser cache every image and thumbnail forever.
    """
    f = DATASET / 'ucounter'
    n = (int(f.read_text()) if f.exists() else 0) + 1
    f.write_text(str(n))
    return n


def _class_id(name):
    """Index of name in classes.txt, appending it if new."""
    path = DATASET / 'classes.txt'
    classes = path.read_text().splitlines() if path.exists() else []
    if name not in classes:
        classes.append(name)
        path.write_text('\n'.join(classes) + '\n')
    return classes.index(name)


def _thumb(path):
    """
    Return a 160-wide JPEG thumbnail, cached in memory.

    Safe because file names are never reused within a dataset.
    """
    key = f'{path}:{path.stat().st_mtime_ns}'
    t = _thumbs.get(key)
    if t is None:
        arr = cv2.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
        if arr is None:
            return b''
        h, w = arr.shape[:2]
        small = cv2.resize(arr, (160, max(1, round(160 * h / w))))
        t = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 50])[1].tobytes()
        if len(_thumbs) > 600:
            _thumbs.clear()
        _thumbs[key] = t
    return t


class Handler(BaseHTTPRequestHandler):
    # Keep-alive: reuse one TCP connection per viewer instead of a new
    # handshake per request. Matters a lot on congested WiFi.
    protocol_version = 'HTTP/1.1'

    def _query_n(self):
        return int(self.path.split('n=')[1].split('&')[0])

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
            self._send((BASE / 'camlabel.html').read_bytes(), 'text/html; charset=utf-8')
        elif self.path.startswith('/frame'):
            with _lock:
                data = _preview
            if data:
                self._send(data, 'image/jpeg', cache='no-store')
            else:
                self.send_error(503)
        elif self.path == '/info':
            with _lock:
                res = list(_cam_res)
            body = json.dumps(
                {
                    'cam': res,
                    'preview_width': int(_cfg['preview_width']),
                    'unlabeled': _entries(UNLABELED, 'u*.jpg'),
                    'labeled': _entries(IMAGES, 'img*.jpg'),
                }
            )
            self._send(body.encode(), 'application/json')
        elif self.path.startswith('/uimg?'):
            self._send_file(UNLABELED / f'u{self._query_n()}.jpg')
        elif self.path.startswith('/img?'):
            self._send_file(IMAGES / f'img{self._query_n()}.jpg')
        elif self.path.startswith('/thumb?'):
            pool, stem = (UNLABELED, 'u') if 'src=u' in self.path else (IMAGES, 'img')
            f = pool / f'{stem}{self._query_n()}.jpg'
            data = _thumb(f) if f.is_file() else b''
            if data:
                self._send(data, 'image/jpeg', cache=CACHE_FOREVER)
            else:
                self.send_error(404)
        elif self.path.startswith('/label?'):
            f = LABELS / f'img{self._query_n()}.txt'
            path = DATASET / 'classes.txt'
            classes = path.read_text().splitlines() if path.exists() else []
            boxes = []
            if f.is_file():
                for line in f.read_text().splitlines():
                    i, cx, cy, w, h = line.split()
                    name = classes[int(i)] if int(i) < len(classes) else i
                    boxes.append(
                        {
                            'cls': name,
                            'cx': float(cx),
                            'cy': float(cy),
                            'w': float(w),
                            'h': float(h),
                        }
                    )
            self._send(json.dumps({'boxes': boxes}).encode(), 'application/json')
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path == '/snap':
            with _lock:
                data = _live
            if not data:
                self.send_error(503)
                return
            with _fs_lock:
                _ensure_dataset()
                n = _next_unlabeled_number()
                (UNLABELED / f'u{n}.jpg').write_bytes(data)
            self._send(json.dumps({'n': n}).encode(), 'application/json')
        elif self.path == '/label':
            req = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            src = UNLABELED / f'u{int(req["n"])}.jpg'
            if not src.is_file():
                self.send_error(404)
                return
            with _fs_lock:
                _ensure_dataset()
                m = max(_nums(IMAGES, 'img*.jpg'), default=0) + 1
                src.rename(IMAGES / f'img{m}.jpg')
                lines = [
                    f'{_class_id(b["cls"])} '
                    f'{b["cx"]:.6f} {b["cy"]:.6f} {b["w"]:.6f} {b["h"]:.6f}'
                    for b in req['boxes']
                ]
                (LABELS / f'img{m}.txt').write_text('\n'.join(lines) + '\n' if lines else '')
            self._send(json.dumps({'img': m}).encode(), 'application/json')
        elif self.path == '/dell':
            req = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            n = int(req['n'])
            for f in (IMAGES / f'img{n}.jpg', LABELS / f'img{n}.txt'):
                if f.is_file():
                    f.unlink()
            self._send(b'ok', 'text/plain')
        elif self.path == '/delu':
            req = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
            f = UNLABELED / f'u{int(req["n"])}.jpg'
            if f.is_file():
                f.unlink()
            self._send(b'ok', 'text/plain')
        else:
            self.send_error(404)

    def _send_file(self, f):
        if f.is_file():
            self._send(f.read_bytes(), 'image/jpeg', cache=CACHE_FOREVER)
        else:
            self.send_error(404)

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


def _ensure_dataset():
    for d in (UNLABELED, IMAGES, LABELS):
        d.mkdir(parents=True, exist_ok=True)


def _spin(node):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


def main():
    _ensure_dataset()
    rclpy.init()
    node = CameraNode()
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

    print(
        f'Camera label dashboard on http://0.0.0.0:{PORT} '
        f'(preview {_cfg["preview_width"]}px, dataset {DATASET})'
    )
    server.serve_forever()

    # Unwind the ROS wait before the interpreter tears the context down;
    # otherwise the spin thread aborts the process from C++.
    rclpy.shutdown()
    spin.join(timeout=2)


if __name__ == '__main__':
    main()
