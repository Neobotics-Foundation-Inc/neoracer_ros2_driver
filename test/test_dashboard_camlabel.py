"""
Dataset, settings and HTTP tests for the camera label dashboard (camlabel.py).

Runs without a ROS graph or a camera: frames are JPEGs built here, and the
HTTP handler is served on a free local port against a dataset in a temporary
folder.
"""

from http.server import ThreadingHTTPServer
import json
import threading
import urllib.error
import urllib.request

from conftest import load_script
import cv2
import numpy as np
import pytest
from sensor_msgs.msg import Image


@pytest.fixture(scope='module')
def cl():
    return load_script('camlabel')


@pytest.fixture
def dataset(cl, monkeypatch, tmp_path):
    root = tmp_path / 'datasets' / 'camlabel'
    monkeypatch.setattr(cl, 'DATASET', root)
    monkeypatch.setattr(cl, 'UNLABELED', root / 'unlabeled')
    monkeypatch.setattr(cl, 'IMAGES', root / 'images')
    monkeypatch.setattr(cl, 'LABELS', root / 'labels')
    cl._ensure_dataset()
    return root


@pytest.fixture
def server(cl, dataset):
    srv = ThreadingHTTPServer(('127.0.0.1', 0), cl.Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield f'http://127.0.0.1:{srv.server_address[1]}'
    srv.shutdown()
    srv.server_close()


def post(url, body=None):
    data = json.dumps(body).encode() if body is not None else b''
    req = urllib.request.Request(url, data=data, method='POST')
    with urllib.request.urlopen(req, timeout=5) as r:
        return r.read()


def jpeg_frame(width=640, height=480):
    msg = Image()
    msg.encoding = 'jpeg'
    msg.width, msg.height = width, height
    msg.data = cv2.imencode('.jpg', np.full((height, width, 3), 120, np.uint8))[1].tobytes()
    return msg


def test_the_dataset_lives_under_the_data_root(cl, data_root_dir):
    assert cl.DATASET == data_root_dir / 'datasets' / 'camlabel'


def test_the_preview_settings_take_the_profile_on_top(cl, car_profile_dir):
    assert cl.load_cfg()['preview_width'] == 320
    (car_profile_dir / 'dashboards').mkdir(parents=True)
    (car_profile_dir / 'dashboards' / 'camlabel.yaml').write_text('preview_width: 480\n')
    cfg = cl.load_cfg()
    assert cfg['preview_width'] == 480
    assert cfg['preview_quality'] == 45


def test_a_jpeg_frame_is_kept_whole_and_previewed(cl, monkeypatch):
    monkeypatch.setattr(cl, '_cfg', {'preview_width': 320, 'preview_quality': 45})
    monkeypatch.setattr(cl.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(cl.CameraNode, 'create_subscription', lambda *a, **k: None, raising=False)
    msg = jpeg_frame()
    cl.CameraNode()._cb(msg)
    # The camera's own JPEG is saved by Snap as it came, at full resolution.
    assert cl._live == bytes(msg.data)
    assert cl._cam_res == [640, 480]
    small = cv2.imdecode(np.frombuffer(cl._preview, np.uint8), cv2.IMREAD_COLOR)
    assert small.shape[:2] == (240, 320)


def test_snap_then_label_writes_a_yolo_dataset(cl, server, dataset, monkeypatch):
    monkeypatch.setattr(cl, '_live', bytes(jpeg_frame().data))
    assert json.loads(post(server + '/snap')) == {'n': 1}
    assert json.loads(post(server + '/snap')) == {'n': 2}
    box = {'cls': 'cone', 'cx': 0.5, 'cy': 0.5, 'w': 0.25, 'h': 0.5}
    assert json.loads(post(server + '/label', {'n': 2, 'boxes': [box]})) == {'img': 1}
    assert (dataset / 'images' / 'img1.jpg').is_file()
    assert not (dataset / 'unlabeled' / 'u2.jpg').exists()
    assert (dataset / 'labels' / 'img1.txt').read_text() == (
        '0 0.500000 0.500000 0.250000 0.500000\n'
    )
    assert (dataset / 'classes.txt').read_text() == 'cone\n'
    with urllib.request.urlopen(server + '/info', timeout=5) as r:
        info = json.loads(r.read())
    assert [n for n, _ in info['unlabeled']] == [1]
    assert [n for n, _ in info['labeled']] == [1]


def test_a_snap_with_no_frame_yet_is_refused(cl, server, monkeypatch):
    monkeypatch.setattr(cl, '_live', b'')
    with pytest.raises(urllib.error.HTTPError) as err:
        post(server + '/snap')
    assert err.value.code == 503


def test_unlabeled_numbers_are_never_reused(cl, dataset):
    first = cl._next_unlabeled_number()
    (dataset / 'unlabeled' / f'u{first}.jpg').write_bytes(b'x')
    (dataset / 'unlabeled' / f'u{first}.jpg').unlink()
    assert cl._next_unlabeled_number() == first + 1


def test_a_new_class_name_is_appended(cl, dataset):
    assert cl._class_id('cone') == 0
    assert cl._class_id('car') == 1
    assert cl._class_id('cone') == 0
    assert (dataset / 'classes.txt').read_text() == 'cone\ncar\n'


def test_the_page_and_the_shared_styles_are_served(cl, server):
    with urllib.request.urlopen(server + '/', timeout=5) as r:
        page = r.read().decode()
    assert 'data-app="Camera label"' in page and '/ui/neoracer.css' in page
    with urllib.request.urlopen(server + '/ui/neoracer.css', timeout=5) as r:
        assert r.headers['Content-Type'].startswith('text/css')
    # A link from another port carries the theme in the query string.
    with urllib.request.urlopen(server + '/?theme=light', timeout=5) as r:
        assert 'data-app="Camera label"' in r.read().decode()
