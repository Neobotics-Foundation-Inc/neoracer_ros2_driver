"""
Pre-flight hardware tests for the NeoRacer.

Each failure names the device and gives a one-line fix hint. Tests for absent
hardware fail. `racecar test` leaves them out (addopts in setup.cfg); run them
on the car from the package root with
`python3 -m pytest -m hardware test/test_hardware.py -v`.

The tests run beside teleop and read the car as `racecar status` does. Teleop's
nodes hold the ESP32's serial port, the webcam and the lidar's UDP port, so no
test opens them: the streams come from their ROS topics, the lidar gets a ping,
and the rest comes from /dev, /sys and lsusb. No test publishes a message or
sends a command, so nothing here moves the car. A stream test fails where
`racecar status` reads FAIL for that stream and warns where it reads WARN.
"""

import dataclasses
import importlib
import math
import os
from pathlib import Path
import re
import subprocess
import time
import warnings

from conftest import load_script
import pytest

diagnose = load_script('diagnose')

ROOT = Path(__file__).resolve().parent.parent
UDEV_RULES = ROOT / 'scripts' / 'udev' / '99-racecar.rules'

# STREAMS gives each stream racecar status's nominal rate and floor. /rc/link
# comes from the same ESP32 `r` frame as /rc/channels, so it takes that rate.
STREAMS = {spec.topic: spec for spec in diagnose.SENSOR_TOPICS}
STREAMS['/rc/link'] = dataclasses.replace(
    STREAMS['/rc/channels'], topic='/rc/link', label='FlySky link'
)

# ESP32_STREAMS lists the ESP32's telemetry streams: the `s` frame
# (/imu/fused, /odom, /encoder/speed), `m` (/mag) and `b` (/battery/voltage).
# TestFlySky reads the `r` frame's streams.
ESP32_STREAMS = sorted(diagnose.BOARD_TOPICS - {'/rc/channels', '/joy'})
SAMPLED = (*ESP32_STREAMS, '/rc/channels', '/rc/link', '/scan', '/camera/color')


def _lsusb_match(vid_pid):
    try:
        out = subprocess.run(['lsusb'], capture_output=True, text=True, timeout=5).stdout
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False
    return vid_pid.lower() in out.lower()


def _udev_ids(link):
    """Return the USB vendor:product pairs that the udev rules link to /dev/`link`."""
    ids = set()
    for line in UDEV_RULES.read_text().splitlines():
        if line.lstrip().startswith('#') or f'SYMLINK+="{link}"' not in line:
            continue
        vid = re.search(r'ATTRS\{idVendor\}=="(\w{4})"', line)
        pid = re.search(r'ATTRS\{idProduct\}=="(\w{4})"', line)
        if vid and pid:
            ids.add(f'{vid.group(1)}:{pid.group(1)}')
    return sorted(ids)


def _device_usb_id(link):
    """Return the vendor:product behind /dev/`link`, read from udev, or None."""
    try:
        out = subprocess.run(
            ['udevadm', 'info', '-q', 'property', '-n', f'/dev/{link}'],
            capture_output=True,
            text=True,
            timeout=5,
        ).stdout
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None
    props = dict(line.split('=', 1) for line in out.splitlines() if '=' in line)
    vid, pid = props.get('ID_VENDOR_ID'), props.get('ID_MODEL_ID')
    return f'{vid}:{pid}'.lower() if vid and pid else None


def _usb_enumerated(link, device, hint):
    """Fail unless /dev/`link`'s device is on the USB bus; warn if our rules lack its id."""
    vid_pid = _device_usb_id(link)
    assert vid_pid and _lsusb_match(vid_pid), f'{device}: not on the USB bus. {hint}'
    known = _udev_ids(link)
    if vid_pid not in known:
        # The dev car's base board (303a:4001) and LED matrix (1209:4001) get
        # their links from another rules file, not from ours.
        warnings.warn(
            f'{device}: {vid_pid} is not in scripts/udev/99-racecar.rules '
            f'({", ".join(known)}); another rules file names /dev/{link}.',
            stacklevel=2,
        )


def _params(node, name):
    """Return the ros__parameters of `node` in config/<name>.yaml."""
    import yaml

    with open(ROOT / 'config' / f'{name}.yaml') as f:
        return yaml.safe_load(f)[node]['ros__parameters']


def _import_or_fail(module, device, hint):
    """Import `module`, or fail naming `device`; a missing CUDA library raises OSError."""
    try:
        return importlib.import_module(module)
    except (ImportError, OSError) as e:
        pytest.fail(f'{device}: {module} is not importable ({e}). {hint}', pytrace=False)


def _judge(check, device, hint=''):
    """Fail where racecar status reads FAIL for this row, and warn where it reads WARN."""
    text = f'{device}: {check.detail}' + (f', {check.data}' if check.data else '')
    assert check.status != diagnose.FAIL, f'{text}. {hint}'.strip()
    if check.status == diagnose.WARN:
        warnings.warn(text, stacklevel=2)


@pytest.fixture(scope='module')
def ros():
    """Sample every stream the tests read in one window, as racecar status does."""
    return diagnose.sample_ros(diagnose.DEFAULT_WINDOW, [STREAMS[t] for t in SAMPLED])


def _stream(ros, topic, device):
    """
    Return racecar status's row for `topic` once the stream can be read at all.

    A stream turned off at launch skips. No ROS graph, or no publisher, fails.
    """
    check = diagnose.note_turned_off(diagnose.rate_checks('hardware', [STREAMS[topic]], ros))[0]
    if check.switched_off:
        pytest.skip(f'{device}: turned off at launch, so nothing publishes {topic}')
    if not ros.available:
        pytest.fail(
            f'{device}: no ROS graph to read {topic} from ({ros.reason}). '
            'Source ~/.bashrc and check that teleop runs (racecar service status).',
            pytrace=False,
        )
    if topic not in ros.published:
        pytest.fail(
            f'{device}: nothing publishes {topic}. Teleop is stopped or one of its nodes '
            'is down; racecar status shows which.',
            pytrace=False,
        )
    return check


def _latest(ros, topic, device):
    """Return the last message on `topic` in the sample window."""
    _stream(ros, topic, device)
    msg = ros.values.get(topic)
    assert msg is not None, f'{device}: no message on {topic} in {diagnose.DEFAULT_WINDOW:g} s'
    return msg


# ---------------------------------------------------------------------------
# LakiBeam 2D lidar: a USB-C network link, usb0 on the Jetson, 192.168.8.2
# ---------------------------------------------------------------------------


@pytest.mark.hardware
class TestLakiBeam:
    DEVICE = 'LakiBeam lidar'
    HINT = "Check the lidar's USB-C cable and its power."

    def test_network_link(self):
        _judge(diagnose.lidar_link_check(), f'{self.DEVICE} link', self.HINT)

    def test_sensor_answers_ping(self):
        _judge(diagnose.lidar_answers_check(True), self.DEVICE, self.HINT)

    def test_scan_stream(self, ros):
        # racecar status warns on a beam count other than 1440.
        check = _stream(ros, '/scan', self.DEVICE)
        _judge(check, f'{self.DEVICE} on /scan', f'{self.HINT} See docs/lidar_scan_health.md.')

    def test_scan_has_returns(self, ros):
        # A scan with every range out of range is the blind lidar of
        # docs/lidar_scan_health.md, which streams at its full rate.
        scan = _latest(ros, '/scan', self.DEVICE)
        hits = [
            r for r in scan.ranges if math.isfinite(r) and scan.range_min <= r <= scan.range_max
        ]
        assert hits, (
            f'{self.DEVICE}: all {len(scan.ranges)} ranges on /scan are out of range, so the '
            'lidar is blind. A sagging battery, a covered window or a scan window changed in '
            'the web panel does this; see docs/lidar_scan_health.md.'
        )


# ---------------------------------------------------------------------------
# ESP32 base board (OSRBOT firmware): IMU, odometry, battery, FlySky receiver
# ---------------------------------------------------------------------------


@pytest.mark.hardware
class TestESP32Board:
    DEVICE = 'ESP32 base board'
    LINK = '/dev/osrbot_base'

    def test_device_symlink_exists(self):
        assert os.path.exists(self.LINK), (
            f'{self.DEVICE}: {self.LINK} is missing. '
            'Check its USB cable, then run racecar udev.'
        )

    def test_usb_enumerated(self):
        _usb_enumerated(
            Path(self.LINK).name,
            self.DEVICE,
            'The USB cable is unplugged or the board has no power.',
        )

    @pytest.mark.parametrize('topic', ESP32_STREAMS)
    def test_telemetry_stream(self, ros, topic):
        check = _stream(ros, topic, self.DEVICE)
        _judge(check, f'{self.DEVICE} on {topic}', "Check the ESP32's USB cable and the battery.")


# ---------------------------------------------------------------------------
# FlySky handset and receiver, read through the ESP32's `r` frame
# ---------------------------------------------------------------------------


@pytest.mark.hardware
class TestFlySky:
    DEVICE = 'FlySky receiver'

    def test_channels_stream(self, ros):
        # racecar status fails a frame without the 10 channels.
        check = _stream(ros, '/rc/channels', self.DEVICE)
        _judge(
            check,
            f'{self.DEVICE} on /rc/channels',
            "Check the ESP32's USB cable and the receiver's wiring to the ESP32.",
        )

    def test_handset_link(self, ros):
        # The firmware sends -1 on every channel while it has no handset
        # signal. With the mode switch DOWN it repeats the last values
        # instead, so a handset lost in that position still reads linked.
        link = _latest(ros, '/rc/link', 'FlySky handset')
        assert link.data, (
            'FlySky handset: /rc/link reads false, so the ESP32 gets no handset signal. '
            'Switch the handset on, or bring it closer to the car.'
        )


# ---------------------------------------------------------------------------
# USB webcam: native MJPG, published as JPEG on /camera/color
# ---------------------------------------------------------------------------


@pytest.mark.hardware
class TestWebcam:
    DEVICE = 'USB webcam'
    LINK = '/dev/osrbot_usb_cam'

    def test_device_symlink_exists(self):
        assert os.path.exists(self.LINK), (
            f'{self.DEVICE}: {self.LINK} is missing. '
            'Check its USB cable, then run racecar udev.'
        )

    def test_usb_enumerated(self):
        _usb_enumerated(Path(self.LINK).name, self.DEVICE, 'The USB cable is unplugged.')

    def test_color_stream(self, ros):
        check = _stream(ros, '/camera/color', self.DEVICE)
        _judge(
            check,
            f'{self.DEVICE} on /camera/color',
            'Check its USB cable, then restart teleop (racecar service restart teleop).',
        )

    def test_frame_decodes(self, ros):
        msg = _latest(ros, '/camera/color', self.DEVICE)
        from neoracer_ros2_driver import inference_lib as il

        try:
            img = il.decode_image(msg.data, msg.encoding, msg.height, msg.width)
        except ValueError as e:
            pytest.fail(
                f'{self.DEVICE}: the frame on /camera/color does not decode ({e})', pytrace=False
            )
        cam = _params('camera_node', 'camera')
        want = (cam['image_height'], cam['image_width'])
        if img.shape[:2] != want:
            warnings.warn(
                f'{self.DEVICE}: {img.shape[1]}x{img.shape[0]} frames, '
                f'config/camera.yaml sets {want[1]}x{want[0]}',
                stacklevel=2,
            )


# ---------------------------------------------------------------------------
# Jetson GPU: CUDA, TensorRT and the Python stack inference_node runs on
# ---------------------------------------------------------------------------


@pytest.mark.hardware
class TestJetsonGPU:
    DEVICE = 'Jetson GPU'
    CUDA_ROOT = Path('/usr/local/cuda')
    SETUP_HINT = 'Run racecar setup ml.'

    @pytest.fixture(autouse=True)
    def _no_autoinstall(self, monkeypatch):
        # Ultralytics pip-installs a missing requirement unless
        # YOLO_AUTOINSTALL is off. PyPI has no TensorRT wheel for the Orin
        # (scripts/setup_ml.sh), so these tests report a missing package and
        # install nothing.
        monkeypatch.setenv('YOLO_AUTOINSTALL', 'False')

    def test_cuda_toolkit_present(self):
        # scripts/setup_ml.sh reads the CUDA version from this file.
        assert (self.CUDA_ROOT / 'version.json').is_file(), (
            f'{self.DEVICE}: no CUDA toolkit at {self.CUDA_ROOT}. '
            'Reflash JetPack with CUDA selected.'
        )

    def test_tensorrt_importable(self):
        _import_or_fail(
            'tensorrt',
            self.DEVICE,
            'JetPack ships the bindings as python3-libnvinfer. ' + self.SETUP_HINT,
        )

    def test_torch_reaches_gpu(self):
        torch = _import_or_fail('torch', self.DEVICE, self.SETUP_HINT)
        assert torch.cuda.is_available(), (
            f'{self.DEVICE}: torch {torch.__version__} cannot reach the GPU '
            f'(torch.cuda.is_available() is False). {self.SETUP_HINT}'
        )

    def test_ultralytics_importable(self):
        _import_or_fail('ultralytics', self.DEVICE, self.SETUP_HINT)

    # The test times TIMED_FRAMES frames after the warmup frame, as upstream
    # times the Coral.
    TIMED_FRAMES = 10

    def test_engine_within_latency_budget(self):
        import numpy as np

        # inference_node holds the model while detections are on; a second
        # copy here would share the GPU and the 8 GB of memory with it.
        running = subprocess.run(
            ['pgrep', '-f', 'lib/neoracer_ros2_driver/inference_node'],
            capture_output=True,
        )
        if running.returncode == 0:
            pytest.skip('inference_node is running; cannot test in isolation')
        # A car without the TensorRT bindings fails here, before Ultralytics
        # loads the engine.
        _import_or_fail('tensorrt', self.DEVICE, self.SETUP_HINT)

        from neoracer_ros2_driver import inference_lib as il

        cfg = _params('inference_node', 'inference')
        model = il.resolve_model_path(cfg['model_path'], [str(ROOT / 'models')])
        assert os.path.isfile(model), (
            f'{self.DEVICE}: {cfg["model_path"]}, the model_path of config/inference.yaml, '
            'is not in models/.'
        )
        # The test feeds a frame of the camera's size with the node's options.
        cam = _params('camera_node', 'camera')
        frame = np.zeros((cam['image_height'], cam['image_width'], 3), dtype=np.uint8)
        options = {
            'imgsz': cfg['imgsz'],
            'conf': cfg['score_threshold'],
            'iou': cfg['iou_threshold'],
            'max_det': max(1, cfg['max_detections']),
            'device': cfg['device'],
            'half': cfg['half'],
            'verbose': False,
        }

        # The first frame pays for the CUDA context and deserializes the engine.
        try:
            from neoracer_ros2_driver.inference_node import load_yolo

            yolo = load_yolo(model)
            yolo.predict(frame, **options)
        except ImportError as e:
            pytest.fail(f'{self.DEVICE}: {e}. {self.SETUP_HINT}', pytrace=False)
        except Exception as e:  # noqa: BLE001 - torch and TensorRT raise broadly
            pytest.fail(
                f'{self.DEVICE}: {Path(model).name} does not run on this car ({e}). '
                'inference_node falls back to the .pt at about 2.4x the latency. Rebuild '
                'the engine on this car with racecar compile (models/README.md).',
                pytrace=False,
            )

        t0 = time.monotonic()
        for _ in range(self.TIMED_FRAMES):
            yolo.predict(frame, **options)
        mean_ms = (time.monotonic() - t0) / self.TIMED_FRAMES * 1000.0

        # The engine has to clear inference.yaml's max_rate_hz ceiling, one
        # frame per 1 / max_rate_hz: 66.7 ms at 15 Hz. The README measures
        # 20.9 ms with teleop running. A max_rate_hz of 0 leaves the node
        # uncapped, so there is no budget to hold.
        if not cfg['max_rate_hz'] > 0:
            pytest.skip(f'max_rate_hz is {cfg["max_rate_hz"]}: no cap, so no budget')
        budget_ms = 1000.0 / cfg['max_rate_hz']
        assert mean_ms < budget_ms, (
            f'{self.DEVICE}: {Path(model).name} averages {mean_ms:.1f} ms a frame, over the '
            f'{budget_ms:.1f} ms that max_rate_hz {cfg["max_rate_hz"]:g} allows. Check the '
            'power mode (sudo nvpmodel -q) and the soc temperature (racecar status).'
        )


# ---------------------------------------------------------------------------
# Python runtime dependencies for the driver
# ---------------------------------------------------------------------------


@pytest.mark.hardware
class TestDependencies:
    # The list holds the Python modules that package.xml declares. Upstream's
    # luma.core, PIL and rosbag2_py are left out: the LED matrix takes text
    # over pyserial, and racecar log records through the ros2 bag command.
    @pytest.mark.parametrize(
        'module',
        [
            'ackermann_msgs',
            'ament_index_python',
            'cv2',
            'diagnostic_msgs',
            'geometry_msgs',
            'nav_msgs',
            'numpy',
            'rcl_interfaces',
            'rclpy',
            'scipy',
            'sensor_msgs',
            'serial',
            'std_msgs',
            'std_srvs',
            'tf2_ros',
            'vision_msgs',
            'yaml',
        ],
    )
    def test_module_importable(self, module):
        try:
            importlib.import_module(module)
        except ImportError as e:
            pytest.fail(f'{module} not importable: {e}. Run racecar setup all.')


# ---------------------------------------------------------------------------
# Checks of this file that run in every racecar test, on the car or off it
# ---------------------------------------------------------------------------


class TestOffCar:
    def test_racecar_test_leaves_hardware_out(self, pytestconfig):
        # setup.cfg's addopts deselects the marker, and a -m on the command
        # line replaces it.
        addopts = pytestconfig.getini('addopts')
        assert '-m' in addopts, 'setup.cfg addopts has no -m "not hardware"'
        assert addopts[addopts.index('-m') + 1] == 'not hardware'
        markers = pytestconfig.getini('markers')
        assert any(m.startswith('hardware:') for m in markers), 'hardware marker not registered'

    def test_device_classes_are_marked(self):
        classes = [
            obj
            for name, obj in globals().items()
            if name.startswith('Test') and isinstance(obj, type) and obj is not TestOffCar
        ]
        assert classes
        for cls in classes:
            marks = [m.name for m in getattr(cls, 'pytestmark', [])]
            assert 'hardware' in marks, f'{cls.__name__} would run in racecar test'

    def test_udev_rules_name_each_device(self):
        for cls in (TestESP32Board, TestWebcam):
            link = Path(cls.LINK).name
            assert _udev_ids(link), f'{UDEV_RULES.name} gives no USB IDs for {link}'
