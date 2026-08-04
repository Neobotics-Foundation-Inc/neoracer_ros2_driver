"""Unit tests for the YOLO helpers in inference_lib (no ROS, no GPU required)."""

import cv2
import numpy as np
import pytest

from neoracer_ros2_driver import inference_lib as il


COCO_NAMES = {0: 'person', 2: 'car', 39: 'bottle'}


def _bgr_frame(height=8, width=12):
    """Build a frame whose three channels are distinguishable from each other."""
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    frame[:, :, 0] = 10   # blue
    frame[:, :, 1] = 20   # green
    frame[:, :, 2] = 30   # red
    return frame


# ---------- decode_image ----------

def test_decode_jpeg_roundtrip():
    frame = _bgr_frame()
    ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 100])
    assert ok

    out = il.decode_image(buf.tobytes(), 'jpeg', frame.shape[0], frame.shape[1])

    assert out.shape == frame.shape
    assert out.dtype == np.uint8
    # JPEG is lossy; a flat image survives well within a few counts per channel.
    assert np.allclose(out, frame, atol=4)


def test_decode_bgr8_is_passthrough():
    frame = _bgr_frame()
    out = il.decode_image(frame.tobytes(), 'bgr8', frame.shape[0], frame.shape[1])
    assert np.array_equal(out, frame)


def test_decode_rgb8_swaps_channels():
    frame = _bgr_frame()
    out = il.decode_image(frame.tobytes(), 'rgb8', frame.shape[0], frame.shape[1])
    assert np.array_equal(out, frame[:, :, ::-1])
    # The blue channel of the output must be the red channel of the input.
    assert out[0, 0, 0] == 30


def test_decode_rejects_unsupported_encoding():
    with pytest.raises(ValueError, match='unsupported image encoding'):
        il.decode_image(b'\x00' * 12, 'mono8', 2, 2)


def test_decode_rejects_size_mismatch():
    with pytest.raises(ValueError, match='expected'):
        il.decode_image(b'\x00' * 10, 'bgr8', 8, 12)


def test_decode_rejects_corrupt_jpeg():
    with pytest.raises(ValueError, match='did not decode'):
        il.decode_image(b'\xff\xd8not-a-jpeg', 'jpeg', 8, 12)


# ---------- resolve_model_path ----------

def test_resolve_prefers_first_matching_dir(tmp_path):
    first = tmp_path / 'share'
    second = tmp_path / 'cwd'
    first.mkdir()
    second.mkdir()
    (first / 'yolo11n.pt').write_bytes(b'')
    (second / 'yolo11n.pt').write_bytes(b'')

    assert il.resolve_model_path(
        'yolo11n.pt', [str(first), str(second)]) == str(first / 'yolo11n.pt')


def test_resolve_passes_through_unknown_name(tmp_path):
    assert il.resolve_model_path('yolo11n.pt', [str(tmp_path)]) == 'yolo11n.pt'


def test_resolve_keeps_absolute_path(tmp_path):
    absolute = str(tmp_path / 'elsewhere.engine')
    assert il.resolve_model_path(absolute, [str(tmp_path)]) == absolute


# ---------- box_to_center_size ----------

def test_box_center_and_size():
    cx, cy, w, h = il.box_to_center_size([10, 20, 50, 60], 640, 480)
    assert (cx, cy, w, h) == (30.0, 40.0, 40.0, 40.0)


def test_box_clipped_to_frame():
    # Letterbox padding can push a corner outside the image.
    cx, cy, w, h = il.box_to_center_size([-20, -10, 100, 60], 640, 480)
    assert (cx, cy, w, h) == (50.0, 30.0, 100.0, 60.0)
    assert 0 <= cx <= 640 and 0 <= cy <= 480


def test_box_fully_outside_frame_collapses():
    cx, cy, w, h = il.box_to_center_size([700, 500, 800, 600], 640, 480)
    assert (w, h) == (0.0, 0.0)
    assert (cx, cy) == (640.0, 480.0)


# ---------- build_detections ----------

def test_build_detections_maps_names_and_geometry():
    dets = il.build_detections(
        xyxy=np.array([[10, 20, 50, 60]], dtype=np.float32),
        scores=np.array([0.9], dtype=np.float32),
        classes=np.array([2]),
        names=COCO_NAMES,
        img_w=640,
        img_h=480,
    )

    assert len(dets) == 1
    assert dets[0]['class_id'] == 'car'
    assert dets[0]['class_index'] == 2
    assert dets[0]['score'] == pytest.approx(0.9)
    assert (dets[0]['cx'], dets[0]['cy']) == (30.0, 40.0)
    assert (dets[0]['w'], dets[0]['h']) == (40.0, 40.0)


def test_build_detections_applies_threshold():
    dets = il.build_detections(
        xyxy=np.array([[0, 0, 10, 10], [0, 0, 20, 20]], dtype=np.float32),
        scores=np.array([0.8, 0.2], dtype=np.float32),
        classes=np.array([0, 0]),
        names=COCO_NAMES,
        img_w=640,
        img_h=480,
        score_threshold=0.5,
    )
    assert [d['score'] for d in dets] == [pytest.approx(0.8)]


def test_build_detections_caps_count_keeping_order():
    dets = il.build_detections(
        xyxy=np.tile(np.array([0, 0, 10, 10], dtype=np.float32), (5, 1)),
        scores=np.array([0.9, 0.8, 0.7, 0.6, 0.5], dtype=np.float32),
        classes=np.zeros(5, dtype=int),
        names=COCO_NAMES,
        img_w=640,
        img_h=480,
        max_detections=2,
    )
    assert [d['score'] for d in dets] == [pytest.approx(0.9), pytest.approx(0.8)]


def test_build_detections_unknown_class_falls_back_to_index():
    dets = il.build_detections(
        xyxy=np.array([[0, 0, 10, 10]], dtype=np.float32),
        scores=np.array([0.9], dtype=np.float32),
        classes=np.array([77]),
        names=COCO_NAMES,
        img_w=640,
        img_h=480,
    )
    assert dets[0]['class_id'] == '77'


def test_build_detections_empty_result():
    dets = il.build_detections(
        xyxy=np.zeros((0, 4), dtype=np.float32),
        scores=np.zeros((0,), dtype=np.float32),
        classes=np.zeros((0,), dtype=int),
        names=COCO_NAMES,
        img_w=640,
        img_h=480,
    )
    assert dets == []
