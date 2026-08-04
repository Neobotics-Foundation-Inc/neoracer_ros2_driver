"""
Pure (ROS-free) helper functions for inference_node.py - the YOLO detector.

Kept import-light and side-effect-free so the frame decode and the box geometry
can be unit-tested without a running ROS graph or a GPU (see
test/test_inference.py). Nothing here imports ultralytics or torch.

Coordinates follow the vision_msgs convention: pixel space, origin at the image
top-left, ``x`` to the right and ``y`` down.
"""

import os

import cv2
import numpy as np

# sensor_msgs/Image encodings the node accepts. 'jpeg' is what this driver's
# camera node publishes (native MJPG passthrough); the two raw formats are here
# so the node also works behind a cv_bridge publisher or a bag replay.
SUPPORTED_ENCODINGS = ('jpeg', 'bgr8', 'rgb8')


def decode_image(data, encoding, height, width):
    """
    Decode sensor_msgs/Image payload fields into an ``(H, W, 3)`` uint8 BGR array.

    BGR is what Ultralytics expects from a numpy input, matching OpenCV; passing
    RGB silently degrades detection quality rather than raising, so the channel
    order is fixed here once. Raises ``ValueError`` on an unsupported encoding or
    a payload that does not match the declared geometry.
    """
    if encoding == 'jpeg':
        img = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            raise ValueError('jpeg payload did not decode')
        return img

    if encoding in ('bgr8', 'rgb8'):
        arr = np.frombuffer(data, dtype=np.uint8)
        if arr.size != height * width * 3:
            raise ValueError(
                f'{encoding} payload is {arr.size} bytes, '
                f'expected {height * width * 3} for {width}x{height}')
        arr = arr.reshape(height, width, 3)
        return arr[:, :, ::-1] if encoding == 'rgb8' else arr

    raise ValueError(f'unsupported image encoding: {encoding}')


def resolve_model_path(model, search_dirs):
    """
    Locate a weights file, returning ``model`` unchanged when nothing matches.

    An absolute path is taken as given. A relative one is looked up in each of
    ``search_dirs`` in order (the package's ``models/`` share directory first, so
    a car-local engine wins over anything else on disk). A bare name that matches
    no file is passed through for Ultralytics to resolve from its own cache,
    which is how the stock ``yolo11n.pt`` reaches a fresh car.
    """
    if os.path.isabs(model):
        return model
    for directory in search_dirs:
        candidate = os.path.join(directory, model)
        if os.path.exists(candidate):
            return candidate
    return model


def box_to_center_size(xyxy, img_w, img_h):
    """
    Convert a pixel-space ``[x1, y1, x2, y2]`` box to ``(cx, cy, w, h)``.

    Corners are ordered, then each is clipped into the frame. Letterbox padding
    lets a box extend past the frame edge, and an unclipped center then sits
    off-image, which downstream steering code reads as a real bearing. A box
    entirely outside the frame collapses to zero width or height on the edge it
    left through, rather than surviving at full size.
    """
    x1, y1, x2, y2 = (float(v) for v in xyxy)
    if x2 < x1:
        x1, x2 = x2, x1
    if y2 < y1:
        y1, y2 = y2, y1

    x1, x2 = (min(max(v, 0.0), float(img_w)) for v in (x1, x2))
    y1, y2 = (min(max(v, 0.0), float(img_h)) for v in (y1, y2))
    return (x1 + x2) / 2.0, (y1 + y2) / 2.0, x2 - x1, y2 - y1


def build_detections(xyxy, scores, classes, names, img_w, img_h,
                     score_threshold=0.0, max_detections=0):
    """
    Turn one YOLO result's arrays into a list of plain detection dicts.

    Each dict is ``{'class_id', 'class_index', 'score', 'cx', 'cy', 'w', 'h'}``
    with the box in pixels. ``names`` maps a class index to its label (Ultralytics
    carries this inside the weights, so there is no separate labels file);
    an index missing from it falls back to its own string form.

    Ultralytics already applies ``conf`` and ``max_det`` during predict, so both
    limits here are a second pass for callers that hand in raw tensors.
    ``max_detections <= 0`` means no cap. Input order is preserved, which for
    Ultralytics output is descending confidence.
    """
    detections = []
    for box, score, index in zip(xyxy, scores, classes):
        if float(score) < score_threshold:
            continue
        if 0 < max_detections <= len(detections):
            break
        index = int(index)
        cx, cy, w, h = box_to_center_size(box, img_w, img_h)
        detections.append({
            'class_id': str(names.get(index, index)),
            'class_index': index,
            'score': float(score),
            'cx': cx,
            'cy': cy,
            'w': w,
            'h': h,
        })
    return detections
