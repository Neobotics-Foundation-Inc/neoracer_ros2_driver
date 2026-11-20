"""Unit tests for scan_match.py: pose algebra and ICP on simulated room scans."""

import math

from conftest import load_script
import numpy as np
import pytest


@pytest.fixture(scope='module')
def sm():
    return load_script('scan_match')


# A 7 x 5 m room with a cabinet and a pillar, as wall segments (x0, y0, x1, y1).
ROOM = [
    (-2.0, -2.0, 5.0, -2.0),
    (5.0, -2.0, 5.0, 3.0),
    (5.0, 3.0, -2.0, 3.0),
    (-2.0, 3.0, -2.0, -2.0),
    (3.0, 1.5, 4.0, 1.5),
    (4.0, 1.5, 4.0, 3.0),
    (0.5, -1.0, 0.8, -1.0),
    (0.8, -1.0, 0.8, -0.7),
    (0.8, -0.7, 0.5, -0.7),
    (0.5, -0.7, 0.5, -1.0),
]
BEAMS = 1080
ANGLE_MIN = -math.pi
ANGLE_INC = 2 * math.pi / BEAMS


def _scan(pose, noise=0.0, seed=0, segments=ROOM):
    """Ranges seen from `pose` (x, y, theta) in the room, by ray casting."""
    rng = np.random.default_rng(seed)
    x, y, th = pose
    ranges = np.full(BEAMS, np.inf)
    for i in range(BEAMS):
        a = th + ANGLE_MIN + i * ANGLE_INC
        dx, dy = math.cos(a), math.sin(a)
        for x0, y0, x1, y1 in segments:
            ex, ey = x1 - x0, y1 - y0
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            t = ((x0 - x) * ey - (y0 - y) * ex) / den
            u = ((x0 - x) * dy - (y0 - y) * dx) / den
            if t > 0 and 0 <= u <= 1:
                ranges[i] = min(ranges[i], t)
    return ranges + rng.normal(0.0, noise, BEAMS)


def _points(sm, ranges):
    return sm.scan_points(ranges, ANGLE_MIN, ANGLE_INC, 0.15, 12.0)


def _relative(sm, a, b):
    return sm.compose(sm.invert(a), b)


class TestPoseAlgebra:
    def test_inverse_composes_to_identity(self, sm):
        a = (1.2, -0.4, 0.7)
        x, y, th = sm.compose(a, sm.invert(a))
        assert (x, y, th) == (pytest.approx(0.0, abs=1e-12),) * 2 + (pytest.approx(0.0),)

    def test_base_and_laser_motion_round_trip(self, sm):
        laser = (0.2383, 0.0, math.pi)
        base = (2.9, 0.1, 0.05)
        back = sm.base_motion(sm.laser_motion(base, laser), laser)
        assert back == pytest.approx(base)

    def test_spin_in_place_moves_an_offset_laser(self, sm):
        # base_link turns half a turn on the spot; a laser 0.24 m ahead of it
        # ends 0.48 m behind where it started, in its own start frame.
        laser = (0.24, 0.0, 0.0)
        x, y, th = sm.laser_motion((0.0, 0.0, math.pi), laser)
        assert (x, y, th) == (pytest.approx(-0.48), pytest.approx(0.0, abs=1e-12), math.pi)


class TestMatch:
    @pytest.mark.parametrize(
        'start,end,guess_error',
        [
            ((0.0, 0.0, 0.0), (0.05, 0.1, math.radians(1083.0)), (0.1, -0.1, 0.05)),
            ((-1.0, 0.0, 0.0), (2.0, 0.15, math.radians(4.0)), (0.2, 0.1, -0.03)),
            ((1.0, 0.5, 1.0), (1.3, 0.2, 1.0 + math.radians(-725.0)), (-0.15, 0.0, 0.08)),
        ],
    )
    def test_recovers_the_motion(self, sm, start, end, guess_error):
        ref = _points(sm, _scan(start, noise=0.005, seed=1))
        cur = _points(sm, _scan(end, noise=0.005, seed=2))
        truth = _relative(sm, start, end)
        guess = tuple(t + e for t, e in zip(truth, guess_error, strict=True))
        result = sm.match_scans(ref, cur, guess)
        x, y, th = result.pose
        wrapped = math.remainder(th - truth[2], 2 * math.pi)
        assert math.hypot(x - truth[0], y - truth[1]) < 0.005
        assert abs(math.degrees(wrapped)) < 0.1
        assert result.rms_m < 0.01 and result.inliers > 0.75

    def test_a_moved_object_is_trimmed(self, sm):
        # A box appears between the two scans; the walls still decide the fit.
        start, end = (0.0, 0.0, 0.0), (2.5, 0.0, 0.0)
        box = [(3.5, -1.0, 3.8, -1.0), (3.8, -1.0, 3.8, -0.7)]
        ref = _points(sm, _scan(start, noise=0.005, seed=3))
        cur = _points(sm, _scan(end, noise=0.005, seed=4, segments=ROOM + box))
        result = sm.match_scans(ref, cur, (2.4, 0.05, 0.02))
        assert math.hypot(result.pose[0] - 2.5, result.pose[1]) < 0.01

    def test_unrelated_scans_score_poorly(self, sm):
        ref = _points(sm, _scan((0.0, 0.0, 0.0)))
        rng = np.random.default_rng(5)
        cur = rng.uniform(-3.0, 3.0, (800, 2))
        result = sm.match_scans(ref, cur, (0.0, 0.0, 0.0))
        assert result.inliers < 0.5 or result.rms_m > 0.03


class TestScanHelpers:
    def test_median_ignores_missing_beams(self, sm):
        got = sm.median_ranges([[1.0, np.inf], [3.0, 2.0], [2.0, 4.0]])
        assert got.tolist() == [2.0, 3.0]
