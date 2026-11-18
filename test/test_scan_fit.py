"""Unit tests for scripts/validation/scan_fit.py."""

import math
from pathlib import Path

from conftest import load_script
import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def sf():
    return load_script('scan_fit')


def _room(sf):
    # 20 x 20 cells of 0.1 m, origin (0, 0); a wall along row 0 (the top edge, y 1.9 to 2.0).
    pixels = np.full((20, 20), 254, dtype=np.uint8)
    pixels[0, :] = 0
    return sf.grid_from_pixels(pixels, (0.0, 0.0), 0.1)


def test_distance_is_continuous(sf):
    grid = _room(sf)
    d = grid.lookup(np.array([1.05, 1.05, 1.05]), np.array([1.95, 1.50, 2.50]))
    assert d[0] == pytest.approx(0.0, abs=1e-9)  # on the wall cell's center
    assert d[1] == pytest.approx(0.45)
    assert d[2] == sf.OUTSIDE_M  # outside the map


def test_compose(sf):
    p = sf.compose((1.0, 0.0, math.pi / 2), (1.0, 0.0, 0.0))
    assert p[0] == pytest.approx(1.0) and p[1] == pytest.approx(1.0)


def test_scan_points_drop_the_self_returns(sf):
    ranges = np.array([sf.MIN_RANGE_M - 0.01, 1.0, np.inf, sf.MAX_RANGE_M + 1.0])
    pts = sf.scan_points(ranges, 0.0, math.pi / 2)
    assert pts == pytest.approx(np.array([[0.0, 1.0]]), abs=1e-12)


def test_a_true_pose_fits_and_a_wrong_one_does_not(sf):
    grid = _room(sf)
    pts = sf.scan_points(np.array([1.0]), math.pi / 2, 0.0)  # wall 1 m ahead along laser +y
    track = np.array([[0.0, 0.0, 0.0, 0.0], [10.0, 0.0, 0.0, 0.0]])
    base = np.array([[0.0, 1.05, 0.95, 0.0], [10.0, 1.05, 0.95, 0.0]])
    good = sf.fit_scans(grid, [(1.0, pts)], track, base, (0.0, 0.0, 0.0))
    shifted = np.array([[0.0, 0.0, -0.4, 0.0], [10.0, 0.0, -0.4, 0.0]])
    bad = sf.fit_scans(grid, [(1.0, pts)], shifted, base, (0.0, 0.0, 0.0))
    assert good.median_m == pytest.approx(0.0, abs=1e-9) and good.inlier == 1.0
    assert bad.median_m == pytest.approx(0.4) and bad.inlier == 0.0


def test_scans_outside_the_tracks_are_skipped(sf):
    grid = _room(sf)
    track = np.array([[0.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]])
    pts = np.array([[1.0, 0.0]])
    assert sf.fit_scans(grid, [(5.0, pts)], track, track, (0.0, 0.0, 0.0)) is None


def test_localized_against_fit(sf):
    # Fit good for 0 to 4 s, bad from 5 s; localized from 1 s to 8 s, then false.
    shares = [(t * 0.5, 0.8 if t * 0.5 < 5 else 0.2) for t in range(21)]
    status = [(0.0, False), (1.0, True), (8.0, False)]
    wrong, missed, span = sf.localized_against_fit(status, shares, window_sec=1.0)
    assert span == pytest.approx(10.0)
    assert wrong == pytest.approx(2.5)  # 5.5 to 8.0: the 1 s window is all bad from 5.5
    assert missed == pytest.approx(0.0)
    status = [(0.0, False), (1.0, True), (2.0, False)]
    _, missed, _ = sf.localized_against_fit(status, shares, window_sec=1.0)
    assert missed == pytest.approx(3.0)  # 2.0 to 5.0, good fit after the first localized


class TestNeoRacer:
    """The range band comes from the SLAM config; LakiBeam scans score as upstream's do."""

    def test_band_is_slam_toolboxs(self, sf):
        config = yaml.safe_load((ROOT / 'config' / 'slam_mapping.yaml').read_text())
        params = config['slam_toolbox']['ros__parameters']
        assert (sf.MIN_RANGE_M, sf.MAX_RANGE_M) == (
            params['min_laser_range'],
            params['max_laser_range'],
        )

    def test_the_car_profile_override_wins(self, sf, tmp_path):
        override = tmp_path / 'slam_mapping.yaml'
        override.write_text('slam_toolbox:\n  ros__parameters:\n    min_laser_range: 0.3\n')
        low, high = sf.laser_band(sf.SLAM_CONFIG, override)
        assert low == 0.3 and high == sf.MAX_RANGE_M

    def test_lakibeam_scan_drops_the_blind_wedge(self, sf):
        # 1440 beams of 0.25 deg from -180 deg; the rear 90 deg reads inf, as on the car.
        angles = -math.pi + np.radians(0.25) * np.arange(1440)
        ranges = np.where(np.abs(angles) > np.radians(135.0), np.inf, 2.0)
        ranges[720] = sf.MIN_RANGE_M / 2  # a self-return straight ahead
        pts = sf.scan_points(ranges, -math.pi, np.radians(0.25))
        assert len(pts) == 1081 - 1  # 1081 beams within 135 deg of the nose, less the self-return
        assert np.hypot(pts[:, 0], pts[:, 1]) == pytest.approx(2.0)
        assert np.all(pts[:, 0] > 2.0 * math.cos(np.radians(135.25)))
