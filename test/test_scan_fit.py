"""Unit tests for scripts/validation/scan_fit.py."""

import math

from conftest import load_script
import numpy as np
import pytest


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
    pts = sf.scan_points(np.array([0.25, 1.0, np.inf, 20.0]), 0.0, math.pi / 2)
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
