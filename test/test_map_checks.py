"""Unit tests for scripts/validation/live_fit.py and compare_maps.py."""

import math

from conftest import load_script
import numpy as np
import pytest


@pytest.fixture(scope='module')
def lf():
    load_script('scan_fit')  # imported by name as a sibling
    return load_script('live_fit')


@pytest.fixture(scope='module')
def cm():
    load_script('scan_fit')
    return load_script('compare_maps')


def _box(size: int = 40) -> np.ndarray:
    # size x size cells, walls on all four edges.
    pixels = np.full((size, size), 254, dtype=np.uint8)
    pixels[0, :] = pixels[-1, :] = pixels[:, 0] = pixels[:, -1] = 0
    return pixels


def _box_points(n: int = 400) -> np.ndarray:
    # A 4 m square of 0.1 m cells; wall cell centers at 0.05 and 3.95. Points from (2, 2).
    side = np.linspace(0.3, 3.7, n // 4)
    walls = np.vstack(
        [
            np.column_stack((side, np.full_like(side, 0.05))),
            np.column_stack((side, np.full_like(side, 3.95))),
            np.column_stack((np.full_like(side, 0.05), side)),
            np.column_stack((np.full_like(side, 3.95), side)),
        ]
    )
    return walls - 2.0


class TestLiveFit:
    def test_true_pose_fits(self, lf):
        grid = lf.sf.grid_from_pixels(_box(), (0.0, 0.0), 0.1)
        s = lf.score(grid, _box_points(), (2.0, 2.0, 0.0), (0.0, 0.0, 0.0))
        # Distances are to wall cell centers: under half a cell along the wall.
        assert s.median_m < 0.05 and s.inlier == 1.0

    def test_search_finds_the_offset(self, lf):
        grid = lf.sf.grid_from_pixels(_box(), (0.0, 0.0), 0.1)
        reported = (2.15, 1.9, math.radians(3.0))
        best = lf.best_nearby(grid, _box_points(), reported, (0.0, 0.0, 0.0), 0.3, 5.0)
        assert best.pose[0] == pytest.approx(2.0, abs=0.03)
        assert best.pose[1] == pytest.approx(2.0, abs=0.03)
        assert math.degrees(best.pose[2]) == pytest.approx(0.0, abs=1.01)
        assert best.inlier > lf.score(grid, _box_points(), reported, (0, 0, 0)).inlier


class TestCompareMaps:
    def test_same_map_agrees(self, cm):
        g = cm.Grid(_box(), (0.0, 0.0), 0.1)
        a = cm.wall_agreement(g, g)
        assert a.cells == 4 * 39 and a.median_m == 0.0 and a.within == 1.0

    def test_shifted_copy_disagrees(self, cm):
        base = cm.Grid(_box(), (0.0, 0.0), 0.1)
        shifted = cm.Grid(_box(), (0.6, 0.0), 0.1)
        a = cm.wall_agreement(base, shifted)
        assert a.beyond > 0.2

    def test_new_area_does_not_count(self, cm):
        base = _box()
        base[:, 20:] = 205  # the right half never observed
        extended = cm.Grid(_box(), (0.0, 0.0), 0.1)
        a = cm.wall_agreement(cm.Grid(base, (0.0, 0.0), 0.1), extended)
        assert a.cells < 4 * 39 and a.within == 1.0

    def test_overlay_extent_and_colors(self, cm):
        base = cm.Grid(_box(20), (0.0, 0.0), 0.1)
        extended = cm.Grid(_box(20), (1.0, 0.0), 0.1)
        img = cm.overlay(base, extended)
        assert img.shape == (20, 30, 3)
        assert tuple(img[10, 0]) == cm.BASE_COLOR
        assert tuple(img[10, 29]) == cm.EXTENDED_COLOR
        # Column 10: base's free interior under extended's left wall.
        assert tuple(img[10, 10]) == cm.EXTENDED_COLOR
        assert tuple(img[0, 15]) == (40, 40, 40)  # top wall drawn by both
