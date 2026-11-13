"""Unit tests for neoracer_ros2_driver/autonomy.py (autonomy_node logic without ROS)."""

from datetime import datetime
import math

import pytest
from neoracer_ros2_driver import autonomy as au
import yaml

LIMITS = au.Limits()


def _cov(xy_sd: float, yaw_sd_deg: float) -> list[float]:
    c = [0.0] * 36
    c[0] = c[7] = xy_sd**2 / 2
    c[35] = math.radians(yaw_sd_deg) ** 2
    return c


class TestLocalized:
    def test_covariance_sd(self):
        xy, yaw = au.covariance_sd(_cov(0.2, 10.0))
        assert xy == pytest.approx(0.2) and yaw == pytest.approx(10.0)

    def test_mapping_needs_only_map_odom_and_scans(self):
        # slam_toolbox publishes its pose only once the car moves.
        assert au.is_localized('mapping', True, 0.1, None, 0, False, LIMITS)

    def test_tracking_amcl(self):
        assert au.is_localized('localization', True, 0.1, (0.11, 5.3), 10, False, LIMITS)

    @pytest.mark.parametrize('sd', [(2.1, 114.0), (0.3, 5.0), (0.1, 25.0)])
    def test_a_lost_amcl(self, sd):
        assert not au.is_localized('localization', True, 0.1, sd, 10, False, LIMITS)

    @pytest.mark.parametrize('scan_age', [None, 0.6])
    def test_lidar_loss(self, scan_age):
        assert not au.is_localized('localization', True, scan_age, (0.1, 5.0), 10, False, LIMITS)
        assert not au.is_localized('mapping', True, scan_age, None, 10, False, LIMITS)

    def test_no_map_odom(self):
        assert not au.is_localized('mapping', False, 0.1, None, 10, False, LIMITS)

    def test_global_search_waits(self):
        assert not au.is_localized('localization', True, 0.1, (0.1, 5.0), 10, True, LIMITS)

    def test_status_level(self):
        assert au.status_level(True, True, 1.0) == au.LEVEL_OK
        assert au.status_level(False, True, 1.0) == au.LEVEL_WARN
        assert au.status_level(False, False, 2.0) == au.LEVEL_WARN  # starting up
        assert au.status_level(False, False, 6.0) == au.LEVEL_ERROR


class TestFiles:
    def test_atomic_yaml(self, tmp_path):
        path = tmp_path / 'm' / 'last_pose.yaml'
        au.write_yaml_atomic(path, {'x': 1.0})
        assert yaml.safe_load(path.read_text()) == {'x': 1.0}
        assert not path.with_name('last_pose.yaml.tmp').exists()

    def test_replace_dir(self, tmp_path):
        final, tmp = tmp_path / 'autosave', tmp_path / 'autosave.tmp'
        final.mkdir()
        (final / 'old').write_text('1')
        tmp.mkdir()
        (tmp / 'new').write_text('2')
        au.replace_dir(tmp, final)
        assert [p.name for p in final.iterdir()] == ['new']
        assert not tmp.exists() and not (tmp_path / 'autosave.old').exists()

    def test_replace_dir_first_time(self, tmp_path):
        tmp = tmp_path / 'autosave.tmp'
        tmp.mkdir()
        au.replace_dir(tmp, tmp_path / 'autosave')
        assert (tmp_path / 'autosave').is_dir()

    def test_pose_round_trip(self, tmp_path):
        path = tmp_path / 'last_pose.yaml'
        au.write_yaml_atomic(path, au.pose_record((1.234567, -2.0, 0.5), datetime(2026, 10, 9)))
        assert au.read_pose(path) == (1.2346, -2.0, 0.5)

    @pytest.mark.parametrize('text', ['', 'x: 1\n', 'not: [yaml'])
    def test_bad_pose_file(self, tmp_path, text):
        path = tmp_path / 'last_pose.yaml'
        path.write_text(text)
        assert au.read_pose(path) is None
        assert au.read_pose(tmp_path / 'missing.yaml') is None

    def test_meta(self):
        meta = au.meta_record(
            'room',
            datetime(2026, 10, 9, 1, 2, 3),
            '0.9.0',
            {'a_mm': 1.0},
            's',
            (1.0, -2.0, 0.5),
            '2026-10-09 01:02:00.250',
            'hall',
        )
        assert meta['saved'] == '2026-10-09 01:02:03' and meta['frames'] == {'a_mm': 1.0}
        assert meta['hostname'] and meta['session'] == 's'
        assert meta['start_pose'] == {'x': 1.0, 'y': -2.0, 'yaw': 0.5}
        assert meta['last_change'] == '2026-10-09 01:02:00.250'
        assert meta['extended_from'] == 'hall'
        assert au.meta_record('room', datetime(2026, 10, 9), '', {}, 's')['start_pose'] is None


def test_status_values():
    got = dict(
        au.status_values(
            mode='mapping', localized=True, tf_age_ms=12.6, pose_sd_m=0.1234, last_save=None
        )
    )
    assert got == {
        'mode': 'mapping',
        'localized': 'true',
        'tf_age_ms': '13',
        'pose_sd_m': '0.123',
        'last_save': '',
    }


class TestResumePoint:
    def _autosave(self, folder, session, time, pose=(1.0, 2.0, 0.5)):
        auto = folder / au.AUTOSAVE_DIR
        auto.mkdir(parents=True, exist_ok=True)
        (auto / 'room.posegraph').write_text('g')
        au.write_yaml_atomic(auto / au.AUTOSAVE_FILE, au.pose_record(pose, time, session=session))

    def test_same_session_resumes_at_the_autosave_pose(self, tmp_path):
        self._autosave(tmp_path, 's1', datetime(2026, 10, 9, 10, 0, 0))
        stem, pose = au.resume_point(tmp_path, 'room', 's1')
        assert stem == tmp_path / 'autosave' / 'room' and pose == (1.0, 2.0, 0.5)

    def test_a_newer_mapping_pose_wins(self, tmp_path):
        self._autosave(tmp_path, 's1', datetime(2026, 10, 9, 10, 0, 0))
        au.write_yaml_atomic(
            tmp_path / au.MAPPING_POSE_FILE,
            au.pose_record((3.0, 4.0, 1.0), datetime(2026, 10, 9, 10, 0, 40), session='s1'),
        )
        assert au.resume_point(tmp_path, 'room', 's1')[1] == (3.0, 4.0, 1.0)

    def test_an_older_or_foreign_mapping_pose_loses(self, tmp_path):
        self._autosave(tmp_path, 's1', datetime(2026, 10, 9, 10, 0, 0))
        au.write_yaml_atomic(
            tmp_path / au.MAPPING_POSE_FILE,
            au.pose_record((3.0, 4.0, 1.0), datetime(2026, 10, 9, 10, 0, 40), session='s0'),
        )
        assert au.resume_point(tmp_path, 'room', 's1')[1] == (1.0, 2.0, 0.5)

    @pytest.mark.parametrize('session', ['s2', ''])
    def test_another_session_starts_fresh(self, tmp_path, session):
        self._autosave(tmp_path, 's1', datetime(2026, 10, 9, 10, 0, 0))
        assert au.resume_point(tmp_path, 'room', session) is None

    def test_no_autosave(self, tmp_path):
        assert au.resume_point(tmp_path, 'room', 's1') is None


def _map_files(folder, name, text='m'):
    folder.mkdir(parents=True, exist_ok=True)
    for suffix in au.MAP_SUFFIXES:
        (folder / f'{name}{suffix}').write_text(text)


def _meta(folder, **fields):
    au.write_yaml_atomic(folder / au.META_FILE, fields)


class TestMapState:
    def test_missing(self, tmp_path):
        state = au.map_state(tmp_path, 'room')
        assert not state.exists and not state.unsaved

    def test_saved_only(self, tmp_path):
        _map_files(tmp_path / 'room', 'room')
        state = au.map_state(tmp_path, 'room')
        assert state.saved and not state.autosaved and not state.unsaved

    def test_autosave_only_is_unsaved(self, tmp_path):
        _map_files(tmp_path / 'room' / au.AUTOSAVE_DIR, 'room')
        state = au.map_state(tmp_path, 'room')
        assert state.exists and not state.saved and state.unsaved

    def test_a_partial_set_is_not_a_map(self, tmp_path):
        (tmp_path / 'room').mkdir()
        (tmp_path / 'room' / 'room.pgm').write_text('m')
        assert not au.map_state(tmp_path, 'room').exists

    @pytest.mark.parametrize(
        'saved,auto,unsaved',
        [
            ('2026-10-09 10:00:00.000', '2026-10-09 10:00:00.000', False),
            ('2026-10-09 10:00:00.000', '2026-10-09 10:00:05.100', True),
            ('', '', False),
        ],
    )
    def test_newer_autosave_by_last_change(self, tmp_path, saved, auto, unsaved):
        folder = tmp_path / 'room'
        _map_files(folder, 'room')
        _map_files(folder / au.AUTOSAVE_DIR, 'room')
        _meta(folder, last_change=saved, saved='2026-10-09 10:00:01')
        # The stop autosave is always written after the save; only new scans count.
        _meta(folder / au.AUTOSAVE_DIR, last_change=auto)
        au.write_yaml_atomic(
            folder / au.AUTOSAVE_DIR / au.AUTOSAVE_FILE, {'time': '2026-10-09 10:00:09'}
        )
        assert au.map_state(tmp_path, 'room').unsaved is unsaved

    def test_newer_autosave_before_last_change_existed(self, tmp_path):
        folder = tmp_path / 'room'
        _map_files(folder, 'room')
        _map_files(folder / au.AUTOSAVE_DIR, 'room')
        _meta(folder, saved='2026-10-09 10:00:00')
        au.write_yaml_atomic(
            folder / au.AUTOSAVE_DIR / au.AUTOSAVE_FILE, {'time': '2026-10-09 10:00:30'}
        )
        state = au.map_state(tmp_path, 'room')
        assert state.unsaved and state.autosave_time == '2026-10-09 10:00:30'

    def test_start_pose(self, tmp_path):
        _map_files(tmp_path / 'room', 'room')
        _meta(tmp_path / 'room', start_pose={'x': 1, 'y': 2, 'yaw': 0.5})
        assert au.map_state(tmp_path, 'room').start_pose == (1.0, 2.0, 0.5)


class TestMapFolders:
    NOW = datetime(2026, 10, 9, 14, 5, 6)

    def test_archive_moves_the_set_and_keeps_the_last_pose(self, tmp_path):
        _map_files(tmp_path, 'room')
        _meta(tmp_path, session='a')
        (tmp_path / au.LAST_POSE_FILE).write_text('x: 1')
        dest = au.archive_map_files(tmp_path, 'room', self.NOW)
        assert dest == tmp_path / au.PREVIOUS_DIR / '20261009-140506'
        assert sorted(p.name for p in dest.iterdir()) == sorted(
            [f'room{s}' for s in au.MAP_SUFFIXES] + [au.META_FILE]
        )
        assert not au.has_map_files(tmp_path, 'room')
        assert (tmp_path / au.LAST_POSE_FILE).exists()

    def test_archive_never_overwrites(self, tmp_path):
        _map_files(tmp_path, 'room', 'first')
        first = au.archive_map_files(tmp_path, 'room', self.NOW)
        _map_files(tmp_path, 'room', 'second')
        second = au.archive_map_files(tmp_path, 'room', self.NOW)
        assert first != second and second.name == '20261009-140506-2'
        assert (first / 'room.pgm').read_text() == 'first'

    def test_archive_nothing(self, tmp_path):
        assert au.archive_map_files(tmp_path, 'room', self.NOW) is None

    def test_replace_folder(self, tmp_path):
        _map_files(tmp_path / 'room', 'room')
        dest = au.replace_folder(tmp_path, 'room', self.NOW)
        assert dest == tmp_path / au.REPLACED_DIR / 'room-20261009-140506'
        assert au.has_map_files(dest, 'room') and not (tmp_path / 'room').exists()

    def test_copy_autosave(self, tmp_path):
        _map_files(tmp_path / au.AUTOSAVE_DIR, 'room', 'auto')
        _meta(tmp_path / au.AUTOSAVE_DIR, last_change='t')
        au.copy_autosave(tmp_path, 'room')
        assert (tmp_path / 'room.posegraph').read_text() == 'auto'
        assert au.read_record(tmp_path / au.META_FILE)['last_change'] == 't'
        assert au.has_map_files(tmp_path / au.AUTOSAVE_DIR, 'room')

    def test_change_time_orders_as_text(self):
        a = au.change_time(datetime(2026, 10, 9, 9, 59, 59, 999000))
        b = au.change_time(datetime(2026, 10, 9, 10, 0, 0, 1000))
        assert a == '2026-10-09 09:59:59.999' and a < b


def test_seed_covariance():
    cov = au.seed_covariance(0.25, 10.0)
    assert len(cov) == 36 and cov[0] == cov[7] == pytest.approx(0.0625)
    assert au.covariance_sd(cov)[1] == pytest.approx(10.0)
    # A seeded start reads above the localized limit until the scan narrows it.
    assert au.covariance_sd(cov)[0] > au.Limits().max_xy_sd_m


def test_shows_seed():
    cov = au.seed_covariance(0.25, 10.0)
    assert au.shows_seed((0.318, 9.3), cov)
    assert not au.shows_seed((0.0, 0.0), cov)


def test_mark_pose_lost(tmp_path):
    path = tmp_path / au.LAST_POSE_FILE
    au.mark_pose_lost(path, datetime(2026, 10, 9))
    assert not path.exists()
    au.write_yaml_atomic(path, au.pose_record((1.0, 2.0, 0.5), datetime(2026, 10, 9, 9)))
    au.mark_pose_lost(path, datetime(2026, 10, 9, 9, 5))
    record = au.read_record(path)
    assert record[au.LOCALIZED_AT_STOP] is False and record['stopped'] == '2026-10-09 09:05:00'
    assert au.read_pose(path) == (1.0, 2.0, 0.5)


def test_localized_hysteresis():
    # 0.2 m: under the limit, above the entry level (0.6 x 0.25 = 0.15 m).
    args = ('localization', True, 0.1, (0.2, 5.0), 10, False, LIMITS)
    assert not au.is_localized(*args)
    assert au.is_localized(*args, was_localized=True)
    assert au.is_localized('localization', True, 0.1, (0.14, 11.0), 10, False, LIMITS)
    assert not au.is_localized('localization', True, 0.1, (0.1, 13.0), 10, False, LIMITS)


class TestScanFit:
    def _walls(self):
        # 20 x 20 cells of 0.1 m, origin (0, 0); a wall along the top row (y 1.9 to 2.0).
        import numpy as np

        occupied = np.zeros((20, 20), dtype=bool)
        occupied[0, :] = True
        return au.WallMask.from_occupied(occupied, (0.0, 0.0), 0.1)

    def test_share_counts_near_wall_and_off_map_misses(self):
        import numpy as np

        walls = self._walls()
        x = np.array([1.0, 1.0, 1.0, 5.0])
        y = np.array([1.95, 1.85, 1.0, 1.95])  # on the wall, one cell off, far, off the map
        assert walls.share(x, y) == pytest.approx(0.5)
        assert walls.share(np.array([]), np.array([])) == 0.0

    def test_from_grid_flips_rows_and_ignores_unknown(self):
        # OccupancyGrid rows start at the bottom: an occupied bottom row is the mask's last row.
        import numpy as np

        width, height = 4, 3
        data = [100, 100, 100, 100] + [-1, 0, 0, 0] + [0, 0, 64, 0]
        walls = au.WallMask.from_grid(data, width, height, (0.0, 0.0), 1.0, occupied_at=65)
        assert walls.near.shape == (3, 4)
        assert walls.near[2].all()  # bottom row in the grid, last row in the mask
        assert not walls.near[0].any()  # unknown (-1) and 64 are not walls
        point = np.array([0.5])
        assert walls.share(point, point) == 1.0  # map point (0.5, 0.5) is in the bottom row

    def test_scan_points_in_map_turns_and_filters(self):
        ranges = [1.0, float('inf'), 0.1, 2.0]
        x, y = au.scan_points_in_map(ranges, 0.0, math.pi / 2, (1.0, 0.0, math.pi), 0.3, 12.0, 4)
        # Beam 0 points along the laser's x, which faces -x in the map; beam 3 at 270 deg.
        assert list(x) == pytest.approx([0.0, 1.0])
        assert list(y) == pytest.approx([0.0, 2.0])

    def test_scan_points_in_map_subsamples(self):
        x, _ = au.scan_points_in_map([1.0] * 1080, 0.0, 0.01, (0.0, 0.0, 0.0), 0.3, 12.0, 180)
        assert len(x) == 180

    def test_gate_hysteresis_and_window(self):
        gate = au.FitGate(window_sec=2.0, drop_share=0.5, enter_share=0.65, min_scores=3)
        assert not gate.passes(0.0, False)  # no scores yet
        for t in (0.0, 0.5, 1.0):
            gate.add(t, 0.6)
        assert not gate.passes(1.0, False)  # under the entry share
        assert gate.passes(1.0, True)  # over the drop share
        for t in (1.5, 2.0, 2.5, 3.0):
            gate.add(t, 0.3)
        assert gate.mean(3.0) == pytest.approx(0.36)  # the window still holds the 1.0 s score
        assert gate.mean(3.1) == pytest.approx(0.3)  # the 0.6 scores aged out
        assert not gate.passes(3.1, True)
        assert gate.mean(10.0) is None  # nothing recent: not judged as good

    def test_localized_needs_fit_in_localization_only(self):
        args = ('localization', True, 0.1, (0.1, 5.0), 10, False, LIMITS)
        assert au.is_localized(*args)
        assert not au.is_localized(*args, fit_ok=False)
        assert au.is_localized('mapping', True, 0.1, None, 0, False, LIMITS, fit_ok=False)
