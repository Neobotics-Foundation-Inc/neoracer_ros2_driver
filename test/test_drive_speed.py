"""Unit tests for drive_speed.py: calibration speeds in m/s as /drive commands."""

from pathlib import Path
import sys
import types

from conftest import load_script as _load
import pytest

CONFIG = Path(__file__).resolve().parent.parent / 'config'
NAN = float('nan')


@pytest.fixture(scope='module')
def ds():
    return _load('drive_speed')


class TestSpeedCommand:
    def test_half_a_meter_per_second_with_the_shipped_limits(self, ds):
        limits = ds.DriveLimits(6.0, 1.0, 1.0)
        assert ds.speed_command(0.5, limits) == pytest.approx(0.5 / 6.0)
        assert ds.speed_command(-0.5, limits) == pytest.approx(-0.5 / 6.0)

    def test_each_direction_uses_its_own_throttle_cap(self, ds):
        limits = ds.DriveLimits(6.0, 0.17, 0.24)
        assert ds.speed_command(0.5, limits) == pytest.approx(0.5 / (6.0 * 0.17))
        assert ds.speed_command(-0.5, limits) == pytest.approx(-0.5 / (6.0 * 0.24))

    def test_zero_stays_zero(self, ds):
        assert ds.speed_command(0.0, ds.DriveLimits(6.0, 0.0, 0.0)) == 0.0

    def test_a_speed_beyond_the_limits_raises(self, ds):
        with pytest.raises(ValueError, match='beyond 1.0'):
            ds.speed_command(2.0, ds.DriveLimits(6.0, 0.17, 0.24))

    def test_a_zero_cap_raises(self, ds):
        with pytest.raises(ValueError, match='not a number above 0'):
            ds.speed_command(0.5, ds.DriveLimits(6.0, 0.0, 1.0))

    def test_a_negative_cap_or_top_speed_raises(self, ds):
        with pytest.raises(ValueError, match='is -0.5, not a number above 0'):
            ds.speed_command(-0.5, ds.DriveLimits(6.0, 1.0, -1.0 / 12))
        with pytest.raises(ValueError, match='not a number above 0'):
            ds.speed_command(0.5, ds.DriveLimits(-6.0, 1.0, 1.0))

    @pytest.mark.parametrize(
        'limits,mps',
        [((NAN, 1.0, 1.0), 0.5), ((6.0, NAN, 1.0), 0.5), ((6.0, 1.0, NAN), -0.5)],
    )
    def test_a_limit_that_is_not_a_number_raises(self, ds, limits, mps):
        # The throttle node turns a NaN /drive speed into full speed.
        with pytest.raises(ValueError, match='not a number above 0'):
            ds.speed_command(mps, ds.DriveLimits(*limits))

    def test_a_speed_that_is_not_a_number_raises(self, ds):
        with pytest.raises(ValueError, match='not a finite number'):
            ds.speed_command(float('nan'), ds.DriveLimits(6.0, 1.0, 1.0))

    def test_segments_keep_steering_and_time(self, ds):
        limits = ds.DriveLimits(6.0, 1.0, 1.0)
        got = ds.drive_segments([(0.0, 0.0, 2.0), (0.5, 0.1, 2.5)], limits)
        assert got == [(0.0, 0.0, 2.0), (pytest.approx(0.5 / 6.0), 0.1, 2.5)]


class TestDriveLimits:
    def test_shipped_config(self, ds, car_profile_dir):
        assert ds.drive_limits(CONFIG) == ds.DriveLimits(6.0, 1.0, 1.0)

    def test_the_profile_overrides_the_shipped_values(self, ds, car_profile_dir):
        params = car_profile_dir / 'params'
        params.mkdir(parents=True)
        (params / 'throttle.yaml').write_text(
            'throttle_node:\n  ros__parameters:\n    max_speed_forward: 0.5\n'
        )
        (params / 'controller.yaml').write_text(
            'controller_node:\n  ros__parameters:\n    max_speed_mps: 4.0\n'
        )
        limits = ds.drive_limits(CONFIG)
        assert limits == ds.DriveLimits(4.0, 0.5, 1.0)
        assert ds.speed_command(0.5, limits) == pytest.approx(0.25)

    def test_missing_values_fall_back_to_the_nodes_defaults(self, ds, car_profile_dir, tmp_path):
        assert ds.drive_limits(tmp_path) == ds.DriveLimits(6.0, 0.17, 0.24)

    def test_no_config_directory_raises(self, ds, car_profile_dir, monkeypatch):
        # The nodes' fallbacks would stand in for the YAML the running nodes loaded.
        common = types.ModuleType('calibrate_common')
        common.install_config_dir = common.source_config_dir = lambda: None
        monkeypatch.setitem(sys.modules, 'calibrate_common', common)
        with pytest.raises(ValueError, match='no config directory found'):
            ds.drive_limits()
