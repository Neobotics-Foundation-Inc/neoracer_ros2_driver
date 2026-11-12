"""Unit tests for scripts/sysinfo.py."""

import subprocess
import sys
import time
from types import SimpleNamespace

from conftest import load_script
import pytest

sysinfo = load_script('sysinfo')

# Read on the dev car (Jetson Orin Nano, JetPack 6) on 2026-11-11.
# Thermal zones as (type, temp file text); None is a zone whose temp read
# fails, as the cv*-thermal zones of the idle vision engine do.
CAR_THERMAL_ZONES = [
    ('cpu-thermal', '50406'),
    ('gpu-thermal', '50562'),
    ('cv0-thermal', None),
    ('cv1-thermal', None),
    ('cv2-thermal', None),
    ('soc0-thermal', '50031'),
    ('soc1-thermal', '51656'),
    ('soc2-thermal', '49843'),
    ('tj-thermal', '51562'),
]
CAR_HWMON_NAMES = ['usb_charger', 'pwmfan', 'ina3221', 'pwm_tach', 'soctherm_oc']
CAR_CPU_MAX_KHZ = '1728000'
CAR_CPU_CUR_KHZ = '1344000'
CAR_NVPMODEL = 'NV Power Mode: 25W\n1\n'
CAR_TEGRASTATS = [
    '11-11-2026 23:28:49 RAM 2771/7620MB (lfb 2x4MB) SWAP 0/3810MB (cached 0MB) '
    'CPU [57%@1190,43%@1190,53%@1190,52%@1190,52%@729,52%@729] GR3D_FREQ 78% '
    'cpu@50.218C soc2@49.875C soc0@49.968C gpu@50.687C tj@51.593C soc1@51.593C '
    'VDD_IN 6776mW/6776mW VDD_CPU_GPU_CV 1985mW/1985mW VDD_SOC 1663mW/1663mW',
    '11-11-2026 23:28:50 RAM 2770/7620MB (lfb 2x4MB) SWAP 0/3810MB (cached 0MB) '
    'CPU [47%@1344,51%@1344,53%@1344,49%@1344,48%@806,48%@806] GR3D_FREQ 76% '
    'cpu@50.343C soc2@49.812C soc0@49.968C gpu@50.718C tj@51.687C soc1@51.687C '
    'VDD_IN 6776mW/6776mW VDD_CPU_GPU_CV 1985mW/1985mW VDD_SOC 1623mW/1643mW',
]
CAR_MEMINFO = (
    'MemTotal:        7803236 kB\n'
    'MemFree:         3123076 kB\n'
    'MemAvailable:    4805300 kB\n'
    'Buffers:          111784 kB\n'
    'Cached:          1724084 kB\n'
)
# df -B1 / on the car's NVMe root: total, used, available bytes.
CAR_DISK = (123887886336, 27156271104, 90391138304)
CAR_UPTIME_SEC = 972.50


def _completed(stdout, returncode=0):
    return SimpleNamespace(stdout=stdout, returncode=returncode)


@pytest.fixture
def fake_run(monkeypatch):
    """Replace subprocess.run in sysinfo; set .result or .exc on the returned stub."""
    stub = SimpleNamespace(result=_completed(''), exc=None, calls=[])

    def run(cmd, **_kwargs):
        stub.calls.append(cmd)
        if stub.exc is not None:
            raise stub.exc
        return stub.result

    fake = SimpleNamespace(run=run, TimeoutExpired=subprocess.TimeoutExpired)
    monkeypatch.setattr(sysinfo, 'subprocess', fake)
    return stub


@pytest.fixture
def car_thermal(tmp_path):
    """A /sys/class/thermal tree with the car's zones; failing zones have no temp file."""
    for i, (kind, temp) in enumerate(CAR_THERMAL_ZONES):
        zone = tmp_path / f'thermal_zone{i}'
        zone.mkdir()
        (zone / 'type').write_text(kind + '\n')
        if temp is not None:
            (zone / 'temp').write_text(temp + '\n')
    return tmp_path


def _hwmon_tree(root, oc_counts=None):
    """Build a /sys/class/hwmon tree with the car's drivers; oc_counts fills soctherm_oc."""
    for i, name in enumerate(CAR_HWMON_NAMES):
        h = root / f'hwmon{i}'
        h.mkdir()
        (h / 'name').write_text(name + '\n')
        if name == 'soctherm_oc':
            for k, count in (oc_counts or {}).items():
                (h / f'oc{k}_event_cnt').write_text(f'{count}\n')
    return root


class TestDecodeThrottled:
    # The car's counters were not read; these counts are made up in the
    # format `grep . /sys/class/hwmon/hwmon4/oc*_event_cnt` prints.
    def test_counts_that_fired_set_their_bits(self):
        flags, active = sysinfo.decode_throttled(
            'oc1_event_cnt:0\noc2_event_cnt:3\noc3_event_cnt:0'
        )
        assert flags == 0b010
        assert active == ['over-current throttling oc2 has occurred (3 events)']

    def test_grep_output_with_paths(self):
        text = '/sys/class/hwmon/hwmon4/oc1_event_cnt:5\n/sys/class/hwmon/hwmon4/oc3_event_cnt:1\n'
        flags, active = sysinfo.decode_throttled(text)
        assert flags == 0b101
        assert len(active) == 2

    def test_clean(self):
        assert sysinfo.decode_throttled('oc1_event_cnt:0\noc2_event_cnt:0\noc3_event_cnt:0') == (
            0,
            [],
        )

    def test_unparsable(self):
        assert sysinfo.decode_throttled('error=1') == (None, [])

    def test_every_name_is_sticky(self):
        # The counters only grow until reboot, so no condition reads as live.
        _, active = sysinfo.decode_throttled('oc1_event_cnt:2\noc2_event_cnt:1')
        assert active and all('has occurred' in name for name in active)

    def test_read_throttled_finds_soctherm_oc_by_name(self, tmp_path):
        root = _hwmon_tree(tmp_path, {1: 0, 2: 0, 3: 4})
        flags, active = sysinfo.read_throttled(root)
        assert flags == 0b100
        assert active == ['over-current throttling oc3 has occurred (4 events)']

    def test_read_throttled_without_soctherm_oc(self, tmp_path):
        (tmp_path / 'hwmon0').mkdir()
        (tmp_path / 'hwmon0' / 'name').write_text('pwmfan\n')
        assert sysinfo.read_throttled(tmp_path) == (None, [])


class TestClassifyRtc:
    def test_above_ok_is_healthy(self):
        status, label = sysinfo.classify_rtc(2.95)
        assert status == 'healthy'
        assert '2.95' in label

    def test_ok_line_is_healthy(self):
        assert sysinfo.classify_rtc(sysinfo.RTC_OK_VOLTS)[0] == 'healthy'

    def test_between_low_and_ok_is_stale(self):
        status, label = sysinfo.classify_rtc(2.75)
        assert status == 'stale'
        assert 'recharge soon' in label

    def test_low_line_is_stale(self):
        assert sysinfo.classify_rtc(sysinfo.RTC_LOW_VOLTS)[0] == 'stale'

    def test_below_low_is_dead(self):
        status, label = sysinfo.classify_rtc(2.6999)
        assert status == 'dead'
        assert 'RECHARGE NOW' in label

    def test_no_reading_is_dead(self):
        assert sysinfo.classify_rtc(None) == ('dead', 'NO READING')


class TestRtcVoltage:
    def test_no_rtc_cell_reading_on_the_jetson(self, fake_run):
        assert sysinfo.read_rtc_voltage() is None
        assert fake_run.calls == []


class TestUnderVoltageAlarm:
    def test_missing_hwmon_is_none(self, monkeypatch):
        monkeypatch.setattr(sysinfo, 'under_voltage_alarm_path', lambda: None)
        assert sysinfo.read_under_voltage_alarm() is None

    @pytest.mark.parametrize('text,expected', [('1\n', True), ('0\n', False)])
    def test_reads_the_sticky_flag(self, monkeypatch, tmp_path, text, expected):
        alarm = tmp_path / 'in0_lcrit_alarm'
        alarm.write_text(text)
        monkeypatch.setattr(sysinfo, 'under_voltage_alarm_path', lambda: alarm)
        assert sysinfo.read_under_voltage_alarm() is expected

    def test_the_jetson_has_no_alarm_file(self):
        assert sysinfo.under_voltage_alarm_path() is None
        assert sysinfo.read_under_voltage_alarm() is None


class TestRunCmd:
    def test_returns_stdout(self, fake_run):
        fake_run.result = _completed('out\n')
        assert sysinfo.run_cmd(['true']) == 'out\n'

    def test_nonzero_exit_is_empty(self, fake_run):
        fake_run.result = _completed('partial', returncode=1)
        assert sysinfo.run_cmd(['false']) == ''

    def test_missing_binary_is_empty(self, fake_run):
        fake_run.exc = FileNotFoundError('nope')
        assert sysinfo.run_cmd(['nope']) == ''


class TestFirstLine:
    def test_returns_the_first_line_and_stops_the_command(self):
        cmd = [sys.executable, '-c', 'import time; print("one", flush=True); time.sleep(30)']
        assert sysinfo.first_line(cmd, timeout=10) == 'one'

    def test_missing_binary_is_empty(self):
        assert sysinfo.first_line(['/nonexistent/tegrastats']) == ''

    def test_a_half_written_line_does_not_outlast_the_timeout(self):
        cmd = [
            sys.executable,
            '-c',
            'import sys, time; sys.stdout.write("half"); sys.stdout.flush(); time.sleep(30)',
        ]
        start = time.monotonic()
        assert sysinfo.first_line(cmd, timeout=1.0) == ''
        assert time.monotonic() - start < 3.0

    def test_stops_the_command_and_closes_its_pipe(self, monkeypatch):
        started = []
        real_popen = subprocess.Popen

        def spy(*args, **kwargs):
            proc = real_popen(*args, **kwargs)
            started.append(proc)
            return proc

        monkeypatch.setattr(sysinfo.subprocess, 'Popen', spy)
        cmd = [sys.executable, '-c', 'import time; print("one", flush=True); time.sleep(30)']
        assert sysinfo.first_line(cmd, timeout=10) == 'one'
        assert started[0].poll() is not None
        assert started[0].stdout.closed


class TestDiagnosticRates:
    @staticmethod
    def _msg(*statuses):
        return SimpleNamespace(
            status=[
                SimpleNamespace(
                    name=name, values=[SimpleNamespace(key=k, value=v) for k, v in values]
                )
                for name, values in statuses
            ]
        )

    def test_extracts_the_actual_frequency(self):
        msg = self._msg(
            (
                'camera: color',
                [('Target frequency (Hz)', '60.0'), ('Actual frequency (Hz)', '59.4')],
            ),
            ('camera: Temperatures', [('Asic Temperature', '54')]),
        )
        out = {}
        sysinfo.read_diagnostic_rates(msg, out)
        assert out == {'/camera/color': pytest.approx(59.4)}

    def test_ignores_an_unparsable_rate(self):
        out = {}
        sysinfo.read_diagnostic_rates(
            self._msg(('camera: depth', [('Actual frequency (Hz)', 'n/a')])), out
        )
        assert out == {}

    def test_names_cover_the_three_realsense_streams(self):
        assert set(sysinfo.REALSENSE_DIAGNOSTIC_NAMES.values()) == {
            '/camera/color',
            '/camera/depth',
            '/imu/realsense',
        }


class TestMemory:
    MEMINFO = 'MemTotal:        8245972 kB\nMemFree:  100 kB\nMemAvailable:    6291456 kB\n'

    def test_mib_totals(self):
        assert sysinfo.parse_meminfo(self.MEMINFO) == {
            'total': 8052,
            'available': 6144,
            'used': 1908,
        }

    def test_missing_field_is_none(self):
        assert sysinfo.parse_meminfo('MemTotal: 1024 kB\n') is None

    def test_the_cars_meminfo_matches_tegrastats(self):
        mem = sysinfo.parse_meminfo(CAR_MEMINFO)
        assert mem == {'total': 7620, 'available': 4692, 'used': 2928}
        assert mem['total'] == sysinfo.parse_tegrastats(CAR_TEGRASTATS[0])['ram_total_mb']


class TestCpuTimes:
    STAT = 'cpu  494969 599 69983 34490 4416 0 9467 0 0 0\ncpu0 1 2 3 4 5 6 7 8 0 0\n'

    def test_idle_and_iowait_are_not_busy(self):
        busy, total = sysinfo.parse_cpu_times(self.STAT)
        assert total == 494969 + 599 + 69983 + 34490 + 4416 + 9467
        assert busy == total - 34490 - 4416

    def test_guest_fields_are_not_double_counted(self):
        # guest (field 9) is already inside user.
        assert sysinfo.parse_cpu_times('cpu 10 0 0 90 0 0 0 0 7 0\n') == (10, 100)

    def test_malformed_is_none(self):
        assert sysinfo.parse_cpu_times('cpu x y\n') is None
        assert sysinfo.parse_cpu_times('intr 1 2\n') is None


class TestArmClock:
    def test_cpufreq_in_mhz(self, tmp_path):
        (tmp_path / 'scaling_cur_freq').write_text(CAR_CPU_CUR_KHZ + '\n')
        (tmp_path / 'cpuinfo_max_freq').write_text(CAR_CPU_MAX_KHZ + '\n')
        assert sysinfo.read_arm_clock(tmp_path) == (1344, 1728)

    def test_missing_cpufreq_is_none(self, tmp_path):
        assert sysinfo.read_arm_clock(tmp_path) == (None, None)

    def test_runs_no_command(self, fake_run, tmp_path):
        sysinfo.read_arm_clock(tmp_path)
        assert fake_run.calls == []


class TestThermal:
    def test_the_cars_zones_skip_the_unreadable_ones(self, car_thermal):
        zones = sysinfo.read_thermal_zones(car_thermal)
        assert set(zones) == {
            'cpu-thermal',
            'gpu-thermal',
            'soc0-thermal',
            'soc1-thermal',
            'soc2-thermal',
            'tj-thermal',
        }
        assert zones['cpu-thermal'] == pytest.approx(50.406)

    def test_soc_temp_is_the_junction_zone(self, car_thermal):
        assert sysinfo.read_soc_temp(car_thermal) == pytest.approx(51.562)

    def test_soc_temp_falls_back_to_the_hottest_zone(self, car_thermal):
        (car_thermal / 'thermal_zone8' / 'temp').unlink()
        assert sysinfo.read_soc_temp(car_thermal) == pytest.approx(51.656)

    def test_no_readable_zone_is_none(self, tmp_path):
        assert sysinfo.read_soc_temp(tmp_path) is None

    def test_eagain_reads_as_no_reading(self):
        # Python surfaces the cv*-thermal EAGAIN as TypeError from read_text().
        def fails():
            raise TypeError('read returned None')

        assert sysinfo._read_millidegrees(SimpleNamespace(read_text=fails)) is None


class TestPowerMode:
    def test_the_cars_nvpmodel_output(self):
        assert sysinfo.parse_power_mode(CAR_NVPMODEL) == ('25W', 1)

    def test_empty_output_is_none(self):
        assert sysinfo.parse_power_mode('') == (None, None)

    def test_read_power_mode_uses_nvpmodel(self, fake_run):
        fake_run.result = _completed(CAR_NVPMODEL)
        assert sysinfo.read_power_mode() == ('25W', 1)
        assert fake_run.calls == [['nvpmodel', '-q']]


class TestTegrastats:
    def test_the_cars_first_line(self):
        out = sysinfo.parse_tegrastats(CAR_TEGRASTATS[0])
        assert (out['ram_used_mb'], out['ram_total_mb']) == (2771, 7620)
        assert (out['swap_used_mb'], out['swap_total_mb']) == (0, 3810)
        assert len(out['cpu']) == 6 and out['cpu'][0] == (57, 1190) and out['cpu'][5] == (52, 729)
        assert out['gpu_load_pct'] == 78
        assert out['temps_c']['tj'] == pytest.approx(51.593)
        assert out['power_mw']['VDD_IN'] == (6776, 6776)

    def test_power_keeps_current_and_average(self):
        out = sysinfo.parse_tegrastats(CAR_TEGRASTATS[1])
        assert out['power_mw']['VDD_SOC'] == (1623, 1643)
        assert out['cpu'][0] == (47, 1344)

    def test_an_offline_core_reads_as_none(self):
        out = sysinfo.parse_tegrastats('RAM 1/2MB CPU [10%@1190,off,20%@729]')
        assert out['cpu'] == [(10, 1190), None, (20, 729)]

    def test_a_line_without_ram_is_none(self):
        assert sysinfo.parse_tegrastats('tegrastats: command not found') is None

    def test_read_tegrastats_without_tegrastats_is_none(self, monkeypatch):
        monkeypatch.setattr(sysinfo, 'first_line', lambda _cmd, _timeout=3.0: '')
        assert sysinfo.read_tegrastats() is None


class TestDisk:
    def test_gib_and_percent(self, monkeypatch):
        gib = 1024**3
        usage = SimpleNamespace(disk_usage=lambda _p: (100 * gib, 25 * gib, 75 * gib))
        monkeypatch.setattr(sysinfo, 'shutil', usage)
        assert sysinfo.read_disk('/') == {
            'total': 100,
            'used': 25,
            'free': 75,
            'percent_used': 25,
        }

    def test_unreadable_path_is_none(self):
        assert sysinfo.read_disk('/nonexistent/racecar/path') is None

    def test_the_cars_nvme_root(self, monkeypatch):
        monkeypatch.setattr(sysinfo, 'shutil', SimpleNamespace(disk_usage=lambda _p: CAR_DISK))
        assert sysinfo.read_disk('/') == {
            'total': 115,
            'used': 25,
            'free': 84,
            'percent_used': 22,
        }


class TestNtp:
    @pytest.mark.parametrize('stdout,expected', [('yes\n', True), ('no\n', False)])
    def test_synchronized_flag(self, fake_run, stdout, expected):
        fake_run.result = _completed(stdout)
        assert sysinfo.ntp_synchronized() is expected

    def test_timedatectl_failure_is_none(self, fake_run):
        fake_run.result = _completed('', returncode=1)
        assert sysinfo.ntp_synchronized() is None


class TestFormatUptime:
    @pytest.mark.parametrize(
        'seconds,expected',
        [(59, '0m'), (125, '2m'), (3 * 3600 + 5 * 60, '3h 5m'), (90061, '1d 1h 1m')],
    )
    def test_compact_form(self, seconds, expected):
        assert sysinfo.format_uptime(seconds) == expected

    def test_the_cars_uptime(self):
        assert sysinfo.format_uptime(CAR_UPTIME_SEC) == '16m'
