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
# The INA3221 power monitor on the dev car, 2026-11-12: labels, bus voltages
# (mV) and currents (mA). in4 to in6 are the shunt voltages; in7 is their sum.
CAR_INA3221_LABELS = {1: 'VDD_IN', 2: 'VDD_CPU_GPU_CV', 3: 'VDD_SOC', 7: 'sum of shunt voltages'}
CAR_INA3221_IN = {1: 5072, 2: 5064, 3: 5072, 4: 6600, 5: 1840, 6: 1640, 7: 12520}
CAR_INA3221_CURR = {1: 1320, 2: 376, 3: 328, 4: 2240}
# The fan on the dev car, 2026-11-12: pwmfan pwm1 and pwm_tach rpm.
CAR_FAN_PWM = '41'
CAR_FAN_RPM = '1641'
# The devfreq entries on the dev car, 2026-11-12, with their cur_freq in Hz.
CAR_DEVFREQ = {
    '15340000.vic': 435200000,
    '15380000.nvjpg': 499200000,
    '15480000.nvdec': 524800000,
    '15540000.nvjpg': 499200000,
    '15a50000.ofa': 537600000,
    '17000000.gpu': 306000000,
}
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
    """Build a /sys/class/thermal tree with the car's zones; failing zones have no temp file."""
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
    # On the dev car on 2026-11-12 all three counters read 0 (test_clean and
    # test_read_throttled_on_the_cars_counters). The non-zero counts are made
    # up, in the format `grep . /sys/class/hwmon/hwmon4/oc*_event_cnt` prints.
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

    def test_read_throttled_on_the_cars_counters(self, tmp_path):
        root = _hwmon_tree(tmp_path, {1: 0, 2: 0, 3: 0})
        assert sysinfo.read_throttled(root) == (0, [])

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


def _power_tree(root):
    """Build a hwmon tree holding the car's INA3221 files."""
    h = root / 'hwmon2'
    h.mkdir(parents=True)
    (h / 'name').write_text('ina3221\n')
    for n, label in CAR_INA3221_LABELS.items():
        (h / f'in{n}_label').write_text(label + '\n')
    for n, mv in CAR_INA3221_IN.items():
        (h / f'in{n}_input').write_text(f'{mv}\n')
    for n, ma in CAR_INA3221_CURR.items():
        (h / f'curr{n}_input').write_text(f'{ma}\n')
    return root


class TestPowerRails:
    def test_the_cars_rails(self, tmp_path):
        rails = sysinfo.read_power_rails(_power_tree(tmp_path))
        assert rails == {
            'VDD_CPU_GPU_CV': {'mv': 5064, 'ma': 376, 'mw': 1904},
            'VDD_IN': {'mv': 5072, 'ma': 1320, 'mw': 6695},
            'VDD_SOC': {'mv': 5072, 'ma': 328, 'mw': 1664},
        }

    def test_the_sum_of_shunt_voltages_is_left_out(self, tmp_path):
        assert 'sum of shunt voltages' not in sysinfo.read_power_rails(_power_tree(tmp_path))

    def test_no_power_monitor_is_empty(self, tmp_path):
        assert sysinfo.read_power_rails(_hwmon_tree(tmp_path)) == {}


class TestFan:
    def _fan_tree(self, root, pwm=CAR_FAN_PWM, rpm=CAR_FAN_RPM):
        for i, (name, file, value) in enumerate(
            [('pwmfan', 'pwm1', pwm), ('pwm_tach', 'rpm', rpm)], start=1
        ):
            h = root / f'hwmon{i}'
            h.mkdir()
            (h / 'name').write_text(name + '\n')
            if value is not None:
                (h / file).write_text(value + '\n')
        return root

    def test_the_cars_fan(self, tmp_path):
        fan = sysinfo.read_fan(self._fan_tree(tmp_path))
        assert fan == {'pwm': 41, 'percent': 16, 'rpm': 1641}

    def test_a_missing_tachometer_reading_is_none(self, tmp_path):
        fan = sysinfo.read_fan(self._fan_tree(tmp_path, rpm=None))
        assert fan == {'pwm': 41, 'percent': 16, 'rpm': None}

    def test_no_fan_is_none(self, tmp_path):
        (tmp_path / 'hwmon0').mkdir()
        (tmp_path / 'hwmon0' / 'name').write_text('ina3221\n')
        assert sysinfo.read_fan(tmp_path) is None


class TestGpu:
    # The car's GPU max_freq and load were not read; 1020000000 and 412 are
    # made up, in the units the files use (Hz, tenths of a percent).
    def _devfreq_tree(self, root, load='412'):
        for name, hz in CAR_DEVFREQ.items():
            d = root / name
            d.mkdir()
            (d / 'cur_freq').write_text(f'{hz}\n')
            if name.endswith('.gpu'):
                (d / 'max_freq').write_text('1020000000\n')
                (d / 'device').mkdir()
                if load is not None:
                    (d / 'device' / 'load').write_text(load + '\n')
        return root

    def test_finds_the_gpu_among_the_cars_engines(self, tmp_path):
        gpu = sysinfo.read_gpu(self._devfreq_tree(tmp_path))
        assert gpu == {'load_pct': 41.2, 'cur_mhz': 306.0, 'max_mhz': 1020.0}

    def test_a_missing_load_file_is_none(self, tmp_path):
        assert sysinfo.read_gpu(self._devfreq_tree(tmp_path, load=None))['load_pct'] is None

    def test_no_gpu_is_none(self, tmp_path):
        assert sysinfo.read_gpu(tmp_path) is None


class TestCpuLoad:
    def test_busy_share_between_samples(self):
        assert sysinfo.cpu_load_pct((100, 1000), (150, 1100)) == 50.0

    def test_first_sample_or_no_change_is_none(self):
        assert sysinfo.cpu_load_pct(None, (150, 1100)) is None
        assert sysinfo.cpu_load_pct((150, 1100), (150, 1100)) is None


class TestHostPoller:
    @pytest.fixture
    def fake_host(self, monkeypatch):
        calls = {'power_mode': 0, 'ntp': 0}
        cpu = iter([(100, 1000), (150, 1100), (175, 1200)])

        def power_mode():
            calls['power_mode'] += 1
            return ('25W', 1)

        def ntp():
            calls['ntp'] += 1
            return True

        monkeypatch.setattr(sysinfo, 'read_power_mode', power_mode)
        monkeypatch.setattr(sysinfo, 'ntp_synchronized', ntp)
        monkeypatch.setattr(sysinfo, 'read_cpu_times', lambda: next(cpu))
        monkeypatch.setattr(sysinfo, 'read_arm_clock', lambda: (1344, 1728))
        monkeypatch.setattr(sysinfo, 'read_throttled', lambda: (0, []))
        monkeypatch.setattr(sysinfo, 'read_uptime', lambda: 991.77)
        monkeypatch.setattr(sysinfo, 'read_soc_temp', lambda: 51.187)
        monkeypatch.setattr(sysinfo, 'read_thermal_zones', lambda: {'tj-thermal': 51.187})
        monkeypatch.setattr(
            sysinfo, 'read_memory', lambda: {'total': 7620, 'available': 4727, 'used': 2893}
        )
        monkeypatch.setattr(
            sysinfo,
            'read_disk',
            lambda: {'total': 115, 'used': 25, 'free': 84, 'percent_used': 22},
        )
        monkeypatch.setattr(
            sysinfo, 'read_gpu', lambda: {'load_pct': 0.0, 'cur_mhz': 306.0, 'max_mhz': None}
        )
        monkeypatch.setattr(
            sysinfo, 'read_power_rails', lambda: {'VDD_IN': {'mv': 5072, 'ma': 1320, 'mw': 6695}}
        )
        monkeypatch.setattr(sysinfo, 'read_fan', lambda: {'pwm': 41, 'percent': 16, 'rpm': 1641})
        return calls

    def test_the_snapshot_holds_every_reading(self, fake_host):
        snap = sysinfo.HostPoller(clock=lambda: 0.0).poll()
        assert set(snap) == {
            'soc_temp_c',
            'temps_c',
            'cpu',
            'memory_mib',
            'disk_gib',
            'gpu',
            'power_rails',
            'fan',
            'throttle',
            'uptime_s',
            'uptime',
            'power_mode',
            'ntp_synchronized',
        }
        assert snap['cpu'] == {'load_pct': None, 'cur_mhz': 1344, 'max_mhz': 1728}
        assert snap['power_mode'] == {'name': '25W', 'id': 1}
        assert snap['throttle'] == {'flags': 0, 'active': []}
        assert snap['uptime'] == '16m'

    def test_cpu_load_comes_from_the_previous_poll(self, fake_host):
        poller = sysinfo.HostPoller(clock=lambda: 0.0)
        poller.poll()
        assert poller.poll()['cpu']['load_pct'] == 50.0
        assert poller.poll()['cpu']['load_pct'] == 25.0

    def test_slow_facts_are_read_once_a_minute(self, fake_host):
        now = [0.0]
        poller = sysinfo.HostPoller(clock=lambda: now[0])
        poller.poll()
        now[0] = 30.0
        poller.poll()
        assert fake_host == {'power_mode': 1, 'ntp': 1}
        now[0] = 61.0
        poller.poll()
        assert fake_host == {'power_mode': 2, 'ntp': 2}

    def test_the_snapshot_is_json(self, fake_host):
        import json

        json.dumps(sysinfo.HostPoller(clock=lambda: 0.0).poll())
