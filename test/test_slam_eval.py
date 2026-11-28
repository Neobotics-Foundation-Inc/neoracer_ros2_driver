"""Unit tests for scripts/slam_eval.py (no ROS graph, no /proc of the car)."""

import math
from pathlib import Path
import re

from conftest import load_script
import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def ev():
    return load_script('slam_eval')


class TestLoad:
    def test_process_ticks_with_a_space_in_the_name(self, ev):
        stat = '42 (python3 x) S 1 2 3 4 5 6 7 8 9 10 111 222 0 0'
        assert ev.process_ticks(stat) == 333

    def test_find_processes(self, ev):
        cmdlines = {
            1: '/usr/bin/python3 /ws/install/neoracer_ros2_driver/lib/neoracer_ros2_driver/'
            'controller --ros-args',
            2: '/opt/ros/humble/lib/slam_toolbox/async_slam_toolbox_node --ros-args',
            3: '/opt/ros/humble/lib/nav2_amcl/amcl --ros-args',
            4: '/usr/bin/python3 /opt/ros/humble/bin/ros2 launch neoracer_ros2_driver '
            'controller.launch.py',
        }
        assert ev.find_processes(cmdlines) == {'controller': [1], 'slam_toolbox': [2], 'amcl': [3]}

    def test_an_exited_process_counts_nothing(self, ev):
        assert ev.tick_delta({1: 100, 2: 500}, {1: 130}) == 30
        assert ev.tick_delta({1: 100}, {1: 130, 3: 9000}) == 30

    def test_summary_flags_a_throttled_run(self, ev):
        samples = [
            ev.Sample(0.0, 50.0, 70.0, 0, 2400, {'slam_toolbox': 30.0}),
            ev.Sample(1.0, 60.0, 72.0, 0x50005, 1000, {'slam_toolbox': 40.0}),
        ]
        text = '\n'.join(ev.summarize(samples))
        assert 'slam_toolbox' in text and '35.0' in text and '40.0' in text
        assert '0x50005' in text and 'Discard' in text and 'min 1000 MHz' in text

    def test_summary_of_a_clean_run(self, ev):
        text = '\n'.join(ev.summarize([ev.Sample(0.0, 50.0, 70.0, 0, 2400, {})]))
        assert '0x0 for the whole run' in text


class TestMarks:
    def test_mean_yaw_wraps(self, ev):
        x, y, yaw = ev.mean_pose([(0, 0, math.radians(179)), (2, 2, math.radians(-179))])
        assert (x, y) == (1, 1) and abs(abs(math.degrees(yaw)) - 180) < 1e-6

    def test_report(self, ev):
        marks = [
            ev.Mark('A', 'amcl', 0.0, 0.0, 0.0),
            ev.Mark('A', 'amcl', 0.02, 0.0, 0.0),
            ev.Mark('B', 'amcl', 3.01, 0.0, 0.0),
        ]
        lines = ev.report_lines(marks, [('A', 'B', 3.0), ('A', 'C', 1.0)])
        assert lines[0] == '== amcl'
        a = next(line for line in lines if line.startswith('A '))
        assert a.split()[1:3] == ['2', '10']  # 2 visits, 10 mm from their mean
        pair = next(line for line in lines if line.startswith('A-B'))
        assert pair.split()[-1] == '+0'  # mean A at 0.01, B at 3.01
        assert not any(line.startswith('A-C') for line in lines)

    def test_taped_file(self, ev, tmp_path):
        f = tmp_path / 'taped.txt'
        f.write_text('# pairs\nA B 2.50\nB C 1.25\n')
        assert ev.read_taped(f) == [('A', 'B', 2.5), ('B', 'C', 1.25)]
        assert ev.read_point(f) == (0.0, 0.0)
        f.write_text('point 0 0.1045\nA B 2.50\n')
        assert ev.read_point(f) == (0.0, 0.1045)
        assert ev.read_taped(f) == [('A', 'B', 2.5)]

    def test_distances_from_a_point_on_the_car(self, ev):
        # Base links 2 m apart along x, the car facing +y at A and -y at B: the
        # left-side points (0, 0.1) sit at x -0.1 and x 2.1, 2.2 m apart.
        marks = [
            ev.Mark('A', 't', 0.0, 0.0, math.pi / 2),
            ev.Mark('B', 't', 2.0, 0.0, -math.pi / 2),
        ]
        lines = ev.report_lines(marks, [('A', 'B', 2.2)], (0.0, 0.1))
        pair = next(line for line in lines if line.startswith('A-B'))
        assert pair.split()[2] == '2.200' and pair.split()[-1] == '+0'


def test_cell_counts(ev):
    assert ev.cell_counts([0, 254, 205, 0, 254, 254]) == (2, 3, 1)


class TestJetsonLoad:
    """The NeoRacer's load readings: sysinfo.py's Jetson readers, swap, and throttling."""

    def test_processes_are_the_neoracer_executables(self, ev):
        setup = (ROOT / 'setup.py').read_text()
        scripts = set(re.findall(r"'(\w+) = neoracer_ros2_driver\.", setup))
        for label, needle in ev.PROCESSES.items():
            if needle.startswith('lib/neoracer_ros2_driver/'):
                assert needle.removeprefix('lib/neoracer_ros2_driver/') in scripts, label
        lidar = (ROOT / 'launch' / 'lidar.launch.py').read_text()
        assert f"executable='{ev.PROCESSES['lidar']}'" in lidar

    def test_swap_in_use_rounds_up(self, ev):
        assert ev.parse_swap_used('SwapTotal: 3901440 kB\nSwapFree: 3901440 kB\n') == 0
        assert ev.parse_swap_used('SwapTotal: 3901440 kB\nSwapFree: 3900940 kB\n') == 1
        assert ev.parse_swap_used('SwapTotal: 0 kB\nSwapFree: 0 kB\n') == 0
        assert ev.parse_swap_used('MemTotal: 7803236 kB\n') is None

    def test_over_current_counters(self, ev, tmp_path):
        oc = tmp_path / 'hwmon4'
        oc.mkdir()
        (oc / 'name').write_text('soctherm_oc\n')
        for k, n in ((1, 0), (2, 5), (3, 0)):
            (oc / f'oc{k}_event_cnt').write_text(f'{n}\n')
        assert ev.read_oc_counts(tmp_path) == {1: 0, 2: 5, 3: 0}
        assert ev.read_oc_counts(tmp_path / 'none') is None

    def test_only_alarms_raised_during_the_run_count(self, ev):
        start = {1: 0, 2: 5, 3: 0}
        assert ev.throttle_flags(start, start) == 0
        assert ev.throttle_flags(start, {1: 2, 2: 6, 3: 0}) == 0b011
        assert ev.throttle_flags(None, start) is None and ev.throttle_flags(start, None) is None

    def test_recorder_reads_the_jetson(self, ev, tmp_path, monkeypatch):
        meminfo = tmp_path / 'meminfo'
        meminfo.write_text(
            'MemTotal:  7803236 kB\nMemAvailable:  4840448 kB\n'
            'SwapTotal:  3901440 kB\nSwapFree:  3891200 kB\n'
        )
        counts = iter([{1: 0, 2: 5}, {1: 1, 2: 5}])
        monkeypatch.setattr(ev, 'MEMINFO', meminfo)
        monkeypatch.setattr(ev, 'read_cmdlines', dict)
        monkeypatch.setattr(ev, 'read_oc_counts', lambda: next(counts))
        monkeypatch.setattr(ev.sysinfo, 'read_cpu_times', lambda: (100, 1000))
        monkeypatch.setattr(ev.sysinfo, 'read_soc_temp', lambda: 51.2)
        monkeypatch.setattr(ev.sysinfo, 'read_arm_clock', lambda: (1344, 1728))
        monkeypatch.setattr(ev.sysinfo, 'read_gpu', lambda: {'load_pct': 23.0})
        rails = {'VDD_IN': {'mv': 5072, 'ma': 1320, 'mw': 6695}}
        monkeypatch.setattr(ev.sysinfo, 'read_power_rails', lambda: rails)
        s = ev.LoadRecorder().sample()
        assert (s.temp_c, s.arm_mhz, s.gpu_pct, s.power_mw) == (51.2, 1344, 23.0, 6695)
        assert (s.mem_used_mib, s.mem_avail_mib, s.swap_used_mib) == (2893, 4727, 10)
        assert s.throttled == 0b001

    def test_summary_flags_swap_and_gives_the_jetson_readings(self, ev):
        jetson = {'gpu_pct': 20.0, 'power_mw': 6695, 'mem_used_mib': 2900, 'mem_avail_mib': 4700}
        samples = [
            ev.Sample(0.0, 50.0, 51.0, 0, 1344, {}, **jetson, swap_used_mib=0),
            ev.Sample(1.0, 60.0, 52.0, 0, 1344, {}, **jetson, swap_used_mib=12),
        ]
        text = '\n'.join(ev.summarize(samples))
        assert 'GPU load' in text and 'Board power           6.7     6.7 W' in text
        assert 'Memory used' in text and 'Memory available   min 4700 MiB' in text
        assert 'Swap in use: 0 MiB at the start, up to 12 MiB. Discard this run.' in text
        assert 'Throttled: 0x0 for the whole run.' in text

    def test_summary_of_a_clean_jetson_run(self, ev):
        text = '\n'.join(ev.summarize([ev.Sample(0.0, 50.0, 51.0, 0, 1344, {}, swap_used_mib=0)]))
        assert 'Swap in use: none for the whole run.' in text and 'Discard' not in text

    def test_swap_from_before_the_run_does_not_count(self, ev):
        samples = [ev.Sample(float(t), 50.0, 51.0, 0, 1344, {}, swap_used_mib=40) for t in (0, 1)]
        text = '\n'.join(ev.summarize(samples))
        assert 'Swap in use: 40 MiB from before the run, no more during it.' in text
        assert 'Discard' not in text

    def test_summary_without_counters_says_so(self, ev):
        text = '\n'.join(ev.summarize([ev.Sample(0.0, 50.0, None, None, None, {})]))
        assert 'Throttled: not read' in text and 'Swap in use: not read.' in text
