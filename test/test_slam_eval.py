"""Unit tests for scripts/slam_eval.py (no ROS graph, no /proc of the car)."""

import math

from conftest import load_script
import pytest


@pytest.fixture(scope='module')
def ev():
    return load_script('slam_eval')


class TestLoad:
    def test_process_ticks_with_a_space_in_the_name(self, ev):
        stat = '42 (python3 x) S 1 2 3 4 5 6 7 8 9 10 111 222 0 0'
        assert ev.process_ticks(stat) == 333

    def test_find_processes(self, ev):
        cmdlines = {
            1: '/usr/bin/python3 /ws/install/lib/racecar_neo_ros2_driver/pit_node --ros-args',
            2: '/opt/ros/jazzy/lib/slam_toolbox/async_slam_toolbox_node --ros-args',
            3: '/opt/ros/jazzy/lib/nav2_amcl/amcl --ros-args',
            4: '/usr/bin/python3 ros2 launch racecar_neo_ros2_driver pit.launch.py',
        }
        assert ev.find_processes(cmdlines) == {'pit_node': [1], 'slam_toolbox': [2], 'amcl': [3]}

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
