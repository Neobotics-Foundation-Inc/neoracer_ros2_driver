"""The controller's board.steering_trim_deg against a scripted board (no serial port)."""

import inspect
import math
import re
import sys
import threading
import time
from types import ModuleType, SimpleNamespace

import pytest


def _load_controller():
    """
    Import controller.py; off the car, with a stand-in serial module for this file only.

    The stand-in and the module loaded with it leave sys.modules again, so the
    other test files still skip what needs pyserial.
    """
    try:
        import serial  # noqa: F401
    except ImportError:
        pass
    else:
        return pytest.importorskip('neoracer_ros2_driver.controller')
    fake = ModuleType('serial')
    fake.SerialException = OSError  # type: ignore[attr-defined]
    sys.modules['serial'] = fake
    try:
        return pytest.importorskip('neoracer_ros2_driver.controller')
    finally:
        sys.modules.pop('serial', None)
        sys.modules.pop('neoracer_ros2_driver.controller', None)


controller = _load_controller()


class Board:
    """Answers `trim get` and `trim set` as NEORACER_V1.2 does; `mode` scripts a failure."""

    def __init__(self, trim=-1.0, mode='ok'):
        self.trim = trim
        self.mode = mode
        self.sent = []

    def command(self, line, prefixes):
        self.sent.append(line.strip())
        if self.mode == 'silent':
            return None
        if line.startswith('trim get'):
            return f'TRIM: {self.trim:.2f}deg center_pwm=1500us range=-5.0..5.0deg'
        value = float(re.match(r'trim set (\S+)', line)[1])
        if self.mode == 'stuck':
            return f'OK: steering trim={value:.2f}deg save queued'
        self.trim = max(-5.0, min(5.0, value))
        if self.mode == 'full':
            self.mode = 'ok'  # the queue drains: the restore goes through
            return f'ERROR: steering trim={self.trim:.2f}deg active, NVS save queue full'
        return f'OK: steering trim={self.trim:.2f}deg save queued'


class Log:
    def __init__(self):
        self.lines = []

    def info(self, msg, **kwargs):
        self.lines.append(msg)

    warn = error = info


class Timer:
    def __init__(self):
        self.cancels = self.resets = 0

    def cancel(self):
        self.cancels += 1

    def reset(self):
        self.resets += 1


def _node(board, known=True):
    node = controller.ControllerNode.__new__(controller.ControllerNode)
    node._board_trim = board.trim if known else math.nan
    node._board_range = (-5.0, 5.0)
    node._board_known = known
    node._board_syncing = False
    node._board_tries = 0
    node._board_timer = Timer()
    node._last_motion = -math.inf
    node._board_command = board.command
    node.shown = []
    node._show_board_trim = node.shown.append
    log = Log()
    node.get_logger = lambda: log
    return node


def test_a_valid_trim_is_written_and_read_back():
    board = Board()
    node = _node(board)
    assert node._write_board_trim(-1.5) == (True, '')
    assert board.trim == -1.5 and node._board_trim == -1.5
    assert board.sent == ['trim set -1.50', 'trim get']


@pytest.mark.parametrize('value', [math.nan, math.inf, 9.0, -5.5, 'left'])
def test_nan_and_out_of_range_values_never_reach_the_board(value):
    board = Board()
    accepted, reason = _node(board)._write_board_trim(value)
    assert not accepted and reason
    assert board.sent == [] and board.trim == -1.0


def test_an_unknown_board_trim_cannot_be_set():
    board = Board()
    accepted, reason = _node(board, known=False)._write_board_trim(-1.5)
    assert not accepted and 'unknown' in reason and board.sent == []


def test_a_write_the_board_did_not_save_is_put_back():
    board = Board(mode='full')
    node = _node(board)
    accepted, reason = node._write_board_trim(-1.5)
    assert not accepted and 'NVS save queue full' in reason
    assert board.trim == -1.0 and node._board_known
    assert board.sent == ['trim set -1.50', 'trim get', 'trim set -1.00', 'trim get']


def test_a_write_the_board_does_not_read_back_is_rejected():
    board = Board(mode='stuck')
    accepted, reason = _node(board)._write_board_trim(-1.5)
    assert not accepted and '-1.00' in reason and board.trim == -1.0


def test_a_silent_board_leaves_the_trim_unknown_and_read_again():
    board = Board(mode='silent')
    node = _node(board)
    accepted, reason = node._write_board_trim(-1.5)
    assert not accepted and 'did not answer' in reason
    assert not node._board_known and math.isnan(node._board_trim)
    assert node._board_tries == 0 and node._board_timer.resets == 1


def test_the_parameter_callback_rejects_a_bad_value_and_passes_other_parameters():
    board = Board()
    node = _node(board)
    bad = node._on_set_parameters([SimpleNamespace(name=node._BOARD_PARAM, value=9.0)])
    assert not bad.successful and 'outside the board range' in bad.reason
    other = node._on_set_parameters([SimpleNamespace(name='odometry.source', value='firmware')])
    assert other.successful and board.sent == []
    good = node._on_set_parameters([SimpleNamespace(name=node._BOARD_PARAM, value=-2.0)])
    assert good.successful and board.trim == -2.0


def test_the_read_at_start_does_not_write_the_board():
    board = Board()
    node = _node(board)
    node._board_syncing = True
    result = node._on_set_parameters([SimpleNamespace(name=node._BOARD_PARAM, value=-1.0)])
    assert result.successful and board.sent == []


# ---- Review fixes: no board command while the car drives; NaN is never sent ----


def test_no_board_command_while_the_car_drives():
    # A silent board would pause the v lines past the board's 500 ms timeout.
    board = Board()
    node = _node(board)
    node._last_motion = time.monotonic()
    accepted, reason = node._write_board_trim(-1.5)
    assert not accepted and 'the car is driving' in reason
    assert board.sent == [] and board.trim == -1.0


def test_a_write_goes_through_once_the_car_has_been_still_for_a_second():
    board = Board()
    node = _node(board)
    node._last_motion = time.monotonic() - node._BOARD_STILL_SEC - 0.1
    assert node._write_board_trim(-1.5) == (True, '')


@pytest.mark.parametrize(
    'speed,moving', [(0.0, False), (0.0005, False), (0.05, True), (-0.05, True), (math.nan, True)]
)
def test_the_motor_command_marks_the_car_moving(speed, moving):
    node = _node(Board())
    node.serial = SimpleNamespace(write=lambda data: None)
    node.serial_lock = threading.Lock()
    node.max_speed_mps, node.max_steering_angle_deg, node.steering_trim_deg = 6.0, 30.0, 0.0
    node.get_clock = lambda: SimpleNamespace(now=lambda: 0)
    msg = SimpleNamespace(drive=SimpleNamespace(speed=speed, steering_angle=0.3))
    node.motor_callback(msg)
    assert node._car_moving() is moving


def test_the_start_read_waits_while_the_car_drives():
    board = Board()
    node = _node(board, known=False)
    node._last_motion = time.monotonic()
    node._read_board_trim()
    assert board.sent == [] and node._board_tries == 0 and not node._board_known


def test_the_start_read_shows_the_boards_trim():
    board = Board()
    node = _node(board, known=False)
    node._read_board_trim()
    assert node._board_known and node._board_trim == -1.0 and node.shown == [-1.0]
    assert node._board_timer.cancels == 1


def test_a_board_that_never_answers_shows_unknown():
    board = Board(mode='silent')
    node = _node(board, known=False)
    for _ in range(node._BOARD_READ_TRIES):
        node._read_board_trim()
    assert board.sent == ['trim get'] * node._BOARD_READ_TRIES
    assert math.isnan(node.shown[-1]) and node._board_timer.cancels == 1


def test_a_board_that_holds_nan_is_reported_and_never_sent_nan():
    # The July audit: `trim set nan` turns the steering to full lock and is saved.
    board = Board(trim=math.nan)
    node = _node(board, known=False)
    node._read_board_trim()
    assert node._board_known and math.isnan(node.shown[-1])
    assert any('not a number' in line for line in node.get_logger().lines)
    board.mode = 'silent'
    accepted, _ = node._write_board_trim(0.0)
    assert not accepted
    assert board.sent == ['trim get', 'trim set 0.00', 'trim get']
    assert node._board_timer.resets == 1


def test_a_board_that_holds_nan_takes_a_finite_trim():
    board = Board(trim=math.nan)
    node = _node(board)
    assert node._write_board_trim(0.0) == (True, '')
    assert board.trim == 0.0


class Serial:
    """Answers each written line from another thread, as the serial reader would."""

    def __init__(self, node, replies):
        self.node = node
        self.replies = replies
        self.written = []

    def write(self, data):
        self.written.append(data.decode())
        lines = self.replies.pop(0) if self.replies else []
        threading.Thread(target=lambda: [self.node._board_reply(r) for r in lines]).start()


def _wired(replies):
    node = controller.ControllerNode.__new__(controller.ControllerNode)
    node._board_cmd_lock = threading.Lock()
    node._board_waiting = None
    node.serial_lock = threading.Lock()
    node.serial = Serial(node, replies)
    node._BOARD_REPLY_TIMEOUT = 0.2
    log = Log()
    node.get_logger = lambda: log
    return node


def test_the_reader_hands_the_awaited_reply_to_the_command():
    reply = 'TRIM: -1.00deg center_pwm=1483us range=-5.0..5.0deg'
    node = _wired([['OK: steering trim=-1.00deg save queued', reply]])
    assert node._board_command('trim get\n', ('TRIM:',)) == reply
    assert node.serial.written == ['trim get\n'] and node._board_waiting is None


def test_a_command_without_a_reply_returns_none_after_the_wait():
    node = _wired([[]])
    started = time.monotonic()
    assert node._board_command('trim get\n', ('TRIM:',)) is None
    assert 0.15 < time.monotonic() - started < 1.0


def test_an_error_line_nobody_waits_for_is_logged():
    node = _wired([])
    node._board_reply('ERROR: firmware update active')
    node._board_reply('TRIM: -1.00deg center_pwm=1483us range=-5.0..5.0deg')
    assert node.get_logger().lines == ['ESP32: ERROR: firmware update active']


def test_a_parameter_file_never_sets_the_shown_trim():
    source = inspect.getsource(controller.ControllerNode._setup_board_trim)
    assert 'ignore_override=True' in source
