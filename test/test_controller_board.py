"""The controller's board.steering_trim_deg against a scripted board (no serial port)."""

import math
import re
import sys
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

    def info(self, msg):
        self.lines.append(msg)

    warn = error = info


def _node(board, known=True):
    node = controller.ControllerNode.__new__(controller.ControllerNode)
    node._board_trim = board.trim if known else math.nan
    node._board_range = (-5.0, 5.0)
    node._board_known = known
    node._board_syncing = False
    node._board_command = board.command
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


def test_a_write_the_board_does_not_read_back_is_refused():
    board = Board(mode='stuck')
    accepted, reason = _node(board)._write_board_trim(-1.5)
    assert not accepted and '-1.00' in reason and board.trim == -1.0


def test_a_silent_board_leaves_the_trim_unknown():
    board = Board(mode='silent')
    node = _node(board)
    accepted, reason = node._write_board_trim(-1.5)
    assert not accepted and 'did not answer' in reason
    assert not node._board_known


def test_the_parameter_callback_refuses_a_bad_value_and_passes_other_parameters():
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
