"""Unit tests for led_matrix_node's serial writes against a fake port (no hardware)."""

import pytest
import rclpy

from neoracer_ros2_driver import led_matrix_node as lm


class FakeSerial:
    """Stand-in for serial.Serial that records every write."""

    instances = []

    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.writes = []
        self.is_open = True
        FakeSerial.instances.append(self)

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def close(self):
        self.is_open = False

    @property
    def text(self):
        """Every write decoded and joined, in order."""
        return b''.join(self.writes).decode('utf-8')


@pytest.fixture
def node(monkeypatch):
    """Build a LedMatrixNode wired to a FakeSerial, torn down after the test."""
    FakeSerial.instances.clear()
    monkeypatch.setattr(lm.serial, 'Serial', FakeSerial)
    rclpy.init()
    node = lm.LedMatrixNode()
    yield node
    if rclpy.ok():
        rclpy.shutdown()


def _msg(text):
    msg = lm.String()
    msg.data = text
    return msg


def test_idle_text_written_on_startup(node):
    assert node.serial.text == 'N\n'


def test_idle_text_restored_on_shutdown(node):
    node._on_text(_msg('NEORACER'))
    node._on_text(_msg(' '))
    serial = node.serial
    node.destroy_node()

    assert serial.text == 'N\nNEORACER\n \nN\n'
    assert serial.is_open is False


def test_incoming_text_is_newline_terminated(node):
    node._on_text(_msg('HELLO'))
    assert node.serial.writes[-1] == b'HELLO\n'


def test_existing_newline_is_not_doubled(node):
    node._on_text(_msg('HELLO\n'))
    assert node.serial.writes[-1] == b'HELLO\n'


def test_write_noops_on_closed_port(node):
    node.serial.close()
    node._write('HELLO')
    assert node.serial.text == 'N\n'


def test_serial_write_failure_is_logged_not_raised(node):
    def boom(data):
        raise lm.serial.SerialException('port went away')

    node.serial.write = boom
    node._on_text(_msg('HELLO'))


def _node_with_args(monkeypatch, args):
    """Build a node under a context started with the given ROS arguments."""
    FakeSerial.instances.clear()
    monkeypatch.setattr(lm.serial, 'Serial', FakeSerial)
    rclpy.init(args=args)
    return lm.LedMatrixNode()


def test_idle_text_is_configurable(monkeypatch):
    try:
        node = _node_with_args(monkeypatch, ['--ros-args', '-p', 'idle_text:=X'])
        node._on_text(_msg('HELLO'))
        node.destroy_node()
        assert node.idle_text == 'X'
        assert FakeSerial.instances[-1].text == 'X\nHELLO\nX\n'
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_main_routes_sigterm_through_rclpy(monkeypatch):
    # systemd stops the node with SIGTERM. Only rclpy's own handler breaks the
    # wait set, so anything else leaves the shutdown idle write unreached.
    FakeSerial.instances.clear()
    monkeypatch.setattr(lm.serial, 'Serial', FakeSerial)
    captured = {}
    real_init = rclpy.init

    def fake_init(**kwargs):
        captured.update(kwargs)
        real_init(args=kwargs.get('args'))

    monkeypatch.setattr(lm.rclpy, 'init', fake_init)
    monkeypatch.setattr(lm.rclpy, 'spin', lambda node: None)
    try:
        lm.main()
        assert captured['signal_handler_options'] is lm.SignalHandlerOptions.ALL
        assert FakeSerial.instances[-1].text == 'N\nN\n'
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_valueless_idle_text_leaves_the_panel_alone(monkeypatch):
    # rcl rejects an empty parameter in a params file, but an override that
    # parses to no value reaches declare_parameter() as None. Startup must not
    # die on it.
    try:
        node = _node_with_args(monkeypatch, ['--ros-args', '-p', 'idle_text:= '])
        node._on_text(_msg('HELLO'))
        node.destroy_node()
        assert node.idle_text is None
        assert FakeSerial.instances[-1].text == 'HELLO\n'
    finally:
        if rclpy.ok():
            rclpy.shutdown()
