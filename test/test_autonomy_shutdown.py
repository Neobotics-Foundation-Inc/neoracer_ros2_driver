"""Tests for autonomy_node.main(): the executor thread ends before the node and the context go."""

import math
import os
import signal
import threading
import time
import types

from neoracer_ros2_driver import autonomy_node as an
import pytest


class FakeExecutor:
    """As on Humble: shutdown() returns while spin() is still in its wait."""

    lag = 0.2  # spin() returns this many seconds after shutdown()
    last: 'FakeExecutor | None' = None

    def __init__(self, num_threads=None):
        self.stop_at = math.inf
        self.spinning = False
        FakeExecutor.last = self

    def add_node(self, node):
        pass

    def spin(self):
        self.spinning = True
        try:
            while time.monotonic() < self.stop_at:
                time.sleep(0.005)
        finally:
            self.spinning = False

    def shutdown(self, timeout_sec=None):
        self.stop_at = time.monotonic() + self.lag
        return True


class FakeNode:
    def __init__(self):
        self.events = []
        self.warnings = []

    def _note(self, name):
        self.events.append((name, FakeExecutor.last.spinning))

    def shutdown(self):
        self._note('shutdown')

    def destroy_node(self):
        self._note('destroy_node')

    def get_logger(self):
        return types.SimpleNamespace(warning=self.warnings.append, info=lambda m: None)


@pytest.fixture
def run_main(monkeypatch):
    nodes = []

    def make_node():
        nodes.append(FakeNode())
        return nodes[-1]

    monkeypatch.setattr(an, 'AutonomyNode', make_node)
    monkeypatch.setattr(an, 'MultiThreadedExecutor', FakeExecutor)
    monkeypatch.setattr(an, 'SignalHandlerOptions', types.SimpleNamespace(NO=0))
    monkeypatch.setattr(an.rclpy, 'init', lambda **k: None, raising=False)
    monkeypatch.setattr(
        an.rclpy, 'try_shutdown', lambda **k: nodes[-1]._note('try_shutdown'), raising=False
    )
    saved = signal.getsignal(signal.SIGINT), signal.getsignal(signal.SIGTERM)
    timers = []

    def run(lag):
        FakeExecutor.lag = lag
        timers.append(threading.Timer(0.1, os.kill, (os.getpid(), signal.SIGINT)))
        timers[-1].start()
        t0 = time.monotonic()
        an.main([])
        return nodes[-1], time.monotonic() - t0

    yield run
    # A SIGINT still pending after main() fails would stop the whole pytest run.
    for timer in timers:
        timer.cancel()
    signal.signal(signal.SIGINT, saved[0])
    signal.signal(signal.SIGTERM, saved[1])
    if FakeExecutor.last is not None:
        FakeExecutor.last.stop_at = 0.0  # let a stuck fake spin() end


def test_executor_thread_ends_before_teardown(run_main):
    node, _ = run_main(lag=0.2)
    # The final status goes out before the wait.
    assert node.events == [('shutdown', True), ('destroy_node', False), ('try_shutdown', False)]
    assert node.warnings == []


def test_a_stuck_executor_thread_does_not_hold_the_stop(run_main):
    node, took = run_main(lag=math.inf)
    assert [e for e, _ in node.events] == ['shutdown', 'destroy_node', 'try_shutdown']
    assert took < 2.0
    assert node.warnings == ['The executor thread was still running at exit.']
