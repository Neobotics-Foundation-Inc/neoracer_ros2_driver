"""
Unit tests for mux_node: mode selection, arming, RC authority, arbitration, passthrough.

mux_node imports rclpy and the message packages, so these run on the car.
"""

from pathlib import Path
from types import SimpleNamespace

from neoracer_ros2_driver import mux_node
from neoracer_ros2_driver.mux_node import (
    arming_ready,
    joy_is_centered,
    MuxMode,
    MuxNode,
    RcAuthority,
    select_command,
    select_mode,
    select_rc_mode,
)
import pytest
import yaml

ROOT = Path(__file__).resolve().parent.parent

GAMEPAD = 4
AUTO = 5


def _btns(**kwargs):
    arr = [0] * 8
    for name, val in kwargs.items():
        idx = {'gp': GAMEPAD, 'ao': AUTO}[name]
        arr[idx] = val
    return arr


class TestModeSelection:
    def test_neither_pressed_is_idle(self):
        assert select_mode(_btns(), GAMEPAD, AUTO) == MuxMode.IDLE

    def test_gamepad_only_is_gamepad(self):
        assert select_mode(_btns(gp=1), GAMEPAD, AUTO) == MuxMode.GAMEPAD

    def test_auto_only_is_autonomy(self):
        assert select_mode(_btns(ao=1), GAMEPAD, AUTO) == MuxMode.AUTONOMY

    def test_both_pressed_is_idle(self):
        assert select_mode(_btns(gp=1, ao=1), GAMEPAD, AUTO) == MuxMode.IDLE

    def test_empty_buttons_is_idle(self):
        assert select_mode([], GAMEPAD, AUTO) == MuxMode.IDLE

    def test_buttons_shorter_than_indices(self):
        assert select_mode([0, 0, 0], GAMEPAD, AUTO) == MuxMode.IDLE

    def test_custom_button_indices(self):
        buttons = [1, 0, 0, 0, 0, 0]
        assert select_mode(buttons, 0, 1) == MuxMode.GAMEPAD
        assert select_mode([0, 1, 0, 0, 0, 0], 0, 1) == MuxMode.AUTONOMY


class TestJoyCentered:
    def test_all_zero_is_centered(self):
        assert joy_is_centered([0.0, 0.0, 0.0, 0.0])

    def test_empty_axes_is_centered(self):
        assert joy_is_centered([])

    def test_below_threshold_is_centered(self):
        assert joy_is_centered([0.05, -0.1, 0.0, 0.15], threshold=0.2)

    def test_above_threshold_is_not_centered(self):
        assert not joy_is_centered([0.0, 0.5, 0.0, 0.0], threshold=0.2)

    def test_negative_above_threshold_is_not_centered(self):
        assert not joy_is_centered([0.0, -0.9, 0.0, 0.0], threshold=0.2)

    def test_xbox_trigger_rest_blocks_without_ignore(self):
        # EasySMX in Xbox-360 mode: axes[2] (LT) and axes[5] (RT) rest at +1.0,
        # not 0.0. Without ignore_axes the arming gate never opens.
        axes = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        assert not joy_is_centered(axes, threshold=0.2)

    def test_xbox_trigger_rest_ok_with_ignore(self):
        # Same axes, but axes 2 and 5 excluded; should arm.
        axes = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        assert joy_is_centered(axes, threshold=0.2, ignore_axes=(2, 5))

    def test_ignore_axes_does_not_excuse_stuck_stick(self):
        # axes 2 and 5 ignored, but axis 1 (left-stick Y) is stuck forward.
        # Must still block arming; ignoring triggers shouldn't excuse sticks.
        axes = [0.0, 0.8, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        assert not joy_is_centered(axes, threshold=0.2, ignore_axes=(2, 5))


# A frame with the mode channel centered; index 1 moves between the two so the
# authority sees live data rather than a frozen buffer.
LIVE_A = [0.0, 0.10, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
LIVE_B = [0.0, 0.12, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def _granted(auth, t0=100.0, hold=1.0):
    """Drive an RcAuthority to the granted state; return the time it happened."""
    auth.observe_channels(LIVE_A)
    auth.observe_channels(LIVE_B)
    auth.update(t0, True, t0, LIVE_B, t0)
    t = t0 + hold + 0.01
    assert auth.update(t, True, t, LIVE_B, t) is True
    return t


class TestSelectRcMode:
    def test_middle_is_idle(self):
        assert select_rc_mode([0.0] * 8, 5, 0.35) == MuxMode.IDLE

    def test_up_is_manual(self):
        ch = [0.0] * 8
        ch[5] = 1.0
        assert select_rc_mode(ch, 5, 0.35) == MuxMode.GAMEPAD

    def test_down_is_autonomous(self):
        ch = [0.0] * 8
        ch[5] = -1.0
        assert select_rc_mode(ch, 5, 0.35) == MuxMode.AUTONOMY

    def test_inside_deadband_is_idle(self):
        ch = [0.0] * 8
        ch[5] = 0.30
        assert select_rc_mode(ch, 5, 0.35) == MuxMode.IDLE

    def test_short_frame_is_idle(self):
        assert select_rc_mode([0.0, 0.0], 5, 0.35) == MuxMode.IDLE

    def test_empty_frame_is_idle(self):
        assert select_rc_mode([], 5, 0.35) == MuxMode.IDLE

    def test_channel_index_is_honored(self):
        ch = [0.0] * 8
        ch[2] = 1.0
        assert select_rc_mode(ch, 2, 0.35) == MuxMode.GAMEPAD
        assert select_rc_mode(ch, 5, 0.35) == MuxMode.IDLE


class TestRcAuthorityGrant:
    def test_not_granted_before_the_hold_elapses(self):
        a = RcAuthority(hold_sec=1.0)
        a.observe_channels(LIVE_A)
        a.observe_channels(LIVE_B)
        assert a.update(100.0, True, 100.0, LIVE_B, 100.0) is False
        assert a.update(100.5, True, 100.5, LIVE_B, 100.5) is False

    def test_granted_after_the_hold(self):
        assert _granted(RcAuthority(hold_sec=1.0)) > 0

    def test_frozen_channels_never_grant(self):
        # An uninitialized buffer holding a plausible constant passes the band
        # check; it must not pass this one.
        a = RcAuthority(hold_sec=1.0)
        t = 100.0
        for _ in range(50):
            a.observe_channels(LIVE_A)
            assert a.update(t, True, t, LIVE_A, t) is False
            t += 0.1

    def test_link_down_is_never_granted(self):
        a = RcAuthority(hold_sec=1.0)
        a.observe_channels(LIVE_A)
        a.observe_channels(LIVE_B)
        assert a.update(100.0, False, 100.0, LIVE_B, 100.0) is False
        assert a.update(200.0, False, 200.0, LIVE_B, 200.0) is False

    def test_empty_channels_are_never_granted(self):
        a = RcAuthority(hold_sec=1.0)
        assert a.update(100.0, True, 100.0, [], 100.0) is False

    def test_missing_stamps_are_never_granted(self):
        a = RcAuthority(hold_sec=1.0)
        assert a.update(100.0, True, None, LIVE_B, None) is False


class TestRcAuthorityRevoke:
    def test_one_bad_frame_revokes_immediately(self):
        a = RcAuthority(hold_sec=1.0)
        t = _granted(a)
        assert a.update(t + 0.01, False, t + 0.01, LIVE_B, t + 0.01) is False

    def test_stale_link_revokes(self):
        a = RcAuthority(timeout_sec=0.5, hold_sec=1.0)
        t = _granted(a)
        # link stamp stops advancing while channels keep arriving
        assert a.update(t + 1.0, True, t, LIVE_B, t + 1.0) is False

    def test_stale_channels_revoke(self):
        a = RcAuthority(timeout_sec=0.5, hold_sec=1.0)
        t = _granted(a)
        assert a.update(t + 1.0, True, t + 1.0, LIVE_B, t) is False

    def test_revoking_restarts_the_full_hold(self):
        a = RcAuthority(hold_sec=1.0)
        t = _granted(a)
        a.update(t + 0.01, False, t + 0.01, LIVE_B, t + 0.01)
        # link returns; authority must serve the hold again, not resume
        a.observe_channels(LIVE_A)
        t2 = t + 1.0
        assert a.update(t2, True, t2, LIVE_A, t2) is False
        assert a.update(t2 + 0.5, True, t2 + 0.5, LIVE_A, t2 + 0.5) is False


class TestRcArming:
    def test_autonomy_is_held_until_the_switch_is_seen_at_middle(self):
        # A car powered on with SWB already down must not go autonomous.
        a = RcAuthority()
        assert a.gate(MuxMode.AUTONOMY) == MuxMode.IDLE
        assert a.gate(MuxMode.GAMEPAD) == MuxMode.IDLE

    def test_middle_arms_and_then_modes_pass(self):
        a = RcAuthority()
        assert a.gate(MuxMode.IDLE) == MuxMode.IDLE
        assert a.gate(MuxMode.AUTONOMY) == MuxMode.AUTONOMY
        assert a.gate(MuxMode.GAMEPAD) == MuxMode.GAMEPAD

    def test_reset_disarms(self):
        a = RcAuthority()
        a.gate(MuxMode.IDLE)
        a.reset()
        assert a.gate(MuxMode.AUTONOMY) == MuxMode.IDLE

    def test_losing_the_link_disarms(self):
        a = RcAuthority(hold_sec=1.0)
        t = _granted(a)
        a.gate(MuxMode.IDLE)
        a.update(t + 0.01, False, t + 0.01, LIVE_B, t + 0.01)
        assert a.gate(MuxMode.AUTONOMY) == MuxMode.IDLE


class TestArmingReady:
    def test_waits_for_the_startup_grace(self):
        assert not arming_ready(0.5, 1.0, [0.0] * 8, 0.2, (2, 5))
        assert arming_ready(1.0, 1.0, [0.0] * 8, 0.2, (2, 5))

    def test_stuck_stick_blocks_after_the_grace(self):
        assert not arming_ready(10.0, 1.0, [0.0, 0.9, 0.0, 0.0], 0.2, ())

    def test_ignored_trigger_rest_does_not_block(self):
        assert arming_ready(10.0, 1.0, [0.0, 0.0, 1.0, 0.0, 0.0, 1.0], 0.2, (2, 5))


class TestSelectCommand:
    def test_gamepad_mode_forwards_a_fresh_gamepad_command(self):
        assert select_command(MuxMode.GAMEPAD, 10.0, 0.5, ('gp', 9.8), ('auto', 9.9)) == 'gp'

    def test_autonomy_mode_forwards_a_fresh_drive_command(self):
        assert select_command(MuxMode.AUTONOMY, 10.0, 0.5, ('gp', 9.8), ('auto', 9.9)) == 'auto'

    def test_idle_forwards_nothing(self):
        assert select_command(MuxMode.IDLE, 10.0, 0.5, ('gp', 9.9), ('auto', 9.9)) is None

    def test_stale_command_is_not_forwarded(self):
        assert select_command(MuxMode.GAMEPAD, 10.0, 0.5, ('gp', 9.4), None) is None

    def test_command_exactly_at_the_timeout_is_forwarded(self):
        assert select_command(MuxMode.AUTONOMY, 10.0, 0.5, None, ('auto', 9.5)) == 'auto'

    def test_never_received_is_not_forwarded(self):
        assert select_command(MuxMode.AUTONOMY, 10.0, 0.5, ('gp', 9.9), None) is None


class _Log:
    def __init__(self) -> None:
        self.lines = []

    def info(self, msg) -> None:
        self.lines.append(msg)

    warn = info


class _Mux:
    """MuxNode's decision methods over plain attributes, without rclpy."""

    _joy_mode = MuxNode._joy_mode
    _rc_has_authority = MuxNode._rc_has_authority
    _publish = MuxNode._publish

    def __init__(self, rc_enable=False) -> None:
        self._log = _Log()
        self._latest_joy = None
        self._joy_stamp = 0.0
        self._joy_timeout = 0.5
        self._joy_connected = False
        self._armed = False
        self._boot_time = 0.0
        self._startup_grace = 1.0
        self._arm_threshold = 0.2
        self._arm_ignore_axes = (2, 5)
        self._gamepad_btn = GAMEPAD
        self._auto_btn = AUTO
        self._rc_enable = rc_enable
        self._rc = RcAuthority(timeout_sec=0.5, hold_sec=1.0)
        self._rc_link = False
        self._rc_link_stamp = None
        self._rc_channels = []
        self._rc_channels_stamp = None
        self._rc_held = False
        self._rc_channel = 6
        self._rc_deadband = 0.35
        self._cmd_timeout = 0.5
        self._latest_gamepad = None
        self._gamepad_stamp = 0.0
        self._latest_auto = None
        self._auto_stamp = 0.0
        self._last_mode = MuxMode.IDLE
        self.passthrough = True
        self.sent = []
        self._pub = SimpleNamespace(publish=self.sent.append)

    def get_parameter(self, name):
        assert name == 'autonomy_passthrough'
        return SimpleNamespace(value=self.passthrough)

    def get_logger(self):
        return self._log

    def joy(self, now, axes=(0.0,) * 8, **buttons):
        self._latest_joy = SimpleNamespace(axes=list(axes), buttons=_btns(**buttons))
        self._joy_stamp = now
        self._joy_connected = True


class TestJoyMode:
    def test_no_controller_publishes_zero(self):
        assert _Mux()._joy_mode(5.0) is None

    def test_arms_then_follows_the_bumpers(self):
        mux = _Mux()
        mux.joy(5.0, gp=1)
        assert mux._joy_mode(5.0) == MuxMode.GAMEPAD
        assert mux._armed
        assert 'Mux armed' in mux._log.lines

    def test_inside_the_startup_grace_publishes_zero(self):
        mux = _Mux()
        mux.joy(0.5, gp=1)
        assert mux._joy_mode(0.5) is None
        assert not mux._armed

    def test_stuck_stick_at_power_on_never_arms(self):
        mux = _Mux()
        mux.joy(5.0, axes=[0.0, 0.9, 0.0, 0.0], ao=1)
        assert mux._joy_mode(5.0) is None

    def test_arming_is_latched(self):
        mux = _Mux()
        mux.joy(5.0)
        mux._joy_mode(5.0)
        mux.joy(6.0, axes=[0.0, 0.9, 0.0, 0.0], gp=1)
        assert mux._joy_mode(6.0) == MuxMode.GAMEPAD

    def test_stale_joy_disconnects(self):
        mux = _Mux()
        mux.joy(5.0, gp=1)
        mux._joy_mode(5.0)
        assert mux._joy_mode(5.6) is None
        assert not mux._joy_connected
        assert any('disconnected' in line for line in mux._log.lines)


LIVE_C = [0.0, 0.14, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class TestRcHasAuthority:
    def test_disabled_never_holds(self):
        mux = _Mux(rc_enable=False)
        mux._rc_link, mux._rc_link_stamp = True, 10.0
        mux._rc_channels, mux._rc_channels_stamp = LIVE_B, 10.0
        assert mux._rc_has_authority(10.0) is False

    def test_grant_and_release_are_logged_once(self):
        mux = _Mux(rc_enable=True)
        mux._rc.observe_channels(LIVE_A)
        mux._rc.observe_channels(LIVE_C)
        mux._rc_link, mux._rc_link_stamp = True, 10.0
        mux._rc_channels, mux._rc_channels_stamp = LIVE_C, 10.0
        assert mux._rc_has_authority(10.0) is False
        mux._rc_link_stamp = mux._rc_channels_stamp = 11.1
        assert mux._rc_has_authority(11.1) is True
        assert mux._rc_has_authority(11.1) is True
        mux._rc_link = False
        assert mux._rc_has_authority(11.2) is False
        assert mux._log.lines == [
            'RC transmitter holds the drive gate',
            'RC transmitter released the drive gate',
        ]


class TestPassthrough:
    """autonomy_passthrough, the NeoRacer default: forward fresh /drive, else zero."""

    @staticmethod
    def _at(monkeypatch, now):
        monkeypatch.setattr(mux_node.time, 'monotonic', lambda: now)
        monkeypatch.setattr(mux_node, 'AckermannDriveStamped', lambda: 'zero')

    def test_fresh_drive_is_forwarded_with_no_controller(self, monkeypatch):
        mux = _Mux()
        mux._latest_auto, mux._auto_stamp = 'drive', 9.8
        self._at(monkeypatch, 10.0)
        mux._publish()
        assert mux.sent == ['drive']

    def test_stale_drive_publishes_zero(self, monkeypatch):
        mux = _Mux()
        mux._latest_auto, mux._auto_stamp = 'drive', 9.4
        self._at(monkeypatch, 10.0)
        mux._publish()
        assert mux.sent == ['zero']

    def test_the_bumpers_and_the_gamepad_are_ignored(self, monkeypatch):
        mux = _Mux()
        mux.joy(10.0, gp=1)
        mux._latest_gamepad, mux._gamepad_stamp = 'gamepad', 9.9
        self._at(monkeypatch, 10.0)
        mux._publish()
        assert mux.sent == ['zero']
        mux._latest_auto, mux._auto_stamp = 'drive', 9.9
        mux._publish()
        assert mux.sent == ['zero', 'drive']

    def test_off_falls_back_to_the_bumper_gate(self, monkeypatch):
        mux = _Mux()
        mux.passthrough = False
        mux._latest_auto, mux._auto_stamp = 'drive', 9.9
        self._at(monkeypatch, 10.0)
        mux._publish()
        assert mux.sent == ['zero']  # no controller, so nothing is armed


def _declared_defaults():
    """Return {name: default} for every declare_parameter call in mux_node.py."""
    import ast
    import re

    text = (ROOT / 'neoracer_ros2_driver' / 'mux_node.py').read_text()
    pairs = re.findall(r"declare_parameter\('(\w+)', ([^)]+?)\)", text)
    return {name: ast.literal_eval(value) for name, value in pairs}


def _shipped():
    with open(ROOT / 'config' / 'mux.yaml') as f:
        return yaml.safe_load(f)['mux_node']['ros__parameters']


class TestShippedConfig:
    def test_code_defaults_match_the_yaml(self):
        declared = _declared_defaults()
        for name, value in _shipped().items():
            assert declared[name] == value, name

    def test_passthrough_is_on_by_default(self):
        assert _declared_defaults()['autonomy_passthrough'] is True

    @pytest.mark.parametrize('source', ['code', 'yaml'])
    def test_rc_authority_is_off_on_the_mode_switch_channel(self, source):
        params = _declared_defaults() if source == 'code' else _shipped()
        assert params['rc_authority_enable'] is False
        assert params['rc_mode_channel'] == 6  # radio Ch7, the mode switch

    def test_there_is_no_override(self):
        text = (ROOT / 'neoracer_ros2_driver' / 'mux_node.py').read_text()
        assert not hasattr(mux_node, 'SafetyOverride')
        assert 'override_timeout_sec' not in text and "'/mux/override" not in text
        assert 'override_timeout_sec' not in _shipped()


def _held(mux, now, mode_value):
    """Give the mux a transmitter that holds the gate, mode channel at mode_value."""
    channels = [0.0] * 10
    channels[6] = mode_value
    mux._rc.observe_channels([0.0] * 10)
    mux._rc.observe_channels([0.1] + [0.0] * 9)
    mux._rc_link, mux._rc_link_stamp = True, now - 1.5
    mux._rc_channels, mux._rc_channels_stamp = channels, now - 1.5
    mux._rc_has_authority(now - 1.5)
    mux._rc_link_stamp = mux._rc_channels_stamp = now


class TestRcGate:
    def test_passthrough_ignores_a_held_transmitter(self, monkeypatch):
        mux = _Mux(rc_enable=True)
        _held(mux, 10.0, 0.9)
        assert mux._rc_has_authority(10.0) is True
        mux._rc_has_authority = lambda now: pytest.fail('passthrough consulted the transmitter')
        mux._latest_gamepad, mux._gamepad_stamp = 'gamepad', 9.9
        mux._latest_auto, mux._auto_stamp = 'drive', 9.9
        TestPassthrough._at(monkeypatch, 10.0)
        mux._publish()
        assert mux.sent == ['drive']

    def test_with_passthrough_off_the_mode_channel_picks_the_source(self, monkeypatch):
        mux = _Mux(rc_enable=True)
        mux.passthrough = False
        _held(mux, 10.0, 0.0)
        mux._latest_gamepad, mux._gamepad_stamp = 'gamepad', 9.9
        mux._latest_auto, mux._auto_stamp = 'drive', 9.9
        TestPassthrough._at(monkeypatch, 10.0)
        mux._publish()  # middle: arms the gate, publishes zero
        mux._rc_channels[6] = 0.9
        mux._publish()
        mux._rc_channels[6] = -0.9
        mux._publish()
        assert mux.sent == ['zero', 'gamepad', 'drive']
