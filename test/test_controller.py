"""Unit tests for the ESP32 bridge helpers in controller_lib (no ROS required)."""

from pathlib import Path

from neoracer_ros2_driver import controller_lib as cl
import yaml

CONTROLLER_YAML = Path(__file__).resolve().parent.parent / 'config' / 'controller.yaml'

# Mapping config matching the config/controller.yaml defaults, in SBUS counts.
DEFAULT_CFG = {
    'rc_min': 240,
    'rc_center': 1025,
    'rc_max': 1810,
    'rc_deadband': 0.05,
    'rc_failsafe_below': 0,
    'num_axes': 6,
    'num_buttons': 11,
    'trigger_axes': [2, 5],
    'throttle_axis': 1,
    'throttle_channel': 2,
    'throttle_sign': 1,
    'steering_axis': 3,
    'steering_channel': 0,
    'steering_sign': 1,
    'mode_channel': 4,
    'mode_mid_thresh': -0.5,
    'mode_high_thresh': 0.5,
    'mode_idle_button': -1,
    'mode_manual_button': 4,
    'mode_autonomy_button': 5,
    'aux_button_channels': [],
    'aux_button_indices': [],
    'aux_button_thresh': 0.5,
}


def _channels(**overrides):
    """Build a 10-channel RC list defaulting every channel to center (1025)."""
    ch = [1025] * 10
    for i, v in overrides.items():
        ch[int(i)] = v
    return ch


# ---------- parse_serial_data ----------


def test_parse_imu():
    data, tag = cl.parse_serial_data(
        'i -0.0072 -0.0061 0.7560 0.6545 -0.0179 -0.1542 9.8467 0.1297 0.0441 -0.0715'
    )
    assert tag == 'i'
    assert data['q_w'] == 0.6545
    assert data['a_z'] == 9.8467
    assert data['g_x'] == 0.1297


def test_parse_odom():
    data, tag = cl.parse_serial_data('o 5.4118 7.2407 0.0 0.0 0.0 0.0 1.4828')
    assert tag == 'o'
    assert data['p_x'] == 5.4118
    assert data['yaw'] == 1.4828


def test_parse_rc():
    data, tag = cl.parse_serial_data('r 1000 1500 2000 -1 1234 1 1 1 1 1')
    assert tag == 'r'
    assert data['channels'] == [1000, 1500, 2000, -1, 1234, 1, 1, 1, 1, 1]


def test_parse_mag():
    data, tag = cl.parse_serial_data('m 0.4330 -0.0040 0.0290')
    assert tag == 'm'
    assert data['mag_x'] == 0.4330


def test_parse_blank_and_unknown():
    assert cl.parse_serial_data('') == (None, None)
    assert cl.parse_serial_data('x 1 2 3') == (None, None)
    # wrong field count for a known tag is ignored, not crashed
    assert cl.parse_serial_data('i 1 2 3') == (None, None)


# ---------- normalize_channel ----------


def test_normalize_center_min_max():
    assert cl.normalize_channel(1500, 1000, 1500, 2000, 0.0) == 0.0
    assert cl.normalize_channel(2000, 1000, 1500, 2000, 0.0) == 1.0
    assert cl.normalize_channel(1000, 1000, 1500, 2000, 0.0) == -1.0


def test_normalize_failsafe_and_deadband():
    # -1 (transmitter off) and anything below failsafe -> neutral
    assert cl.normalize_channel(-1, 1000, 1500, 2000, 0.05) == 0.0
    # within deadband of center -> 0
    assert cl.normalize_channel(1510, 1000, 1500, 2000, 0.05) == 0.0
    # clamps beyond range
    assert cl.normalize_channel(2500, 1000, 1500, 2000, 0.0) == 1.0


# ---------- motor_to_command ----------


def test_motor_to_command_scaling_and_format():
    assert cl.motor_to_command(1.0, 0.0, 6.0, 30.0, 0.0) == 'v 6.000 0.00\n'
    assert cl.motor_to_command(-1.0, 1.0, 6.0, 30.0, 0.0) == 'v -6.000 30.00\n'
    # clamps out-of-range inputs and applies trim
    assert cl.motor_to_command(2.0, -2.0, 6.0, 30.0, 4.0) == 'v 6.000 -26.00\n'


# ---------- rc_to_joy ----------


def test_rc_to_joy_shape_and_trigger_rest():
    axes, buttons = cl.rc_to_joy(_channels(), DEFAULT_CFG)
    assert len(axes) == 6 and len(buttons) == 11
    # triggers rest at +1.0
    assert axes[2] == 1.0 and axes[5] == 1.0
    # centered sticks -> 0
    assert axes[1] == 0.0 and axes[3] == 0.0


def test_rc_to_joy_throttle_and_steering():
    axes, _ = cl.rc_to_joy(_channels(**{'2': 1810, '0': 240}), DEFAULT_CFG)
    assert axes[1] == 1.0  # throttle channel 2 full -> axis 1
    assert axes[3] == -1.0  # steering channel 0 min -> axis 3


def test_rc_to_joy_mode_switch_manual_autonomy_idle():
    # mode channel 4 high -> autonomy button 5
    _, buttons = cl.rc_to_joy(_channels(**{'4': 1810}), DEFAULT_CFG)
    assert buttons[5] == 1 and buttons[4] == 0
    # mid -> manual button 4
    _, buttons = cl.rc_to_joy(_channels(**{'4': 1025}), DEFAULT_CFG)
    assert buttons[4] == 1 and buttons[5] == 0
    # low -> neither
    _, buttons = cl.rc_to_joy(_channels(**{'4': 240}), DEFAULT_CFG)
    assert buttons[4] == 0 and buttons[5] == 0


def test_rc_to_joy_failsafe_is_neutral():
    # transmitter off: every channel -1 -> sticks neutral, no mode buttons
    axes, buttons = cl.rc_to_joy([-1] * 10, DEFAULT_CFG)
    assert axes[1] == 0.0 and axes[3] == 0.0
    assert buttons[4] == 0 and buttons[5] == 0
    assert axes[2] == 1.0 and axes[5] == 1.0


def test_rc_to_joy_aux_buttons():
    cfg = dict(DEFAULT_CFG, aux_button_channels=[6], aux_button_indices=[0])
    _, buttons = cl.rc_to_joy(_channels(**{'6': 1810}), cfg)
    assert buttons[0] == 1


# ---------- rc_to_channels ----------


def test_rc_to_channels_preserves_firmware_order_and_length():
    # Centered stick reads 0.0; full deflection reads +-1.0, in channel order.
    out = cl.rc_to_channels(_channels(**{'0': 1810, '9': 240}), DEFAULT_CFG)
    assert len(out) == 10
    assert out[0] == 1.0
    assert out[9] == -1.0
    assert all(v == 0.0 for v in out[1:9])


def test_rc_to_channels_failsafe_is_neutral():
    assert cl.rc_to_channels([-1] * 10, DEFAULT_CFG) == [0.0] * 10


def test_rc_to_channels_on_the_sbus_scale():
    # The car's own readings: sticks centred near 1023, full throw at the
    # firmware's 240 and 1810, a switch held low at 240. Each reads as itself.
    out = cl.rc_to_channels([1023, 240, 1810, 1025, 240, 240, 240, 1810, 1025, 1025], DEFAULT_CFG)
    assert out == [0.0, -1.0, 1.0, 0.0, -1.0, -1.0, -1.0, 1.0, 0.0, 0.0]


def test_default_cfg_matches_the_shipped_yaml():
    shipped = yaml.safe_load(CONTROLLER_YAML.read_text())['controller_node']['ros__parameters']
    for key in ('rc_min', 'rc_center', 'rc_max', 'rc_deadband', 'rc_failsafe_below'):
        assert shipped[key] == DEFAULT_CFG[key], key


def test_rc_to_channels_empty_and_none():
    assert cl.rc_to_channels(None, DEFAULT_CFG) == []
    assert cl.rc_to_channels([], DEFAULT_CFG) == []


# ---------- battery_fraction ----------


def test_battery_fraction_3s():
    assert cl.battery_fraction(12.6) == 1.0
    assert cl.battery_fraction(10.8) == 0.0
    assert abs(cl.battery_fraction(11.7) - 0.5) < 1e-9
    assert cl.battery_fraction(13.0) == 1.0
    assert cl.battery_fraction(9.0) == 0.0


# ---------- rc_link_up ----------


def test_rc_link_up_for_sbus_counts():
    assert cl.rc_link_up(_channels())
    assert cl.rc_link_up(_channels(**{'0': 240, '1': 1810}))
    assert cl.rc_link_up(_channels(**{'0': 0, '1': 2047}))  # both band edges inclusive


def test_rc_link_up_with_switches_held_low():
    # Switches at their low end read 240; a live link, as on the car.
    assert cl.rc_link_up(_channels(**{'4': 240, '5': 240, '6': 240}))


def test_rc_link_down_when_the_transmitter_is_off():
    assert not cl.rc_link_up([-1] * 10)


def test_rc_link_down_on_one_dead_channel():
    assert not cl.rc_link_up(_channels(**{'3': -1}))


def test_rc_link_down_on_an_impossible_count():
    assert not cl.rc_link_up(_channels(**{'3': 2600}))
    assert not cl.rc_link_up(_channels(**{'3': 2048}))
    assert not cl.rc_link_up(_channels(**{'3': -2}))


def test_rc_link_down_with_no_channels():
    assert not cl.rc_link_up([])
    assert not cl.rc_link_up(None)
