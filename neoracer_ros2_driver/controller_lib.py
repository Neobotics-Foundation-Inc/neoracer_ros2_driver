"""
Pure (ROS-free) helper functions for controller.py - the ESP32 serial bridge.

Kept import-light and side-effect-free so the parsing and the FlySky->Joy
mapping can be unit-tested without a running ROS graph (see test/test_controller.py).

The Seeed OSRbot ESP32 firmware emits newline-terminated ASCII lines:
- ``i qx qy qz qw ax ay az gx gy gz``   IMU   (quaternion, accel m/s^2, gyro rad/s)
- ``o px py pz vx vy vz yaw``           odom  (position m, velocity m/s, yaw rad)
- ``r c1 c2 ... c10``                   FlySky RC channels (SBUS counts; -1 == firmware failsafe)
- ``m x y z``                           magnetometer (Gauss)
- ``TRIM:`` / ``OK:`` / ``ERROR:``        replies to ``trim get`` and ``trim set``
"""

import math
import re
from typing import Any

from neoracer_ros2_driver.limits import clamp

# The ESP32 reports the receiver's channels as raw SBUS counts, 0 to 2047
# (sticks 240 to 1807, centre 1023), and -1 on every channel on receiver
# failsafe. A reading below RC_FAILSAFE_BELOW maps to neutral, so only that
# -1 counts as no signal; a real 240, a stick or switch at its end, is a value.
RC_COUNT_MIN = 0
RC_COUNT_MAX = 2047
RC_FAILSAFE_BELOW = RC_COUNT_MIN

# 3S LiPo bounds for the charge fraction in /battery/state (the vendor stack's
# battery_voltage_min/max).
BATTERY_3S_V_MIN = 10.8
BATTERY_3S_V_MAX = 12.6


def battery_fraction(volts, v_min=BATTERY_3S_V_MIN, v_max=BATTERY_3S_V_MAX):
    """Return the pack's charge fraction in [0, 1], linear between v_min and v_max."""
    return clamp((volts - v_min) / (v_max - v_min), 0.0, 1.0)


def motor_to_command(
    speed, steering_angle, max_speed_mps, max_steering_angle_deg, steering_trim_deg=0.0
):
    """
    Map a normalized ``/motor`` command to the firmware ``v <m/s> <deg>`` string.

    ``speed`` and ``steering_angle`` are clamped to ``[-1, 1]`` and scaled to
    physical units; ``steering_trim_deg`` is a constant per-vehicle offset.
    """
    speed_mps = clamp(speed) * max_speed_mps
    steer_deg = clamp(steering_angle) * max_steering_angle_deg + steering_trim_deg
    return f'v {speed_mps:.3f} {steer_deg:.2f}\n'


def parse_serial_data(line: str):
    """
    Parse one line from the ESP32 and return ``(data, tag)``.

    ``tag`` is one of ``'s'``, ``'i'``, ``'o'``, ``'r'``, ``'m'``, ``'b'`` and
    ``data`` is a dict of parsed fields. Returns ``(None, None)`` for blank/unknown
    lines so the caller can ignore them. Raises ``ValueError``/``IndexError``
    on malformed numeric fields, which the caller logs and skips.
    """
    data: dict[str, Any] = {}

    parts = line.split()
    if not parts:
        return None, None

    cmd_type = parts[0]

    if cmd_type == 's' and len(parts) == 18:
        # s px py pz vx vy vz yaw qx qy qz qw ax ay az gx gy gz
        # NEORACER_V1.1 firmware (2026-07) merges the legacy 'o' and 'i'
        # frames into one ~200 Hz state frame. Field order verified against
        # the vendor chassis parser and a live capture.
        p_x, p_y, p_z = map(float, parts[1:4])
        v_x, v_y, v_z = map(float, parts[4:7])
        yaw = float(parts[7])
        q_x, q_y, q_z, q_w = map(float, parts[8:12])
        a_x, a_y, a_z = map(float, parts[12:15])
        g_x, g_y, g_z = map(float, parts[15:18])

        data['p_x'] = p_x
        data['p_y'] = p_y
        data['p_z'] = p_z
        data['v_x'] = v_x
        data['v_y'] = v_y
        data['v_z'] = v_z
        data['yaw'] = yaw
        data['q_x'] = q_x
        data['q_y'] = q_y
        data['q_z'] = q_z
        data['q_w'] = q_w
        data['a_x'] = a_x
        data['a_y'] = a_y
        data['a_z'] = a_z
        data['g_x'] = g_x
        data['g_y'] = g_y
        data['g_z'] = g_z

        return data, 's'

    if cmd_type == 'i' and len(parts) == 11:
        # i qx qy qz qw ax ay az gx gy gz
        q_x, q_y, q_z, q_w = map(float, parts[1:5])  # quaternion
        a_x, a_y, a_z = map(float, parts[5:8])  # accelerometer
        g_x, g_y, g_z = map(float, parts[8:11])  # gyroscope

        data['q_x'] = q_x
        data['q_y'] = q_y
        data['q_z'] = q_z
        data['q_w'] = q_w
        data['a_x'] = a_x
        data['a_y'] = a_y
        data['a_z'] = a_z
        data['g_x'] = g_x
        data['g_y'] = g_y
        data['g_z'] = g_z

        return data, 'i'

    if cmd_type == 'o' and len(parts) == 8:
        # o px py pz vx vy vz yaw
        p_x, p_y, p_z = map(float, parts[1:4])
        v_x, v_y, v_z = map(float, parts[4:7])
        yaw = float(parts[7])

        data['p_x'] = p_x
        data['p_y'] = p_y
        data['p_z'] = p_z
        data['v_x'] = v_x
        data['v_y'] = v_y
        data['v_z'] = v_z
        data['yaw'] = yaw

        return data, 'o'

    if cmd_type == 'r' and len(parts) >= 2:
        # r c1 c2 ... cN   (FlySky RC channels, ints; -1 == firmware failsafe)
        data['channels'] = [int(float(v)) for v in parts[1:]]
        return data, 'r'

    if cmd_type == 'm' and len(parts) == 4:
        # m x y z   (magnetometer, Gauss)
        data['mag_x'], data['mag_y'], data['mag_z'] = map(float, parts[1:4])
        return data, 'm'

    if cmd_type == 'b' and len(parts) == 2:
        # b volts   (pack voltage, ~0.5 Hz, V1.1 firmware)
        data['voltage'] = float(parts[1])
        return data, 'b'

    return None, None


# The identity axis remap: the firmware's IMU axes are already REP-103 body
# axes (x forward, y left, z up), as measured on the dev car on 2026-11-12.
IDENTITY_AXIS_ORDER = (0, 1, 2)
IDENTITY_AXIS_SIGN = (1.0, 1.0, 1.0)


def remap_axes(vec, order, sign, scale=1.0):
    """Reorder, sign-flip and scale a 3-vector: out[i] = scale * sign[i] * vec[order[i]]."""
    return tuple(scale * float(sign[i]) * float(vec[int(order[i])]) for i in range(3))


def axis_remap_problem(order, sign):
    """
    Return why an axis remap is not a signed permutation of the axes, or ''.

    order must hold 0, 1 and 2 once each; sign must hold three values, each
    1.0 or -1.0. The controller falls back to the identity on a problem, so a
    mistyped car profile cannot scramble /imu/fused.
    """
    try:
        if len(order) != 3 or sorted(int(i) for i in order) != [0, 1, 2]:
            return f'axis order {list(order)} is not a permutation of [0, 1, 2]'
        if len(sign) != 3 or any(float(s) not in (1.0, -1.0) for s in sign):
            return f'axis sign {list(sign)} must hold three values, each 1.0 or -1.0'
    except (TypeError, ValueError):
        return f'axis order {order!r} and sign {sign!r} must be lists of numbers'
    return ''


def is_identity_remap(order, sign):
    """Report whether an axis remap leaves every axis where it is."""
    return [int(i) for i in order] == list(IDENTITY_AXIS_ORDER) and [
        float(s) for s in sign
    ] == list(IDENTITY_AXIS_SIGN)


def quaternion_from_euler(roll, pitch, yaw):
    """Convert an euler attitude (rad) to a quaternion ``[x, y, z, w]``."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = sr * cp * cy - cr * sp * sy  # x
    q[1] = cr * sp * cy + sr * cp * sy  # y
    q[2] = cr * cp * sy - sr * sp * cy  # z
    q[3] = cr * cp * cy + sr * sp * sy  # w
    return q


def normalize_channel(raw, rc_min, rc_center, rc_max, deadband, failsafe_below=RC_FAILSAFE_BELOW):
    """
    Map a raw RC reading (an SBUS count) to ``[-1.0, 1.0]`` about its center.

    A reading below ``failsafe_below`` (the ``-1`` the firmware sends while in
    failsafe) returns ``0.0``, so a failsafe never commands full
    scale. The two sides of center are scaled independently so an asymmetric
    ``rc_min``/``rc_max`` still yields a clean +-1 range. Values within
    ``deadband`` of center snap to ``0.0``.
    """
    if raw is None or raw < failsafe_below:
        return 0.0
    if raw >= rc_center:
        span = max(1.0, float(rc_max - rc_center))
    else:
        span = max(1.0, float(rc_center - rc_min))
    norm = clamp((raw - rc_center) / span)
    if abs(norm) < deadband:
        return 0.0
    return norm


def _set_button(buttons, idx, value=1):
    """Set ``buttons[idx] = value`` when ``idx`` is a valid in-range index."""
    if idx is not None and 0 <= idx < len(buttons):
        buttons[idx] = value


def rc_link_up(channels, count_min=RC_COUNT_MIN, count_max=RC_COUNT_MAX):
    """
    Report whether the transmitter link is up: every raw channel is an SBUS count.

    The firmware sends -1 on every channel while it is in failsafe.
    Use this, not the normalized channels, for transmitter presence: the
    failsafe mapping turns a dead channel and a stick held low into the same
    neutral value. One frame's verdict; freshness belongs to the consumer.
    """
    return bool(channels) and all(count_min <= c <= count_max for c in channels)


def rc_to_channels(channels, cfg):
    """
    Normalize every FlySky channel to ``[-1, 1]`` for ``/rc/channels``.

    The same calibration ``rc_to_joy`` applies, but with no axis/button mapping
    on top: channels stay in firmware order so the student library and a
    ``ros2 topic echo`` both see the transmitter as it is wired. A no-signal
    channel reads ``0.0``, matching the Joy failsafe.
    """
    return [
        normalize_channel(
            raw,
            cfg['rc_min'],
            cfg['rc_center'],
            cfg['rc_max'],
            cfg['rc_deadband'],
            cfg['rc_failsafe_below'],
        )
        for raw in (channels or [])
    ]


def rc_to_joy(channels, cfg):
    """
    Convert FlySky RC channels into ``(axes, buttons)`` for a sensor_msgs/Joy.

    The mapping is fully driven by ``cfg`` (a dict built from ROS parameters) so
    the exact FlySky channel order - which depends on the transmitter's mixer -
    can be tuned on-hardware without code changes. The produced Joy satisfies
    three consumers at once:
      * gamepad_node    -> reads ``axes[throttle_axis]`` / ``axes[steering_axis]``
      * mux_node        -> reads ``buttons[manual]`` (LB) / ``buttons[autonomy]`` (RB),
                           and treats ``trigger_axes`` as resting at +1.0
      * student library -> Xbox-style axis/button layout

    ``channels`` is the list from the ESP32 ``r`` line (``-1`` == receiver
    failsafe); a failsafe channel maps to neutral so the car stays idle.
    """
    n_axes = cfg['num_axes']
    axes = [0.0] * n_axes
    # Xbox triggers rest at +1.0; the mux excludes these from its arming check.
    for ti in cfg['trigger_axes']:
        if 0 <= ti < n_axes:
            axes[ti] = 1.0

    def chan(i):
        if channels is None or i is None or not (0 <= i < len(channels)):
            return None
        return channels[i]

    def norm(i):
        return normalize_channel(
            chan(i),
            cfg['rc_min'],
            cfg['rc_center'],
            cfg['rc_max'],
            cfg['rc_deadband'],
            cfg['rc_failsafe_below'],
        )

    # --- Driving axes ---
    ta, tc = cfg['throttle_axis'], cfg['throttle_channel']
    if 0 <= ta < n_axes and tc >= 0:
        axes[ta] = cfg['throttle_sign'] * norm(tc)
    sa, sc = cfg['steering_axis'], cfg['steering_channel']
    if 0 <= sa < n_axes and sc >= 0:
        axes[sa] = cfg['steering_sign'] * norm(sc)

    buttons = [0] * cfg['num_buttons']

    # --- Mode switch -> manual / autonomy enable ---
    # below mid => idle, between mid and high => manual (LB), above high => autonomy (RB).
    # controller.yaml sets the thresholds to the firmware's two states: a count
    # below 1500 (switch up or middle) is manual, 1500 or more (down) autonomy.
    mc = cfg['mode_channel']
    if mc >= 0 and chan(mc) is not None and chan(mc) >= cfg['rc_failsafe_below']:
        m = norm(mc)
        if m >= cfg['mode_high_thresh']:
            _set_button(buttons, cfg['mode_autonomy_button'])
        elif m >= cfg['mode_mid_thresh']:
            _set_button(buttons, cfg['mode_manual_button'])
        else:
            _set_button(buttons, cfg['mode_idle_button'])

    # --- Generic 2-position aux switches -> arbitrary Joy buttons ---
    for ch_i, btn_i in zip(cfg['aux_button_channels'], cfg['aux_button_indices'], strict=False):
        if norm(ch_i) >= cfg['aux_button_thresh']:
            _set_button(buttons, btn_i)

    return axes, buttons


# ---- Board steering trim (`trim get` / `trim set`) ----
# The firmware trim is added to every steering pulse, from the Jetson and from
# the handset. Replies, as NEORACER_V1.2 sends them:
#   trim get       -> "TRIM: -1.00deg center_pwm=1483us range=-5.0..5.0deg"
#   trim set <deg> -> "OK: steering trim=-1.50deg save queued", or "ERROR: ..."
#                     when the number does not parse or the NVS save queue is
#                     full (the trim is then active but not saved).
# The firmware clamps to its range but accepts NaN and saves it, so the value is
# checked here before it is sent.
BOARD_REPLY_PREFIXES = ('TRIM:', 'OK:', 'ERROR:')
TRIM_RANGE_DEFAULT = (-5.0, 5.0)
# The firmware prints the trim with two decimals.
TRIM_CONFIRM_TOL_DEG = 0.01
_TRIM_REPLY = re.compile(
    r'^TRIM:\s*(?P<deg>\S+?)deg\s+center_pwm=(?P<pwm>-?\d+)us\s+'
    r'range=(?P<low>-?\d+(?:\.\d+)?)\.\.(?P<high>-?\d+(?:\.\d+)?)deg'
)


def is_board_reply(line: str) -> bool:
    """Return whether a serial line answers a board command rather than streaming data."""
    return line.startswith(BOARD_REPLY_PREFIXES)


# A /motor speed within this of 0 leaves the car still.
STILL_SPEED = 1e-3


def motor_moving(speed: float) -> bool:
    """
    Return whether a /motor speed drives the car.

    NaN counts as driving: clamp() turns it into full speed.
    """
    return not abs(speed) <= STILL_SPEED


def parse_trim_reply(line: str) -> tuple[float, float, float] | None:
    """
    Return (trim_deg, low_deg, high_deg) from a `trim get` reply, or None.

    trim_deg is NaN when the board holds NaN; the range is always finite.
    """
    match = _TRIM_REPLY.match(line.strip())
    if match is None:
        return None
    try:
        deg, low, high = (float(match[k]) for k in ('deg', 'low', 'high'))
    except ValueError:
        return None
    if not all(math.isfinite(v) for v in (low, high)) or low >= high:
        return None
    return deg, low, high


def trim_set_command(trim_deg: float) -> str:
    """Return the `trim set` line for a trim in degrees (positive steers left)."""
    return f'trim set {trim_deg:.2f}\n'


def trim_problem(value: Any, low: float, high: float) -> str:
    """Return why `value` cannot be sent as the board trim, or '' when it can."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return f'the trim must be a number of degrees, not {value!r}'
    if not math.isfinite(value):
        return f'the trim must be a finite number of degrees, not {value}'
    if not low <= value <= high:
        return f'the trim {value:g} deg is outside the board range {low:g} to {high:g} deg'
    return ''


def trim_write_outcome(
    requested: float, set_reply: str | None, get_reply: str | None
) -> tuple[bool, str]:
    """
    Return (accepted, reason) for a `trim set` followed by a `trim get`.

    Accepted only when the board answered OK and reads back the requested
    trim, to the two decimals it prints.
    """
    if set_reply is None:
        return False, 'the board did not answer trim set'
    if not set_reply.startswith('OK:'):
        return False, f'the board answered: {set_reply}'
    read = parse_trim_reply(get_reply) if get_reply is not None else None
    if read is None:
        return False, 'the board did not answer trim get after trim set'
    if abs(read[0] - round(requested, 2)) > TRIM_CONFIRM_TOL_DEG:
        return False, f'the board reads {read[0]:+.2f} deg after trim set {requested:+.2f}'
    return True, ''
