"""
Pure (ROS-free) helper functions for controller.py - the ESP32 serial bridge.

Kept import-light and side-effect-free so the parsing and the FlySky->Joy
mapping can be unit-tested without a running ROS graph (see test/test_controller.py).

The Seeed OSRbot ESP32 firmware emits newline-terminated ASCII lines:
- ``i qx qy qz qw ax ay az gx gy gz``   IMU   (quaternion, accel m/s^2, gyro rad/s)
- ``o px py pz vx vy vz yaw``           odom  (position m, velocity m/s, yaw rad)
- ``r c1 c2 ... c10``                   FlySky RC channels (ints; -1 == no signal)
- ``m x y z``                           magnetometer (Gauss)
"""

import math

# A raw RC channel reading below this is treated as "no signal" (failsafe).
# The firmware reports -1 on every channel when the FlySky transmitter is off,
# so anything below a valid pulse width (~1000us) maps to neutral, never full-scale.
RC_FAILSAFE_BELOW = 500


def clamp(value, low=-1.0, high=1.0):
    """Clamp ``value`` to the inclusive ``[low, high]`` range."""
    return max(low, min(high, value))


def motor_to_command(speed, steering_angle, max_speed_mps,
                     max_steering_angle_deg, steering_trim_deg=0.0):
    """
    Map a normalized ``/motor`` command to the firmware ``v <m/s> <deg>`` string.

    ``speed`` and ``steering_angle`` are clamped to ``[-1, 1]`` and scaled to
    physical units; ``steering_trim_deg`` is a constant per-vehicle offset.
    """
    speed_mps = clamp(speed) * max_speed_mps
    steer_deg = clamp(steering_angle) * max_steering_angle_deg + steering_trim_deg
    return f"v {speed_mps:.3f} {steer_deg:.2f}\n"


def parse_serial_data(line: str):
    """
    Parse one line from the ESP32 and return ``(data, tag)``.

    ``tag`` is one of ``'s'``, ``'i'``, ``'o'``, ``'r'``, ``'m'``, ``'b'`` and
    ``data`` is a dict of parsed fields. Returns ``(None, None)`` for blank/unknown
    lines so the caller can ignore them. Raises ``ValueError``/``IndexError``
    on malformed numeric fields, which the caller logs and skips.
    """
    data = {}

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
        a_x, a_y, a_z = map(float, parts[5:8])       # accelerometer
        g_x, g_y, g_z = map(float, parts[8:11])      # gyroscope

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
        # r c1 c2 ... cN   (FlySky RC channels, ints; -1 == no signal)
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


def normalize_channel(raw, rc_min, rc_center, rc_max, deadband,
                      failsafe_below=RC_FAILSAFE_BELOW):
    """
    Map a raw RC pulse width to ``[-1.0, 1.0]`` about its center.

    A reading below ``failsafe_below`` (e.g. the firmware's ``-1`` no-signal
    sentinel) returns ``0.0`` so a powered-off transmitter never commands full
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
    norm = max(-1.0, min(1.0, (raw - rc_center) / span))
    if abs(norm) < deadband:
        return 0.0
    return norm


def _set_button(buttons, idx, value=1):
    """Set ``buttons[idx] = value`` when ``idx`` is a valid in-range index."""
    if idx is not None and 0 <= idx < len(buttons):
        buttons[idx] = value


def rc_to_channels(channels, cfg):
    """
    Normalize every FlySky channel to ``[-1, 1]`` for ``/rc/channels``.

    The same calibration ``rc_to_joy`` applies, but with no axis/button mapping
    on top: channels stay in firmware order so the student library and a
    ``ros2 topic echo`` both see the transmitter as it is wired. A no-signal
    channel reads ``0.0``, matching the Joy failsafe.
    """
    return [
        normalize_channel(raw, cfg['rc_min'], cfg['rc_center'], cfg['rc_max'],
                          cfg['rc_deadband'], cfg['rc_failsafe_below'])
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

    ``channels`` is the list from the ESP32 ``r`` line (``-1`` == no signal); a
    failsafe/no-signal channel maps to neutral so the car stays idle.
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
            chan(i), cfg['rc_min'], cfg['rc_center'], cfg['rc_max'],
            cfg['rc_deadband'], cfg['rc_failsafe_below'],
        )

    # --- Driving axes ---
    ta, tc = cfg['throttle_axis'], cfg['throttle_channel']
    if 0 <= ta < n_axes and tc >= 0:
        axes[ta] = cfg['throttle_sign'] * norm(tc)
    sa, sc = cfg['steering_axis'], cfg['steering_channel']
    if 0 <= sa < n_axes and sc >= 0:
        axes[sa] = cfg['steering_sign'] * norm(sc)

    buttons = [0] * cfg['num_buttons']

    # --- 3-position mode switch -> manual / autonomy enable ---
    # below mid => idle, between mid and high => manual (LB), above high => autonomy (RB).
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
    for ch_i, btn_i in zip(cfg['aux_button_channels'], cfg['aux_button_indices']):
        if norm(ch_i) >= cfg['aux_button_thresh']:
            _set_button(buttons, btn_i)

    return axes, buttons
