"""
Command mux: gates /gamepad_drive (LB) or /drive (RB) onto /mux_out.

Timer-driven so the controller and the watchdog see a steady publish rate.
Zeroes on joy disconnect or stale upstream commands (command_timeout_sec).

On the NeoRacer the ESP32 firmware is the gate: it follows the Jetson's `v`
lines while the handset's mode switch is down (autonomy), and also when the
handset is off or out of range. So autonomy_passthrough is
on by default: the mux forwards any fresh /drive and publishes zero once it
goes stale. The bumper gate, the boot-time arming and the FlySky authority
below apply only with autonomy_passthrough off. config/mux.yaml describes the
whole model.

A FlySky transmitter can take the gate instead of the bumpers. That is off by
default (rc_authority_enable): the link predicate is the least-verified part of
the path, and with the mode switch down a lost handset is not reported on
/rc/link. Where it is enabled and a live transmitter is present, the mode
channel selects idle, manual or autonomous and the bumpers are ignored;
otherwise the bumpers govern. On the NeoRacer handset the mode switch reads the
other way round, so it stays off; config/mux.yaml says what enabling it would
do.

Upstream's safety-controller override (/mux/override) is not part of this mux.
"""

from collections.abc import Sequence
from enum import auto, Enum
import time
from typing import TypeVar

from ackermann_msgs.msg import AckermannDriveStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32MultiArray


class MuxMode(Enum):
    IDLE = auto()
    GAMEPAD = auto()
    AUTONOMY = auto()


def select_mode(buttons: Sequence[int], gamepad_btn: int, auto_btn: int) -> MuxMode:
    """Pick the mux mode from the latest /joy button state."""
    gp = len(buttons) > gamepad_btn and bool(buttons[gamepad_btn])
    ao = len(buttons) > auto_btn and bool(buttons[auto_btn])
    if gp and not ao:
        return MuxMode.GAMEPAD
    if ao and not gp:
        return MuxMode.AUTONOMY
    return MuxMode.IDLE


def select_rc_mode(channels: Sequence[float], channel: int, deadband: float) -> MuxMode:
    """
    Pick the mux mode from the RC mode channel.

    Up (positive) is manual, down (negative) is autonomous, the middle band is
    idle. A channel index past the end of the frame reads as idle rather than
    raising, so a short frame cannot arm anything.
    """
    if channel < 0 or channel >= len(channels):
        return MuxMode.IDLE
    value = float(channels[channel])
    if value > deadband:
        return MuxMode.GAMEPAD
    if value < -deadband:
        return MuxMode.AUTONOMY
    return MuxMode.IDLE


class RcAuthority:
    """
    Decide whether a live FlySky transmitter may hold the drive gate.

    Kept out of the node so the rules can be exercised without ROS. Granting is
    deliberately slow and revoking immediate: the link must look good
    continuously for hold_sec before it takes the gate, and a single bad frame
    hands the gate straight back to the bumpers.
    """

    def __init__(self, timeout_sec: float = 0.5, hold_sec: float = 1.0) -> None:
        self._timeout = timeout_sec
        self._hold = hold_sec
        self._valid_since: float | None = None
        self._seen_change = False
        self._last_channels: tuple[float, ...] | None = None
        self.armed = False

    def reset(self) -> None:
        """Drop any progress toward authority, including the arming state."""
        self._valid_since = None
        self._seen_change = False
        self.armed = False

    def observe_channels(self, channels: Sequence[float]) -> None:
        """
        Record a channel frame, tracking whether the data ever moves.

        A receiver jitters; an uninitialized or frozen buffer holding a
        plausible mid-band constant does not. Authority is withheld until at
        least one frame differs from the one before it, which the band check
        alone cannot catch. This gates the initial grant only: a transmitter
        held perfectly still after that keeps the gate.
        """
        current = tuple(float(c) for c in channels)
        if self._last_channels is not None and current != self._last_channels:
            self._seen_change = True
        self._last_channels = current

    def update(
        self,
        now: float,
        link_up: bool,
        link_stamp: float | None,
        channels: Sequence[float],
        channels_stamp: float | None,
    ) -> bool:
        """Return whether the transmitter holds the gate at time `now`."""
        fresh = (
            link_stamp is not None
            and channels_stamp is not None
            and (now - link_stamp) <= self._timeout
            and (now - channels_stamp) <= self._timeout
        )
        if not (fresh and link_up and channels):
            self.reset()
            return False
        if self._valid_since is None:
            self._valid_since = now
        if not self._seen_change:
            return False
        return (now - self._valid_since) >= self._hold

    def gate(self, mode: MuxMode) -> MuxMode:
        """
        Hold the mode at idle until the switch has been seen at middle.

        The analogue of the joystick arming check: a car powered on with the
        switch already down must not go autonomous the moment the link comes up.
        """
        if mode == MuxMode.IDLE:
            self.armed = True
        return mode if self.armed else MuxMode.IDLE


def joy_is_centered(
    axes: Sequence[float], threshold: float = 0.2, ignore_axes: Sequence[int] = ()
) -> bool:
    """
    Return True when every non-ignored axis magnitude is below threshold.

    Xbox-mode triggers (axes 2 and 5 on the EasySMX) rest at +1.0, not 0.0,
    so they must be excluded from the arming check or the mux never arms.
    """
    ignore = set(ignore_axes)
    return all(abs(float(a)) < threshold for i, a in enumerate(axes) if i not in ignore)


def arming_ready(
    since_boot: float,
    grace_sec: float,
    axes: Sequence[float],
    threshold: float,
    ignore_axes: Sequence[int],
) -> bool:
    """Return True once the startup grace has passed and the sticks are centered."""
    return since_boot >= grace_sec and joy_is_centered(axes, threshold, ignore_axes)


T = TypeVar('T')


def select_command(
    mode: MuxMode,
    now: float,
    timeout: float,
    gamepad: tuple[T, float] | None,
    autonomy: tuple[T, float] | None,
) -> T | None:
    """
    Return the upstream command `mode` forwards, or None to publish zero.

    Each source is a (message, arrival time) pair, None before the first message.
    """
    latest = {MuxMode.GAMEPAD: gamepad, MuxMode.AUTONOMY: autonomy}.get(mode)
    if latest is None or (now - latest[1]) > timeout:
        return None
    return latest[0]


class MuxNode(Node):
    def __init__(self) -> None:
        super().__init__('mux_node')

        self.declare_parameter('gamepad_enable_button', 4)
        self.declare_parameter('autonomy_enable_button', 5)
        self.declare_parameter('joystick_timeout_sec', 0.5)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('startup_grace_sec', 1.0)
        self.declare_parameter('arm_axis_threshold', 0.2)
        self.declare_parameter('arm_ignore_axes', [2, 5])
        # On the NeoRacer the ESP32 firmware is the gate: it follows the
        # Jetson's `v` lines while the handset's mode switch is down
        # (autonomy), and also when the handset is off or out of range. So the
        # mux forwards fresh /drive and lets the firmware decide. Set false only
        # for a USB gamepad with real bumpers.
        self.declare_parameter('autonomy_passthrough', True)

        # FlySky authority; off by default (see module docstring).
        self.declare_parameter('rc_authority_enable', False)
        self.declare_parameter('rc_mode_channel', 6)  # 0-indexed; radio Ch7, the mode switch
        self.declare_parameter('rc_mode_deadband', 0.35)
        self.declare_parameter('rc_timeout_sec', 0.5)
        self.declare_parameter('rc_link_hold_sec', 1.0)
        self.declare_parameter('rc_link_topic', '/rc/link')
        self.declare_parameter('rc_channels_topic', '/rc/channels')

        self._gamepad_btn = self.get_parameter('gamepad_enable_button').value
        self._auto_btn = self.get_parameter('autonomy_enable_button').value
        self._joy_timeout = self.get_parameter('joystick_timeout_sec').value
        self._cmd_timeout = self.get_parameter('command_timeout_sec').value
        publish_rate = self.get_parameter('publish_rate_hz').value
        self._startup_grace = self.get_parameter('startup_grace_sec').value
        self._arm_threshold = self.get_parameter('arm_axis_threshold').value
        self._arm_ignore_axes = tuple(self.get_parameter('arm_ignore_axes').value)

        self._rc_enable = bool(self.get_parameter('rc_authority_enable').value)
        self._rc_channel = int(self.get_parameter('rc_mode_channel').value)
        self._rc_deadband = float(self.get_parameter('rc_mode_deadband').value)
        self._rc = RcAuthority(
            timeout_sec=float(self.get_parameter('rc_timeout_sec').value),
            hold_sec=float(self.get_parameter('rc_link_hold_sec').value),
        )
        self._rc_link = False
        self._rc_link_stamp: float | None = None
        self._rc_channels: list[float] = []
        self._rc_channels_stamp: float | None = None
        self._rc_held = False

        self._latest_joy: Joy | None = None
        self._joy_stamp = 0.0
        self._joy_connected = False

        self._latest_gamepad: AckermannDriveStamped | None = None
        self._gamepad_stamp = 0.0

        self._latest_auto: AckermannDriveStamped | None = None
        self._auto_stamp = 0.0

        self._last_mode = MuxMode.IDLE
        self._armed = False
        self._boot_time = time.monotonic()

        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(AckermannDriveStamped, '/mux_out', qos)
        self.create_subscription(Joy, '/joy', self._joy_cb, qos)
        self.create_subscription(AckermannDriveStamped, '/gamepad_drive', self._gamepad_cb, qos)
        self.create_subscription(AckermannDriveStamped, '/drive', self._auto_cb, qos)

        # Subscribed only when rc_authority_enable is set.
        if self._rc_enable:
            self.create_subscription(
                Bool, self.get_parameter('rc_link_topic').value, self._rc_link_cb, qos
            )
            self.create_subscription(
                Float32MultiArray,
                self.get_parameter('rc_channels_topic').value,
                self._rc_channels_cb,
                qos,
            )

        self.create_timer(1.0 / publish_rate, self._publish)

        passthrough = bool(self.get_parameter('autonomy_passthrough').value)
        rc_state = 'off'
        if self._rc_enable:
            rc_state = 'on, unused while autonomy_passthrough is on' if passthrough else 'on'
        self.get_logger().info(
            f'Mux ready: gamepad btn={self._gamepad_btn}, autonomy btn={self._auto_btn}, '
            f'joy timeout={self._joy_timeout}s, cmd timeout={self._cmd_timeout}s, '
            f'rate={publish_rate}Hz, '
            f'rc_authority={rc_state}, '
            f'autonomy_passthrough={"on" if passthrough else "off"}'
        )

    def _joy_cb(self, msg: Joy) -> None:
        self._latest_joy = msg
        self._joy_stamp = time.monotonic()
        if not self._joy_connected:
            self._joy_connected = True
            self.get_logger().info('Controller connected')

    def _rc_link_cb(self, msg: Bool) -> None:
        self._rc_link = bool(msg.data)
        self._rc_link_stamp = time.monotonic()

    def _rc_channels_cb(self, msg: Float32MultiArray) -> None:
        self._rc_channels = list(msg.data)
        self._rc_channels_stamp = time.monotonic()
        self._rc.observe_channels(self._rc_channels)

    def _gamepad_cb(self, msg: AckermannDriveStamped) -> None:
        self._latest_gamepad = msg
        self._gamepad_stamp = time.monotonic()

    def _auto_cb(self, msg: AckermannDriveStamped) -> None:
        self._latest_auto = msg
        self._auto_stamp = time.monotonic()

    def _rc_has_authority(self, now: float) -> bool:
        """Check whether a live transmitter holds the gate, and log transitions."""
        if not self._rc_enable:
            return False
        held = self._rc.update(
            now,
            self._rc_link,
            self._rc_link_stamp,
            self._rc_channels,
            self._rc_channels_stamp,
        )
        if held != self._rc_held:
            self._rc_held = held
            self.get_logger().info(
                'RC transmitter holds the drive gate'
                if held
                else 'RC transmitter released the drive gate'
            )
        return held

    def _joy_mode(self, now: float) -> MuxMode | None:
        """
        Pick the mode from the gamepad, or None when it cannot be trusted.

        None means publish zero and log no mode change: the controller is
        missing, has gone stale, or the boot-time arming check has not passed.
        """
        joy = self._latest_joy
        if joy is None or (now - self._joy_stamp) > self._joy_timeout:
            if self._joy_connected and joy is not None:
                self._joy_connected = False
                self.get_logger().warn('Controller disconnected; publishing zero')
            return None

        # Boot-time arming: require an idle period plus a centered Joy frame
        # before honoring bumper presses, so a stuck stick at power-on can't move the robot.
        if not self._armed:
            if arming_ready(
                now - self._boot_time,
                self._startup_grace,
                joy.axes,
                self._arm_threshold,
                self._arm_ignore_axes,
            ):
                self._armed = True
                self.get_logger().info('Mux armed')
            else:
                return None

        return select_mode(joy.buttons, self._gamepad_btn, self._auto_btn)

    def _publish(self) -> None:
        now = time.monotonic()
        out = AckermannDriveStamped()

        if self.get_parameter('autonomy_passthrough').value:
            # Forward fresh /drive and ignore /joy; the firmware's mode switch
            # is the gate. A stale /drive (a stopped program) publishes zero.
            command = select_command(
                MuxMode.AUTONOMY,
                now,
                self._cmd_timeout,
                None,
                None if self._latest_auto is None else (self._latest_auto, self._auto_stamp),
            )
            self._pub.publish(out if command is None else command)
            return

        if self._rc_has_authority(now):
            mode = self._rc.gate(
                select_rc_mode(self._rc_channels, self._rc_channel, self._rc_deadband)
            )
        else:
            joy_mode = self._joy_mode(now)
            if joy_mode is None:
                self._pub.publish(out)
                self._last_mode = MuxMode.IDLE
                return
            mode = joy_mode

        command = select_command(
            mode,
            now,
            self._cmd_timeout,
            None if self._latest_gamepad is None else (self._latest_gamepad, self._gamepad_stamp),
            None if self._latest_auto is None else (self._latest_auto, self._auto_stamp),
        )
        if command is not None:
            out = command

        if mode != self._last_mode:
            self.get_logger().info(f'Mode -> {mode.name}')
            self._last_mode = mode

        self._pub.publish(out)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MuxNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
