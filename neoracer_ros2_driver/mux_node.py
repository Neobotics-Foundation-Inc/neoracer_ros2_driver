"""
Command mux: gates /gamepad_drive (LB) or /drive (RB) onto /mux_out.

Timer-driven so the Maestro stays fed and the watchdog sees a steady publish
rate. Zeroes on joy disconnect or stale upstream commands (>0.5s).
"""

from enum import auto, Enum
import time

from ackermann_msgs.msg import AckermannDriveStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Joy


class MuxMode(Enum):
    IDLE = auto()
    GAMEPAD = auto()
    AUTONOMY = auto()


def select_mode(buttons, gamepad_btn: int, auto_btn: int) -> MuxMode:
    """Pick the mux mode from the latest /joy button state."""
    gp = len(buttons) > gamepad_btn and bool(buttons[gamepad_btn])
    ao = len(buttons) > auto_btn and bool(buttons[auto_btn])
    if gp and not ao:
        return MuxMode.GAMEPAD
    if ao and not gp:
        return MuxMode.AUTONOMY
    return MuxMode.IDLE


def joy_is_centered(axes, threshold: float = 0.2, ignore_axes=()) -> bool:
    """
    Return True when every non-ignored axis magnitude is below threshold.

    Xbox-mode triggers (axes 2 and 5 on the EasySMX) rest at +1.0, not 0.0,
    so they must be excluded from the arming check or the mux never arms.
    """
    ignore = set(ignore_axes)
    return all(
        abs(float(a)) < threshold
        for i, a in enumerate(axes)
        if i not in ignore
    )


class MuxNode(Node):
    def __init__(self):
        super().__init__('mux_node')

        self.declare_parameter('gamepad_enable_button', 4)
        self.declare_parameter('autonomy_enable_button', 5)
        self.declare_parameter('joystick_timeout_sec', 0.5)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('startup_grace_sec', 1.0)
        self.declare_parameter('arm_axis_threshold', 0.2)
        self.declare_parameter('arm_ignore_axes', [2, 5])
        # THIS car gates autonomy with a HARDWARE PCB switch (SWC) that never
        # reaches ROS. So the mux must not gate autonomy itself - it just forwards
        # /drive; the PCB switch physically decides if it moves the car. ON by
        # default because that is this car's design; set false only for a USB
        # gamepad with real buttons.
        self.declare_parameter('autonomy_passthrough', True)

        self._gamepad_btn = self.get_parameter('gamepad_enable_button').value
        self._auto_btn = self.get_parameter('autonomy_enable_button').value
        self._joy_timeout = self.get_parameter('joystick_timeout_sec').value
        self._cmd_timeout = self.get_parameter('command_timeout_sec').value
        publish_rate = self.get_parameter('publish_rate_hz').value
        self._startup_grace = self.get_parameter('startup_grace_sec').value
        self._arm_threshold = self.get_parameter('arm_axis_threshold').value
        self._arm_ignore_axes = tuple(self.get_parameter('arm_ignore_axes').value)

        self._latest_joy: Joy = None
        self._joy_stamp = 0.0
        self._joy_connected = False

        self._latest_gamepad: AckermannDriveStamped = None
        self._gamepad_stamp = 0.0

        self._latest_auto: AckermannDriveStamped = None
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
        self.create_subscription(
            AckermannDriveStamped, '/gamepad_drive', self._gamepad_cb, qos
        )
        self.create_subscription(
            AckermannDriveStamped, '/drive', self._auto_cb, qos
        )

        self.create_timer(1.0 / publish_rate, self._publish)

        self.get_logger().info(
            f'Mux ready: gamepad btn={self._gamepad_btn}, autonomy btn={self._auto_btn}, '
            f'joy timeout={self._joy_timeout}s, cmd timeout={self._cmd_timeout}s, '
            f'rate={publish_rate}Hz'
        )

    def _joy_cb(self, msg: Joy):
        self._latest_joy = msg
        self._joy_stamp = time.monotonic()
        if not self._joy_connected:
            self._joy_connected = True
            self.get_logger().info('Controller connected')

    def _gamepad_cb(self, msg: AckermannDriveStamped):
        self._latest_gamepad = msg
        self._gamepad_stamp = time.monotonic()

    def _auto_cb(self, msg: AckermannDriveStamped):
        self._latest_auto = msg
        self._auto_stamp = time.monotonic()

    def _publish(self):
        now = time.monotonic()
        out = AckermannDriveStamped()

        if self.get_parameter('autonomy_passthrough').value:
            # Forward /drive when fresh; ignore /joy entirely. Software always
            # offers the command, the PCB SWC switch is the physical gate. Stale
            # /drive (lab stopped) -> zeros via cmd_timeout, so it self-safes.
            if (self._latest_auto is not None
                    and (now - self._auto_stamp) <= self._cmd_timeout):
                out = self._latest_auto
            self._pub.publish(out)
            return

        joy = self._latest_joy
        if joy is None or (now - self._joy_stamp) > self._joy_timeout:
            if self._joy_connected and joy is not None:
                self._joy_connected = False
                self.get_logger().warn('Controller disconnected - publishing zero')
            self._pub.publish(out)
            self._last_mode = MuxMode.IDLE
            return

        # Boot-time arming: require an idle period plus a centered Joy frame
        # before honoring bumper presses, so a stuck stick at power-on can't move the robot.
        if not self._armed:
            grace_elapsed = (now - self._boot_time) >= self._startup_grace
            if grace_elapsed and joy_is_centered(
                joy.axes, self._arm_threshold, self._arm_ignore_axes,
            ):
                self._armed = True
                self.get_logger().info('Mux armed')
            else:
                self._pub.publish(out)
                self._last_mode = MuxMode.IDLE
                return

        mode = select_mode(joy.buttons, self._gamepad_btn, self._auto_btn)

        if mode == MuxMode.GAMEPAD:
            if (
                self._latest_gamepad is not None
                and (now - self._gamepad_stamp) <= self._cmd_timeout
            ):
                out = self._latest_gamepad
        elif mode == MuxMode.AUTONOMY:
            if (
                self._latest_auto is not None
                and (now - self._auto_stamp) <= self._cmd_timeout
            ):
                out = self._latest_auto

        if mode != self._last_mode:
            self.get_logger().info(f'Mode -> {mode.name}')
            self._last_mode = mode

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MuxNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
