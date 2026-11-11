"""Joy -> /gamepad_drive passthrough. All caps live in throttle_node."""

from collections.abc import Sequence

from ackermann_msgs.msg import AckermannDriveStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Joy

from .limits import clamp


def joy_to_drive(
    axes: Sequence[float],
    throttle_axis: int,
    steering_axis: int,
    throttle_sign: float,
    steering_sign: float,
) -> tuple[float, float] | None:
    """Return (speed, steering) in [-1, 1], or None when the frame lacks either axis."""
    if len(axes) <= max(throttle_axis, steering_axis):
        return None
    speed = float(axes[throttle_axis]) * throttle_sign
    steering = float(axes[steering_axis]) * steering_sign
    return clamp(speed), clamp(steering)


class GamepadNode(Node):
    def __init__(self) -> None:
        super().__init__('gamepad_node')

        self.declare_parameter('throttle_axis', 1)
        self.declare_parameter('steering_axis', 3)
        self.declare_parameter('throttle_sign', 1)
        self.declare_parameter('steering_sign', 1)

        self._throttle_axis = self.get_parameter('throttle_axis').value
        self._steering_axis = self.get_parameter('steering_axis').value
        self._throttle_sign = self.get_parameter('throttle_sign').value
        self._steering_sign = self.get_parameter('steering_sign').value

        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(AckermannDriveStamped, '/gamepad_drive', qos)
        self.create_subscription(Joy, '/joy', self._joy_cb, qos)

        self.get_logger().info(
            f'Gamepad ready: throttle axis={self._throttle_axis} '
            f'(sign={self._throttle_sign}), '
            f'steering axis={self._steering_axis} (sign={self._steering_sign})'
        )

    def _joy_cb(self, msg: Joy) -> None:
        command = joy_to_drive(
            msg.axes,
            self._throttle_axis,
            self._steering_axis,
            self._throttle_sign,
            self._steering_sign,
        )
        if command is None:
            return
        drive = AckermannDriveStamped()
        drive.drive.speed, drive.drive.steering_angle = command
        self._pub.publish(drive)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GamepadNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
