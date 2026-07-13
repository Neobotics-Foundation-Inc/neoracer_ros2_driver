"""Joy -> /gamepad_drive passthrough. All caps live in throttle_node."""

from ackermann_msgs.msg import AckermannDriveStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Joy


class GamepadNode(Node):
    def __init__(self):
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
        self._pub = self.create_publisher(
            AckermannDriveStamped, '/gamepad_drive', qos
        )
        self.create_subscription(Joy, '/joy', self._joy_cb, qos)

        self.get_logger().info(
            f'Gamepad ready: throttle axis={self._throttle_axis} '
            f'(sign={self._throttle_sign}), '
            f'steering axis={self._steering_axis} (sign={self._steering_sign})'
        )

    def _joy_cb(self, msg: Joy):
        if len(msg.axes) <= max(self._throttle_axis, self._steering_axis):
            return
        speed = float(msg.axes[self._throttle_axis]) * self._throttle_sign
        steering = float(msg.axes[self._steering_axis]) * self._steering_sign
        drive = AckermannDriveStamped()
        drive.drive.speed = max(-1.0, min(1.0, speed))
        drive.drive.steering_angle = max(-1.0, min(1.0, steering))
        self._pub.publish(drive)


def main(args=None):
    rclpy.init(args=args)
    node = GamepadNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
