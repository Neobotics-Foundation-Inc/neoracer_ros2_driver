"""
Throttle: clamps /mux_out to [-1, 1] and scales by per-direction caps onto /motor.

Single source of truth for actuator caps. +speed = forward, +steering = left.
"""

from ackermann_msgs.msg import AckermannDriveStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


def scale_speed(speed: float, max_fwd: float, max_back: float) -> float:
    speed = max(-1.0, min(1.0, speed))
    return speed * (max_fwd if speed >= 0 else max_back)


def scale_steering(angle: float, max_steer: float) -> float:
    return max(-1.0, min(1.0, angle)) * max_steer


class ThrottleNode(Node):
    def __init__(self):
        super().__init__('throttle_node')

        self.declare_parameter('max_speed_forward', 0.17)
        self.declare_parameter('max_speed_backward', 0.24)
        self.declare_parameter('max_steering', 0.625)

        self._max_fwd = self.get_parameter('max_speed_forward').value
        self._max_back = self.get_parameter('max_speed_backward').value
        self._max_steer = self.get_parameter('max_steering').value

        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(AckermannDriveStamped, '/motor', qos)
        self.create_subscription(
            AckermannDriveStamped, '/mux_out', self._drive_cb, qos
        )

        self.get_logger().info(
            f'Throttle ready: max_fwd={self._max_fwd}, max_back={self._max_back}, '
            f'max_steer={self._max_steer} (all as fractions of full PWM swing)'
        )

    def _drive_cb(self, msg: AckermannDriveStamped):
        out = AckermannDriveStamped()
        out.header = msg.header
        out.drive.speed = scale_speed(msg.drive.speed, self._max_fwd, self._max_back)
        out.drive.steering_angle = scale_steering(msg.drive.steering_angle, self._max_steer)
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ThrottleNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
