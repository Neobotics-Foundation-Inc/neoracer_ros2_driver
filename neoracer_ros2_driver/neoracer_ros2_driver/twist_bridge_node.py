#!/usr/bin/env python3

"""
Bridge Nav2's ``/cmd_vel`` (Twist, m/s and rad/s) onto the normalized
``/drive`` topic the mux arbitrates.

Nav2 and the osracer planners speak Twist in physical units. The neoracer
pipeline speaks normalized Ackermann: ``/drive`` -> mux -> throttle ->
``/motor`` -> controller, where the controller multiplies by
``max_speed_mps`` / ``max_steering_angle_deg``. This node divides the same
constants back out, so a Twist of 1.5 m/s arrives at the firmware as
1.5 m/s regardless of the throttle caps in between.

Steering comes from the bicycle model: ``steer = atan(wheelbase * omega / v)``.
An Ackermann car cannot turn in place, so when the planner asks for rotation
at near-zero speed the bridge commands ``min_turn_speed_mps`` forward to give
the front axle authority (the creep-arc behaviour the physical car needs).

Autonomy stays behind the same gate as any student program: the mux only
passes ``/drive`` through while SWB is down. Flipping SWB up takes the car
back regardless of what Nav2 wants.
"""

import math

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist


class TwistBridgeNode(Node):
    """Convert /cmd_vel Twist commands to normalized /drive Ackermann."""

    def __init__(self):
        super().__init__('twist_bridge_node')

        # Must match config/controller.yaml so normalization round-trips.
        self.max_speed_mps = self.declare_parameter('max_speed_mps', 6.0).value
        self.max_steering_angle_deg = self.declare_parameter(
            'max_steering_angle_deg', 30.0).value
        self.wheelbase = self.declare_parameter('wheelbase', 0.285).value

        # Slowest speed at which the servo still steers the car; commanded
        # when the planner wants to turn but sends |v| below this.
        self.min_turn_speed_mps = self.declare_parameter(
            'min_turn_speed_mps', 0.4).value

        # Publish zero once if the planner goes quiet (the mux also zeroes
        # on its own command_timeout_sec; this just makes the stop explicit).
        self.cmd_timeout_sec = self.declare_parameter('cmd_timeout_sec', 0.5).value

        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)

        self._last_cmd_time = None
        self._stopped = True
        self.create_timer(0.1, self._check_stale)

        self.get_logger().info(
            f'[INFO] twist_bridge up: /cmd_vel -> /drive '
            f'(max {self.max_speed_mps} m/s, {self.max_steering_angle_deg} deg, '
            f'wheelbase {self.wheelbase} m)')

    def on_cmd_vel(self, msg: Twist):
        """Convert one Twist to a normalized Ackermann command."""
        v = msg.linear.x
        omega = msg.angular.z

        # Creep-arc: rotation needs forward motion on an Ackermann platform.
        if abs(omega) > 1e-3 and abs(v) < self.min_turn_speed_mps:
            v = math.copysign(self.min_turn_speed_mps, v if abs(v) > 1e-3 else 1.0)

        if abs(v) > 1e-3:
            steer_rad = math.atan(self.wheelbase * omega / v)
        else:
            steer_rad = 0.0

        speed_norm = max(-1.0, min(1.0, v / self.max_speed_mps))
        steer_norm = max(-1.0, min(1.0,
                         math.degrees(steer_rad) / self.max_steering_angle_deg))

        self._publish(speed_norm, steer_norm)
        self._last_cmd_time = self.get_clock().now()
        self._stopped = False

    def _check_stale(self):
        """Send one explicit zero when the planner stops publishing."""
        if self._stopped or self._last_cmd_time is None:
            return
        age = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if age > self.cmd_timeout_sec:
            self._publish(0.0, 0.0)
            self._stopped = True

    def _publish(self, speed: float, steering: float):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = speed
        msg.drive.steering_angle = steering
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TwistBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
