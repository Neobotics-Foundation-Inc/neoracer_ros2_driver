#!/usr/bin/env python3

"""
Broadcast the ``odom -> base_footprint`` transform from ``/odom``.

The Neoracer ESP32 bridge (``controller`` node) publishes wheel + IMU
odometry as ``nav_msgs/Odometry`` on ``/odom`` with ``frame_id = odom``
and ``child_frame_id = base_footprint``, but it does not broadcast the
matching TF. slam_toolbox and Nav2 both look up ``odom -> base_footprint``
on /tf, so this node mirrors each ``/odom`` message onto the transform
tree. It is the minimum glue that lets the SLAM stack consume the
driver's odometry unchanged.

Dependencies: rclpy, nav_msgs, tf2_ros, geometry_msgs.
"""

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


class OdomTfNode(Node):
    """Re-publish ``/odom`` poses as the ``odom -> base_footprint`` TF."""

    def __init__(self):
        super().__init__('odom_tf_node')

        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter(
            'base_frame', 'base_footprint').value
        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value

        self.broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(
            Odometry, self.odom_topic, self._on_odom, qos_profile_sensor_data)

        self.get_logger().info(
            '[INFO] odom_tf node ready, broadcasting %s -> %s from %s'
            % (self.odom_frame, self.base_frame, self.odom_topic))

    def _on_odom(self, msg):
        """Mirror one Odometry message onto /tf."""
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
