#!/bin/env python3

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class FakeOdometryNode(Node):
    def __init__(self):
        super().__init__("fake_odometry")

        self.declare_parameter("base_height", 0.30)
        self.declare_parameter("base_frame", "base")
        self.declare_parameter("odom_frame", "odom")

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odometry_publisher = self.create_publisher(Odometry, "odometry/filtered", 10)

        self.timer = self.create_timer(0.01, self.publish_odom_cb)

    def publish_odom_cb(self):
        timestamp = self.get_clock().now().to_msg()

        transform_msg = TransformStamped()
        transform_msg.header.stamp = timestamp
        transform_msg.child_frame_id = self.get_parameter("base_frame").value
        transform_msg.header.frame_id = self.get_parameter("odom_frame").value
        transform_msg.transform.translation.x, transform_msg.transform.translation.y = 0.0, 0.0
        transform_msg.transform.translation.z = self.get_parameter("base_height").value
        (
            transform_msg.transform.rotation.x,
            transform_msg.transform.rotation.y,
            transform_msg.transform.rotation.z,
            transform_msg.transform.rotation.w,
        ) = 0.0, 0.0, 0.0, 1.0
        self.tf_broadcaster.sendTransform(transform_msg)

        odometry_msg = Odometry()
        odometry_msg.header.stamp = timestamp
        odometry_msg.child_frame_id = self.get_parameter("base_frame").value
        odometry_msg.header.frame_id = self.get_parameter("odom_frame").value
        odometry_msg.pose.pose.position.x, odometry_msg.pose.pose.position.y = 0.0, 0.0
        odometry_msg.pose.pose.position.z = self.get_parameter("base_height").value
        (
            odometry_msg.pose.pose.orientation.x,
            odometry_msg.pose.pose.orientation.y,
            odometry_msg.pose.pose.orientation.z,
            odometry_msg.pose.pose.orientation.w,
        ) = 0.0, 0.0, 0.0, 1.0
        odometry_msg.twist.twist.linear.x, odometry_msg.twist.twist.linear.y, odometry_msg.twist.twist.linear.z = (
            0.0,
            0.0,
            0.0,
        )
        odometry_msg.twist.twist.angular.x, odometry_msg.twist.twist.angular.y, odometry_msg.twist.twist.angular.z = (
            0.0,
            0.0,
            0.0,
        )
        self.odometry_publisher.publish(odometry_msg)


def main(args=None):
    rclpy.init(args=args)

    node = FakeOdometryNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
