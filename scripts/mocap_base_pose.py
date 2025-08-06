#!/bin/env python3
"""
Gives odometry of the robot_base based on motion capture readings
Motion capture : Qualisys with RealTime protocol version 1.24
"""

import PyKDL
import asyncio
import qtm_rt
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class MocapOdometryNode(Node):
    def __init__(self):
        super().__init__("mocap_base_estimator")

        # ros2 parameters ======================================================
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("wanted_body", "Go2")  # go2, cube or None
        self.declare_parameter("qualisys_ip", "192.168.75.4")
        self.declare_parameter(
            "publishing_freq", 110
        )  # in Hz : due to the discretisation the frequency may be slightly lower than what is it set to. Max limit of 300Hz (set in MoCap software)
        self.declare_parameter("mimic_go2_odometry", 0)
        self.mocap_as_pose_estimate = bool(self.get_parameter("mimic_go2_odometry").value)
        # change name of base_frame according to use case
        if self.mocap_as_pose_estimate:
            self.declare_parameter("base_frame", "base")
        else:
            self.declare_parameter("base_frame", "base_frame")

        # Check if publishing freq is a correct value
        if (
            isinstance(self.get_parameter("publishing_freq").value, int) is False
            or self.get_parameter("publishing_freq").value > 300
            or self.get_parameter("publishing_freq").value < 0
        ):
            self.get_logger().error(
                "Invalid value for publishing_freq parameter, must be an interger belonging to [0;300]"
            )

            self.destroy_node()
            return

        # publishers ===========================================================
        self.tf_broadcaster = TransformBroadcaster(self)

        if self.mocap_as_pose_estimate:  # publish odometry if mocap is used as a perfect pose estimator
            self.odometry_publisher = self.create_publisher(Odometry, "odometry/filtered", 10)

        # startup info =========================================================
        self.get_logger().info("MoCap started with parameters:")
        self.get_logger().info("base_frame: " + self.get_parameter("base_frame").value)
        self.get_logger().info("odom_frame: " + self.get_parameter("odom_frame").value)
        self.get_logger().info("qualisys_ip: " + self.get_parameter("qualisys_ip").value)
        self.get_logger().info("wanted_body: " + self.get_parameter("wanted_body").value)
        self.get_logger().info("publishing_freq: " + str(self.get_parameter("publishing_freq").value))
        self.get_logger().info("mimic_go2_odometry: " + str(self.mocap_as_pose_estimate))

        # Connecting to the motion capture =====================================
        asyncio.ensure_future(self.setup())
        self.loop = asyncio.get_event_loop()
        self.loop.run_forever()

    async def setup(self):
        """Connects to the Motion Capture system and sets callback on packet received"""
        connection = await qtm_rt.connect(
            self.get_parameter("qualisys_ip").value, version="1.24"
        )  # version changed to 1.24

        if connection is None:
            self.get_logger().error("Could not connect to the Motion Capture")
            #! for now the node does not destroy itself when failing to connect
            self.destroy_node()
            return

        xml_string = await connection.get_parameters(parameters=["6d"])
        self.body_index = self.create_body_index(xml_string)

        # disconnect if the wanted body isn't streamed
        if (self.get_parameter("wanted_body").value is None) or (
            self.get_parameter("wanted_body").value not in self.body_index
        ):
            self.get_logger().error("Wanted body not found or wanted_body is None")
            connection.disconnect()

        await connection.stream_frames(components=["6d"], on_packet=self.on_packet)

    def on_packet(self, packet):
        """Callback function called every time a data packet arrives from QTM"""

        # Time management
        self.new_timestamp = packet.timestamp * 0.000001  # convert from µs to seconds

        if (self.new_timestamp - self.prec_timestamp) >= (1 / self.get_parameter("publishing_freq").value):
            info, bodies = packet.get_6d()

            i = 0  # index
            for body in bodies:
                # select wanted body
                if self.body_index[self.get_parameter("wanted_body").value] == i:
                    position, rotation = body

                    # Quaternions from rotation matrix (transpose of matrix sent by mocap)
                    # fmt: off
                    rotation_matrix = PyKDL.Rotation(rotation[0][0], rotation[0][3], rotation[0][6],
                                                     rotation[0][1], rotation[0][4], rotation[0][7],
                                                     rotation[0][2], rotation[0][5], rotation[0][8])
                    # fmt: on

                    qx, qy, qz, qw = rotation_matrix.GetQuaternion()

                    timestamp = self.get_clock().now().to_msg()

                    # Transform from odom_frame (unmoving) to base_frame (tied to robot base)
                    self.transform_msg.header.stamp = timestamp

                    if not self.mocap_as_pose_estimate:  # change the name of the transformation published so that it doesn't interfer with robot control when used as ground truth
                        self.transform_msg.child_frame_id = "base_mocap"
                    else:
                        self.transform_msg.child_frame_id = self.get_parameter("base_frame").value

                    self.transform_msg.header.frame_id = self.get_parameter("odom_frame").value

                    # translation + convert from millimeter to meter
                    self.transform_msg.transform.translation.x = position[0] * 0.001
                    self.transform_msg.transform.translation.y = position[1] * 0.001
                    self.transform_msg.transform.translation.z = position[2] * 0.001

                    # rotation
                    self.transform_msg.transform.rotation.x = qx
                    self.transform_msg.transform.rotation.y = qy
                    self.transform_msg.transform.rotation.z = qz
                    self.transform_msg.transform.rotation.w = qw

                    self.tf_broadcaster.sendTransform(self.transform_msg)

                    if self.mocap_as_pose_estimate:  # publish odometry if mocap is used as a perfect pose estimator
                        self.odometry_msg.header.stamp = timestamp
                        self.odometry_msg.child_frame_id = self.get_parameter("base_frame").value
                        self.odometry_msg.header.frame_id = self.get_parameter("odom_frame").value

                        # position + convert to meter
                        self.odometry_msg.pose.pose.position.x = position[0] * 0.001
                        self.odometry_msg.pose.pose.position.y = position[1] * 0.001
                        self.odometry_msg.pose.pose.position.z = position[2] * 0.001

                        # orientation
                        self.odometry_msg.pose.pose.orientation.x = qx
                        self.odometry_msg.pose.pose.orientation.y = qy
                        self.odometry_msg.pose.pose.orientation.z = qz
                        self.odometry_msg.pose.pose.orientation.w = qw

                        # linear speed
                        self.odometry_msg.twist.twist.linear.x = 0.0
                        self.odometry_msg.twist.twist.linear.y = 0.0
                        self.odometry_msg.twist.twist.linear.z = 0.0

                        # angular speed
                        self.odometry_msg.twist.twist.angular.x = 0.0
                        self.odometry_msg.twist.twist.angular.y = 0.0
                        self.odometry_msg.twist.twist.angular.z = 0.0
                        self.odometry_publisher.publish(self.odometry_msg)

                else:
                    pass  # ignore other bodies

                i += 1
            self.prec_timestamp = self.new_timestamp

    def create_body_index(self, xml_string):
        """Extract a name to index dictionary from 6dof settings xml,
        Taken from qualisys_python_sdk/examples/stream_6dof_example.py"""

        xml = ET.fromstring(xml_string)

        for index, body in enumerate(xml.findall("*/Body/Name")):
            self.body_index[body.text.strip()] = index

        return self.body_index

    # Class attributes
    transform_msg = TransformStamped()
    odometry_msg = Odometry()
    body_index = {}
    new_timestamp = 0
    prec_timestamp = 0


def main(args=None):
    rclpy.init(args=args)

    node = MocapOdometryNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
