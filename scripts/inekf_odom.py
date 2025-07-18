#!/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from unitree_go.msg import LowState
import pinocchio as pin

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor
from inekf import RobotState, NoiseParams, InEKF, Kinematics
from go2_description.loader import loadGo2


# ==============================================================================
# Main Class
# ==============================================================================
class Inekf(Node):
    def __init__(self):
        super().__init__("inekf")

        self.base_frame = self.declare_parameter("base_frame", "base").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        robot_fq = self.declare_parameter(
            "robot_freq", 500.0, ParameterDescriptor(description="Frequency at which the robot publish its state")
        ).value
        self.dt = 1.0 / robot_fq

        # Load robot model
        self.robot = loadGo2()
        self.foot_frame_name = [prefix + "_foot" for prefix in ["FL", "FR", "RL", "RR"]]
        self.foot_frame_id = [self.robot.model.getFrameId(frame_name) for frame_name in self.foot_frame_name]
        self.imu_frame_id = self.robot.model.getFrameId("imu")

        # In/Out topics
        self.lowstate_subscription = self.create_subscription(LowState, "/lowstate", self.listener_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, "/odometry/filtered", 1)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Invariant EKF
        gravity = np.array([0, 0, -9.81])

        initial_state = RobotState()
        initial_state.setRotation(np.eye(3))
        initial_state.setVelocity(np.zeros(3))
        initial_state.setPosition(np.zeros(3))
        initial_state.setGyroscopeBias(np.zeros(3))
        initial_state.setAccelerometerBias(np.zeros(3))

        # Initialize state covariance
        noise_params = NoiseParams()
        noise_params.setGyroscopeNoise(0.01)
        noise_params.setAccelerometerNoise(0.1)
        noise_params.setGyroscopeBiasNoise(0.00001)
        noise_params.setAccelerometerBiasNoise(0.0001)
        noise_params.setContactNoise(0.001)

        self.filter = InEKF(initial_state, noise_params)
        self.filter.setGravity(gravity)

    def listener_callback(self, msg):
        # Format IMU measurements
        imu_state = np.concatenate([msg.imu_state.gyroscope, msg.imu_state.accelerometer])

        # Propagate filter
        self.filter.propagate(imu_state, self.dt)

        # TODO: use IMU quaternion for extra correction step ?

        # FEET KINEMATIC data ==================================================
        contact_list, pose_list, covariance_list = self.feet_transformations(msg)

        contact_pairs = []
        kinematics_list = []
        for i in range(len(self.foot_frame_name)):
            contact_pairs.append((i, contact_list[i]))

            velocity = np.zeros(3)

            kinematics = Kinematics(i, pose_list[i].translation, covariance_list[i], velocity, np.eye(3) * 0.001)
            kinematics_list.append(kinematics)

        self.filter.setContacts(contact_pairs)
        self.filter.correctKinematics(kinematics_list)

        self.publish_state(self.filter.getState(), msg.imu_state.gyroscope)

    def _unitree_to_urdf_vec(self, vec):
        # fmt: off
        return  [vec[3],  vec[4],  vec[5],
                 vec[0],  vec[1],  vec[2],
                 vec[9],  vec[10], vec[11],
                 vec[6],  vec[7],  vec[8],]
        # fmt: on

    def feet_transformations(self, state_msg):
        # Get sensor measurement
        q_unitree = [j.q for j in state_msg.motor_state[:12]]
        v_unitree = [j.dq for j in state_msg.motor_state[:12]]
        f_unitree = state_msg.foot_force

        # Rearrange joints according to urdf
        q_pin = np.array([0] * 6 + [1] + self._unitree_to_urdf_vec(q_unitree))
        v_pin = np.array([0] * 6 + self._unitree_to_urdf_vec(v_unitree))
        f_pin = [f_unitree[i] for i in [1, 0, 3, 2]]

        # Compute positions and velocities
        pin.forwardKinematics(self.robot.model, self.robot.data, q_pin, v_pin)
        pin.updateFramePlacements(self.robot.model, self.robot.data)
        pin.computeJointJacobians(self.robot.model, self.robot.data)

        # Make message
        contact_list = []
        pose_list = []
        covariance_list = []
        for i in range(4):
            if f_pin[i] >= 20:
                contact_list.append(True)
            else:
                contact_list.append(False)

            pose_list.append(self.robot.data.oMf[self.foot_frame_id[i]])

            Jc = pin.getFrameJacobian(self.robot.model, self.robot.data, self.foot_frame_id[i], pin.LOCAL)[:3, 6:]
            cov_pose = Jc @ np.eye(12) * 1e-3 @ Jc.transpose()
            covariance_list.append(cov_pose)

        return contact_list, pose_list, covariance_list

    def publish_state(self, filter_state, twist_angular_vel):
        # Get filter state
        timestamp = self.get_clock().now().to_msg()

        state_rotation = filter_state.getRotation()
        state_position = filter_state.getPosition()
        state_velocity = state_rotation.T @ filter_state.getX()[0:3, 3:4]
        state_velocity = state_velocity.reshape(-1)

        state_quaternion = pin.Quaternion(state_rotation)
        state_quaternion.normalize()

        # TF2 messages
        transform_msg = TransformStamped()
        transform_msg.header.stamp = timestamp
        transform_msg.child_frame_id = self.base_frame
        transform_msg.header.frame_id = self.odom_frame

        transform_msg.transform.translation.x = state_position[0]
        transform_msg.transform.translation.y = state_position[1]
        transform_msg.transform.translation.z = state_position[2]

        transform_msg.transform.rotation.x = state_quaternion.x
        transform_msg.transform.rotation.y = state_quaternion.y
        transform_msg.transform.rotation.z = state_quaternion.z
        transform_msg.transform.rotation.w = state_quaternion.w

        self.tf_broadcaster.sendTransform(transform_msg)

        # Odometry topic
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.child_frame_id = self.base_frame
        odom_msg.header.frame_id = self.odom_frame

        odom_msg.pose.pose.position.x = state_position[0]
        odom_msg.pose.pose.position.y = state_position[1]
        odom_msg.pose.pose.position.z = state_position[2]

        odom_msg.pose.pose.orientation.x = state_quaternion.x
        odom_msg.pose.pose.orientation.y = state_quaternion.y
        odom_msg.pose.pose.orientation.z = state_quaternion.z
        odom_msg.pose.pose.orientation.w = state_quaternion.w

        odom_msg.twist.twist.linear.x = state_velocity[0]
        odom_msg.twist.twist.linear.y = state_velocity[1]
        odom_msg.twist.twist.linear.z = state_velocity[2]

        odom_msg.twist.twist.angular.x = float(twist_angular_vel[0])
        odom_msg.twist.twist.angular.y = float(twist_angular_vel[1])
        odom_msg.twist.twist.angular.z = float(twist_angular_vel[2])

        self.odom_publisher.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)

    inekf_node = Inekf()

    rclpy.spin(inekf_node)

    inekf_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
