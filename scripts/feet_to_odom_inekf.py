#!/bin/env python3

import rclpy
from rclpy.node import Node

from go2_odometry.msg import OdometryVector
from unitree_go.msg import LowState
from geometry_msgs.msg import PoseWithCovariance
from scipy.spatial.transform import Rotation

import numpy as np
import pinocchio as pin
from go2_description import loadGo2

class FeetToOdom(Node):

    def __init__(self):
        super().__init__('feet_to_odom')

        self.pos_publisher = self.create_publisher(OdometryVector, 'odometry/feet_pos', 10)
        self.subscription = self.create_subscription(
            LowState,
            '/lowstate',
            self.listener_callback,
            10)

        self.robot = loadGo2()
        self.foot_frame_name = [prefix + "_foot" for prefix in ["FL", "FR", "RL", "RR"]]
        self.foot_frame_id = [self.robot.model.getFrameId(frame_name) for frame_name in self.foot_frame_name]
        self.imu_frame_id = self.robot.model.getFrameId("imu")

    def _unitree_to_urdf_vec(self, vec):
        return  [vec[3],  vec[4],  vec[5],
                 vec[0],  vec[1],  vec[2],
                 vec[9],  vec[10], vec[11],
                 vec[6],  vec[7],  vec[8],]


    def listener_callback(self, state_msg):
        # Get sensor measurement
        q_unitree = [j.q for j in state_msg.motor_state[:12]]
        v_unitree = [j.dq for j in state_msg.motor_state[:12]]
        fc_unitree = state_msg.foot_force

        # Rearrange joints according to urdf
        q = np.array([0]*6 + [1] + self._unitree_to_urdf_vec(q_unitree))
        v = np.array([0]*6 + self._unitree_to_urdf_vec(v_unitree))
        f_contact = [fc_unitree[i] for i in [1, 0, 3, 2]]

        # Compute positions and velocities
        ## f = foot, i = imu, b = base
        self.robot.forwardKinematics(q, v)
        pin.updateFramePlacements(self.robot.model, self.robot.data)
        bMf_list = [self.robot.data.oMf[id] for id in self.foot_frame_id]

        # Make message
        pos_msg = OdometryVector()
        pos_msg.header.stamp = self.get_clock().now().to_msg()
        pos_list = []
        feet_list = []
        for i in range(4):
            if(f_contact[i]<10):
                continue # Feet in the air : skip

            feet_list.append(self.foot_frame_name[i])
            pose_foot = PoseWithCovariance()
            pose_foot.covariance = [0.] * 36

            pose_foot.pose.position.x = bMf_list[i].translation[0]
            pose_foot.pose.position.y = bMf_list[i].translation[1]
            pose_foot.pose.position.z = bMf_list[i].translation[2]

            foot_rotation = bMf_list[i].rotation
            r = Rotation.from_matrix([[foot_rotation[0][0], foot_rotation[0][1], foot_rotation[0][2]],
                                  [foot_rotation[1][0], foot_rotation[1][1], foot_rotation[1][2]],
                                  [foot_rotation[2][0], foot_rotation[2][1], foot_rotation[2][2]]])
            
            qx, qy, qz, qw = r.as_quat()
            pose_foot.pose.orientation.x = qx
            pose_foot.pose.orientation.y = qy
            pose_foot.pose.orientation.z = qz
            pose_foot.pose.orientation.w = qw

            pos_list.append(pose_foot)

        pos_msg.feet_names = feet_list
        pos_msg.pose_vec = pos_list

        self.pos_publisher.publish(pos_msg)

def main(args=None):
    rclpy.init(args=args)

    feet_to_odom = FeetToOdom()

    rclpy.spin(feet_to_odom)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    feet_to_odom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()