#!/bin/env python3

import rclpy
from rclpy.node import Node

from go2_odometry.msg import OdometryVector
from unitree_go.msg import LowState
from geometry_msgs.msg import PoseWithCovariance

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
        self.initialize_pose = True

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

        if np.min(f_contact) > 30:
            self.initialize_pose = False
            #self.get_logger().info('Initialization over')

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
            if(f_contact[i] >= 20 or self.initialize_pose):
                feet_list.append(True)
            else:
                feet_list.append(False)
            pose_foot = PoseWithCovariance()
            pose_foot.covariance = [0.] * 36

            pose_foot.pose.position.x = bMf_list[i].translation[0]
            pose_foot.pose.position.y = bMf_list[i].translation[1]
            pose_foot.pose.position.z = bMf_list[i].translation[2]

            quat = pin.Quaternion(bMf_list[i].rotation)
            quat.normalize()
            
            pose_foot.pose.orientation.x = quat.x
            pose_foot.pose.orientation.y = quat.y
            pose_foot.pose.orientation.z = quat.z
            pose_foot.pose.orientation.w = quat.w

            pos_list.append(pose_foot)

        pos_msg.contact_states = feet_list
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