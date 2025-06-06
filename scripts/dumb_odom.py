#!/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
import pinocchio as pin
from go2_description import loadGo2

from unitree_go.msg import LowState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

# ==============================================================================
# Main Class
# ==============================================================================
class DumbOdom(Node):

    def __init__(self):
        super().__init__('DumbOdom')
        print("==GO2 DumbOdom launched==")
        # ROS2 =================================================================
        self.declare_parameter("base_frame", "base")
        self.declare_parameter("odom_frame", "odom")
        
        qos_profile_keeplast = QoSProfile(history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
        self.lowstate_subscription_ = self.create_subscription(
            LowState,
            '/lowstate',
            self.listener_callback,
            10)
        
        self.robot = loadGo2()
        self.foot_frame_name = [prefix + "_foot" for prefix in ["FL", "FR", "RL", "RR"]]
        self.foot_frame_id = [self.robot.model.getFrameId(frame_name) for frame_name in self.foot_frame_name]
        
        self.odom_publisher = self.create_publisher(Odometry, "/odometry/filtered", qos_profile_keeplast)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.transform_msg = TransformStamped()
        self.odom_msg = Odometry()

        self.foot_frame_name = [prefix + "_foot" for prefix in ["FL", "FR", "RL", "RR"]]
    
    def _unitree_to_urdf_vec(self, vec):
        return  [vec[3],  vec[4],  vec[5],
                 vec[0],  vec[1],  vec[2],
                 vec[9],  vec[10], vec[11],
                 vec[6],  vec[7],  vec[8],]

    def listener_callback(self, state_msg):
        timestamp = self.get_clock().now().to_msg()
        self.transform_msg.header.stamp = timestamp
        self.transform_msg.child_frame_id = "base" #self.get_parameter("base_frame").value
        self.transform_msg.header.frame_id = "odom" #self.get_parameter("odom_frame").value

        self.odom_msg.header.stamp = timestamp
        self.odom_msg.child_frame_id = "base" #self.get_parameter("base_frame").value
        self.odom_msg.header.frame_id = "odom" #self.get_parameter("odom_frame").value

        # Get sensor measurement
        q_unitree = [j.q for j in state_msg.motor_state[:12]]
        v_unitree = [j.dq for j in state_msg.motor_state[:12]]
        fc_unitree = state_msg.foot_force
        gyro_measure = [float(state_msg.imu_state.gyroscope[0]),
                        float(state_msg.imu_state.gyroscope[1]),
                        float(state_msg.imu_state.gyroscope[2])]

        # Get true rotation
        true_quat = np.array([-float(state_msg.imu_state.quaternion[1]), 
                              -float(state_msg.imu_state.quaternion[2]),
                              -float(state_msg.imu_state.quaternion[3]),
                              -float(state_msg.imu_state.quaternion[0])
        ])

        # Rearrange joints according to urdf
        q = np.array([0]*6 + [1] + self._unitree_to_urdf_vec(q_unitree))
        q[3:7] = true_quat
        v = np.array([0]*6 + self._unitree_to_urdf_vec(v_unitree))
        f_contact = [fc_unitree[i] for i in [1, 0, 3, 2]]

        # Compute positions and velocities
        ## f = foot, i = imu, b = base
        self.robot.forwardKinematics(q, v)
        pin.updateFramePlacements(self.robot.model, self.robot.data)
        bMf_list = [self.robot.data.oMf[id] for id in self.foot_frame_id]
        Vf_list = [pin.getFrameVelocity(self.robot.model, self.robot.data, id, pin.LOCAL) for id in self.foot_frame_id]

        # Make message
        Vf_in_contact = np.array([0.0, 0.0, 0.0])
        Bf_in_contact = np.array([0.0, 0.0, 0.0])
        nb_contact = 0
        for i in range(4):
            if(f_contact[i]<20):
                continue # Feet in the air : skip
            Vf_in_contact -= Vf_list[i].linear
            Bf_in_contact -= bMf_list[i].translation
            nb_contact += 1
        
        if nb_contact > 0:
            Vf_in_contact = Vf_in_contact / nb_contact
            Bf_in_contact = Bf_in_contact / nb_contact

        # ROS2 transform
        self.transform_msg.transform.translation.x = 0.0
        self.transform_msg.transform.translation.y = 0.0
        self.transform_msg.transform.translation.z = Bf_in_contact[2]

        self.odom_msg.pose.pose.position.x = 0.0
        self.odom_msg.pose.pose.position.y = 0.0
        self.odom_msg.pose.pose.position.z = Bf_in_contact[2]

        #self.get_logger().info('new quat' + str(quat))

        self.transform_msg.transform.rotation.x = true_quat[0]
        self.transform_msg.transform.rotation.y = true_quat[1]
        self.transform_msg.transform.rotation.z = true_quat[2]
        self.transform_msg.transform.rotation.w = true_quat[3] 

        self.odom_msg.pose.pose.orientation.x = true_quat[0]
        self.odom_msg.pose.pose.orientation.y = true_quat[1]
        self.odom_msg.pose.pose.orientation.z = true_quat[2]
        self.odom_msg.pose.pose.orientation.w = true_quat[3] 

        self.odom_msg.twist.twist.linear.x = Vf_in_contact[0]
        self.odom_msg.twist.twist.linear.y = Vf_in_contact[1]
        self.odom_msg.twist.twist.linear.z = Vf_in_contact[2]

        self.odom_msg.twist.twist.angular.x = gyro_measure[0]
        self.odom_msg.twist.twist.angular.y = gyro_measure[1]
        self.odom_msg.twist.twist.angular.z = gyro_measure[2]
        

        self.tf_broadcaster.sendTransform(self.transform_msg)
        self.odom_publisher.publish(self.odom_msg)


def main(args=None):
    rclpy.init(args=args)

    odom_node = DumbOdom()
    
    rclpy.spin(odom_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odom_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
