#!/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from go2_odometry.msg import OdometryVector
import pinocchio as pin

import copy
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from inekf import (
    RobotState,
    NoiseParams,
    InEKF,
    Kinematics
)

# ==============================================================================
# Main Class
# ==============================================================================
class Inekf(Node):

    def __init__(self):
        super().__init__('inekf')
        print("==GO2 InEKF launched==")
        # ROS2 =================================================================
        self.declare_parameter("base_frame", "base")
        self.declare_parameter("odom_frame", "odom")
        
        qos_profile_keeplast = QoSProfile(history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
        self.pos_feet_subscriber = self.create_subscription(OdometryVector,
                                                            'odometry/feet_pos', 
                                                            self.listener_feet_callback,
                                                            qos_profile_keeplast)
        self.imu_subscription = self.create_subscription(Imu,
                                                        '/imu',
                                                        self.listener_callback,
                                                        qos_profile_keeplast)
        
        self.odom_publisher = self.create_publisher(Odometry, "/odometry/filtered", qos_profile_keeplast)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.transform_msg = TransformStamped()
        self.odom_msg = Odometry()

        self.foot_frame_name = [prefix + "_foot" for prefix in ["FL", "FR", "RL", "RR"]]
        self.foot_frame_ids = [0, 1, 2, 3]
        
        # Filter and global variables ==========================================
        self.DT_MIN = 1e-6
        self.DT_MAX = 1
        self.dt = 0


        # Data from go2 ========================================================
        self.imu_measurement = np.zeros(6)
        self.imu_measurement_prev = np.zeros(6)
        self.quaternion_measurement = np.zeros(4)
        self.feet_contacts = np.zeros(4)
        self.contacts={0:False,
                       1:False,
                       2:False,
                       3:False}
        
        self.joints_unitree_2_urdf = [1,0,3,2] # index = urdf convention, value = unitree joint numbering
        
        self.estimated_contact_positions = {0:0, #! not sure if correct
                                                1:0,
                                                2:0,
                                                3:0}
        
        # double = float in python
        self.t : float = 0
        self.t_prev : float = 0

        # Invariant EKF
        initial_state = RobotState()

        R0 = np.array([[1, 0, 0],
               [0, 1, 0],
               [0, 0, 1]]) # Initial rotation 
        
        v0 = np.zeros(3) # initial velocity
        p0 = np.array([0, 0, 0.07]) # initial position in simulation
        bg0 = np.zeros(3) # initial gyroscope bias
        ba0 = np.zeros(3) # initial accelerometer bias
        gravity = np.array([0, 0, -9.81])

        initial_state.setRotation(R0)
        initial_state.setVelocity(v0)
        initial_state.setPosition(p0)
        initial_state.setGyroscopeBias(bg0)
        initial_state.setAccelerometerBias(ba0)

        # Initialize state covariance
        noise_params = NoiseParams()
        noise_params.setGyroscopeNoise(0.01)
        noise_params.setAccelerometerNoise(0.1)
        noise_params.setGyroscopeBiasNoise(0.00001)
        noise_params.setAccelerometerBiasNoise(0.0001)
        noise_params.setContactNoise(0.001)

        self.filter = InEKF(initial_state, noise_params)
        self.filter.setGravity(gravity)

    def listener_feet_callback(self, state_msg):
        self.t = state_msg.header.stamp.sec + state_msg.header.stamp.nanosec * 1e-9 

        pose_vec = state_msg.pose_vec
        vel_vec = state_msg.vel_vec
        contact_states = state_msg.contact_states
        #self.get_logger().info('Feet in contact ' + str(state_msg.feet_names))
        
        contact_list = []
        kinematics_list = []

        current_base_rotation = self.filter.getState().getRotation()

        for i in range(len(self.foot_frame_name)):
            contact_list.append((i, contact_states[i]))
            pose = pose_vec[i]

            full_cov = pose.covariance[:9]
            full_cov = np.squeeze(full_cov)
            full_cov = full_cov.reshape(3,3)
            
            quat = pin.Quaternion(pose.pose.orientation.w,
                                  pose.pose.orientation.x, 
                                  pose.pose.orientation.y,
                                  pose.pose.orientation.z)
            quat.normalize()

            translation = np.zeros(3)
            translation[0] = pose.pose.position.x
            translation[1] = pose.pose.position.y
            translation[2] = pose.pose.position.z


            contact_cov = pose.covariance[:9]
            contact_cov = np.squeeze(contact_cov)
            contact_cov = contact_cov.reshape(3,3)

            contact_velocity = np.zeros(3)
            contact_velocity[0] = vel_vec[i].x
            contact_velocity[1] = vel_vec[i].y
            contact_velocity[2] = vel_vec[i].z
            contact_velocity = current_base_rotation.transpose() @ contact_velocity
            
            # Only if vel_foot is an approximate of velocity base (no sliding)
            """ vel_foot_in_local = vel_foot + np.cross(self.imu_measurement[:3], translation)
            vel_foot_in_local = current_base_rotation @ vel_foot_in_local

            velocity_base = -vel_foot_in_local """
            cov_vel_base = np.eye(3) * 0.001

            cov_vel_slip = np.eye(3) * 1e-6

            kinematics = Kinematics(i, translation, full_cov, np.zeros(3), cov_vel_base, contact_velocity, cov_vel_slip)
            #kinematics = Kinematics(i, translation, full_cov, np.zeros(3), cov_vel_base)
            kinematics_list.append(kinematics)

            #self.get_logger().info('Base contact of ' + self.foot_frame_name[i] + ' is ' + str(pose_matrix[:3,3]))
            #self.get_logger().info('Rotation of ' + self.foot_frame_name[i] + ' is ' + str(pose_matrix[:3,:3]))
        
        #self.get_logger().info('Contact list' + str(contact_list))
        self.filter.setContacts(contact_list)
        self.filter.correctKinematics(kinematics_list)


    def listener_callback(self, state_msg):
        # TODO verify if timestamp can be used as time
        self.t = state_msg.header.stamp.sec + state_msg.header.stamp.nanosec * 1e-9 
        self.dt = self.t - self.t_prev
    
        # IMU measurement - used for propagation ===============================
        #! gyroscope meas in filter are radians
        self.imu_measurement[0] = state_msg.angular_velocity.x
        self.imu_measurement[1] = state_msg.angular_velocity.y
        self.imu_measurement[2] = state_msg.angular_velocity.z

        self.imu_measurement[3] = state_msg.linear_acceleration.x
        self.imu_measurement[4] = state_msg.linear_acceleration.y
        self.imu_measurement[5] = state_msg.linear_acceleration.z

        # breakpoint()
        if(self.dt > self.DT_MIN and self.dt < self.DT_MAX):
            #propagate using previous measurement
            self.propagate()

        # KINEMATIC data =======================================================
        self.quaternion_measurement[0] = state_msg.orientation.x
        self.quaternion_measurement[1] = state_msg.orientation.y
        self.quaternion_measurement[2] = state_msg.orientation.z
        self.quaternion_measurement[3] = state_msg.orientation.w
        # TODO normalize quaternion && find quat type


        # TODO: add feet vel and feet pos (additionnal info on base)
        self.t_prev = self.t
        self.imu_measurement_prev = self.imu_measurement

    def propagate(self):
        # Transform from odom_frame (unmoving) to base_frame (tied to robot base)
        timestamp = self.get_clock().now().to_msg()
        self.transform_msg.header.stamp = timestamp
        self.transform_msg.child_frame_id = "base" #self.get_parameter("base_frame").value
        self.transform_msg.header.frame_id = "odom" #self.get_parameter("odom_frame").value

        self.odom_msg.header.stamp = timestamp
        self.odom_msg.child_frame_id = "base" #self.get_parameter("base_frame").value
        self.odom_msg.header.frame_id = "odom" #self.get_parameter("odom_frame").value
        
        self.filter.propagate(self.imu_measurement_prev, self.dt)
        
        new_state = self.filter.getState()
        new_r = new_state.getRotation()
        new_p = new_state.getPosition()
        new_v = new_r.T @ new_state.getX()[0:3,3:4]
        new_v = new_v.reshape(-1)
        #self.get_logger().info('imu measure ' + str(self.imu_measurement_prev))

        quat = pin.Quaternion(new_r)
        quat.normalize()
        
        # ROS2 transform
        self.transform_msg.transform.translation.x = new_p[0]
        self.transform_msg.transform.translation.y = new_p[1]
        self.transform_msg.transform.translation.z = new_p[2]

        self.odom_msg.pose.pose.position.x = new_p[0]
        self.odom_msg.pose.pose.position.y = new_p[1]
        self.odom_msg.pose.pose.position.z = new_p[2]


        self.transform_msg.transform.rotation.x = quat.x
        self.transform_msg.transform.rotation.y = quat.y
        self.transform_msg.transform.rotation.z = quat.z
        self.transform_msg.transform.rotation.w = quat.w 

        self.odom_msg.pose.pose.orientation.x = quat.x
        self.odom_msg.pose.pose.orientation.y = quat.y
        self.odom_msg.pose.pose.orientation.z = quat.z
        self.odom_msg.pose.pose.orientation.w = quat.w 

        self.odom_msg.twist.twist.linear.x = new_v[0]
        self.odom_msg.twist.twist.linear.y = new_v[1]
        self.odom_msg.twist.twist.linear.z = new_v[2]

        self.odom_msg.twist.twist.angular.x = self.imu_measurement_prev[0]
        self.odom_msg.twist.twist.angular.y = self.imu_measurement_prev[1]
        self.odom_msg.twist.twist.angular.z = self.imu_measurement_prev[2]
        

        self.tf_broadcaster.sendTransform(self.transform_msg)
        self.odom_publisher.publish(self.odom_msg)


    # Attributes ===============================================================

    estimated_landmarks = {} # int:int
    contacts = {} # int:bool
    estimated_contact_positions = {} # int:int


def main(args=None):
    rclpy.init(args=args)

    inekf_node = Inekf()
    
    rclpy.spin(inekf_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    inekf_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
