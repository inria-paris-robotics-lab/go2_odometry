#!/bin/env python3

import math
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from go2_odometry.msg import OdometryVector

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
        self.imu_measurement = np.zeros((6,1))
        self.imu_measurement_prev = np.zeros((6,1))
        self.quaternion_measurement = np.zeros((4))
        self.quaternion = np.zeros((4,1))
        self.feet_contacts = np.zeros((4))
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
        
        quat_init = np.array([-0.00588, 0.0416, -0.00319,  0.999])
        #quat_init = np.array([-0.00111727, -0.0520058, -0.02145091,  0.99841575])
        rot_init = Rotation.from_quat(quat_init)
        
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
        feet_names_in_contact = state_msg.feet_names
        #self.get_logger().info('Feet in contact ' + str(state_msg.feet_names))
        
        contact_list = []
        kinematics_list = []

        for i in range(len(self.foot_frame_name)):
            if self.foot_frame_name[i] in feet_names_in_contact:
                
                contact_list.append((i, True))
                pose = pose_vec[i]
                
                r = Rotation.from_quat([pose.pose.orientation.x, 
                                        pose.pose.orientation.y,
                                        pose.pose.orientation.z,
                                        pose.pose.orientation.w])

                pose_matrix = np.eye(4)
                pose_matrix[:3,:3] = r.as_matrix()
                pose_matrix[0,3] = pose.pose.position.x
                pose_matrix[1,3] = pose.pose.position.y
                pose_matrix[2,3] = pose.pose.position.z

                kinematics = Kinematics()
                kinematics.id = i
                kinematics.pose = pose_matrix
                kinematics.covariance = np.eye(6) * 0.01 #pose.covariance

                kinematics_list.append(kinematics)

                #self.get_logger().info('Base contact of ' + self.foot_frame_name[i] + ' is ' + str(pose_matrix[:3,3]))
                #self.get_logger().info('Rotation of ' + self.foot_frame_name[i] + ' is ' + str(pose_matrix[:3,:3]))
        
        #self.get_logger().info('Contact list' + str(contact_list))
        self.filter.setContacts(contact_list)
        self.filter.correctKinematics(kinematics_list)
        new_pose = self.filter.getState().getPosition()
        #if len(kinematics_list) > 0:
            #for i in range(len(kinematics_list)):
                #new_c = self.filter.getState().getX()[:3,5+i]
                #self.get_logger().info('World contact ' + str(i) + ' is ' + str(new_c))
        #self.get_logger().info('Correct kinematics ' + str(new_pose))
        """ if len(kinematics_list) > 0:
            exit() """
        if np.isnan(new_pose[0]):
            exit()




    def listener_callback(self, state_msg):
        # TODO verify if timestamp can be used as time
        self.t = state_msg.header.stamp.sec + state_msg.header.stamp.nanosec * 1e-9 
        self.dt = self.t - self.t_prev
    
        # IMU measurement - used for propagation ===============================
        #! gyroscope meas in filter are radians
        self.imu_measurement[0][0] = state_msg.angular_velocity.x
        self.imu_measurement[1][0] = state_msg.angular_velocity.y
        self.imu_measurement[2][0] = state_msg.angular_velocity.z

        self.imu_measurement[3][0] = state_msg.linear_acceleration.x
        self.imu_measurement[4][0] = state_msg.linear_acceleration.y
        self.imu_measurement[5][0] = state_msg.linear_acceleration.z

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
        new_v = new_state.getX()[0:3,3:4]
        #self.get_logger().info('imu measure ' + str(self.imu_measurement_prev))
        #self.get_logger().info('Position ' + str(new_p))

        r = Rotation.from_matrix([[new_r[0][0], new_r[0][1], new_r[0][2]],
                                  [new_r[1][0], new_r[1][1], new_r[1][2]],
                                  [new_r[2][0], new_r[2][1], new_r[2][2]]])
        
        # ROS2 transform
        self.transform_msg.transform.translation.x = new_p[0]
        self.transform_msg.transform.translation.y = new_p[1]
        self.transform_msg.transform.translation.z = new_p[2]

        self.odom_msg.pose.pose.position.x = new_p[0]
        self.odom_msg.pose.pose.position.y = new_p[1]
        self.odom_msg.pose.pose.position.z = new_p[2]

        x,y,z,w = r.as_quat()
        #print("RPY rotation:", r.as_euler("xyz",degrees=True))
        quat = np.array([x,y,z,w])
        #self.get_logger().info('new quat' + str(quat))

        self.transform_msg.transform.rotation.x = x
        self.transform_msg.transform.rotation.y = y
        self.transform_msg.transform.rotation.z = z
        self.transform_msg.transform.rotation.w = w 

        self.odom_msg.pose.pose.orientation.x = x
        self.odom_msg.pose.pose.orientation.y = y
        self.odom_msg.pose.pose.orientation.z = z
        self.odom_msg.pose.pose.orientation.w = w 

        self.odom_msg.twist.twist.linear.x = float(new_v[0])
        self.odom_msg.twist.twist.linear.y = float(new_v[1])
        self.odom_msg.twist.twist.linear.z = float(new_v[2])

        self.odom_msg.twist.twist.angular.x = float(self.imu_measurement_prev[0])
        self.odom_msg.twist.twist.angular.y = float(self.imu_measurement_prev[1])
        self.odom_msg.twist.twist.angular.z = float(self.imu_measurement_prev[2])
        

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
