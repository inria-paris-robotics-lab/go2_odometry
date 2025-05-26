#!/bin/env python3

import rclpy
from rclpy.node import Node

from go2_odometry.msg import OdometryVector
from unitree_go.msg import LowCmd, LowState
from geometry_msgs.msg import PoseWithCovariance, Vector3
from nav_msgs.msg import Odometry

import copy
import numpy as np
import pinocchio as pin
from go2_description.loader import loadGo2
import coal
from simulation_args import SimulationArgs
from simulation_utils import (
    removeBVHModelsIfAny,
    setPhysicsProperties,
    Simulation,
    addSystemCollisionPairs
)

class FeetToOdom(Node):

    def __init__(self):
        super().__init__('feet_to_odom')

        self.pos_publisher = self.create_publisher(OdometryVector, 'odometry/feet_pos', 10)
        self.subscription = self.create_subscription(
            LowState,
            '/lowstate',
            self.listener_callback,
            10)
        self.cmd_subscription =  self.create_subscription(LowCmd, "/lowcmd", self.listener_callback_cmd, 10)
        #self.odom_subscription =  self.create_subscription(Odometry, "/odometry/filtered", self.listener_callback_odom, 10)
        self.tau_cmd = np.zeros(12)
        self.estimate_force = np.zeros(12)

        self.qbase = np.zeros(7)
        self.vbase = np.zeros(6)
        self.qbase[6] = 1

        self.robot = loadGo2()
        self.rmodel = self.robot.model
        for i in range(self.rmodel.nq):
            self.rmodel.lowerPositionLimit[i] = np.finfo("d").min
            self.rmodel.upperPositionLimit[i] = np.finfo("d").max
        self.rmodel.lowerDryFrictionLimit[:] = 0
        self.rmodel.upperDryFrictionLimit[:] = 0
        self.rmodel.damping[:] = 0

        self.foot_frame_name = [prefix + "_foot" for prefix in ["FL", "FR", "RL", "RR"]]
        self.foot_frame_id = [self.rmodel.getFrameId(frame_name) for frame_name in self.foot_frame_name]
        self.imu_frame_id = self.rmodel.getFrameId("imu")
        self.initialize_pose = True
        
        geom_model = self.robot.collision_model
        visual_model = self.robot.visual_model
        self.rdata = self.rmodel.createData()
        self.feet_names = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]
        
        ### Force estimation via Simple
        # Set Simulator
        q0 = np.array([ 0.   ,  0.   ,  0,  0.   ,  0.   ,  0.   ,  1.   ,  
                       0.068, 0.785, -1.44 , -0.068,  0.785, -1.44 ,  
                       0.068,  0.785, -1.44 ,-0.068,  0.785, -1.44 ])
        self.q_previous = copy.deepcopy(q0)
        self.v_previous = np.zeros(18)
        args = SimulationArgs()
        setPhysicsProperties(geom_model, args.material, args.compliance)
        removeBVHModelsIfAny(geom_model)
        addSystemCollisionPairs(self.rmodel, geom_model, q0)

        self.contact_modes = [#[1, 1, 1, 1],
                 [0, 0, 0, 0],
                 [1, 0, 0, 0],
                 [1, 1, 0, 0],
                 #[1,1,1,0],
                 #[0,1,1,1],
                 [0,0,1,1],
                 [0,0,0,1],
                 [0,1,1,0],
                 [1,0,0,1],
                 [0,1,0,1],
                 [1,0,1,0],
                 [0,0,1,0],
                 [0,1,0,0],
                 #[1,0,1,1],
                 #[1,1,0,1]               
        ]
        self.fext = [pin.Force(np.zeros(6)) for _ in range(self.robot.model.njoints)]
        self.dt_sim = 1e-2

        # Remove all pair of collision
        i = 0
        while i < len(geom_model.collisionPairs):
            cp = geom_model.collisionPairs[i]
            geom_model.removeCollisionPair(cp)
        
        # Add one collision pair per foot
        pin.forwardKinematics(self.rmodel, self.rdata, q0)
        pin.updateFramePlacements(self.rmodel, self.rdata)
        universe_fid = self.rmodel.getFrameId("universe")
        universe_jid = self.rmodel.frames[universe_fid].parentJoint
        self.radius = 0.01
        for name in self.feet_names:
            foot_id = self.rmodel.getFrameId(name)
            geom_name = name + "_0"
            geom_id = geom_model.getGeometryId(geom_name)
            pose = self.rdata.oMf[foot_id]
            pose_object = pin.SE3.Identity()
            pose_object.translation = pose.translation
            pose_object.translation[2] -= self.radius
            geom_object = pin.GeometryObject(
                "sphere_" + name, universe_fid, universe_jid, coal.Sphere(self.radius), pose_object
            )

            ig_frame = geom_model.addGeometryObject(geom_object)
            geom_model.addCollisionPair(pin.CollisionPair(ig_frame, geom_id))
        
        # Create the simulator objectconstraints_forces
        self.simulator = Simulation(self.rmodel, geom_model, visual_model, q0, np.zeros(self.rmodel.nv), args) 


    def _unitree_to_urdf_vec(self, vec):
        return  [vec[3],  vec[4],  vec[5],
                 vec[0],  vec[1],  vec[2],
                 vec[9],  vec[10], vec[11],
                 vec[6],  vec[7],  vec[8],]
    
    def setContactPose(self, feet_translation, z_offsets = [0, 0, 0, 0]):
        ij = 0
        for name in self.feet_names:
            geom_name = "sphere_" + name
            feet_trans = copy.deepcopy(feet_translation[ij])
            feet_trans[2] -= self.radius
            feet_trans[2] -= z_offsets[ij]
            geom_id = self.simulator.simulator.geom_model.getGeometryId(geom_name)
            self.simulator.simulator.geom_model.geometryObjects[geom_id].placement.translation = feet_trans
            ij += 1
    
    def listener_callback_cmd(self, cmd_msg):
        tau_unitree = [j.tau for j in cmd_msg.motor_cmd]
        self.tau_cmd = np.array(self._unitree_to_urdf_vec(tau_unitree))
    
    """ def listener_callback_odom(self, odom_msg):
        self.qbase[0] = odom_msg.pose.pose.position.x
        self.qbase[1] = odom_msg.pose.pose.position.y
        self.qbase[2] = odom_msg.pose.pose.position.z
        self.qbase[3] = odom_msg.pose.pose.orientation.x
        self.qbase[4] = odom_msg.pose.pose.orientation.y
        self.qbase[5] = odom_msg.pose.pose.orientation.z
        self.qbase[6] = odom_msg.pose.pose.orientation.w
        
        self.vbase[0] = odom_msg.twist.twist.linear.x
        self.vbase[1] = odom_msg.twist.twist.linear.y
        self.vbase[2] = odom_msg.twist.twist.linear.z
        self.vbase[3] = odom_msg.twist.twist.angular.x
        self.vbase[4] = odom_msg.twist.twist.angular.y
        self.vbase[5] = odom_msg.twist.twist.angular.z """



    def listener_callback(self, state_msg):
        # Get sensor measurement
        q_unitree = [j.q for j in state_msg.motor_state[:12]]
        v_unitree = [j.dq for j in state_msg.motor_state[:12]]
        tau_unitree = [j.tau_est for j in state_msg.motor_state[:12]]
        fc_unitree = state_msg.foot_force

        # Rearrange joints according to urdf
        q = np.array([0]*6 + [1] + self._unitree_to_urdf_vec(q_unitree))
        v = np.array([0]*6 + self._unitree_to_urdf_vec(v_unitree))
        tau = self._unitree_to_urdf_vec(tau_unitree)
        #qworld = np.concatenate((self.qbase, self._unitree_to_urdf_vec(q_unitree)))
        #vworld = np.concatenate((self.vbase, self._unitree_to_urdf_vec(v_unitree)))
        #x_measured = np.concatenate((qworld, vworld))
        f_contact = [fc_unitree[i] for i in [1, 0, 3, 2]]

        if np.max(self.tau_cmd) > 1:
            self.initialize_pose = False
            #self.get_logger().info('Initialization over')

        # Compute positions and velocities constraints_forces
        ## f = foot, i = imu, b = base
        pin.forwardKinematics(self.rmodel, self.rdata, q, v)
        pin.updateFramePlacements(self.rmodel, self.rdata)
        pin.computeJointJacobians(self.rmodel, self.rdata)
        bMf_list = [self.rdata.oMf[id] for id in self.foot_frame_id]
        bMf_trans_list = [self.rdata.oMf[id].translation for id in self.foot_frame_id]
        foot_vel_list = []
        for i in range(4):
            vpin = pin.getFrameVelocity(self.rmodel, self.rdata, self.foot_frame_id[i], pin.LOCAL_WORLD_ALIGNED)
            foot_vel_list.append(vpin.linear)
        
        """ pin.forwardKinematics(self.rmodel, self.rdata, qworld)
        pin.updateFramePlacements(self.rmodel, self.rdata)
        world_feet_pose = [self.rdata.oMf[id].translation for id in self.foot_frame_id] """


        ### Estimate force and contact
        torque_simple = np.zeros(18)
        torque_simple[6:] = self.tau_cmd #tau
        
        self.setContactPose(bMf_trans_list)
        self.simulator.simulator.stepPGS(q, v, torque_simple, self.fext, self.dt_sim)
        forces_simple = self.simulator.simulator.constraints_problem.frictional_point_constraints_forces
        col_pairs = self.simulator.simulator.constraints_problem.pairs_in_collision

        id_count = 0
        m_force = np.zeros(12)
        for i in range(4):
            if i in col_pairs:
                m_force[i * 3:i * 3 + 3] = forces_simple[id_count:id_count + 3]
                id_count += 3

        """ col_pairs_list = []
        force_simple_list = []
        norm_diff_v = []
        
        for cm in self.contact_modes:
            self.setContactPose(bMf_trans_list, cm)
            self.simulator.simulator.stepPGS(self.q_previous, self.v_previous, torque_simple, self.fext, self.dt_sim)
            forces_simple = self.simulator.simulator.constraints_problem.frictional_point_constraints_forces
            col_pairs = self.simulator.simulator.constraints_problem.pairs_in_collision

            force_simple_list.append(forces_simple)
            col_pairs_list.append(col_pairs.tolist()[:])
            norm_diff_v.append(np.linalg.norm(self.simulator.simulator.vnew[6:] - v[6:]))
        
        id_min = np.argmin(norm_diff_v)
        m_force = np.zeros(12)
        id_count = 0
        for i in range(4):
            if i in col_pairs_list[id_min]:
                m_force[i * 3:i * 3 + 3] = force_simple_list[id_min][id_count:id_count + 3]
                id_count += 3
        
        self.q_previous = q
        self.v_previous = v """
       
        # Filter force estimates
        b = 0.
        self.estimate_force = (1 - b) * m_force + b * self.estimate_force

        # Make message
        pos_msg = OdometryVector()
        pos_msg.header.stamp = self.get_clock().now().to_msg()
        pos_list = []
        feet_list = []
        force_list = []
        vel_list = []
        for i in range(4):
            if(f_contact[i] >= 20  or self.initialize_pose): #f_contact[i] >= 20  self.estimate_force[2 + i * 3] >= 10
                feet_list.append(True)
            else:
                feet_list.append(False)
            pose_foot = PoseWithCovariance()
            force_foot = Vector3()
            vel_foot = Vector3()

            Jc = pin.getFrameJacobian(self.rmodel, self.rdata, self.foot_frame_id[i], pin.LOCAL)[:3,6:]
            cov_pose = Jc @ np.eye(12) * 2e-6 @ Jc.transpose()
            pose_foot.covariance = [0.] * 36
            pose_foot.covariance[:9] = cov_pose.flatten().tolist()

            pose_foot.pose.position.x = bMf_list[i].translation[0]
            pose_foot.pose.position.y = bMf_list[i].translation[1]
            pose_foot.pose.position.z = bMf_list[i].translation[2]

            force_foot.x = self.estimate_force[i * 3]
            force_foot.y = self.estimate_force[1 + i * 3]
            force_foot.z = self.estimate_force[2 + i * 3]
            
            vel_foot.x = foot_vel_list[i][0]
            vel_foot.y = foot_vel_list[i][1]
            vel_foot.z = foot_vel_list[i][2]
            

            quat = pin.Quaternion(bMf_list[i].rotation)
            quat.normalize()
            
            pose_foot.pose.orientation.x = quat.x
            pose_foot.pose.orientation.y = quat.y
            pose_foot.pose.orientation.z = quat.z
            pose_foot.pose.orientation.w = quat.w

            pos_list.append(pose_foot)
            force_list.append(force_foot)
            vel_list.append(vel_foot)

        pos_msg.contact_states = feet_list
        pos_msg.pose_vec = pos_list
        pos_msg.force_vec = force_list
        pos_msg.vel_vec = vel_list

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