/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   kinematics.cpp
 *  @author Ross Hartley
 *  @brief  Example of invariant filtering for contact-aided inertial navigation
 *  @date   September 25, 2018
 **/

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "InEKF.h"

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "nav_msgs/msg/odometry.hpp"


#define DT_MIN 1e-6
#define DT_MAX 1

using namespace std;
using namespace inekf;


class InekfNode : public rclcpp::Node
{
    public:
    InekfNode()
    : Node("inekf_node")
    {
        
        // // ROS2 Init
        lowstate_subscription_ = this->create_subscription<unitree_go::msg::LowState>("lowstate", 10, std::bind(&InekfNode::lowstate_callback_, this, std::placeholders::_1));

        // // Initialize noise parameters
        // NoiseParams noise_params;
        // noise_params.setGyroscopeNoise(0.01);
        // noise_params.setAccelerometerNoise(0.1);
        // noise_params.setGyroscopeBiasNoise(0.00001);
        // noise_params.setAccelerometerBiasNoise(0.0001);
        // noise_params.setContactNoise(0.01);


        // Initialize initial state

        Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(5,5);
        Eigen::MatrixXd theta = Eigen::MatrixXd::Zero(6,1);
        Eigen::MatrixXd P = Eigen::MatrixXd::Identity(15,15);

        Eigen::MatrixXd v(3,3);

        v << 1, 2, 3, 4, 5, 6,7,8,9;
        //  "v.head(3) =" << endl << v.head(3) << endl << endl;
        // cout << "v.tail<3>() = " << endl << v.tail<3>() << endl << endl;
        // RCLCPP_INFO_STREAM(this->get_logger(),v.segment(1,4));
        // cout << "after 'v.segment(1,4) *= 2', v =" << endl << v << endl;
        // RCLCPP_INFO_STREAM(this->get_logger(),identity);

        // RCLCPP_INFO_STREAM(this->get_logger(),identity);
        // RCLCPP_INFO_STREAM(this->get_logger(),theta);
        // RCLCPP_INFO_STREAM(this->get_logger(),P);

        //auto* state = new RobotState(); //allocation sur la pile (Stack) //! delete state
         //! understand why this works and below does not -> see what is the diff?
        // RobotState state(identity,theta,P); //? Allocation sur le tas (Heap)


        // RCLCPP_INFO_STREAM(this->get_logger(),(*state).getX());
        //* All zeroes for now
        // Eigen::Matrix3d R_init = Eigen::Matrix3d::Identity(); // initial orientation
        // Eigen::Vector3d v_init = Eigen::Vector3d::Zero();     // initial velocity
        // Eigen::Vector3d p_init = Eigen::Vector3d::Zero();     // initial position
        // Eigen::Vector3d bg_init = Eigen::Vector3d::Zero();    // initial gyroscope bias
        // Eigen::Vector3d ba_init = Eigen::Vector3d::Zero();    // initial accelerometer bias
        // Eigen::Matrix<double,15,15> P_init = Eigen::Matrix<double,15,15>::Zero();

        // state.setRotation(R_init);
        // state.setVelocity(v_init);
        // state.setPosition(p_init);
        // state.setGyroscopeBias(bg_init);
        // state.setAccelerometerBias(ba_init);
        // state.setP(P_init);
        // filter_.setState(state);


        // Initialize filter

        // RCLCPP_INFO_STREAM(this->get_logger(),"Noise parameters are initialized to: \n"
        // << filter_.getNoiseParams()
        // << "Robot's state is initialized to: \n" 
        // << filter_.getState());

    }

    private:

        void lowstate_callback_(const unitree_go::msg::LowState::SharedPtr msg)
        { // fills imu and foot contact values
            // this->imu_measurement_ << msg->imu_state.gyroscope[0], //angular vel x
            //                          msg->imu_state.gyroscope[1], //angular vel y
            //                          msg->imu_state.gyroscope[2], //angular vel z

            //                          msg->imu_state.accelerometer[0], // linear acc x
            //                          msg->imu_state.accelerometer[1], // linear acc y
            //                          msg->imu_state.accelerometer[2]; // linear acc z

            // double dt = this->t_ - this->t_prev_;
            // if (dt > DT_MIN && dt < DT_MAX) 
            // {
            //     this->filter_.Propagate(imu_measurement_prev_, dt);
            // }

            vector<pair <int,bool> > contacts; //? move to class attribute?
            bool foot_contact;

            // RCLCPP_INFO_STREAM(this->get_logger(),"Foot force " << msg->foot_force[0] << " " << msg->foot_force[1] << " " <<msg->foot_force[2] << " " << msg->foot_force[3]);
            // //! f_contact = [fc_unitree[i] for i in [1, 0, 3, 2]] dans feet_to_odom: ordre a modifier pour matcher urdf
            for(int foot_id = 0; foot_id < 4 ; foot_id++ ) 
            {
                foot_contact = (msg->foot_force[foot_id] > 20) ? 1 : 0; // converting float foot contact to bool
                contacts.push_back(pair<int,bool> (foot_id, foot_contact)); 
            }

            RCLCPP_INFO_STREAM(this->get_logger(),"Foot contact " << contacts[0].first << contacts[0].second << " " << contacts[1].first << contacts[1].second << " " << contacts[2].first << contacts[2].second << " " << contacts[3].first << contacts[3].second);
            // this->filter_.setContacts(contacts);
     

        }


        // // Kalman variables
        InEKF filter_; // pointer to the filter so it can be initialized in the init
        Eigen::Matrix<double,6,1> imu_measurement_;
        Eigen::Matrix<double,6,1> imu_measurement_prev_;
        double t_ = 0;
        double t_prev_ = 0;

        // // ROS 2
        rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_subscription_;
		// rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_; 
        // //subscriber to kinetics? -> tf listener?
        // //publisher of robot_state

};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InekfNode>());
  rclcpp::shutdown();
  return 0;
}
