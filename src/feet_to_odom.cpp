#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>

#include <unitree_go/msg/low_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "go2_odometry/msg/odometry_vector.hpp"

class FeetToOdom : public rclcpp::Node
{
public:
    explicit FeetToOdom()
    : Node("inekf")
    {
        rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
        qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        
        // Write subscribers to feet odometry and IMU
        pos_feet_publisher_ = this->create_publisher<go2_odometry::msg::OdometryVector>(
            "odometry/feet_pos", 
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos)
        );
        
        lowstate_subscriber_ = this->create_subscription<unitree_go::msg::LowState>(
            "lowstate", 
            10, 
            std::bind(&FeetToOdom::listener_callback, this, std::placeholders::_1)
        );

        unitree_to_urdf_vec_ = {3, 4, 5,
            0, 1, 2,
            9, 10, 11,
            6, 7, 8};
        
        force_order_ = {1, 0, 3, 2};

        q_.resize(19);
        q_.setZero();
        q_[6] = 1;
        f_contact_.resize(4);
        f_contact_.setZero();

        // Load robot
        std::string urdf_path = GO2_DESCRIPTION_MODEL_DIR "/go2.urdf";
        std::string srdf_path = GO2_DESCRIPTION_MODEL_DIR "/go2.srdf";
        std::string base_joint_name ="root_joint";
        
        pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), model_);
        data_ = pinocchio::Data(model_);
        
        foot_frame_names_ = {"FL_foot", "FR_foot", "RL_foot", "RR_foot"};
        for (auto it = foot_frame_names_.cbegin(); it != foot_frame_names_.cend(); ++it) {
            foot_frame_ids_.push_back(model_.getFrameId(*it));
        }
    }

    void listener_callback(const unitree_go::msg::LowState::SharedPtr msg)
    {   
        // Rearrange joints according to urdf
        for (std::size_t i = 0; i < 12; i ++) {
            q_[7 + i] = msg->motor_state[unitree_to_urdf_vec_[i]].q;
        }
        for (std::size_t i = 0; i < 4; i ++) {
            f_contact_[i] = msg->foot_force[force_order_[i]];
        }

        // Compute positions and velocities
        // f = foot, i = imu, b = base
        pinocchio::forwardKinematics(model_, data_, q_);
        pinocchio::updateFramePlacements(model_, data_);
        std::vector<pinocchio::SE3> bMf_list;
        for (auto it = foot_frame_ids_.cbegin(); it != foot_frame_ids_.cend(); ++it) {
            bMf_list.push_back(data_.oMf[*it]);
        }
        
        // Create message
        go2_odometry::msg::OdometryVector pose_msg;
        pose_msg.header.stamp = this->get_clock()->now();

        std::vector<geometry_msgs::msg::PoseWithCovariance> pose_list;
        std::vector<bool> contact_states;
        for (std::size_t i = 0; i < 4; i++) {
            if (f_contact_[i] > 20) contact_states.push_back(true);
            else contact_states.push_back(false);

            geometry_msgs::msg::PoseWithCovariance pose_foot;
            pose_foot.covariance.fill(0);

            pose_foot.pose.position.x = bMf_list[i].translation()[0];
            pose_foot.pose.position.y = bMf_list[i].translation()[1];
            pose_foot.pose.position.z = bMf_list[i].translation()[2];

            Eigen::Quaternion<double> quat(bMf_list[i].rotation());
            
            pose_foot.pose.orientation.x = quat.x();
            pose_foot.pose.orientation.y = quat.y();
            pose_foot.pose.orientation.z = quat.z();
            pose_foot.pose.orientation.w = quat.w();

            pose_list.push_back(pose_foot);
        }
        pose_msg.contact_states = contact_states;
        pose_msg.pose_vec = pose_list;

        pos_feet_publisher_->publish(pose_msg);
    }

    rclcpp::Publisher<go2_odometry::msg::OdometryVector>::SharedPtr pos_feet_publisher_;
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_subscriber_;

    std::vector<long> unitree_to_urdf_vec_;
    std::vector<long > force_order_;
    Eigen::VectorXd q_;
    Eigen::VectorXd f_contact_;
    std::vector<std::size_t> foot_frame_ids_;
    std::vector<std::string> foot_frame_names_;
    pinocchio::Model model_;
    pinocchio::Data data_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FeetToOdom>());
  rclcpp::shutdown();
  return 0;
}