#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>
#include <Eigen/Core>

#include <unitree_go/msg/low_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <inekf/InEKF.hpp>
#include <inekf/RobotState.hpp>

#include "go2_odometry/msg/odometry_vector.hpp"

class InEKFNode : public rclcpp::Node
{
public:
    explicit InEKFNode()
    : Node("inekf")
    {
        this->declare_parameter("base_frame", rclcpp::PARAMETER_STRING);
        this->declare_parameter("odom_frame", rclcpp::PARAMETER_STRING);

        rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
        qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
        qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        
        // Write subscribers to feet odometry and IMU
        pos_feet_subscriber_ = this->create_subscription<go2_odometry::msg::OdometryVector>(
            "odometry/feet_pos", 
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos), 
            std::bind(&InEKFNode::listener_feet_callback, this, std::placeholders::_1)
        );

        lowstate_subscriber_ = this->create_subscription<unitree_go::msg::LowState>(
            "lowstate", 
            10, 
            std::bind(&InEKFNode::listener_lowstate_callback, this, std::placeholders::_1)
        );
        
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu", 
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos), 
            std::bind(&InEKFNode::listener_callback, this, std::placeholders::_1)
        );
        
        // Write publishers and TF
        odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "odometry/filtered", 
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos)
        ); 

        
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        geometry_msgs::msg::TransformStamped t;

        // Robot-related quantities
        foot_frame_names_ = {"FL_foot", "FR_foot", "RL_foot", "RR_foot"};
        foot_frame_ids_ = {0, 1, 2, 3};

        // Time variables 
        DT_MIN_ = 1e-6;
        DT_MAX_ = 1;
        dt_ = 0;
        t_ = 0;
        t_prev_ = 0;
        imu_measurement_.resize(6);
        imu_measurement_prev_.resize(6);

        // Setup filter
        inekf::RobotState initial_state;
        // Initialize state mean
        Eigen::Matrix3d R0;
        Eigen::Vector3d v0, p0, bg0, ba0, gravity;
        R0 << 1, 0, 0, // initial orientation
            0, 1, 0,  // IMU frame is rotated 90deg about the x-axis
            0, 0, 1;
        v0 << 0, 0, 0;  // initial velocity
        p0 << 0, 0, 0.07;  // initial position
        bg0 << 0, 0, 0; // initial gyroscope bias
        ba0 << 0, 0, 0; // initial accelerometer bias
        gravity << 0, 0, -9.81; 

        Eigen::Quaternion<double> q_init;
        q_init.x() = -0.00111831;
        q_init.y() = -0.05200578;
        q_init.z() = -0.02147064;
        q_init.w() = 0.99841532;

        initial_state.setRotation(R0); //q_init.normalized().toRotationMatrix());
        initial_state.setVelocity(v0);
        initial_state.setPosition(p0);
        initial_state.setGyroscopeBias(bg0);
        initial_state.setAccelerometerBias(ba0);

        // Initialize state covariance
        inekf::NoiseParams noise_params;
        noise_params.setGyroscopeNoise(0.01);
        noise_params.setAccelerometerNoise(0.1);
        noise_params.setGyroscopeBiasNoise(0.00001);
        noise_params.setAccelerometerBiasNoise(0.0001);
        noise_params.setContactNoise(0.0001);

        // Initialize filter
        filter_ = std::make_shared<inekf::InEKF>(initial_state, noise_params);
        filter_->setGravity(gravity);
    }

    void listener_lowstate_callback(const unitree_go::msg::LowState::SharedPtr msg)
    {
        quat_lowstate_.x() = msg->imu_state.quaternion[0];
        quat_lowstate_.y() = msg->imu_state.quaternion[1];
        quat_lowstate_.z() = msg->imu_state.quaternion[2];
        quat_lowstate_.w() = msg->imu_state.quaternion[3];
    }

    void listener_feet_callback(const go2_odometry::msg::OdometryVector::SharedPtr msg)
    {   
        std::vector<std::pair<int, bool>> contact_list;
        inekf::vectorKinematics kinematics_list;

        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity() * 0.0001;
        Eigen::Matrix3d covariance_vel = Eigen::Matrix3d::Identity() * 0.0001;

        for (std::size_t i = 0; i < foot_frame_names_.size(); ++i) {
            contact_list.push_back(std::pair<int, bool>(i, msg->contact_states[i]));
            
            position << msg->pose_vec[i].pose.position.x, msg->pose_vec[i].pose.position.y, msg->pose_vec[i].pose.position.z;

            inekf::Kinematics frame(i, position, covariance, velocity, covariance_vel);
            kinematics_list.push_back(frame);
        }
        auto new_state = filter_->getState();
        filter_->setContacts(contact_list);
        filter_->correctKinematics(kinematics_list);
    }

    void listener_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        t_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        dt_ = t_ - t_prev_;
    
        // IMU measurement - used for propagation ===============================
        // gyroscope meas in filter are radians
        imu_measurement_[0] = msg->angular_velocity.x;
        imu_measurement_[1] = msg->angular_velocity.y;
        imu_measurement_[2] = msg->angular_velocity.z;

        imu_measurement_[3] = msg->linear_acceleration.x;
        imu_measurement_[4] = msg->linear_acceleration.y;
        imu_measurement_[5] = msg->linear_acceleration.z;

        if(dt_ > DT_MIN_ && dt_ < DT_MAX_) {
            //propagate using previous measurement
            propagate();
        }

        t_prev_ = t_;
        imu_measurement_prev_ = imu_measurement_;
    }

    void propagate() {
        // Transform from odom_frame (unmoving) to base_frame (tied to robot base)
        rclcpp::Time now = this->get_clock()->now();

        transform_msg_.header.stamp = now;
        transform_msg_.child_frame_id = "base";
        transform_msg_.header.frame_id = "odom";

        odometry_msg_.header.stamp = now;
        odometry_msg_.child_frame_id = "base";
        odometry_msg_.header.frame_id = "odom";
        
        filter_->propagate(imu_measurement_prev_, dt_);
        
        auto new_state = filter_->getState();
        auto new_r = new_state.getRotation();
        auto new_p = new_state.getPosition();
        auto new_v = new_state.getVelocity();

        //new_state.setRotation(quat_lowstate_.normalized().toRotationMatrix());
        Eigen::MatrixXd X = new_state.getX();
        long dimX = new_state.dimX();
        for (std::size_t i = 5; i < dimX; i++) {
            X(2, i) = 0.023;
        }
        new_state.setX(X); 
        filter_->setState(new_state);


        //new_p << 0., 0., 0.355;
        
        Eigen::Quaternion<double> q(new_r);
        
        transform_msg_.transform.translation.x = new_p[0];
        transform_msg_.transform.translation.y = new_p[1];
        transform_msg_.transform.translation.z = new_p[2];

        odometry_msg_.pose.pose.position.x = new_p[0];
        odometry_msg_.pose.pose.position.y = new_p[1];
        odometry_msg_.pose.pose.position.z = new_p[2];

        odometry_msg_.twist.twist.linear.x = new_v[0];
        odometry_msg_.twist.twist.linear.y = new_v[1];
        odometry_msg_.twist.twist.linear.z = new_v[2];

        odometry_msg_.twist.twist.angular.x = imu_measurement_[0];
        odometry_msg_.twist.twist.angular.y = imu_measurement_[1];
        odometry_msg_.twist.twist.angular.z = imu_measurement_[2];

        transform_msg_.transform.rotation.x = q.x();
        transform_msg_.transform.rotation.y = q.y();
        transform_msg_.transform.rotation.z = q.z();
        transform_msg_.transform.rotation.w = q.w();

        odometry_msg_.pose.pose.orientation.x = q.x();
        odometry_msg_.pose.pose.orientation.y = q.y();
        odometry_msg_.pose.pose.orientation.z = q.z();
        odometry_msg_.pose.pose.orientation.w = q.w();
        

        tf_broadcaster_->sendTransform(transform_msg_);
        odometry_publisher_->publish(odometry_msg_);
    }
    
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_subscriber_;
    rclcpp::Subscription<go2_odometry::msg::OdometryVector>::SharedPtr pos_feet_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    nav_msgs::msg::Odometry odometry_msg_;
    geometry_msgs::msg::TransformStamped transform_msg_;

    std::vector<std::string> foot_frame_names_;
    std::vector<std::size_t> foot_frame_ids_;

    double DT_MIN_;
    double DT_MAX_;
    double dt_;
    double t_;
    double t_prev_;

    Eigen::VectorXd imu_measurement_;
    Eigen::VectorXd imu_measurement_prev_;
    Eigen::Quaternion<double> quat_lowstate_;

    std::shared_ptr<inekf::InEKF> filter_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InEKFNode>());
  rclcpp::shutdown();
  return 0;
}