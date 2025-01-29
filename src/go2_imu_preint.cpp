// IMU preintegration node
#include "core/preintegration.hpp"
#include "utils/tools.hpp"

//ros2
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "kdl/frames.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

/*
Pour tester avec sin :
subscriber qui ajoute valeur à un vector a chaque fois qu'une donnée IMU est envoyée
Timer avec freq plus petite qui préintègre et publie au callback
*/

class ImuPreIntNode : public rclcpp::Node
{
    public:
        ImuPreIntNode()
        : Node("imu_preint")
        {
            // RCLCPP_DEBUG(this->get_logger(), "My log message %d", 4); //! kept for ref: to be removed once devel done

            this->declare_parameter("preintegration_freq", 200); // in Hz


            lowstate_subscription_ = this->create_subscription<unitree_go::msg::LowState>("lowstate", 10, std::bind(&ImuPreIntNode::state_callback, this, std::placeholders::_1));
            preintegration_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("preintegration_odom",10);
            
			// Recuperation de la pose précédente pour l'ajouter au delta calcule par la preintegration
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
			odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("odometry/filtered", 10, std::bind(&ImuPreIntNode::odometry_callback, this, std::placeholders::_1));

            // Preintegration timer
            this->timer_ = this->create_wall_timer(std::chrono::milliseconds(1000/(this->get_parameter("preintegration_freq").as_int())), std::bind(&ImuPreIntNode::preintegration_callback, this)); // TODO replace time hardcoded by ros param
        
            // Setting preintegration parameters
            using namespace preintegration;

            this->params->setGravity(Eigen::Vector3d::UnitZ() * -9.81);
            this->params->setGyroNoiseSigma(1e-6); //(1e-4);
            this->params->setAccNoiseSigma(6e-2); //(1e-3);
            this->params->setGyroBiasNoiseSigma(1e-6);
            this->params->setAccBiasNoiseSigma(1e-5);
        }

    private:
		void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
		{ // Saves the linear speed locally for later addition
			this->prec_linear_twist_.x = msg->twist.twist.linear.x;
			this->prec_linear_twist_.y = msg->twist.twist.linear.y;
			this->prec_linear_twist_.z = msg->twist.twist.linear.z;
		}

		void state_callback(const unitree_go::msg::LowState::SharedPtr msg)
		{ // fills the acceleration and gyroscope vectors every time a message is recieved
		
			// Angular velocity
			Eigen::Vector3d data_gyro ; 
			data_gyro << msg->imu_state.gyroscope[0], // X
                         msg->imu_state.gyroscope[1], // Y
                         msg->imu_state.gyroscope[2]; // Z
			this->gyros_.push_back(data_gyro);

			// Linear acceleration
			Eigen::Vector3d data_acc ;
			data_acc << msg->imu_state.accelerometer[0], // X
						msg->imu_state.accelerometer[1], // Y
						msg->imu_state.accelerometer[2]; // Z
			this->accs_.push_back(data_acc);
		}

    	void preintegration_callback()
        {
            // Setting preintegration parameters
            using namespace preintegration;

            // Initialize the preintegration object
            preintegration::EquivariantPreintegration<double> pim(params);
        
            // Example IMU measurements
            size_t size = this->accs_.size();
            double dt = 1.0/this->get_parameter("preintegration_freq").as_int(); // conversion to seconds

            for (int j = 0; j < size; ++j) 
            {
                pim.integrateMeasurement(accs_[j], gyros_[j], dt);
            }

            preintegration_msg_.header.frame_id = "odom";
            preintegration_msg_.child_frame_id = "imu";

            preintegration_msg_.header.stamp = this->get_clock()->now();
            preintegration_msg_.pose.pose.position.x = pim.deltaPij()[0] + prec_pose_.x;
            preintegration_msg_.pose.pose.position.y = pim.deltaPij()[1] + prec_pose_.y;
            preintegration_msg_.pose.pose.position.z = pim.deltaPij()[2] + prec_pose_.z;


            KDL::Rotation rotation_matrix(pim.deltaRij()(0,0), pim.deltaRij()(0,1), pim.deltaRij()(0,2),
                                          pim.deltaRij()(1,0), pim.deltaRij()(1,1), pim.deltaRij()(1,2),
                                          pim.deltaRij()(2,0), pim.deltaRij()(2,1), pim.deltaRij()(2,2));

            double x,y,z,w;
            //TODO: repair linking during compilation
            // rotation_matrix.GetQuaternion(x,y,z,w); 

            //! Replacing GetQuaternion function with copy pasted code from source because of inclusion failure 
            double trace = rotation_matrix(0,0) + rotation_matrix(1,1) + rotation_matrix(2,2);
            double epsilon=1E-12;
            if( trace > epsilon ){
                double s = 0.5 / sqrt(trace + 1.0);
                w = 0.25 / s;
                x = ( rotation_matrix(2,1) - rotation_matrix(1,2) ) * s;
                y = ( rotation_matrix(0,2) - rotation_matrix(2,0) ) * s;
                z = ( rotation_matrix(1,0) - rotation_matrix(0,1) ) * s;
            }else{
                if ( rotation_matrix(0,0) > rotation_matrix(1,1) && rotation_matrix(0,0) > rotation_matrix(2,2) ){
                    double s = 2.0 * sqrt( 1.0 + rotation_matrix(0,0) - rotation_matrix(1,1) - rotation_matrix(2,2));
                    w = (rotation_matrix(2,1) - rotation_matrix(1,2) ) / s;
                    x = 0.25 * s;
                    y = (rotation_matrix(0,1) + rotation_matrix(1,0) ) / s;
                    z = (rotation_matrix(0,2) + rotation_matrix(2,0) ) / s;
                } else if (rotation_matrix(1,1) > rotation_matrix(2,2)) {
                    double s = 2.0 * sqrt( 1.0 + rotation_matrix(1,1) - rotation_matrix(0,0) - rotation_matrix(2,2));
                    w = (rotation_matrix(0,2) - rotation_matrix(2,0) ) / s;
                    x = (rotation_matrix(0,1) + rotation_matrix(1,0) ) / s;
                    y = 0.25 * s;
                    z = (rotation_matrix(1,2) + rotation_matrix(2,1) ) / s;
                }else {
                    double s = 2.0 * sqrt( 1.0 + rotation_matrix(2,2) - rotation_matrix(0,0) - rotation_matrix(1,1) );
                    w = (rotation_matrix(1,0) - rotation_matrix(0,1) ) / s;
                    x = (rotation_matrix(0,2) + rotation_matrix(2,0) ) / s;
                    y = (rotation_matrix(1,2) + rotation_matrix(2,1) ) / s;
                    z = 0.25 * s;
                }
            }

            geometry_msgs::msg::Quaternion preint_quat;
            preint_quat.x = x;
            preint_quat.y = y;
            preint_quat.z = z;
            preint_quat.w = w;
            preintegration_msg_.pose.pose.orientation = preint_quat;

            preintegration_msg_.twist.twist.linear.x = pim.deltaVij()[0] + this->prec_linear_twist_.x;
            preintegration_msg_.twist.twist.linear.y = pim.deltaVij()[1] + this->prec_linear_twist_.y;
            preintegration_msg_.twist.twist.linear.z = pim.deltaVij()[2] + this->prec_linear_twist_.z; 

            preintegration_publisher_->publish(preintegration_msg_);

            // Display preintegrated measurements
            // RCLCPP_INFO_STREAM(this->get_logger(),
			// "Preintegration Matrix:\n" << pim.Upsilon().asMatrix() << "\n" <<
            // "Preintegrated Rotation:\n" << pim.deltaRij() << "\n" << 
            // "Preintegrated Velocity: " << pim.deltaVij().transpose() << "\n" <<
            // "Preintegrated Position: " << pim.deltaPij().transpose() << "\n" <<
            // "Preintegration Time: " << pim.deltaTij() << "\n" <<
            // "Preintegration Covariance:\n" << pim.Cov() << 
			// "------------------------------------------" <<
			// "\n\n");
            
            //! clear the vectors
            this->accs_.clear();
            this->gyros_.clear();

			try 
			{
				// Get current IMU pose (transform from odom to imu)
				this->t_ = tf_buffer_->lookupTransform("odom","imu", tf2::TimePointZero);

			} 
			catch (const tf2::TransformException & ex) 
			{
				RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s","odom", "imu", ex.what());
				return; 
      		}
			prec_pose_.x = t_.transform.translation.x;
			prec_pose_.y = t_.transform.translation.y;
			prec_pose_.z = t_.transform.translation.z;


            RCLCPP_INFO_STREAM(this->get_logger(),
            "Sortie EKF:" <<
            "\nX:" << t_.transform.translation.x <<
			"\nY:" << t_.transform.translation.y <<
			"\nZ:" << t_.transform.translation.z
            );
        }

        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_subscription_;
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr preintegration_publisher_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

		nav_msgs::msg::Odometry preintegration_msg_;
		geometry_msgs::msg::TransformStamped t_;
		geometry_msgs::msg::Vector3 prec_pose_;
		geometry_msgs::msg::Vector3 prec_linear_twist_;

        std::vector<Eigen::Vector3d> accs_;
        std::vector<Eigen::Vector3d> gyros_;

        std::shared_ptr<preintegration::PreintegrationParams<double>> params = std::make_shared<preintegration::PreintegrationParams<double>>(); // preintegration parmeter object
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuPreIntNode>());
  rclcpp::shutdown();
  return 0;
}

