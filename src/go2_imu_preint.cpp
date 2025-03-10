// IMU preintegration node
#include "core/preintegration.hpp"
#include "utils/tools.hpp"

//ros2
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "kdl/frames.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"

#include <iostream>
#include <fstream>

#define LOGGING_ENABLED 1
 
class ImuPreIntNode : public rclcpp::Node
{
    public:
        ImuPreIntNode()
        : Node("imu_preint")
        {
            if(LOGGING_ENABLED)
            {
                // getting date & time as a string
                time_t t = time(0);
                struct tm * now = localtime( & t );
                char buffer [80];
                strftime(buffer,80,"%Y-%m-%d_%H-%M-%S.csv",now);
                
                std::string filepath  = "go2_state_logs/" + std::string(buffer);
                RCLCPP_INFO_STREAM(this->get_logger(), "Creating log @ " << filepath);
                
                this->myfile.open (filepath);
                myfile << "Time,Accel X,Accel Y,Accel Z,Vel X,Vel Y,Vel Z,Pose X,Pose Y,Pose Z,Gyro X,Gyro Y,Gyro Z,Orientation X,Orientation Y,Orientation Z\n";

                this->t_since_start = (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count())/1000.0;
            }
            

            this->declare_parameter("preintegration_freq", 100); // in Hz
            this->declare_parameter("imu_freq", 500);

            
            // ROS2 subscribers / publishers ===================================
            lowstate_subscription_ = this->create_subscription<unitree_go::msg::LowState>("lowstate", 10, std::bind(&ImuPreIntNode::state_callback, this, std::placeholders::_1));
            preintegration_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("preintegration_odom",10);
            
			// Recuperation de la pose précédente pour l'ajouter au delta calcule par la preintegration
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            
            
			odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("odometry/filtered", 10, std::bind(&ImuPreIntNode::odometry_callback, this, std::placeholders::_1));

            // Preintegration timer
            this->timer_ = this->create_wall_timer(std::chrono::milliseconds(1000/(this->get_parameter("preintegration_freq").as_int())), std::bind(&ImuPreIntNode::preintegration_callback, this));
        
            // Setting preintegration parameters 
            using namespace preintegration;

            this->params->setGravity(Eigen::Vector3d::UnitZ() * -9.81);
            this->dt_integration = 1.0/this->get_parameter("imu_freq").as_int(); // conversion to seconds

            // Noise parameters ================================================

            // VECTOR values 
            Eigen::Vector<double,3> AccNoiseSigma(0.0463, 0.05, 0.043);
            Eigen::Vector<double,3> AccBiasNoiseSigma(0, 0, 0);
            Eigen::Vector<double,3> GyroNoiseSigma(0.0089, 0.0081, 0.0083);
            Eigen::Vector<double,3> GyroBiasNoiseSigma(0, 0, 0) ;

            this->params->setAccNoiseSigma(AccNoiseSigma); //(1e-3); 
            this->params->setAccBiasNoiseSigma(AccBiasNoiseSigma); 

            this->params->setGyroNoiseSigma(GyroNoiseSigma); //(1e-4); 
            this->params->setGyroBiasNoiseSigma(GyroBiasNoiseSigma); 

            // NULL values 
            // this->params->setAccNoiseSigma(0); //(1e-3); 
            // this->params->setAccBiasNoiseSigma(0); 

            // this->params->setGyroNoiseSigma(0); //(1e-4); 
            // this->params->setGyroBiasNoiseSigma(0); 

            // Saving precedent pose & vel for now, looping on only imu preintegration
            this->prec_pose_.x = 0;
			this->prec_pose_.y = 0;
			this->prec_pose_.z = 0;
            this->prec_linear_twist_.x = 0;
			this->prec_linear_twist_.y = 0;
			this->prec_linear_twist_.z = 0;
            this->prec_rotation_ << 1,0,0,
                                    0,1,0,
                                    0,0,1;
            
        }

    private:
		void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
		{ // Saves the linear speed locally for later addition
        //!shutdown 4 test : for now looped on itself but needs to be output of inekf 
			// this->prec_linear_twist_.x = msg->twist.twist.linear.x;
			// this->prec_linear_twist_.y = msg->twist.twist.linear.y;
			// this->prec_linear_twist_.z = msg->twist.twist.linear.z;
		}

		void state_callback(const unitree_go::msg::LowState::SharedPtr msg)
		{ // fills the acceleration and gyroscope vectors every time a message is recieved
		
			// Angular velocity
			Eigen::Vector3d data_gyro ; 
			data_gyro << msg->imu_state.gyroscope[0], // - static bias on X // X
                         msg->imu_state.gyroscope[1], // - static bias on Y // Y
                         msg->imu_state.gyroscope[2]; // - static bias on Z // Z
			this->gyros_.push_back(data_gyro);

            

			// Linear acceleration
			Eigen::Vector3d data_acc ;
            Eigen::Vector3d bias;
            bias <<  0,0,0;//-0.139, 0.0291, -0.21; //-0.15,-0.13,-0.18; //0.14 , 0.07, -0.23;
            Eigen::Vector3d bias_comp = (imu_orientation_.transpose()*bias);
            

			data_acc << msg->imu_state.accelerometer[0] - bias_comp[0], // - static bias on X 
						msg->imu_state.accelerometer[1] - bias_comp[1], // - static bias on Y 
						msg->imu_state.accelerometer[2] - bias_comp[2]; // - gravity - static bias on Z 
            this->accs_.push_back(data_acc);

            // Orientation (for gravity compensation)
            Eigen::Quaterniond imu_quat(msg->imu_state.quaternion[3],
                                        msg->imu_state.quaternion[0],
                                        msg->imu_state.quaternion[1],
                                        msg->imu_state.quaternion[2]
                                        );
            
            imu_orientation_ = imu_quat.toRotationMatrix();

            RCLCPP_INFO_STREAM(this->get_logger(), "orientation IMU:\n"<< imu_orientation_);
            RCLCPP_INFO_STREAM(this->get_logger(), "accel IMU vu par preint:\n"<< accs_[0]-(imu_orientation_.transpose()*params->getGravity()));
            

            // ! Accel Value Printing ==========================================
            // RCLCPP_INFO_STREAM(this->get_logger(),"accel brute - avec compensation bias\n" <<
            //                     msg->imu_state.accelerometer[0] << "\t" <<
            //                     msg->imu_state.accelerometer[1] << "\t" <<
            //                     msg->imu_state.accelerometer[2] << "\n" <<
                                // msg->imu_state.accelerometer[0] - bias_comp[0] << "\t\t" <<
                                // msg->imu_state.accelerometer[1] - bias_comp[1] << "\t" <<
                                // msg->imu_state.accelerometer[2] - bias_comp[2]<<
                                // "\n");
            // ! ===============================================================

            
            if(LOGGING_ENABLED)
            {
                double timestamp_log = (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count())/1000.0 - this->t_since_start;
                this->myfile <<timestamp_log <<","<< msg->imu_state.accelerometer[0]<<","<<msg->imu_state.accelerometer[1]<<","<<msg->imu_state.accelerometer[2]<<",,,,,,,"<< msg->imu_state.gyroscope[0] <<","<< msg->imu_state.gyroscope[0] <<","<< msg->imu_state.gyroscope[0]<<"\n";
            }
            

			
		}

    	void preintegration_callback()
        {
            
            // Setting preintegration parameters
            using namespace preintegration;

            // Initialize the preintegration object
            preintegration::EquivariantPreintegration<double> pim(params);
        
            // size of IMU measurements
            size_t size = this->accs_.size();

            // RCLCPP_INFO_STREAM(this->get_logger(),"Numbers of measurements:" << size << "\ndt : " << this->dt_integration);
            
            for (int j = 0; j < size; ++j) 
            {
                pim.integrateMeasurement(accs_[j], gyros_[j], imu_orientation_, this->dt_integration);
            }

            // clear the vectors between each run
            this->accs_.clear();
            this->gyros_.clear();

            // ! Sending data ==================================================
            // fill odometry message with data
            preintegration_msg_.header.frame_id = "odom";
            preintegration_msg_.child_frame_id = "base";

            preintegration_msg_.header.stamp = this->get_clock()->now();
            preintegration_msg_.pose.pose.position.x = pim.deltaPij()[0] + prec_pose_.x;
            preintegration_msg_.pose.pose.position.y = pim.deltaPij()[1]+ prec_pose_.y;
            preintegration_msg_.pose.pose.position.z = pim.deltaPij()[2] + prec_pose_.z;


            Eigen::Matrix3d delta_rotation_matrix_{{pim.deltaRij()(0,0),
                                                    pim.deltaRij()(0,1),
                                                    pim.deltaRij()(0,2)},

                                                    {pim.deltaRij()(1,0),
                                                    pim.deltaRij()(1,1),
                                                    pim.deltaRij()(1,2)},

                                                    {pim.deltaRij()(2,0),
                                                    pim.deltaRij()(2,1),
                                                    pim.deltaRij()(2,2)}};

            Eigen::Matrix3d current_rotation = delta_rotation_matrix_ * prec_rotation_; // add the delta to the precedent rotation to get current rotation 
            
            // Convert from rotation matrix to quaternion
            Eigen::Quaterniond quat_from_rot(current_rotation);
            
            // tranfert values to ros msg
            geometry_msgs::msg::Quaternion preint_quat;
            preint_quat.w = quat_from_rot.w(); // TODO: replace by eigen?
            preint_quat.x = quat_from_rot.x();
            preint_quat.y = quat_from_rot.y();
            preint_quat.z = quat_from_rot.z();

            preintegration_msg_.pose.pose.orientation = preint_quat;

            preintegration_msg_.twist.twist.linear.x = pim.deltaVij()[0] + this->prec_linear_twist_.x;
            preintegration_msg_.twist.twist.linear.y = pim.deltaVij()[1] + this->prec_linear_twist_.y;
            preintegration_msg_.twist.twist.linear.z = pim.deltaVij()[2] + this->prec_linear_twist_.z; 

            preintegration_publisher_->publish(preintegration_msg_);

            // // Save the data for the next loop
            // this->prec_pose_.x = preintegration_msg_.pose.pose.position.x;
            // this->prec_pose_.y = preintegration_msg_.pose.pose.position.y;
            // this->prec_pose_.z = preintegration_msg_.pose.pose.position.z;

            
            this->prec_linear_twist_.x = preintegration_msg_.twist.twist.linear.x ;
            this->prec_linear_twist_.y = preintegration_msg_.twist.twist.linear.y ;
            this->prec_linear_twist_.z = preintegration_msg_.twist.twist.linear.z ; 
            this->prec_rotation_ = current_rotation;

            //! test avec integration manuelle pour la pose
            if(pim.deltaVij()[0]!=0)
            {
                this->prec_pose_.x += this->prec_linear_twist_.x*(this->dt_integration*size);
                this->prec_pose_.y += this->prec_linear_twist_.y*(this->dt_integration*size);
                this->prec_pose_.z += this->prec_linear_twist_.z*(this->dt_integration*size);
            }
            

            // LOGGING =========================================================
            if(LOGGING_ENABLED)
            {
                Eigen::Vector3d ea = current_rotation.eulerAngles(2,1,0);
                double timestamp_log = (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count())/1000.0 - this->t_since_start;
                this->myfile << timestamp_log<<",,,," <<
                this->prec_linear_twist_.x<<","<<
                this->prec_linear_twist_.y<<","<< 
                this->prec_linear_twist_.z<<","<<
                this->prec_pose_.x<<","<<
                this->prec_pose_.y<<","<<
                this->prec_pose_.z<<",,,,"<<
                ea[0]<<","<<
                ea[1]<<","<<
                ea[2]<<
                "\n";
            }
            
            // =================================================================

            // ! PRINT STATE ===================================================
            // RCLCPP_INFO_STREAM(this->get_logger(), "pose \t"<< this->prec_pose_.x <<"\t"<<
            //                                                    this->prec_pose_.y<< "\t" <<
            //                                                    this->prec_pose_.z);
            // RCLCPP_INFO_STREAM(this->get_logger(), "vel \t"<< this->prec_linear_twist_.x << "\t"<< 
            //                                                   this->prec_linear_twist_.y << "\t"<<
            //                                                   this->prec_linear_twist_.z
            //                                                   <<"\n");
            

            //* Display preintegrated measurements 
            // RCLCPP_INFO_STREAM(this->get_logger(),
			// "Preintegration Matrix:\n" << pim.Upsilon().asMatrix() << "\n" <<
            // "Preintegrated Rotation:\n" << pim.deltaRij() << "\n" << 
            // "Preintegrated Velocity: " << pim.deltaVij().transpose() << "\n" <<
            // "Preintegrated Position: " << pim.deltaPij().transpose() << "\n" <<
            // "Preintegration Time: " << pim.deltaTij() << "\n" <<
            // "Preintegration Covariance:\n" << pim.Cov() << 
			// "------------------------------------------" <<
            // "Preintegration bias:\n" << pim.b()[0] << ", " << pim.b()[1] << ", " << pim.b()[2] << 
			// "\n\n");
            // ! ===============================================================
            
            

            //* Fill and publish tf 
			// try 
			// {
			// 	// Get current IMU pose (transform from odom to imu)
			// 	this->t_ = tf_buffer_->lookupTransform("odom","imu", tf2::TimePointZero);
			// } 
			// catch (const tf2::TransformException & ex) 
			// {
			// 	RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s","odom", "imu", ex.what());
			// 	return; 
      		// }

            geometry_msgs::msg::TransformStamped t_;            
            t_.header.stamp = this->get_clock()->now();
            t_.header.frame_id = "map";
            t_.child_frame_id = "base";

            t_.transform.translation.x = this->prec_pose_.x;
            t_.transform.translation.y = this->prec_pose_.y;
            t_.transform.translation.z = this->prec_pose_.z;

            t_.transform.rotation = preint_quat;

            // Send the transformation
            tf_broadcaster_->sendTransform(t_);


        }

// Attributes ==================================================================
        std::ofstream myfile; // logging file

        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_subscription_;
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr preintegration_publisher_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;



		nav_msgs::msg::Odometry preintegration_msg_;
		geometry_msgs::msg::TransformStamped t_;
		geometry_msgs::msg::Vector3 prec_pose_;
		geometry_msgs::msg::Vector3 prec_linear_twist_;
        Eigen::Matrix3d prec_rotation_;

        std::vector<Eigen::Vector3d> accs_;
        std::vector<Eigen::Vector3d> gyros_;
        Eigen::Matrix3d imu_orientation_;

        // Time keeping
        double timestamp = 0;
        double prec_timestamp = 0;
        double dt_integration = 0;
        double t_since_start = 0.0;
        int loopnb = 0; //!debug

        std::shared_ptr<preintegration::PreintegrationParams<double>> params = std::make_shared<preintegration::PreintegrationParams<double>>(); // preintegration parmeter object
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuPreIntNode>());
  rclcpp::shutdown();
  return 0;
}

