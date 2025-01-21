// IMU preintegration node
#include "core/preintegration.hpp"
#include "utils/tools.hpp"

//ros2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "unitree_go/msg/low_state.hpp"


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

            this->declare_parameter("preintegration_period_ms", 100); // in ms

            lowstate_subscription_ = this->create_subscription<unitree_go::msg::LowState>("lowstate", 10, std::bind(&ImuPreIntNode::state_callback, this, std::placeholders::_1));
            
            // Preintegration timer
            this->timer_ = this->create_wall_timer(std::chrono::milliseconds(this->get_parameter("preintegration_period_ms").as_int()), std::bind(&ImuPreIntNode::preintegration_callback, this)); // TODO replace time hardcoded by ros param
        
            // Publisher for tf ?
            // TODO


            // setting preintegration parameters
            using namespace preintegration;

            this->params->setGravity(Eigen::Vector3d::UnitZ() * -9.81);
            this->params->setGyroNoiseSigma(1e-4);
            this->params->setAccNoiseSigma(1e-3);
            this->params->setGyroBiasNoiseSigma(1e-6);
            this->params->setAccBiasNoiseSigma(1e-5);
        }

    private:

    void state_callback(const unitree_go::msg::LowState::SharedPtr msg)
    { // fills the acceleration and gyroscope vectors every time a message is recieved
        // RCLCPP_INFO(this->get_logger(), "Acc\n %f \n %f \n %f", msg->imu_state.accelerometer[0],msg->imu_state.accelerometer[1],msg->imu_state.accelerometer[2]); //! DEBUG

        // Angular velocity
        Eigen::Vector3d data_gyro ; 
        data_gyro << msg->imu_state.gyroscope[0], // X
                     msg->imu_state.gyroscope[1], // Y
                     msg->imu_state.gyroscope[2]; // Z
        this->gyros.push_back(data_gyro);

        // Linear acceleration
        Eigen::Vector3d data_acc ;
        data_acc << msg->imu_state.accelerometer[0], // X
                    msg->imu_state.accelerometer[1], // Y
                    msg->imu_state.accelerometer[2]; // Z
        this->accs.push_back(data_acc);

    }

/*
        void subscription_callback(std_msgs::msg::Float64::SharedPtr msg)
        {  // Called when imu data is published: stores the last X data point in a list for preintegration later 

            RCLCPP_INFO(this->get_logger(), "Preint loop\n ");

            // Eigen::Vector3d data ; 
            // data << msg->data, 0, 0;
            // // RCLCPP_INFO_STREAM(this->get_logger(), "vector" << data);
            // this->accs.push_back(data);

        }
*/
        void preintegration_callback()
        {
            using namespace preintegration;

            // Initialize the preintegration object
            preintegration::EquivariantPreintegration<double> pim(params);
        
            // Example IMU measurements
            size_t size = this->accs.size();
            double dt = 0.001*this->get_parameter("preintegration_period_ms").as_int(); // conversion to seconds

            for (int j = 0; j < size; ++j) 
            {
                pim.integrateMeasurement(accs[j], gyros[j], dt);
            }
            std::cout << "Preintegrated Velocity: " << pim.deltaVij().transpose() << "\n" << std::endl;
            std::cout << "Preintegrated Position: " << pim.deltaPij().transpose() << "\n" << std::endl;

            // if(size >100) //! How to limit memory size? save position to bigger position? add at each preintegration and clear vectors?
            // {
            //     this->accs.clear();
            //     this->gyros.clear();
            // }
            
            // // Display preintegrated measurements
            // std::cout << "Preintegration Matrix:\n"
            //         << pim.Upsilon().asMatrix() << "\n" << std::endl;
            // std::cout << "Preintegrated Rotation:\n"
            //         << pim.deltaRij() << "\n" << std::endl;
            // std::cout << "Preintegrated Velocity: " << pim.deltaVij().transpose() << "\n" << std::endl;
            // std::cout << "Preintegrated Position: " << pim.deltaPij().transpose() << "\n" << std::endl;
            // std::cout << "Preintegration Time: " << pim.deltaTij() << "\n" << std::endl;
            // std::cout << "Preintegration Covariance:\n" << pim.Cov() << "\n\n" << std::endl;
            
            //TODO add clear or the vectors here?
            this->accs.clear();
            this->gyros.clear();
        }


        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_subscription_;
        std::vector<Eigen::Vector3d> accs;
        std::vector<Eigen::Vector3d> gyros;
        sensor_msgs::msg::Imu imu_msg_;
        std::shared_ptr<preintegration::PreintegrationParams<double>> params = std::make_shared<preintegration::PreintegrationParams<double>>(); // preintegration parmeter object
 
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuPreIntNode>());
  rclcpp::shutdown();
  return 0;
}