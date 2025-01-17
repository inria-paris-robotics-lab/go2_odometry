// imu preintegration 
#include "core/preintegration.hpp"
#include "utils/tools.hpp"

//ros2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"

//
// #include <vector>

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
            // RCLCPP_DEBUG(this->get_logger(), "My log message %d", 4); //* kept for ref

            subscription_ = this->create_subscription<std_msgs::msg::Float64>("fake_imu_noise", 10, std::bind(&ImuPreIntNode::subscription_callback, this, std::placeholders::_1));
            timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&FakeImuNode::preintegration_callback, this));
            
            // setting preintegration parameters
            using namespace preintegration;

            params->setGravity(Eigen::Vector3d::UnitZ() * -9.81);
            params->setGyroNoiseSigma(1e-4);
            params->setAccNoiseSigma(1e-3);
            params->setGyroBiasNoiseSigma(1e-6);
            params->setAccBiasNoiseSigma(1e-5);
        }

    private:

        void subscription_callback(std_msgs::msg::Float64::SharedPtr msg)
        {  /* Called when imu data is published: stores the last 100 data point in a list for preintegration later */

            Eigen::Vector3d data ; 
            data << msg->data, 0, 0;
            RCLCPP_INFO_STREAM(this->get_logger(), "vector" << data);
            this->accs.push_back(data);

        }

        void preintegration_callback()
        {
            using namespace preintegration;

            // Initialize the preintegration object
            EquivariantPreintegration<double> pim(params);
        
            // Example IMU measurements
            size_t size = this->accs.size();
            double dt = 0.01; //TODO : change based on timestamps from data received
            this->accs // datato preintegrate

            for (int j = 0; j < size; ++j) 
            {
                pim.integrateMeasurement(accs[j], gyros[j], dt);
            }
        }

/*
        int main_loop()
        {
            using namespace preintegration;

            // Define the parameters
            std::shared_ptr<PreintegrationParams<double>> params = std::make_shared<PreintegrationParams<double>>();
            params->setGravity(Eigen::Vector3d::UnitZ() * -9.81);
            params->setGyroNoiseSigma(1e-4);
            params->setAccNoiseSigma(1e-3);
            params->setGyroBiasNoiseSigma(1e-6);
            params->setAccBiasNoiseSigma(1e-5);

            // Initialize the preintegration object
            EquivariantPreintegration<double> pim(params);

            // Example IMU measurements
            size_t n = 1000;
            double dt = 0.01;
            std::vector<Eigen::Vector3d> accs = utils::randomAcc<double>(-10, 10, n);
            std::vector<Eigen::Vector3d> gyros = utils::randomGyro<double>(-1, 1, n);

            // for (int i = 0; i < n; ++i) {
            //         std::cout << accs[i] << "\t" << std::endl;
            // }



            // Integrate IMU measurements
            for (int j = 0; j < n; ++j) {
                pim.integrateMeasurement(accs[j], gyros[j], dt);
            }

            // Display preintegrated measurements
            std::cout << "Preintegration Matrix:\n"
                    << pim.Upsilon().asMatrix() << "\n" << std::endl;
            std::cout << "Preintegrated Rotation:\n"
                    << pim.deltaRij() << "\n" << std::endl;
            std::cout << "Preintegrated Velocity: " << pim.deltaVij().transpose() << "\n" << std::endl;
            std::cout << "Preintegrated Position: " << pim.deltaPij().transpose() << "\n" << std::endl;
            std::cout << "Preintegration Time: " << pim.deltaTij() << "\n" << std::endl;
            std::cout << "Preintegration Covariance:\n" << pim.Cov() << "\n\n" << std::endl;
            

            rclcpp::shutdown();
            return 0;

        }
*/

        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
        std::vector<Eigen::Vector3d> accs;
        std::shared_ptr<PreintegrationParams<double>> params = std::make_shared<PreintegrationParams<double>>(); // preintegration parmeter object

 
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuPreIntNode>());
  rclcpp::shutdown();
  return 0;
}