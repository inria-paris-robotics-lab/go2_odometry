#include "core/preintegration.hpp"
#include "utils/tools.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"


class ImuPreIntNode : public rclcpp::Node
{
    public:
        ImuPreIntNode():Node("imu_preint")
        {
            // RCLCPP_DEBUG(ImuPreIntNode->get_logger(), "My log message %d", 4);
        }

    private:
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

            // Integrate IMU measurements
            for (int j = 0; j < n; ++j) {
                pim.integrateMeasurement(accs[j], gyros[j], dt);
            }

            // Display preintegrated measurements
            std::cout << "Preintegration Matrix:\n"
                    << pim.Upsilon().asMatrix() << std::endl;
            std::cout << "Preintegrated Rotation:\n"
                    << pim.deltaRij() << std::endl;
            std::cout << "Preintegrated Velocity: " << pim.deltaVij().transpose() << std::endl;
            std::cout << "Preintegrated Position: " << pim.deltaPij().transpose() << std::endl;
            std::cout << "Preintegration Time: " << pim.deltaTij() << std::endl;
            std::cout << "Preintegration Covariance:\n" << pim.Cov() << "\n\n" << std::endl;
            
            return 0;

        }
    */
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuPreIntNode>());
  rclcpp::shutdown();
  return 0;
}