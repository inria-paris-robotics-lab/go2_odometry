#include <chrono>
#include <math.h>
#include <time.h>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"

/*
Pour tester avec sin :
subscriber qui ajoute valeur à un vector a chaque fois qu'une donnée IMU est envoyée
Timer avec freq plus petite qui préintègre et publie au callback
*/

class FakeImuNode : public rclcpp::Node
{
    public:
        FakeImuNode()
        : Node("fake_imu")
        {
            // RCLCPP_DEBUG(this->get_logger(), "My log message %d", 4); //* kept for ref
            this->i=0;
            srand(time(NULL));   // Initialization, should only be called once.
            publisher_prenoise_ = this->create_publisher<std_msgs::msg::Float64>("fake_imu_no_noise", 10);
            publisher_postnoise_ = this->create_publisher<std_msgs::msg::Float64>("fake_imu_noise", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&FakeImuNode::timer_callback, this));
        }

    private:

    void timer_callback()
    {
        this->i = this->i + 0.1;

        // Print the time in nanoseconds
        std_msgs::msg::Float64 msg_noise;
        std_msgs::msg::Float64 msg_no_noise;
        msg_no_noise.data = sin(i);

        float noise = (rand()%201-100); // random number between -100 and 100
        noise = noise /100.0;
        msg_noise.data = msg_no_noise.data + noise;  



        this->publisher_prenoise_->publish(msg_no_noise);
        this->publisher_postnoise_->publish(msg_noise);

        // RCLCPP_DEBUG(this->get_logger(), "time %d", rclcpp::Clock(RCL_ROS_TIME).now()); //rclcpp::clock(RCL_ROS_TIME).now()
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_prenoise_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_postnoise_;
    // sensor_msgs::msg::Imu() = imu_msg;
    rclcpp::Clock clock;
    float i;
    
 
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeImuNode>());
  rclcpp::shutdown();
  return 0;
}