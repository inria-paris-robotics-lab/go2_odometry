#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class StateConverterNode : public rclcpp::Node
{
    public:
        StateConverterNode()
        : Node("state_converter")
        , nq(12)
        , urdf_joint_names_({
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
          })
        , urdf_to_sdk_index_({
                3,  4,  5,
                0,  1,  2,
                9, 10, 11,
                6,  7,  8,
          })
        {
            lowstate_subscription_ = this->create_subscription<unitree_go::msg::LowState>("lowstate", 10, std::bind(&StateConverterNode::state_callback, this, std::placeholders::_1));
            jointstate_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_state", 10);

            assert(urdf_joint_names_.size() == nq);
            assert(urdf_to_sdk_index_.size() == nq);
            jointstate_msg_.name = urdf_joint_names_;
            jointstate_msg_.position.resize(nq);
            jointstate_msg_.velocity.resize(nq);
            jointstate_msg_.effort.resize(nq);
        }

    protected:
        const size_t nq;
        const std::vector<std::string> urdf_joint_names_;
        const std::vector<size_t> urdf_to_sdk_index_;

    private:
        void state_callback(const unitree_go::msg::LowState::SharedPtr msg);

        sensor_msgs::msg::JointState jointstate_msg_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointstate_publisher_;
        rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_subscription_;

};

void StateConverterNode::state_callback(const unitree_go::msg::LowState::SharedPtr msg)
{
    jointstate_msg_.header.stamp = this->get_clock()->now();
    for(size_t index_urdf = 0; index_urdf < nq; index_urdf++) {
        const size_t index_sdk = urdf_to_sdk_index_[index_urdf];
        jointstate_msg_.position[index_urdf] = msg->motor_state[index_sdk].q;
        jointstate_msg_.velocity[index_urdf] = msg->motor_state[index_sdk].dq;
        jointstate_msg_.effort[index_urdf] = msg->motor_state[index_sdk].tau_est;
    }
    jointstate_publisher_->publish(jointstate_msg_);
    std::cout << "aaaaa" << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateConverterNode>());
  rclcpp::shutdown();
  return 0;
}