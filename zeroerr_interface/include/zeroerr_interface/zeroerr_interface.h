#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;




class ZeroErrInterface : public rclcpp::Node
{
    public:

        ZeroErrInterface();
        ~ZeroErrInterface();


    private:
        sensor_msgs::msg::JointState joint_states;
        sensor_msgs::msg::JointState joint_commands;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_state_pub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_cmd_sub_;

        rclcpp::TimerBase::SharedPtr cyclic_pdo_timer_;
        
        
        void init_();
        void cyclic_pdo_loop_();


        void check_master_state();
        void check_slave_config_states();
        void check_domain_state();
        
        




};
