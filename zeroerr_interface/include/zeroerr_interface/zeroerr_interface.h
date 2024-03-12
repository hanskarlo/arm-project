#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/sensor_msgs/msg/joint_state.hpp"

#include "ec_defines.h"

using namespace std::chrono_literals;




class ZeroErrInterface : public rclcpp::Node
{
    public:

        ZeroErrInterface();
        ~ZeroErrInterface();


    private:
        int joint_no_ = 0;
        double stamp = 0;
        bool operational = false;


        sensor_msgs::msg::JointState joint_states_;
        std::vector<uint32_t> joint_commands_;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_state_pub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_cmd_sub_;

        rclcpp::TimerBase::SharedPtr cyclic_pdo_timer_;
        rclcpp::TimerBase::SharedPtr joint_state_pub_timer_;
        
        void read_sdos(int joint_no);
        bool configure_pdos_();
        bool set_drive_parameters();
        bool init_();

        void state_transition_(int joint_no);
        void cyclic_pdo_loop_();

        void joint_state_pub_();

        void check_master_state();
        bool check_slave_config_states(int joint_no);
        void check_domain_state();
        
        void arm_cmd_cb_(sensor_msgs::msg::JointState::UniquePtr arm_cmd);
        




};