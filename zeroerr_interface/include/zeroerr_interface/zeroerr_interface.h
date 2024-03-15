#include <chrono>
#include <string>
// #include <thread>
// #include <mutex>
#include "ec_defines.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/sensor_msgs/msg/joint_state.hpp"


#define PI          3.1415926538979323846
#define MAX_COUNT   524288

// Macro converting eRob encoder counts to radians
#define COUNT_TO_RAD(x) ((x * (2 * PI)) / MAX_COUNT)

// Macro converting radians to eRob encoder count
#define RAD_TO_COUNT(x) ((x * MAX_COUNT) / (2 * PI))


using namespace std::chrono_literals;


class ZeroErrInterface : public rclcpp::Node
{
    public:
        ZeroErrInterface();
        ~ZeroErrInterface();


    private:
        const std::chrono::milliseconds CYCLIC_DATA_PERIOD = 1ms;
        const std::chrono::milliseconds JOINT_STATE_PERIOD = 10ms;

        int joint_no_ = 0;
        unsigned long counter_ = 0;
        double stamp_ = 0;
        bool joints_OP_ = false;
        bool joints_op_enabled_ = false;

        sensor_msgs::msg::JointState joint_states_;
        std::vector<uint32_t> joint_commands_;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_state_pub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_cmd_sub_;

        rclcpp::TimerBase::SharedPtr cyclic_pdo_timer_;
        rclcpp::TimerBase::SharedPtr joint_state_pub_timer_;
        
        
        bool configure_pdos_();
        bool set_drive_parameters_();
        bool init_();

        bool state_transition_();
        void cyclic_pdo_loop_();

        void read_sdos(int joint_no);
        void check_master_state_();
        bool check_slave_config_states_();
        void check_domain_state_();
        
        void joint_state_pub_();
        void arm_cmd_cb_(sensor_msgs::msg::JointState::UniquePtr arm_cmd);

};