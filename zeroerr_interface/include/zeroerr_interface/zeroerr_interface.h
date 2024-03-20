#include <chrono>
#include <string>
#include <float.h>
// #include <thread>
// #include <mutex>
#include "ec_defines.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


#define PI          3.1415926538979323846
#define TWO_PI      (2 * PI)
#define MAX_COUNT   524288

// Macro converting eRob encoder counts to radians
#define COUNT_TO_RAD(x) ((x * TWO_PI) / MAX_COUNT)

// Macro converting radians to eRob encoder count
#define RAD_TO_COUNT(x) ((x * MAX_COUNT) / TWO_PI)

bool toggle = false;

using namespace std::chrono_literals;


class ZeroErrInterface : public rclcpp::Node
{
    public:
        ZeroErrInterface();
        ~ZeroErrInterface();


    private:
        const std::string LOGGER = "zeroerr_interface";

        std::chrono::milliseconds CYCLIC_DATA_PERIOD;
        const std::chrono::milliseconds JOINT_STATE_PERIOD = 10ms;

        const uint32_t SYNC0_CYCLE = PERIOD_NS;
        const int32_t SYNC0_SHIFT = 0;

        int sync_ref_counter = 0;
        struct timespec wakeupTime;

        int joint_no_ = 0;
        unsigned long counter_ = 0;
        double stamp_ = 0;
        bool joints_OP_ = false;
        bool joints_op_enabled_ = false;

        unsigned long loop_start_time_;
        unsigned long loop_last_start_time;
        unsigned long loop_end_time_;
        unsigned long wakeup_time_;
        unsigned long latency_ns_;
        unsigned long exec_ns_;
        unsigned long period_ns_;
        unsigned long latency_max_ns_ = 0;
        unsigned long latency_min_ns_ = 0;
        unsigned long exec_max_ns_ = 0;
        unsigned long exec_min_ns_ = 0;
        unsigned long period_max_ns_ = 0;
        unsigned long period_min_ns_ = 0;

        sensor_msgs::msg::JointState joint_states_;
        std::vector<int32_t> joint_states_enc_counts_;
        std::vector<int32_t> joint_commands_;

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