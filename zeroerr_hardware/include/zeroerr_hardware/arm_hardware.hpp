//* Useful links 
// https://control.ros.org/master/doc/getting_started/getting_started.html#overview-hardware-components
// https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html

#ifndef __ZEROERR_HARDWARE_H__
#define __ZEROERR_HARDWARE_H__

#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

namespace zeroerr_hardware
{

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class ArmHardwareInterface : public hardware_interface::SystemInterface
    {
        public:

            //* hardware_interface::SystemInterface method overrides
            /**
             * @brief 
             * 
             * @param info 
             * @return CallbackReturn 
             */
            CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

            /**
             * @brief 
             * 
             * @param previous_state 
             * @return CallbackReturn 
             */
            CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

            /**
             * @brief 
             * 
             * @return std::vector<hardware_interface::StateInterface> 
             */
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            /**
             * @brief 
             * 
             * @return std::vector<hardware_interface::CommandInterface> 
             */
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            /**
             * @brief 
             * 
             * @param previous_state 
             * @return hardware_interface::CallbackReturn 
             */
            hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state);

            /**
             * @brief 
             * 
             * @param previous_state 
             * @return hardware_interface::CallbackReturn 
             */
            hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state);

            /**
             * @brief 
             * 
             * @param time 
             * @param period 
             * @return hardware_interface::return_type 
             */
            hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

            /**
             * @brief 
             * 
             * @param time 
             * @param period 
             * @return hardware_interface::return_type 
             */
            hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;


        private:

            /**
             * @brief Get the hardware state/command topic parameter from ros2_control urdf 
             * 
             * @param param_name 
             * @param default_value 
             * @return topic name string
             */
            inline const auto get_hw_topic_param(const std::string& param_name, const std::string& default_value)
            {
                auto it = info_.hardware_parameters.find(param_name);
                
                // if topic parameter found in ros2_control urdf
                if (it != info_.hardware_parameters.end())
                    return it->second; // return corresponding topic 
                else
                    return default_value; // return default_value as topic
            }

            // Hardware interface node
            rclcpp::Node::SharedPtr hw_node_;

            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_commands_pub_;

            sensor_msgs::msg::JointState latest_arm_state_;

            // Position state interface vector
            std::vector<double> arm_position_state_;

            // Position command interface vector
            std::vector<double> arm_position_commands_;
            
            // Flag indicating enabling joint commands to be sent
            bool commands_ready_ = false;
            
            const double PI = 3.141592654;
            const double COUNT_THRESHOLD = 0.005992112; // 500 enc counts to rad
    };

    
}


#endif // __ZEROERR_HARDWARE_H__