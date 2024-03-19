// #include "zeroerr_hardware/arm_hardware.hpp"
#include "../include/zeroerr_hardware/arm_hardware.hpp"
#include "pluginlib/class_list_macros.hpp"

#define LOGGER "ArmHardwareInterface"
#define LOG_INFO(msg)  RCLCPP_INFO(rclcpp::get_logger(LOGGER), msg) 
#define LOG_ERR(msg)   RCLCPP_FATAL(rclcpp::get_logger(LOGGER), msg) 

#define NUM_JOINTS 6

namespace zeroerr_hardware
{
    hardware_interface::CallbackReturn ArmHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
    {
        //* Check if hardware interface info initialized from URDF.
        // If succcessful, hardware_interface::SystemInterface::info_ will contain hardware info
        // from URDF
        LOG_INFO("[on_init] loading/parsing URDF");
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            // RCLCPP_FATAL(rclcpp::get_logger(LOGGER), "on_init failed to load/parse URDF");
            LOG_ERR("[on_init] failed to load/parse URDF");

            return CallbackReturn::ERROR;
        }
        LOG_INFO("[on_init] ros2_control URDF loaded");

        // if (info_.joints.size() == NUM_JOINTS)
        //     LOG_INFO("[on_init] 6 joints loaded from urdf");

        // for (int i = 0; i < NUM_JOINTS; i++)
        //     LOG_INFO(info_.joints[i].name.c_str());

        //* Arm has 6 joints (6 positions)
        arm_position_state_.assign(NUM_JOINTS, 0.0);
        arm_position_commands_.assign(NUM_JOINTS, 0.0);
        latest_arm_state_.position.assign(NUM_JOINTS, 0.0);
        // velocity_states_.assign(6, 0.0);
        // velocity_commands_.assign(6. 0.0);


        //* Add random ID to prevent warnings about multiple publishers within the same node
        rclcpp::NodeOptions options;
        options.arguments({ "--ros-args", "-r", "__node:=" + info_.name });

        
        //* Make node to publish/subscribe to arm topics
        LOG_INFO("[on_init] Making ros2 control arm hardware interface node");
        hw_node_ = rclcpp::Node::make_shared("_", options);


        //* Make state interface subscriber
        LOG_INFO("[on_init] Making arm/state subscriber");
        joint_states_sub_ = hw_node_->create_subscription<sensor_msgs::msg::JointState>(
            get_hw_topic_param("state_interface_topic", "arm/state"),
            rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr joint_state){ latest_arm_state_ = *joint_state; }
        );

        //! Temp
        for (uint i = 0; i < NUM_JOINTS; i++)
            latest_arm_state_.position[i] = 0.0;


        //* Make command interface publisher
        LOG_INFO("[on_init] Making arm/commands publisher");
        joint_commands_pub_ = hw_node_->create_publisher<sensor_msgs::msg::JointState>(
            get_hw_topic_param("command_interface_topic", "arm/command"),
            rclcpp::QoS(1)
        );


        LOG_INFO("[on_init] Finished successfully!");
        return CallbackReturn::SUCCESS;
    }


    hardware_interface::CallbackReturn ArmHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // TODO: Configure ZeroErr eRob actuators by way of topic

        return CallbackReturn::SUCCESS;
    } 


    std::vector<hardware_interface::StateInterface> ArmHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // for (uint i = 0; i < arm_position_state_.size(); i++)
        //     arm_position_state_[i] = 0.0;

        //* Link arm_position_state_ to state interface
        for (uint i = 0; i < info_.joints.size(); i++)
        {
            for (uint j = 0; j < info_.joints[i].state_interfaces.size(); j++)
            {
                state_interfaces.emplace_back(
                    hardware_interface::StateInterface(
                        info_.joints[i].name,
                        info_.joints[i].state_interfaces[j].name,
                        &arm_position_state_[i]
                    )
                );
            }
        }

        //? Add velocity state interface?

        return state_interfaces;
    }


    std::vector<hardware_interface::CommandInterface> ArmHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        //* Link position_commands_ to command interface
        for (uint i = 0; i < info_.joints.size(); i++) // for every joint
        {
            for (uint j = 0; j < info_.joints[i].command_interfaces.size(); j++) // for every command interface of ith joint
            {
                command_interfaces.emplace_back(
                    hardware_interface::CommandInterface(
                        info_.joints[i].name,                           // ith joint name
                        info_.joints[i].command_interfaces[j].name,     // jth command interface name of ith joint    
                        &arm_position_commands_[i]                      // ith joint command    
                    )
                );
            }
        }

        //? Add velocity command interface?

        return command_interfaces;
    }


    hardware_interface::return_type ArmHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // if node shutdown(), re-spin
        if (rclcpp::ok()) rclcpp::spin_some(hw_node_); 

        //* Update arm_position_state_ with position read from arm/state topic
        for (uint i = 0; i < NUM_JOINTS; i++)
            arm_position_state_[i] = latest_arm_state_.position[i];

        return hardware_interface::return_type::OK;
    }


    hardware_interface::return_type ArmHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        /**
         * The initial joint commands are all 0, i.e. the joint trajectory controller commands
         * all joints to go to 0. Joint commands must be the same as the current joint states 
         * before issuing commands. 
         */
        if (!commands_ready_)
        {
            uint count = 0;
            for (uint i = 0; i < NUM_JOINTS; i++)
            {
                if (arm_position_commands_[i] == 0.0 && latest_arm_state_.position[i] != 0.0)
                {
                    arm_position_commands_[i] = latest_arm_state_.position[i];
                }
                else if (arm_position_commands_[i] != 0.0)
                {
                    count++;
                }
            }

            if (count == NUM_JOINTS)
                commands_ready_ = true;
            else
                return hardware_interface::return_type::OK;
        }


        // JointState message to be read by arm
        sensor_msgs::msg::JointState arm_commands;

        arm_commands.name.resize(NUM_JOINTS);
        arm_commands.position.assign(NUM_JOINTS, 0.0);

        // Get timestamp
        auto timestamp = hw_node_->now();
        arm_commands.header.stamp = timestamp;

        
        //* Fill arm_commands with updated arm_position_commands from command interface
        for (uint i = 0; i < NUM_JOINTS; i++)
        {
            arm_commands.name[i] = info_.joints[i].name;
            arm_commands.position[i] = arm_position_commands_[i];

            // RCLCPP_INFO(
            //     rclcpp::get_logger(LOGGER), "Got command %.5f for j%d!",
            //     arm_commands.position[i], (i + 1)
            // );
        }
        // RCLCPP_INFO(rclcpp::get_logger(LOGGER), "[write] j6 state: %f", latest_arm_state_.position[5]);
        // RCLCPP_INFO(rclcpp::get_logger(LOGGER), "[write] j6 arm_position_command: %f", arm_position_commands_[5]);
        // RCLCPP_INFO(rclcpp::get_logger(LOGGER), "[write] j6 arm/command msg: %f", arm_commands.position[5]);

        //* Publish arm_commands to arm/command topic
        if (rclcpp::ok())
        {
            // LOG_INFO("[write] publishing command: ");        
            joint_commands_pub_->publish(arm_commands);
        }

        return hardware_interface::return_type::OK;
    }


    hardware_interface::CallbackReturn ArmHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // TODO: Begin cyclic data (PDO) exchange by way of topic

        return CallbackReturn::SUCCESS;
    }


    hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // TODO: Cease cyclic data (PDO) exchange and stop ethercat master by way of topic

        return CallbackReturn::SUCCESS;
    }

}

PLUGINLIB_EXPORT_CLASS(zeroerr_hardware::ArmHardwareInterface, hardware_interface::SystemInterface);