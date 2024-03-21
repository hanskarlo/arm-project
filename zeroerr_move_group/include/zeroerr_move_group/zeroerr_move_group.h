#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "zeroerr_msgs/msg/arm_joint_space.hpp"
#include "zeroerr_msgs/msg/arm_point.hpp"


class ArmMoveGroup
{
    public:
        ArmMoveGroup();
        ~ArmMoveGroup();

        rclcpp::Node::SharedPtr node_;

    private:
        const uint NUM_JOINTS = 6;
        const float PI = 3.141592654;
        const std::string LOGGER = "arm_move_group";
        const std::string PLANNING_GROUP = "arm_group";
        const std::string NODE_NAME = "arm_move_group";

        rclcpp::Subscription<zeroerr_msgs::msg::ArmJointSpace>::SharedPtr arm_joint_space_sub_;
        rclcpp::Subscription<zeroerr_msgs::msg::ArmPoint>::SharedPtr arm_point_sub_;
        void arm_joint_space_cb_(zeroerr_msgs::msg::ArmJointSpace::SharedPtr goal_msg);
        void arm_point_cb_(zeroerr_msgs::msg::ArmPoint::SharedPtr goal_msg);


        zeroerr_msgs::msg::ArmJointSpace arm_joint_space_cmd_;
        zeroerr_msgs::msg::ArmPoint arm_point_cmd_;
        moveit::planning_interface::MoveGroupInterfacePtr move_group_;
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
        // moveit::core::JointModelGroup* joint_model_group_;

};