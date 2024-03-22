#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <std_msgs/msg/bool.hpp>

#include "zeroerr_msgs/msg/joint_space_target.hpp"
#include "zeroerr_msgs/msg/pose_target_array.hpp"
#include "zeroerr_msgs/msg/pose_target.hpp"

using namespace std::chrono_literals;

class ArmMoveGroup
{
    public:
        ArmMoveGroup();
        ~ArmMoveGroup();

        void begin_();

        rclcpp::Node::SharedPtr node_;
        rclcpp::Node::SharedPtr mg_node_;



    private:
        const uint NUM_JOINTS = 6;
        const float PI = 3.141592654;
        const std::string PLANNING_GROUP = "arm_group";
        const std::string NODE_NAME = "arm_move_group";
        
        bool joint_space_goal_recv_ = false;
        bool pose_goal_recv_ = false;

        rclcpp::Subscription<zeroerr_msgs::msg::JointSpaceTarget>::SharedPtr arm_joint_space_sub_;
        rclcpp::Subscription<zeroerr_msgs::msg::JointSpaceTarget>::SharedPtr pose_array_sub_;
        rclcpp::Subscription<zeroerr_msgs::msg::PoseTarget>::SharedPtr arm_point_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_execute_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_stop_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_clear_sub_;


        void arm_joint_space_cb_(zeroerr_msgs::msg::JointSpaceTarget::SharedPtr goal_msg);
        void pose_array_cb_(zeroerr_msgs::msg::PoseTargetArray::SharedPtr pose_array_msg);
        void arm_pose_cb_(zeroerr_msgs::msg::PoseTarget::SharedPtr goal_msg);
        void arm_execute_cb_(const std_msgs::msg::Bool::SharedPtr execute_msg);
        void arm_stop_cb_(const std_msgs::msg::Bool::SharedPtr stop_msg);
        void arm_clear_cb_(const std_msgs::msg::Bool::SharedPtr clear_msg);
        void timer_cb_();

        zeroerr_msgs::msg::JointSpaceTarget arm_joint_space_cmd_;
        zeroerr_msgs::msg::PoseTarget arm_point_cmd_;
        moveit_msgs::msg::CollisionObject table_;


        moveit::planning_interface::MoveGroupInterface::Plan plan_;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};