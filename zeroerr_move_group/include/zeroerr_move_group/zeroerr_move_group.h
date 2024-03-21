#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include "zeroerr_msgs/msg/arm_joint_space.hpp"
#include "zeroerr_msgs/msg/arm_point.hpp"

zeroerr_msgs::msg::ArmJointSpace arm_joint_space_cmd_;
zeroerr_msgs::msg::ArmPoint arm_point_cmd_;