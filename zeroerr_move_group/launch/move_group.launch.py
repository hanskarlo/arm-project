from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("zeroerr_arm", package_name="zeroerr_config").to_moveit_configs()

    # MoveGroupInterface demo executable
    move_group = Node(
        # name="arm_move_group",
        package="zeroerr_move_group",
        executable="zeroerr_move_group",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True} #! Will not receive joint_states if False
        ],
    )

    return LaunchDescription([move_group])