from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("zeroerr_arm", package_name="arm_config").to_moveit_configs()

    visualization_param = DeclareLaunchArgument(
        "visualize_trajectories",
        default_value="True",
        description="Visualize trajectories in RVIZ2. Make 'false' if not launching RVIZ2."
    )

    servo_param = DeclareLaunchArgument(
        "servoing",
        default_value="False",
        description="'True' if servoing, 'False' if not."
    )


    # MoveGroupInterface demo executable
    move_group = Node(
        # name="arm_move_group",
        package="arm_move_group",
        executable="arm_move_group",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True}, #! Will not receive joint_states if False
            {"visualize_trajectory": LaunchConfiguration("visualize_trajectories")},
            {"servoing": LaunchConfiguration("servoing")}
        ],
    )

    return LaunchDescription(
        [
            visualization_param,
            servo_param,
            move_group
        ]
    )