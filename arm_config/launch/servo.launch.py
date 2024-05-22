import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_param_builder import ParameterBuilder
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # Use real hardware
    """
    Change hardware type parameter in zeroerr_arm_servo.ros2_control.xacro to use
    arm_hardware/ArmHardwareInterface plugin (actual hardware)
    """
    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="mock_components",
        description="ROS 2 control hardware interface type to use for the launch file -- possible values: [mock_components, real]",
    )

    moveit_config = (
        MoveItConfigsBuilder("ArmProject", package_name="arm_config")
        .robot_description(
            file_path="config/zeroerr_arm_servo.urdf.xacro",
            mappings={"ros2_control_hardware_type": LaunchConfiguration("ros2_control_hardware_type")},
        )
        .robot_description_semantic(file_path="config/zeroerr_arm.srdf")
        .robot_description_kinematics(file_path="config/servo_kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, 
            publish_robot_description_semantic=True
        )
        .joint_limits(file_path="config/servo_joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=[
                "ompl", 
                "chomp", 
                "pilz_industrial_motion_planner", 
                "stomp"
            ]
        )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    arm_interface = Node(
        package="arm_ethercat_interface",
        executable="arm_ethercat_interface",
        output="screen",
    )

    # RViz
    rviz_config = os.path.join(
        get_package_share_directory("arm_config"),
        "config", 
        "servo.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "arm_Link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("arm_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_group_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_group_controller", "-c", "/controller_manager"],
        # prefix=["sudo -E"]
    )


    servo_params = {
        "moveit_servo": ParameterBuilder("arm_config")
        .yaml("config/servo_parameters.yaml")
        .to_dict()
    }
    # print(servo_params)

    # This sets the update rate and planning group name for the acceleration limiting filter.
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "arm_group"}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        parameters=[
            servo_params,
            acceleration_filter_update_period,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            ros2_control_hardware_type,
            rviz_node,
            static_tf_node,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_group_spawner,
            servo_node
        ]
    )
