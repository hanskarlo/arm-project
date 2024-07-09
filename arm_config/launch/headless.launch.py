import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_param_builder import ParameterBuilder
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # Use real hardware
    """
    Change hardware type parameter in zeroerr_arm.ros2_control.xacro to use
    arm_hardware/ArmHardwareInterface plugin (actual hardware)
    """
    ros2_control_hardware_type = DeclareLaunchArgument(
        "hardware_type",
        default_value="real",
        description="ROS 2 control hardware interface type to use for the launch file -- possible values: [sim, real]",
    )

    control_mode = DeclareLaunchArgument(
        "control_mode",
        default_value="plan",
        description="Control mode of arm -- possible values: [plan, servo]",
    )


    moveit_config = (
        MoveItConfigsBuilder("ArmProject", package_name="arm_config")
        .robot_description(
            file_path="config/zeroerr_arm.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration("hardware_type"),
                "control_mode": LaunchConfiguration("control_mode")},
        )
        .robot_description_semantic(file_path="config/zeroerr_arm.srdf")
        .planning_scene_monitor(
            publish_robot_description=True, 
            publish_robot_description_semantic=True
        )
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
        remappings=[
            ('/robot_description', '/arm/robot_description')
        ]
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
        "moveit.rviz"
    )

    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["1.0", "1.0", "1.0", "0.0", "0.0", "0.0", "odom", "arm_Link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="arm_state_publisher",
        output="both",
        remappings=[
            ('/robot_description', '/arm/robot_description')
        ],
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using real hardware
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

    domain_bridge_node = Node(
        package="domain_bridge",
        executable="domain_bridge",
        arguments=[
            "/home/aroarm0/arm_iron_ws/src/zeroerr_arm/arm_config/config/arm_bridge.yaml"
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c", "/controller_manager",
        ],
    )

    arm_group_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_group_controller", 
                   "-c", "/controller_manager"],
    )

    arm_move_group = Node(
        package="arm_move_group",
        executable="arm_move_group",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True}, #! Will not receive joint_states if False
            {"visualize_trajectory": False},
            {"servoing": False}
        ],
        
    )

    add_robot_scene = Node(
        package="arm_move_group",
        executable="add_robot_scene.py",
    )

    robot_server = Node(
        package="arm_move_group",
        executable="robot_server.py"
    )

    
    return LaunchDescription(
        [
            ros2_control_hardware_type,
            control_mode,
            static_tf_node,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_group_spawner,
            domain_bridge_node,
            arm_move_group,
            add_robot_scene,
            robot_server,
        ]
    )
