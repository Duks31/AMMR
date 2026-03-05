import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="True")

    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("robot_arm_description"),
                    "urdf",
                    "robot_arm_description.xacro",
                ),
                " is_ignition:=true",
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        condition=UnlessCondition(use_sim_time),
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": False},
            os.path.join(
                get_package_share_directory("robot_arm_controller"),
                "config",
                "robot_arm_controllers.yaml",
            ),
        ],
        condition=UnlessCondition(use_sim_time),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "50",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "50",
        ],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "50",
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            robot_state_publisher_node,
            controller_manager,
            joint_state_broadcaster_spawner,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[arm_controller_spawner, gripper_controller_spawner],
                )
            ),
        ]
    )
