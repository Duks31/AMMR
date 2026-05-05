import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    RegisterEventHandler,
    IncludeLaunchDescription,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource # Added for the bridge

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cika_description_dir = get_package_share_directory("cika_description")
    cika_bringup_dir     = get_package_share_directory("cika_bringup")
    # Locate the foxglove_bridge share directory
    foxglove_bridge_dir  = get_package_share_directory("foxglove_bridge")

    controllers_yaml = os.path.join(cika_description_dir, "config", "controllers_real.yaml")
    joy_ps5_params   = os.path.join(cika_bringup_dir, "config", "joy_ps5.yaml")
    ekf_config_path = os.path.join(cika_bringup_dir, "config", "ekf.yaml")

    # ── Launch arguments ────────────────────────────────────────────────────
    serial_port_arg = DeclareLaunchArgument(
        name="serial_port",
        default_value="/dev/ttyUSB0",
        description="Serial port for the micro-ROS agent (drive ESP32)",
    )

    teleop_arg = DeclareLaunchArgument(
        name="teleop",
        default_value="true",
        choices=["true", "false"],
        description="Start PS5 joystick teleop",
    )

    # ── Robot description — xacro with use_sim:=false ───────────────────────
    robot_description_content = ParameterValue(
        Command([
            "xacro ",
            os.path.join(cika_description_dir, "urdf", "cika.xacro"),
            " use_sim:=false",
        ]),
        value_type=str,
    )

    # ── Nodes ────────────────────────────────────────────────────────────────

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description_content,
            "use_sim_time": False,
        }],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            controllers_yaml,
            {"use_sim_time": False},
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        output="screen",
    )

    skid_steer_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "skid_steer_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        output="screen",
        remappings=[
            ("/cmd_vel", "/skid_steer_controller/cmd_vel_unstamped"),
        ],
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[joy_ps5_params],
        condition=IfCondition(LaunchConfiguration("teleop")),
    )

    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        parameters=[joy_ps5_params],
        remappings=[("/cmd_vel", "/skid_steer_controller/cmd_vel_unstamped")],
        condition=IfCondition(LaunchConfiguration("teleop")),
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config_path, {"use_sim_time": False}],
    )

    # ── Foxglove Bridge ──────────────────────────────────────────────────────
    foxglove_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(foxglove_bridge_dir, "launch", "foxglove_bridge_launch.xml")
        )
    )

    # ── Sequencing ───────────────────────────────────────────────────────────
    delayed_jsb = TimerAction(
        period=10.0,
        actions=[joint_state_broadcaster_spawner],
    )

    delayed_skid_steer = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                TimerAction(period=2.0, actions=[skid_steer_controller_spawner]),
            ],
        )
    )

    return LaunchDescription([
        serial_port_arg,
        teleop_arg,

        robot_state_publisher_node,
        ros2_control_node,

        delayed_jsb,
        delayed_skid_steer,

        joy_node,
        teleop_node,
        ekf_node,
        
        # Add the bridge to the returned LaunchDescription
        foxglove_bridge,
    ])