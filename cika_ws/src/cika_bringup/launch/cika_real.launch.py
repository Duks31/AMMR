"""
cika_real.launch.py
====================
Bringup launch for the physical cika robot (no Gazebo).

Starts:
  - robot_state_publisher  (URDF with use_sim:=false → CikaDriveInterface)
  - ros2_control_node      (controller_manager — owns controller lifecycle)
  - joint_state_broadcaster
  - skid_steer_controller  (diff_drive_controller)
  - micro-ROS agent        (serial bridge to drive ESP32)
  - PS5 joystick teleop    (optional, enabled by default)

Usage:
  ros2 launch cika_bringup cika_real.launch.py
  ros2 launch cika_bringup cika_real.launch.py teleop:=false
  ros2 launch cika_bringup cika_real.launch.py serial_port:=/dev/ttyUSB1
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    RegisterEventHandler,
    ExecuteProcess,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cika_description_dir = get_package_share_directory("cika_description")
    cika_bringup_dir     = get_package_share_directory("cika_bringup")

    controllers_yaml = os.path.join(cika_description_dir, "config", "controllers.yaml")
    joy_ps5_params   = os.path.join(cika_bringup_dir, "config", "joy_ps5.yaml")

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
    # This swaps gz_ros2_control/GazeboSimSystem → cika_hardware/CikaDriveInterface
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

    # On real hardware the controller_manager is NOT owned by Gazebo —
    # we start it explicitly here.
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

    # micro-ROS agent — bridges the drive ESP32 to ROS2.
    # NOTE: microros_ws must be sourced before calling this launch file.
    # Run: source ~/code/ws/AMMR/microros_ws/install/setup.bash
    # Or add it to your ~/.bashrc alongside the cika_ws source line.
    micro_ros_agent = ExecuteProcess(
        cmd=[
            "ros2", "run", "micro_ros_agent", "micro_ros_agent",
            "serial",
            "--dev", LaunchConfiguration("serial_port"),
            "-b", "115200",
        ],
        output="screen",
        name="micro_ros_agent",
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
        # Keep the same remapping as the sim launch so Nav2 / teleop work unchanged
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

    # ── Sequencing ───────────────────────────────────────────────────────────
    # Wait for ros2_control_node before spawning controllers.
    # We use a TimerAction rather than OnProcessExit because ros2_control_node
    # is a long-running process (it never exits during normal operation).
    # 5 s gives the controller_manager time to initialise and load the plugin.

    delayed_jsb = TimerAction(
        period=10.0,
        actions=[joint_state_broadcaster_spawner],
    )

    # Spawn skid_steer_controller only after joint_state_broadcaster is active
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

        # Start micro-ROS agent first so the ESP32 can connect
        micro_ros_agent,

        robot_state_publisher_node,
        ros2_control_node,

        delayed_jsb,
        delayed_skid_steer,

        joy_node,
        teleop_node,
    ])