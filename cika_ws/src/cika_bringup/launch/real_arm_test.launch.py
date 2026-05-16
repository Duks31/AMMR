import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cika_description_dir = get_package_share_directory("cika_description")

    controllers_yaml = os.path.join(
        cika_description_dir, "config", "controllers_real.yaml"
    )

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

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        output="screen",
    )

    # ── Sequencing ───────────────────────────────────────────────────────────

    # Wait 10s for hardware to activate before spawning joint_state_broadcaster
    delayed_jsb = TimerAction(
        period=10.0,
        actions=[joint_state_broadcaster_spawner],
    )

    # Spawn arm_controller only after joint_state_broadcaster exits successfully
    delayed_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                TimerAction(period=2.0, actions=[arm_controller_spawner]),
            ],
        )
    )

    # Spawn gripper_controller only after arm_controller exits successfully
    delayed_gripper = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[
                TimerAction(period=10.0, actions=[gripper_controller_spawner]),
            ],
        )
    )

    return LaunchDescription([
        robot_state_publisher_node,
        ros2_control_node,
        delayed_jsb,
        delayed_arm,
        delayed_gripper,
    ])