import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    base_controller = get_package_share_directory("base_controller")

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge",
        parameters=[
            {
                "config_file": os.path.join(base_controller, "config", "bridge.yaml"),
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    # Delay spawning to give the controller_manager time to fully start
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
        parameters=[{"use_sim_time": True}],
    )
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "50",
        ],
        parameters=[{"use_sim_time": True}]
    )

    # Start diff_drive only after joint_state_broadcaster is active
    delayed_jsb = TimerAction(
        period=10.0,
        actions=[joint_state_broadcaster_spawner],
    )

    delayed_diff_drive = TimerAction(
        period=12.0,
        actions=[diff_drive_spawner],
    )

    return LaunchDescription(
        [
            bridge,
            delayed_jsb,
            delayed_diff_drive,
        ]
    )
