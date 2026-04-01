"""
LAUNCH BOTH GAZEBO AND CONTROLLER
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_arm_description"),
                "launch",
                "gazebo.launch.py"
            )
        )
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_arm_controller"),
                "launch",
                "controller.launch.py"
            )
        ),
        launch_arguments={"use_sim_time": "True"}.items()
    )

    return LaunchDescription([
        gazebo_launch,
        TimerAction(
            period=5.0,
            actions=[controller_launch]
        ),
    ])