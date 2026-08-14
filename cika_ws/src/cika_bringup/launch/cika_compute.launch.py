#cika_compute.launch.py 
#this is a launch file that runs on a system together with cika_real.launch.py. It contains the heavy compute nodes that are not run on the robot itself (Current State RPI 4b).

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    AnyLaunchDescriptionSource,
)
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cika_bringup_dir = get_package_share_directory("cika_bringup")
    cika_navigation_dir = get_package_share_directory("cika_navigation")
    foxglove_bridge_dir = get_package_share_directory("foxglove_bridge")
    cika_description_dir = get_package_share_directory("cika_description")

    ekf_real_path    = os.path.join(cika_bringup_dir,     "config", "ekf_real.yaml")

    
    robot_description_content = ParameterValue(
        Command([
            "xacro ",
            os.path.join(cika_description_dir, "urdf", "cika.xacro"),
            " use_sim:=false",
        ]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description_content,
            "use_sim_time": False,
        }],
    )

    # ── Arguments ─────────────────────────────────────────────────────────────

    # ── Heavy Compute Nodes ───────────────────────────────────────────────────
    laser_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[os.path.join(cika_bringup_dir, "config", "laser_filter.yaml")],
        remappings=[
            ("scan", "/scan_raw"),
            ("scan_filtered", "/scan"),
        ],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_real_path, {"use_sim_time": False}],
    )

    foxglove_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(foxglove_bridge_dir, "launch", "foxglove_bridge_launch.xml")
        )
    )

    return LaunchDescription(
        [
            ekf_node,
            laser_filter_node,
            robot_state_publisher_node,
            foxglove_bridge,
        ]
    )
