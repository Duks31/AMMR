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


def generate_launch_description():
    cika_bringup_dir = get_package_share_directory("cika_bringup")
    cika_navigation_dir = get_package_share_directory("cika_navigation")
    foxglove_bridge_dir = get_package_share_directory("foxglove_bridge")

    ekf_real_path = os.path.join(cika_bringup_dir, "config", "ekf_real.yaml")

    # ── Arguments ─────────────────────────────────────────────────────────────
    use_perception_arg = DeclareLaunchArgument(
        name="use_perception",
        default_value="false",
        choices=["true", "false"],
        description="Launch YOLOv8 inference node",
    )

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

    inference_node = Node(
        package="cika_perception",
        executable="inference_node.py",
        name="inference_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_perception")),
    )

    foxglove_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(foxglove_bridge_dir, "launch", "foxglove_bridge_launch.xml")
        )
    )

    return LaunchDescription(
        [
            use_perception_arg,
            laser_filter_node,
            inference_node,
            ekf_node,
            foxglove_bridge,
        ]
    )
