import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter, SetRemap


def generate_launch_description():

    cika_bringup = get_package_share_directory("cika_bringup")
    nav2_bringup = get_package_share_directory("nav2_bringup")
    slam_toolbox_dir = get_package_share_directory("slam_toolbox")

    # ── Default paths (sim) ───────────────────────────────────────────────────
    default_nav2_params = os.path.join(cika_bringup, "config", "nav2_params_real.yaml")
    default_slam_params = os.path.join(
        cika_bringup, "config", "mapper_params_online_async.yaml"
    )
    # default_rtabmap_db = os.path.expanduser("~/cika_maps/cika_map.db")

    # SLAM Toolbox and Nav2 use standard .yaml map files, not RTAB-Map's .db
    default_map_yaml = os.path.expanduser("~/cika_maps/cika_map.yaml")

    # ── Arguments ─────────────────────────────────────────────────────────────
    mode_arg = DeclareLaunchArgument(
        name="mode",
        default_value="navigation",
        choices=["slam", "navigation"],
        description="slam = build map, navigation = localize (AMCL) + Nav2",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        description="Use simulation clock",
    )

    # Callers override these for hardware:
    #   ekf_config:=<path>/ekf_real.yaml
    #   nav2_params:=<path>/nav2_params_hw.yaml

    # ekf_config_arg = DeclareLaunchArgument(
    #     name="ekf_config",
    #     default_value=default_ekf_config,
    #     description="Path to EKF yaml (swap for hardware)",
    # )

    nav2_params_arg = DeclareLaunchArgument(
        name="nav2_params",
        default_value=default_nav2_params,
        description="Path to Nav2 params yaml (swap for hardware)",
    )

    slam_params_arg = DeclareLaunchArgument(
        name="slam_params_file",
        default_value=default_slam_params,
        description="Path to SLAM Toolbox params",
    )

    map_yaml_arg = DeclareLaunchArgument(
        name="map",
        default_value=default_map_yaml,
        description="Path to standard ROS 2 map yaml file",
    )

    # rtabmap_db_arg = DeclareLaunchArgument(
    #     name="rtabmap_db",
    #     default_value=default_rtabmap_db,
    #     description="Path to RTAB-Map database file",
    # )

    use_sim_time = LaunchConfiguration("use_sim_time")
    mode = LaunchConfiguration("mode")
    nav2_params = LaunchConfiguration("nav2_params")
    slam_params_file = LaunchConfiguration("slam_params_file")
    map_yaml = LaunchConfiguration("map")
    # rtabmap_db = LaunchConfiguration("rtabmap_db")

    # ── Global sim time parameter ─────────────────────────────────────────────
    set_sim_time = SetParameter(name="use_sim_time", value=use_sim_time)

    # ── SLAM Toolbox (SLAM mode only) ─────────────────────────────────────────
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_params_file,
        }.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'slam'"])),
    )

    # ── AMCL & Map Server (Navigation mode only) ──────────────────────────────
    # This replaces the localization functionality of RTAB-Map
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, "launch", "localization_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": map_yaml,
            "params_file": nav2_params,
        }.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'navigation'"])),
    )

    # ── Nav2 (Navigation mode only) ───────────────────────────────────────────
    nav2_launch = GroupAction(
        condition=IfCondition(PythonExpression(["'", mode, "' == 'navigation'"])),
        actions=[
            SetRemap(src="/cmd_vel", dst="/skid_steer_controller/cmd_vel_unstamped"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup, "launch", "navigation_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": nav2_params,
                }.items(),
            ),
        ],
    )

    # ── Nav2 (navigation mode only) ───────────────────────────────────────────
    nav2_launch = GroupAction(
        condition=IfCondition(PythonExpression(["'", mode, "' == 'navigation'"])),
        actions=[
            SetRemap(src="/cmd_vel", dst="/skid_steer_controller/cmd_vel_unstamped"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup, "launch", "navigation_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": nav2_params,
                }.items(),
            ),
        ],
    )

    return LaunchDescription(
        [
            mode_arg,
            use_sim_time_arg,
            nav2_params_arg,
            slam_params_arg,
            map_yaml_arg,
            set_sim_time,
            slam_toolbox_launch,
            localization_launch,
            nav2_launch,
        ]
    )
