import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter, SetRemap

def launch_setup(context, *args, **kwargs):
    use_sim = LaunchConfiguration("use_sim_time").perform(context)
    mode = LaunchConfiguration("mode").perform(context)

    cika_bringup = get_package_share_directory("cika_bringup")
    nav2_bringup = get_package_share_directory("nav2_bringup")
    slam_toolbox_dir = get_package_share_directory("slam_toolbox")

    if use_sim == "true":
        ekf_params = os.path.join(cika_bringup, "config", "ekf_sim.yaml")
        nav2_params = os.path.join(cika_bringup, "config", "nav2_params_sim.yaml")
    else:
        ekf_params = os.path.join(cika_bringup, "config", "ekf_real.yaml")
        nav2_params = os.path.join(cika_bringup, "config", "nav2_params_real.yaml")

    slam_params = os.path.join(cika_bringup, "config", "mapper_params_online_async.yaml")
    map_yaml = os.path.expanduser("~/cika_maps/cika_map.yaml")

    launch_actions = []

    launch_actions.append(
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_params, {"use_sim_time": use_sim == "true"}]
        )
    )

    if mode == "slam":
        launch_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(slam_toolbox_dir, "launch", "online_async_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim,
                    "slam_params_file": slam_params,
                }.items(),
            )
        )
    elif mode == "navigation":
        launch_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup, "launch", "localization_launch.py")),
                launch_arguments={"use_sim_time": use_sim, "map": map_yaml, "params_file": nav2_params}.items(),
            )
        )
        launch_actions.append(
            GroupAction(
                actions=[
                    SetRemap(src="/cmd_vel", dst="/skid_steer_controller/cmd_vel_unstamped"),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(os.path.join(nav2_bringup, "launch", "navigation_launch.py")),
                        launch_arguments={"use_sim_time": use_sim, "params_file": nav2_params}.items(),
                        ),
                    ]
                )
            )
    return launch_actions

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="slam", choices=["slam", "navigation"], description="Mode: slam or navigation"),
            DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time"),

            SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),

            OpaqueFunction(function=launch_setup),    

        ]
    )