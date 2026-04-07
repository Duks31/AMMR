import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get package directories
    bringup_dir = get_package_share_directory("cika_bringup")
    nav_dir = get_package_share_directory("cika_navigation")

    mode_arg = DeclareLaunchArgument(
        name="mode",
        default_value="slam",
        choices=["slam", "navigation"],
        description="slam = build map, navigation = localize + Nav2",
    )
    mode = LaunchConfiguration("mode")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"gui": "false", "use_sim_time": "true"}.items(),
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_dir, "launch", "cika_nav.launch.py")), 
        launch_arguments={
            "use_sim_time": "true",
            "mode": mode
        }.items(),
    )

    # Include Display Launch
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, "launch", "display.launch.py")),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    delayed_nav = TimerAction(period=10.0, actions=[nav_launch])
    delayed_display = TimerAction(period=15.0, actions=[display_launch])

    return LaunchDescription([
        mode_arg, # --- 3. RETURN IT: Make sure the master launch knows it exists ---
        gazebo_launch,
        delayed_nav,
        delayed_display
    ])
