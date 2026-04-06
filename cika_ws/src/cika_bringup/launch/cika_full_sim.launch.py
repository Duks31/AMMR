import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get package directories
    bringup_dir = get_package_share_directory("cika_bringup")
    nav_dir = get_package_share_directory("cika_navigation")

    # 1. Include Gazebo Launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, "launch", "gazebo.launch.py")),
        launch_arguments={
            "gui": "false", 
            "use_sim_time": "true"
        }.items(),
    )

    # 2. Include SLAM Launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_dir, "launch", "slam.launch.py")),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # 3. Include Display Launch
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, "launch", "display.launch.py")),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    delayed_slam = TimerAction(period=10.0, actions=[slam_launch])
    
    delayed_display = TimerAction(period=15.0, actions=[display_launch])

    return LaunchDescription([
        gazebo_launch,
        delayed_slam,
        delayed_display
    ])
