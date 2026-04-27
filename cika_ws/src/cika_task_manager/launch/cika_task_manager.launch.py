import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory("cika_task_manager")
    config = os.path.join(pkg, "config", "task_manager.yaml")

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation clock",
    )

    task_manager_node = Node(
        package="cika_task_manager",
        executable="task_manager_node.py",
        name="task_manager_node",
        output="screen",
        parameters=[config, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        task_manager_node,
    ])