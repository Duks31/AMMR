import os
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    is_sim = LaunchConfiguration("is_sim")
    
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )

    moveit_config = (
        MoveItConfigsBuilder("robot_arm", package_name="cika_manipulator")
        .robot_description(file_path=os.path.join(
            get_package_share_directory("cika_description"), 
            "urdf",
            "cika.xacro" 
            )
        )
        .robot_description_semantic(file_path="config/cika.srdf") 
        .trajectory_execution(file_path="config/moveit_controllers.yaml") 
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    task_server_node = Node(
        package="cika_manipulator",
        executable="task_server",
        name="task_server",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": is_sim}
        ]
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), 
                    {"use_sim_time": is_sim},
                    {"publish_robot_description_semantic": True}],
        arguments=["--ros-args", "--log-level", "info"],
    )

    return LaunchDescription(
        [
            is_sim_arg,
            move_group_node, 
            task_server_node
        ]
    )