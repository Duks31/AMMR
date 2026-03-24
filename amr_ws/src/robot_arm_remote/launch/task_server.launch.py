import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # 1. Declare arguments
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="True"
    )
    is_sim = LaunchConfiguration("is_sim")

    # 2. Build the MoveIt Configuration (The Brain)
    # We MUST include the ompl pipeline here so the Server knows how to calculate time!
    moveit_config = (
        MoveItConfigsBuilder("robot_arm", package_name="robot_arm_moveit")
        .robot_description(file_path=os.path.join(
            get_package_share_directory("robot_arm_description"),
            "urdf",
            "robot_arm_description.xacro"
            )
        )
        .robot_description_semantic(file_path="config/robot_arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"]) # <-- Keeps your custom OMPL math!
        .to_moveit_configs()
    )

    # 3. Create the Task Server Node
    task_server_node = Node(
        package="robot_arm_remote",
        executable="task_server",
        name="task_server",
        output="screen",
        # The Server NEEDS these parameters to understand the robot's physical limits
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": is_sim}
        ]
    )

    return LaunchDescription([
        is_sim_arg,
        task_server_node
    ])