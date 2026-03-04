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

    # Note: Double-check your description package name and URDF file name below!
    moveit_config = (
        MoveItConfigsBuilder("robot_arm", package_name="robot_arm_moveit")
        .robot_description(file_path=os.path.join(
            get_package_share_directory("robot_arm_description"), # <-- CHANGE THIS if your URDF package has a different name
            "urdf",
            "robot_arm_description.xacro" # <-- CHANGE THIS to your exact URDF/Xacro file name
            )
        )
        .robot_description_semantic(file_path="config/robot_arm.srdf") # Matches your screenshot
        .trajectory_execution(file_path="config/moveit_controllers.yaml") # Matches your screenshot
        .to_moveit_configs()
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

    # RViz
    rviz_config = os.path.join(
        get_package_share_directory("robot_arm_moveit"),
        "config",
        "moveit.rviz",
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": is_sim}
        ],
    )

    return LaunchDescription(
        [
            is_sim_arg,
            move_group_node, 
            rviz_node
        ]
    )