import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # LOGIC: Locate the 'Turntable_description' package folder.
    turntable_description_dir = get_package_share_directory("robot_arm_description")

    # SYNTAX: Creates a command-line argument named 'model'.
    # Allows you to override the file path: 'ros2 launch ... model:=/new/path/robot.xacro'
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(
            turntable_description_dir, "urdf", "robot_arm_description.xacro"
        ),
        description="Absolute path to robot urdf file"
    )

    # SYNTAX: This is a nested substitution.
    # 1. LaunchConfiguration("model") gets the path from the argument above.
    # 2. Command([...]) runs the shell command 'xacro /path/to/file'.
    # 3. ParameterValue(...) captures the text output of that command so it can be passed as a string parameter.
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),
                                       value_type=str)

    # LOGIC: This node listens to joint angles and publishes the 3D transforms (TF) of the robot.
    # It needs the URDF (robot_description) to know how long the robot parts are.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # LOGIC: Opens a small GUI window with sliders.
    # It publishes 'sensor_msgs/msg/JointState' messages to move the robot without physics.
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    # LOGIC: Launches the main visualization tool (RViz).
    # The '-d' argument tells it to load a pre-saved layout.
    # CHANGED: Updated folder from 'config' to 'rviz'
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(turntable_description_dir, "rviz", "display.rviz")],
    )

    # SYNTAX: The return list determines the order of execution. 
    # Arguments usually come first so they are available for the nodes.
    return LaunchDescription([
        model_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])