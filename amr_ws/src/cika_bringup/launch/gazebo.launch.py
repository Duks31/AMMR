import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cika_description = get_package_share_directory("cika_description")
    ros_gz_sim = get_package_share_directory("ros_gz_sim")

    controllers_yaml = os.path.join(cika_description, "config", "controllers.yaml")

    bridge_config = os.path.join(
        get_package_share_directory("cika_bringup"), "config", "ros_gz_bridge.yaml"
    )

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(cika_description, "urdf", "cika.xacro"),
        description="Absolute path to robot xacro file",
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(cika_description).parent.resolve())],
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str,
    )

    world_path = os.path.join(cika_description, "worlds", "world.sdf")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments=[("gz_args", "-v 2 -r " + world_path)],
    )

    # === DELAYED SPAWN (fixes Visual already exists + race condition) ===
    gz_spawn_entity = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    "-topic",
                    "robot_description",
                    "-name",
                    "cika",
                    "-x",
                    "0.0",
                    "-y",
                    "0.0",
                    "-z",
                    "0.15",
                ],
            )
        ],
    )

    # GazeboSimROS2ControlPlugin owns this
    # ros2_control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[
    #         {"robot_description": robot_description},
    #         controllers_yaml,
    #         {"use_sim_time": True},
    #     ],
    #     output="screen",
    # )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_config}],
        output="screen",
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
            )
        ],
    )

    arm_controller_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "arm_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
            )
        ],
    )

    gripper_controller_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "gripper_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
            )
        ],
    )

    skid_steer_controller_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "skid_steer_controller",
                    "--controller-manager",
                    "/controller_manager",
                    "--controller-manager-timeout",
                    "30",
                ],
                remappings=[
                    ("/cmd_vel", "/skid_steer_controller/cmd_vel_unstamped"),
                ],
            )
        ],
    )

    nvidia_offload = SetEnvironmentVariable(name="__NV_PRIME_RENDER_OFFLOAD", value="1")
    nvidia_vendor = SetEnvironmentVariable(
        name="__GLX_VENDOR_LIBRARY_NAME", value="nvidia"
    )

    return LaunchDescription(
        [
            model_arg,
            gazebo_resource_path,
            nvidia_offload,
            nvidia_vendor,
            robot_state_publisher_node,
            gazebo,
            gz_spawn_entity,
            # ros2_control_node, ← REMOVED, GazeboSimROS2ControlPlugin owns this
            ros_gz_bridge,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            gripper_controller_spawner,
            skid_steer_controller_spawner,
        ]
    )
