import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

# The core modules for creating a launch setup
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

# The core modules for running ROS nodes
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Dynamically locates the installation folder of your specific package.
    robot_arm_dir = get_package_share_directory("robot_arm_description")

    # Declares a command-line argument allowing you to dynamically override the URDF path during launch.
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(
            robot_arm_dir, "urdf", "robot_arm_description.xacro"
        ),
        description="Absolute path to the 6-DOF arm urdf file"
    )

    # Injects environment variables so the simulation knows exactly where to search for your STL meshes.
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(robot_arm_dir).parent.resolve())]
    )
    
    ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[str(Path(robot_arm_dir).parent.resolve())]
    )
    
    # CRITICAL FIX 1: You are on ROS 2 Humble. You MUST use Ignition. 
    # This ensures the URDF requests the ign_ros2_control-system plugin which exists on your system.
    is_ignition = "True"

    # Executes the xacro shell command to convert your macro-heavy XACRO file into a raw URDF string.
    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition
        ]),
        value_type=str
    )

    # Initializes the robot_state_publisher to compute the Transform (TF) tree for the arm.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True 
        }]
    )

    # Invokes the Gazebo physics server and loads a blank world.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={'gz_args': '-v 4 -r empty.sdf'}.items()
    )

    # Connects to the running Gazebo server and physically injects (spawns) your URDF into the simulation.
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "robot_arm"],
    )

    # Creates a dedicated bridge to translate messages between Gazebo's isolated network and the ROS 2 network.
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ]
    )

    # Compiles the final execution stack.
    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        ign_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge
    ])