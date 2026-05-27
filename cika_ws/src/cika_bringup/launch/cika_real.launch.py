import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    RegisterEventHandler,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
    cika_description_dir = get_package_share_directory("cika_description")
    cika_bringup_dir     = get_package_share_directory("cika_bringup")
    cika_navigation_dir  = get_package_share_directory("cika_navigation")
    foxglove_bridge_dir  = get_package_share_directory("foxglove_bridge")

    controllers_yaml = os.path.join(cika_description_dir, "config", "controllers_real.yaml")
    joy_ps5_params   = os.path.join(cika_bringup_dir,     "config", "joy_ps5_real.yaml")
    ekf_real_path    = os.path.join(cika_bringup_dir,     "config", "ekf_real.yaml")
    nav2_hw_path     = os.path.join(cika_bringup_dir,     "config", "nav2_params_real.yaml")
    rtabmap_db_path  = os.path.expanduser("~/cika_maps/cika_map.db")

    # ── Arguments ─────────────────────────────────────────────────────────────
    teleop_arg = DeclareLaunchArgument(name="teleop", default_value="true")
    nav_arg    = DeclareLaunchArgument(name="nav", default_value="false")
    mode_arg   = DeclareLaunchArgument(name="mode", default_value="navigation")
    use_perception_arg = DeclareLaunchArgument(name="use_perception", default_value="false")

    # ── Robot description ─────────────────────────────────────────────────────
    robot_description_content = ParameterValue(
        Command(["xacro ", os.path.join(cika_description_dir, "urdf", "cika.xacro"), " use_sim:=false"]),
        value_type=str,
    )

    # ── Core Nodes ────────────────────────────────────────────────────────────
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content, "use_sim_time": False}],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[{"robot_description": robot_description_content}, controllers_yaml, {"use_sim_time": False}],
    )

    # ── RPLIDAR (Official Launch + Motor Start) ───────────────────────────────
    delayed_lidar = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("sllidar_ros2"), "launch", "sllidar_c1_launch.py")
                ),
                launch_arguments={
                    "serial_port": "/dev/rplidar",
                    "serial_baudrate": "460800",
                    "frame_id": "lidar_1",
                    "scan_frequency": "8.0",      # Lowered a bit for stability
                    "angle_compensate": "true",
                }.items(),
            )
        ]
    )

    # Multiple motor start attempts with delays
    delayed_motor = TimerAction(
        period=18.0,
        actions=[
            ExecuteProcess(cmd=['ros2', 'service', 'call', '/start_motor', 'std_srvs/srv/Empty'], output='screen'),
            TimerAction(period=4.0, actions=[ExecuteProcess(cmd=['ros2', 'service', 'call', '/start_motor', 'std_srvs/srv/Empty'], output='screen')]),
            TimerAction(period=8.0, actions=[ExecuteProcess(cmd=['ros2', 'service', 'call', '/start_motor', 'std_srvs/srv/Empty'], output='screen')]),
        ]
    )

    laser_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[os.path.join(cika_bringup_dir, "config", "laser_filter.yaml")],
        remappings=[("scan", "/scan_raw"), ("scan_filtered", "/scan")],
    )

    # ... (keep your other nodes: inference, madgwick, ekf, etc.)

    inference_node = Node(...)   # your existing one
    madgwick_node = Node(...)    # your existing one
    ekf_node = Node(...)         # your existing one

    # Controller spawners (keep as is)
    joint_state_broadcaster_spawner = Node(...)   # your existing
    skid_steer_controller_spawner = Node(...)     # your existing

    # Teleop, Foxglove, Nav...
    joy_node = Node(...) 
    teleop_node = Node(...)
    foxglove_bridge = IncludeLaunchDescription(...) 
    delayed_nav = TimerAction(period=20.0, actions=[your nav_launch])   # increased a bit

    return LaunchDescription([
        teleop_arg,
        mode_arg,
        use_perception_arg,
        nav_arg,
        
        robot_state_publisher_node,
        delayed_lidar,           # ← Early
        delayed_motor,           # ← Motor attempts
        ros2_control_node,
        laser_filter_node,
        madgwick_node,
        ekf_node,
        inference_node,
        
        delayed_jsb,             # your existing
        delayed_skid_steer,      # your existing
        joy_node,
        teleop_node,
        foxglove_bridge,
        delayed_nav,
    ])