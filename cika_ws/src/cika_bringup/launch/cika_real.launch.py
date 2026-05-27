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

from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue


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
    teleop_arg = DeclareLaunchArgument(
        name="teleop",
        default_value="true",
        choices=["true", "false"],
        description="Start PS5 joystick teleop",
    )

    nav_arg = DeclareLaunchArgument(
        name="nav",
        default_value="false",
        choices=["true", "false"],
        description="Launch RTAB-Map + Nav2 after bringup (requires a saved map for navigation mode)",
    )

    mode_arg = DeclareLaunchArgument(
        name="mode",
        default_value="navigation",
        choices=["slam", "navigation"],
        description="slam = build map, navigation = localize + Nav2",
    )

    use_perception_arg = DeclareLaunchArgument(
        name="use_perception",
        default_value="false",
        choices=["true", "false"],
        description="Launch inference node (requires OAK-D)",
    )

    # ── Robot description ─────────────────────────────────────────────────────
    robot_description_content = ParameterValue(
        Command([
            "xacro ",
            os.path.join(cika_description_dir, "urdf", "cika.xacro"),
            " use_sim:=false",
        ]),
        value_type=str,
    )

    # ── Core nodes ────────────────────────────────────────────────────────────
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description_content,
            "use_sim_time": False,
        }],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            controllers_yaml,
            {"use_sim_time": False},
        ],
    )

    delayed_lidar = TimerAction(
    period=20.0, # Give the system 5 seconds to settle before hitting the LiDAR
    actions=[
        Node(
            package="sllidar_ros2",
            executable="sllidar_node",
            name="sllidar_node",
            output="screen",
            parameters=[{
                "serial_port": "/dev/rplidar",
                "serial_baudrate": 460800,
                "frame_id": "lidar_1",
                "angle_compensate": True,
                "scan_mode": "Standard",
                "scan_frequency": 10.0,
            }],
            remappings=[("scan", "/scan_raw")],
            )
        ]
    )

    delayed_motor_start = TimerAction(
    period=12.0,   # Give lidar more time
    actions=[
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/start_motor', 'std_srvs/srv/Empty'],
            output='screen',
            shell=True
            )
        ]
    )

    laser_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[os.path.join(cika_bringup_dir, "config", "laser_filter.yaml")],
        remappings=[
            ("scan",          "/scan_raw"),
            ("scan_filtered", "/scan"),
        ],
    )

    inference_node = Node(
        package="cika_perception",
        executable="inference_node.py",
        name="inference_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_perception")),
    )

    madgwick_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick",
        parameters=[{
            "use_mag": False,
            "gain": 0.1,
            "publish_tf": False,
            "world_frame": "enu",
        }],
        remappings=[
            ("/imu/data_raw", "/imu/raw"),
            ("/imu/data",     "/imu/filtered"),
        ],
        output="screen",
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_real_path, {"use_sim_time": False}],
    )

    # ── Controller spawning (sequenced) ───────────────────────────────────────
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        output="screen",
    )

    skid_steer_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "skid_steer_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
        ],
        output="screen",
        remappings=[("/cmd_vel", "/skid_steer_controller/cmd_vel_unstamped")],
    )

    # ── Teleop ────────────────────────────────────────────────────────────────
    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[joy_ps5_params],
        condition=IfCondition(LaunchConfiguration("teleop")),
    )

    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        parameters=[joy_ps5_params],
        remappings=[("/cmd_vel", "/skid_steer_controller/cmd_vel_unstamped")],
        condition=IfCondition(LaunchConfiguration("teleop")),
    )

    # ── Foxglove bridge ───────────────────────────────────────────────────────
    foxglove_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(foxglove_bridge_dir, "launch", "foxglove_bridge_launch.xml")
        )
    )

    # ── Nav stack (optional, delayed until controllers are up) ────────────────
    # Delay of 15s gives ros2_control + controller spawners time to fully settle.
    # Increase to 20s if you see RTAB-Map failing to find /odometry/filtered on boot.
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cika_navigation_dir, "launch", "cika_nav.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "mode":         LaunchConfiguration("mode"),
            # "ekf_config":   ekf_real_path,
            "nav2_params":  nav2_hw_path,
            "rtabmap_db":   rtabmap_db_path,
        }.items(),
        condition=IfCondition(LaunchConfiguration("nav")),
    )

    delayed_nav = TimerAction(period=15.0, actions=[nav_launch])

    # ── Sequencing ────────────────────────────────────────────────────────────
    delayed_jsb = TimerAction(
        period=10.0,
        actions=[joint_state_broadcaster_spawner],
    )

    delayed_skid_steer = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                TimerAction(period=2.0, actions=[skid_steer_controller_spawner]),
            ],
        )
    )

    return LaunchDescription([
        teleop_arg,
        nav_arg,
        mode_arg,
        use_perception_arg,
        robot_state_publisher_node,
        ros2_control_node,
        delayed_lidar,
        delayed_motor_start,
        laser_filter_node,
        inference_node,
        madgwick_node,
        ekf_node,
        delayed_jsb,
        delayed_skid_steer,
        joy_node,
        teleop_node,
        foxglove_bridge,
        delayed_nav,
    ])