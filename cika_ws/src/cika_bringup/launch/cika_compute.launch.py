import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    AnyLaunchDescriptionSource,
)
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cika_bringup_dir = get_package_share_directory("cika_bringup")
    cika_navigation_dir = get_package_share_directory("cika_navigation")
    foxglove_bridge_dir = get_package_share_directory("foxglove_bridge")

    ekf_real_path = os.path.join(cika_bringup_dir, "config", "ekf_real.yaml")

    cika_description_dir = get_package_share_directory("cika_description")
    
    robot_description_content = ParameterValue(
        Command([
            "xacro ",
            os.path.join(cika_description_dir, "urdf", "cika.xacro"),
            " use_sim:=false",
        ]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description_content,
            "use_sim_time": False,
        }],
    )

    # ── Arguments ─────────────────────────────────────────────────────────────
    use_perception_arg = DeclareLaunchArgument(
        name="use_perception",
        default_value="true",
        choices=["true", "false"],
        description="Launch YOLOv8 inference node",
    )

    # ── Heavy Compute Nodes ───────────────────────────────────────────────────
    laser_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[os.path.join(cika_bringup_dir, "config", "laser_filter.yaml")],
        remappings=[
            ("scan", "/scan_raw"),
            ("scan_filtered", "/scan"),
        ],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_real_path, {"use_sim_time": False}],
    )

    inference_node = Node(
        package="cika_perception",
        executable="inference_node.py",
        name="inference_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_perception")),
    )

    vo_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'use_sim_time':             False,
            'frame_id':                 'base_footprint',
            'odom_frame_id':            'vo_odom',
            'publish_tf':               False,
            'approx_sync':              True,
            'approx_sync_max_interval': 0.1,
            'queue_size':               10,
            'Odom/Strategy':            '0',
            'Vis/EstimationType':       '1',
            'Odom/ResetCountdown':      '0',
            'Vis/FeatureType':          '6',
            'Vis/MaxFeatures':          '200',
            'Vis/MinInliers':           '10',
            'Odom/GuessMotion':         'true',
            'Odom/FilteringStrategy':   '1',
        }],
        remappings=[
            ('rgb/image',       '/oak/rgb/image_raw'),
            ('rgb/camera_info', '/oak/rgb/camera_info'),
            ('depth/image',     '/oak/stereo/image_raw'),
            ('odom',            '/vo'),
        ],
    )

    foxglove_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(foxglove_bridge_dir, "launch", "foxglove_bridge_launch.xml")
        )
    )

    return LaunchDescription(
        [
            use_perception_arg,
            laser_filter_node,
            robot_state_publisher_node,
            inference_node,
            vo_node,    
            ekf_node,
            foxglove_bridge,
        ]
    )
