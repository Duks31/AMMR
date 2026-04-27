import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    cika_navigation = get_package_share_directory("cika_navigation")
    nav2_bringup = get_package_share_directory("nav2_bringup")

    # ── Paths ─────────────────────────────────────────────────────────────────
    nav2_params_path = os.path.join(cika_navigation, "config", "nav2_params.yaml")
    map_yaml_path = os.path.join(cika_navigation, "maps", "cika_map.yaml")
    ekf_config_path = os.path.join(cika_navigation, "config", "ekf.yaml")
    rtabmap_db_path = os.path.join(cika_navigation, "maps", "cika_warehouse.db")

    # ── Arguments ─────────────────────────────────────────────────────────────
    mode_arg = DeclareLaunchArgument(
        name="mode",
        default_value="navigation",
        choices=["slam", "navigation"],
        description="slam = build map, navigation = localize + Nav2",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation clock",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    rtabmap_db_path = os.path.join(cika_navigation, "maps", "cika_map.db")

    mode = LaunchConfiguration("mode")

    # ── Global sim time parameter ─────────────────────────────────────────────
    set_sim_time = SetParameter(name="use_sim_time", value=use_sim_time)

    # ── EKF node (runs in both modes) ─────────────────────────────────────────
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config_path, {"use_sim_time": use_sim_time}],
    )

    # ─────────────────────────────────────────────────────────────────────────
    # SLAM MODE — RTAB-Map builds the map, no Nav2
    # ─────────────────────────────────────────────────────────────────────────
    slam_params = {
        "frame_id": "base_link",
        "odom_frame_id": "odom",
        "camera_frame_id": "OAKDcamera_1_optical",
        "subscribe_depth": True,
        "subscribe_scan": True,
        "subscribe_rgb": True,
        "approx_sync": True,
        "publish_tf": True,
        "approx_sync_max_interval": 0.05,
        "use_sim_time": use_sim_time,
        "database_path": rtabmap_db_path,
        "Mem/IncrementalMemory": "true",
        "Mem/InitWMWithAllNodes": "false",
        "Mem/SaveDepth16Format": "false",
        "Mem/DepthCompressionFormat": ".png",
        "Reg/Strategy": "1",
        "Reg/Force3DoF": "true",
        "Icp/PointToPlane": "false",
        "Icp/VoxelSize": "0.05",
        "Icp/Iterations": "30",
        "Icp/MaxTranslation": "1.0",
        "Icp/MaxCorrespondenceDistance": "0.1",
        "RGBD/NeighborLinkRefining": "true",
        "RGBD/ProximityBySpace": "true",
        "RGBD/ProximityMaxGraphDepth": "0",
        "RGBD/OptimizeFromGraphEnd": "true",
        "RGBD/AngularUpdate": "0.01",
        "RGBD/LinearUpdate": "0.01",
        "RGBD/Enabled": "true",
        "RGBD/DepthMax": "8.0",
        "RGBD/OptimizeMaxError": "5.0",
        "Grid/Sensor": "0",
        "Grid/RangeMin": "0.12",
        "Grid/RangeMax": "12.0",
        "Grid/FootprintRadius": "0.35",
        "Grid/CellSize": "0.05",
        "Grid/3D": "false",
        "Grid/RayTracing": "true",
        "Grid/DepthDecimation": "4",
        "Grid/MaxObstacleHeight": "2.0",
        "Grid/MinGroundHeight": "0.05",
        "Grid/DepthMax": "8.0",
        "Rtabmap/DetectionRate": "2",
        "Optimizer/GravitySigma": "0.3",
        "Vis/CorType": "0",
        "Vis/FeatureType": "8",
        "Vis/MinInliers": "15",
        "odom_tf_angular_variance": 0.05,
        "odom_tf_linear_variance": 0.1,
        "wait_for_transform": 0.5,
    }

    rtabmap_remappings = [
        ("rgb/image", "/oak/rgb/image_raw"),
        ("depth/image", "/oak/stereo/image_raw"),
        ("rgb/camera_info", "/oak/rgb/camera_info"),
        ("depth/camera_info", "/oak/stereo/camera_info"),
        ("scan", "/scan_raw"),
        ("odom", "/odometry/filtered"),
        ("vo", "/vo"),
    ]

    rtabmap_slam_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[slam_params],
        remappings=rtabmap_remappings,
        arguments=["-d"],  # delete db on start — clean map every SLAM session
        condition=IfCondition(PythonExpression(["'", mode, "' == 'slam'"])),
    )

    # ─────────────────────────────────────────────────────────────────────────
    # NAVIGATION MODE — RTAB-Map localizes, Nav2 navigates
    # ─────────────────────────────────────────────────────────────────────────
    nav_params = dict(slam_params)  # start from same base params
    nav_params.update(
        {
            # Localization mode — incremental memory OFF, loads existing db
            "Mem/IncrementalMemory": "false",
            "Mem/InitWMWithAllNodes": "true",  # load all nodes from db for localization
            "database_path": rtabmap_db_path,  # load the saved warehouse map db
        }
    )

    rtabmap_localization_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[nav_params],
        remappings=rtabmap_remappings,
        condition=IfCondition(PythonExpression(["'", mode, "' == 'navigation'"])),
    )

    # ── Nav2 bringup (navigation mode only) ──────────────────────────────────
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_path,
        }.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'navigation'"])),
    )

    # ── RTAB-Map visualizer (both modes) ─────────────────────────────────────
    rtabmap_viz_node = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        name="rtabmap_viz",
        output="screen",
        parameters=[
            {
                "frame_id": "base_link",
                "odom_frame_id": "odom",
                "subscribe_depth": True,
                "subscribe_scan": True,
                "approx_sync": True,
                "use_sim_time": use_sim_time,
                "cloud_decimation": 4,
                "cloud_max_depth": 8.0,
            }
        ],
        remappings=rtabmap_remappings,
    )

    return LaunchDescription(
        [
            mode_arg,
            use_sim_time_arg,
            set_sim_time,
            ekf_node,
            rtabmap_slam_node,
            rtabmap_localization_node,
            nav2_navigation_launch,
            # rtabmap_viz_node, # Saving CPU by not launching the viz
        ]
    )
