import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter, SetRemap


def generate_launch_description():

    cika_navigation = get_package_share_directory("cika_navigation")
    cika_bringup = get_package_share_directory("cika_bringup")
    nav2_bringup = get_package_share_directory("nav2_bringup")

    # ── Default paths (sim) ───────────────────────────────────────────────────
    default_nav2_params = os.path.join(cika_bringup, "config", "nav2_params_real.yaml")
    # default_ekf_config  = os.path.join(cika_bringup,    "config", "ekf_sim.yaml")
    default_rtabmap_db = os.path.expanduser("~/cika_maps/cika_map.db")

    # ── Arguments ─────────────────────────────────────────────────────────────
    mode_arg = DeclareLaunchArgument(
        name="mode",
        default_value="navigation",
        choices=["slam", "navigation"],
        description="slam = build map, navigation = localize + Nav2",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        description="Use simulation clock",
    )

    # Callers override these for hardware:
    #   ekf_config:=<path>/ekf_real.yaml
    #   nav2_params:=<path>/nav2_params_hw.yaml

    # ekf_config_arg = DeclareLaunchArgument(
    #     name="ekf_config",
    #     default_value=default_ekf_config,
    #     description="Path to EKF yaml (swap for hardware)",
    # )

    nav2_params_arg = DeclareLaunchArgument(
        name="nav2_params",
        default_value=default_nav2_params,
        description="Path to Nav2 params yaml (swap for hardware)",
    )

    rtabmap_db_arg = DeclareLaunchArgument(
        name="rtabmap_db",
        default_value=default_rtabmap_db,
        description="Path to RTAB-Map database file",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    mode = LaunchConfiguration("mode")
    # ekf_config   = LaunchConfiguration("ekf_config")
    nav2_params = LaunchConfiguration("nav2_params")
    rtabmap_db = LaunchConfiguration("rtabmap_db")

    # ── Global sim time parameter ─────────────────────────────────────────────
    set_sim_time = SetParameter(name="use_sim_time", value=use_sim_time)

    # # ── EKF node ──────────────────────────────────────────────────────────────
    # ekf_node = Node(
    #     package="robot_localization",
    #     executable="ekf_node",
    #     name="ekf_filter_node",
    #     output="screen",
    #     parameters=[ekf_config, {"use_sim_time": use_sim_time}],
    # )

    # ── RTAB-Map shared params ────────────────────────────────────────────────
    rtabmap_base_params = {
        "frame_id": "base_footprint",  # changed from base_link to base_footprint to match the new URDF
        "odom_frame_id": "odom",
        "camera_frame_id": "oak_rgb_camera_optical_frame",
        "subscribe_depth": False,  # Only Lidar slam for now, set to true if you want to use RGBD slam with the OAK-D
        "subscribe_scan": True,
        "subscribe_scan_cloud": False,
        "subscribe_rgb": False,  # Only Lidar slam for now, set to true if you want to use RGBD slam with the OAK-D
        "approx_sync": True,
        "publish_tf": True,
        "approx_sync_max_interval": 0.2,
        "topic_queue_size": 30,  # ← Added
        "sync_queue_size": 30,  # ← Added
        "qos_scan": 1,  # ← RELIABLE (matches your publisher)
        "qos_odom": 1,  # ← RELIABLE
        "use_sim_time": use_sim_time,
        "Mem/SaveDepth16Format": "false",
        # "Mem/DepthCompressionFormat": ".png",
        "Reg/Strategy": "1",
        "Reg/Force3DoF": "true",
        "Icp/PointToPlane": "false",
        "Icp/VoxelSize": "0.05",
        "Icp/Iterations": "50",
        "Icp/MaxTranslation": "1.0",
        "Icp/MaxCorrespondenceDistance": "0.3",
        "RGBD/NeighborLinkRefining": "true",
        "RGBD/ProximityBySpace": "true",
        "RGBD/ProximityMaxGraphDepth": "0",
        "RGBD/ProximityPathMaxNeighbors": "10",
        "RGBD/OptimizeFromGraphEnd": "true",
        "RGBD/AngularUpdate": "0.1",
        "RGBD/LinearUpdate": "0.1",
        "RGBD/Enabled": "true",
        # "RGBD/DepthMax": "8.0",
        "RGBD/OptimizeMaxError": "3.0",
        "Grid/Sensor": "0",  # 0=scan, 1=cloud, 2=cloud2
        "Grid/RangeMin": "0.12",
        "Grid/RangeMax": "12.0",
        "Grid/FootprintRadius": "0.35",
        "Grid/CellSize": "0.05",
        "Grid/3D": "false",
        "Grid/RayTracing": "true",
        # "Grid/DepthDecimation": "4",
        "Grid/MaxObstacleHeight": "2.0",
        "Grid/MinGroundHeight": "0.05",
        # "Grid/DepthMax": "8.0",
        "Rtabmap/DetectionRate": "2",
        "Optimizer/Robust": "true",
        "Optimizer/GravitySigma": "0.0",
        # "Vis/CorType": "0",
        # "Vis/FeatureType": "8",
        # "Vis/MinInliers": "15",
        "odom_tf_angular_variance": 0.3,
        "odom_tf_linear_variance": 0.5,
        "wait_for_transform": 0.5,
    }

    rtabmap_remappings = [
        ("rgb/image", "/oak/rgb/image_raw"),
        ("depth/image", "/oak/stereo/image_raw"),
        ("rgb/camera_info", "/oak/rgb/camera_info"),
        ("depth/camera_info", "/oak/stereo/camera_info"),
        ("scan", "/scan"),
        ("odom", "/odometry/filtered"),
        ("vo", "/vo"),
    ]

    # ── SLAM mode ─────────────────────────────────────────────────────────────
    slam_params = dict(rtabmap_base_params)
    slam_params.update(
        {
            "database_path": rtabmap_db,
            "Mem/IncrementalMemory": "true",
            "Mem/InitWMWithAllNodes": "false",
        }
    )

    rtabmap_slam_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[slam_params],
        remappings=rtabmap_remappings,
        arguments=["-d"],  # deletes db on start — back up first if needed!
        condition=IfCondition(PythonExpression(["'", mode, "' == 'slam'"])),
    )

    # ── Navigation / localization mode ────────────────────────────────────────
    nav_params = dict(rtabmap_base_params)
    nav_params.update(
        {
            "database_path": rtabmap_db,
            "Mem/IncrementalMemory": "false",
            "Mem/InitWMWithAllNodes": "true",
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

    # ── Nav2 (navigation mode only) ───────────────────────────────────────────
    nav2_launch = GroupAction(
        condition=IfCondition(PythonExpression(["'", mode, "' == 'navigation'"])),
        actions=[
            SetRemap(src="/cmd_vel", dst="/skid_steer_controller/cmd_vel_unstamped"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup, "launch", "navigation_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": nav2_params,
                }.items(),
            ),
        ],
    )

    return LaunchDescription(
        [
            mode_arg,
            use_sim_time_arg,
            # ekf_config_arg,
            nav2_params_arg,
            rtabmap_db_arg,
            set_sim_time,
            # ekf_node,
            rtabmap_slam_node,
            rtabmap_localization_node,
            nav2_launch,
        ]
    )
