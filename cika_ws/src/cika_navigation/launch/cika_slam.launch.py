from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    ekf_config_path = os.path.join(
        get_package_share_directory("cika_navigation"), "config", "ekf.yaml"
    )

    # ── Shared parameters ────────────────────────────────────────────────────
    rtabmap_params = [
        {
            "frame_id": "base_link",
            "odom_frame_id": "odom",
            "camera_frame_id": "OAKDcamera_1_optical",
            "subscribe_depth": True,
            "subscribe_scan": True,
            "subscribe_rgb": True,
            "subscribe_odom_info": False,
            "approx_sync": True,
            "publish_tf": True,
            "approx_sync_max_interval": 0.05,  # 50ms — generous for sim
            "queue_size": 10,
            "use_sim_time": use_sim_time,
            "qos_overrides./tf_static.publisher.durability": "transient_local",
            "Reg/Strategy": "1",
            "Reg/Force3DoF": "true",
            "Icp/PointToPlane": "false",  # 2D scan, not pointcloud
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
            "Grid/Sensor": "0",  # 0 = laser scan, 1 = depth, 2 = depth + scan
            "Grid/RangeMin": "0.12",  # match your lidar min range
            "Grid/RangeMax": "12.0",  # match your LiDAR max range (string!)
            "Grid/FootprintRadius": "0.35",  # rough radius of cika's base in meters
            "Grid/CellSize": "0.05",  # 5cm resolution — good for Nav2
            "Grid/3D": "false",
            "Grid/RayTracing": "true",
            "Grid/DepthDecimation": "4",  # Drops 75% of pixels. Massive RAM saver.
            "Grid/MaxObstacleHeight": "2.0",  # Ignores the warehouse ceiling
            "Grid/MinGroundHeight": "0.05",  # Ignores the floor (saves drawing thousands of flat points)
            "Grid/DepthMax": "8.0",
            "Odom/Strategy": "0",
            "Odom/PublishVOMsg": "true",
            "Odom/Sensor": "0",
            "RGBD/Enabled": "true",
            "Mem/IncrementalMemory": "true",
            "Mem/InitWMWithAllNodes": "false",
            "Mem/SaveDepth16Format": "false",
            "Mem/DepthCompressionFormat": ".png",
            "RGBD/DepthMax": "8.0",
            "RGBD/OptimizeMaxError": "5.0",
            "Rtabmap/DetectionRate": "2",
            "Optimizer/GravitySigma": "0.3",  # Helps lock the map flat to the ground
            "odom_tf_angular_variance": 0.05,
            "odom_tf_linear_variance": 0.1,
            "Vis/CorType": "0",  # 0=Features, 1=Optical Flow
            "Vis/FeatureType": "8",  # 8=GFTT/ORB (Good for Ubuntu 22.04/Humble)
            "Vis/MinInliers": "15",
        }
    ]

    rtabmap_remappings = [
        ("rgb/image", "/oak/rgb/image_raw"),
        ("depth/image", "/oak/stereo/image_raw"),
        ("rgb/camera_info", "/oak/rgb/camera_info"),
        ("depth/camera_info", "/oak/stereo/camera_info"),
        ("scan", "/scan_raw"),
        ("odom", "/odometry/filtered"),
        ("vo", "/vo"),
    ]

    # ── RTAB-Map SLAM node ───────────────────────────────────────────────────
    rtabmap_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=rtabmap_params,
        remappings=rtabmap_remappings,
        arguments=["-d"],  # -d = delete existing database on start (good for dev)
    )

    # ── RTAB-Map visualizer ──────────────────────────────────────────────────
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

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config_path, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use simulation clock"
            ),
            ekf_node,
            rtabmap_node,
            rtabmap_viz_node,
        ]
    )
