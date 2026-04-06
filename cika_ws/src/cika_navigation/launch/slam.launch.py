from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # ── Shared parameters ────────────────────────────────────────────────────
    rtabmap_params = [
        {
            # Frames
            "frame_id": "base_link",
            "odom_frame_id": "odom",
            "camera_frame_id": "OAKDcamera_1_optical",
            # Subscriptions
            "subscribe_depth": True,
            "subscribe_scan": True,
            "subscribe_rgb": True,
            "subscribe_odom_info": False,
            # Sync
            "approx_sync": True,
            "approx_sync_max_interval": 0.05,  # 50ms — generous for sim
            "queue_size": 10,
            # Sim time — NOTE: string key, not variable key
            "use_sim_time": use_sim_time,
            # QoS fix
            "qos_overrides./tf_static.publisher.durability": "transient_local",
            # RTAB-Map strategy
            # 1 = Icp (good with LiDAR), 0 = Visual
            "Reg/Strategy": "1",
            "Reg/Force3DoF": "true",
            "Icp/PointToPlane": "false",  # 2D scan, not pointcloud
            "Icp/VoxelSize": "0.05",
            "Icp/Iterations": "30",
            "Icp/MaxTranslation": "1.0",
            "Icp/MaxCorrespondenceDistance": "0.1",
            # Loop closure / graph
            "RGBD/NeighborLinkRefining": "true",
            "RGBD/ProximityBySpace": "true",
            "RGBD/ProximityMaxGraphDepth": "0",
            "RGBD/OptimizeFromGraphEnd": "false",
            # Grid map (occupancy for Nav2 + 3D for Rviz)
            "Grid/Sensor": "2",  # 0 = laser scan, 1 = depth, 2 = depth + scan
            "Grid/RangeMin": "0.12",  # match your lidar min range
            "Grid/RangeMax": "12.0",  # match your LiDAR max range (string!)
            "Grid/FootprintRadius": "0.35",  # rough radius of cika's base in meters
            "Grid/CellSize": "0.05",  # 5cm resolution — good for Nav2
            "Grid/3D": "true",
            # Visual odometry (disabled — we use wheel odom)
            "Odom/Strategy": "0",
            "RGBD/Enabled": "true",
            # Memory
            "Mem/IncrementalMemory": "true",
            "Mem/InitWMWithAllNodes": "false",
            "Mem/SaveDepth16Format": "true",
            "Mem/DepthCompressionFormat": ".png",
            "RGBD/DepthMax": "10.0",
            "RGBD/OptimizeMaxError": "5.0",
            "Grid/FromDepth": "true",
            "Optimizer/GravitySigma": "0.3",  # Helps lock the map flat to the ground
        }
    ]

    rtabmap_remappings = [
        ("rgb/image", "/oak/rgb/image_raw"),
        ("depth/image", "/oak/stereo/image_raw"),
        ("rgb/camera_info", "/oak/rgb/camera_info"),
        ("depth/camera_info", "/oak/stereo/camera_info"),
        ("scan", "/scan"),
        ("odom", "/skid_steer_controller/odom"),
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
            }
        ],
        remappings=rtabmap_remappings,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use simulation clock"
            ),
            rtabmap_node,
            rtabmap_viz_node,
        ]
    )
