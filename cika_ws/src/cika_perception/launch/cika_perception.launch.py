import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # ── Paths ────────────────────────────────────────────────────────────
    perception_dir = get_package_share_directory('cika_perception')
    depthai_driver_dir = get_package_share_directory('depthai_ros_driver')

    # Path to your Luxonis-generated JSON config (which automatically loads the .blob)
    nn_config_path = os.path.join(perception_dir, 'models', '300_epoch_best.json')

    # ── Nodes & Drivers ──────────────────────────────────────────────────

    # 1. The Official OAK-D Driver (Configured for YOLO + SLAM)
    oak_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(depthai_driver_dir, 'launch', 'camera.launch.py')
        ),
        launch_arguments={
            'name': 'oak',
            'camera.i_pipeline_type': 'yolo',         # Enables YOLO + Spatial Detection
            'nn.i_nn_config_path': nn_config_path,    # Loads your custom AI
            'stereo.i_align_depth': 'true',           # CRITICAL FOR SLAM: Maps depth to RGB
            'camera.i_enable_pointcloud': 'true',     # CRITICAL FOR SLAM: Generates 3D points
        }.items()
    )

    # 2. Your Custom Inference Translator
    inference_node = Node(
        package='cika_perception',
        executable='inference_node.py',
        name='inference_node',
        output='screen'
    )

    return LaunchDescription([
        oak_driver,
        inference_node
    ])