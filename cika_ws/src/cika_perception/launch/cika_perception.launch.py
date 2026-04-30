import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    perception_dir = get_package_share_directory('cika_perception')
    depthai_driver_dir = get_package_share_directory('depthai_ros_driver')

    # Path to your Luxonis-generated JSON config
    nn_config_path = os.path.join(perception_dir, 'models', '300_epoch_best.json')

    return LaunchDescription([
        # ── OVERRIDES ────────────────────────────────────────────────────────
        # This forces the official driver to use our custom pipeline
        SetParameter(name='camera.i_pipeline_type', value='yolo'),
        SetParameter(name='nn.i_nn_config_path', value=nn_config_path),
        SetParameter(name='stereo.i_align_depth', value=True),
        SetParameter(name='camera.i_enable_pointcloud', value=True),

        # ── NODES ────────────────────────────────────────────────────────────
        # 1. Official OAK-D Driver (It will now inherit the parameters above)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_driver_dir, 'launch', 'camera.launch.py')
            ),
            launch_arguments={'name': 'oak'}.items()
        ),

        # 2. Your Custom Inference Translator
        Node(
            package='cika_perception',
            executable='inference_node.py',
            name='inference_node',
            output='screen'
        )
    ])