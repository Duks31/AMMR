import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    perception_dir = get_package_share_directory('cika_perception')
    depthai_driver_dir = get_package_share_directory('depthai_ros_driver')

    # Path to the custom YAML we just created
    custom_params_file = os.path.join(perception_dir, 'config', 'camera.yaml')

    return LaunchDescription([
        # 1. Official OAK-D Driver, forced to load OUR parameters file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_driver_dir, 'launch', 'camera.launch.py')
            ),
            launch_arguments={
                'name': 'oak',
                'params_file': custom_params_file
            }.items()
        ),

        # 2. Your Custom Inference Translator
        Node(
            package='cika_perception',
            executable='inference_node.py',
            name='inference_node',
            output='screen'
        )
    ])