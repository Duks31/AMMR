#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cika_perception',
            executable='inference_node.py',
            name='inference_node',
            output='screen'
        )
    ])