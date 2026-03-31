from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    rtabmap_node = Node(
        package="rtabmap_slam", executable="rtabmap", output="screen",
        parameters=[{
            'frame_id': 'base_link',
            'subscribe_depth': True,
            'subscribe_scan': True,
            'approx_sync': True,
            use_sim_time: use_sim_time,

            'Reg/Strategy': '1',
            'RGBD/NeibhorsLinkRefining': 'True',
            'Grid/RangeMax': 10.0,
        }],

        remappings=[
            ('rgb/image', '/oak/rgb/image_raw'),
            ('depth/image', '/oak/stereo/image_raw'),
            ('rgb/camera_info', '/oak/rgb/camera_info'),
            ('scan', '/scan'),
            ('odom', '/skid_steer_controller/odom'),
        ], 

        arguments=["-d"]
    )

    rtabmap_viz = Node(
        package="rtabmap_viz", executable="rtabmap_viz", output="screen",
        parameters=[{
            'frame_id': 'base_link',
            'subscribe_depth': True,
            'subscribe_scan': True,
            'approx_sync': True,
            'use_sim_time': use_sim_time,
        }],

        remappings=[
            ('rgb/image', '/oak/rgb/image_raw'),
            ('depth/image', '/oak/stereo/image_raw'),
            ('rgb/camera_info', '/oak/rgb/camera_info'),
            ('scan', '/scan'),
            ('odom', '/skid_steer_controller/odom'),
        ]
    )

    return LaunchDescription([
        rtabmap_node,
        rtabmap_viz,
    ])