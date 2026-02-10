import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('trackdlo_bringup')

    return LaunchDescription([
        # RealSense camera node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('realsense2_camera'),
                    'launch', 'rs_launch.py')
            ]),
            launch_arguments={
                'depth_module.depth_profile': '1280x720x15',
                'rgb_camera.color_profile': '1280x720x15',
                'align_depth.enable': 'true',
                'pointcloud.enable': 'true',
                'temporal_filter.enable': 'true',
                'decimation_filter.enable': 'true',
                'ordered_pc': 'true',
            }.items(),
        ),

        # Static TF publishers
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_camera_color_optical_frame_tf',
            arguments=[
                '--x', '0.5308947503950723',
                '--y', '0.030109485611943067',
                '--z', '0.5874',
                '--qx', '-0.7071068',
                '--qy', '0.7071068',
                '--qz', '0',
                '--qw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_color_optical_frame',
            ],
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d', os.path.join(bringup_dir, 'rviz', 'tracking.rviz')
            ],
        ),
    ])
