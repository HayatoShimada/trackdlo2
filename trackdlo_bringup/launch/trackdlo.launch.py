import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('trackdlo_bringup')
    params_file = os.path.join(bringup_dir, 'config', 'trackdlo_params.yaml')

    return LaunchDescription([
        # TrackDLO C++ tracking node
        Node(
            package='trackdlo_perception',
            executable='trackdlo',
            name='trackdlo',
            output='screen',
            parameters=[params_file],
        ),

        # Python initialization node
        Node(
            package='trackdlo_perception',
            executable='init_tracker',
            name='init_tracker',
            output='screen',
            parameters=[params_file],
        ),
    ])
