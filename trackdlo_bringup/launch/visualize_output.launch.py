import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('trackdlo_bringup')

    use_bag = LaunchConfiguration('bag')
    use_eval = LaunchConfiguration('eval')

    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value='false'),
        DeclareLaunchArgument('eval', default_value='false'),

        # Static TF publishers (when using bag files)
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
            condition=IfCondition(use_bag),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_color_optical_frame_to_camera_color_frame_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0.5', '--qy', '-0.5', '--qz', '0.5', '--qw', '0.5',
                '--frame-id', 'camera_color_optical_frame',
                '--child-frame-id', 'camera_color_frame',
            ],
            condition=IfCondition(use_bag),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_color_frame_to_camera_link_tf',
            arguments=[
                '--x', '-0.000351057737134',
                '--y', '-0.0148385819048',
                '--z', '-0.000117231989861',
                '--qx', '0.00429561594501',
                '--qy', '0.000667857821099',
                '--qz', '-0.00226634810679',
                '--qw', '0.999987959862',
                '--frame-id', 'camera_color_frame',
                '--child-frame-id', 'camera_link',
            ],
            condition=IfCondition(use_bag),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_to_camera_depth_frame_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1.0',
                '--frame-id', 'camera_link',
                '--child-frame-id', 'camera_depth_frame',
            ],
            condition=IfCondition(use_bag),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_depth_frame_to_camera_depth_optical_frame_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '-0.5', '--qy', '0.5', '--qz', '-0.5', '--qw', '0.5',
                '--frame-id', 'camera_depth_frame',
                '--child-frame-id', 'camera_depth_optical_frame',
            ],
            condition=IfCondition(use_bag),
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
