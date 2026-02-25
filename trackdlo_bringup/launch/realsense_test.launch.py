import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    bringup_dir = get_package_share_directory('trackdlo_bringup')
    params_file = os.path.join(bringup_dir, 'config', 'realsense_params.yaml')

    segmentation = LaunchConfiguration('segmentation')
    rviz = LaunchConfiguration('rviz')

    # use_external_mask is true when segmentation is not 'hsv'
    use_external_mask = PythonExpression(["'", segmentation, "' != 'hsv'"])

    # RealSense camera launch (resolved at runtime, not at parse time)
    realsense_dir = get_package_share_directory('realsense2_camera')
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_dir, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'camera_namespace': '',
            'depth_module.depth_profile': '1280x720x15',
            'rgb_camera.color_profile': '1280x720x15',
            'align_depth.enable': 'true',
            'pointcloud.enable': 'false',
            'temporal_filter.enable': 'true',
            'decimation_filter.enable': 'true',
        }.items(),
    )

    return [
        realsense_launch,

        # --- Static TF: base_link -> camera_link ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_camera_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link',
            ],
        ),

        # --- TrackDLO C++ tracking node ---
        Node(
            package='trackdlo_perception',
            executable='trackdlo',
            name='trackdlo',
            output='screen',
            parameters=[
                params_file,
                {'use_external_mask': use_external_mask},
            ],
            respawn=True,
            respawn_delay=3.0,
        ),

        # --- Python initialization node ---
        Node(
            package='trackdlo_perception',
            executable='init_tracker',
            name='init_tracker',
            output='screen',
            parameters=[
                params_file,
                {'use_external_mask': use_external_mask},
            ],
        ),

        # --- HSV Tuner (only when segmentation:=hsv_tuner) ---
        Node(
            package='trackdlo_utils',
            executable='hsv_tuner',
            name='hsv_tuner',
            output='screen',
            parameters=[params_file],
            condition=LaunchConfigurationEquals('segmentation', 'hsv_tuner'),
        ),

        # --- SAM2 Segmentation (only when segmentation:=sam2) ---
        Node(
            package='trackdlo_utils',
            executable='sam2_segmentation',
            name='sam2_segmentation',
            output='screen',
            parameters=[params_file],
            condition=LaunchConfigurationEquals('segmentation', 'sam2'),
        ),

        # --- RViz2 ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d', os.path.join(bringup_dir, 'rviz', 'tracking.rviz')
            ],
            condition=IfCondition(rviz),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'segmentation', default_value='hsv',
            description='Segmentation method: hsv (default), hsv_tuner, or sam2',
            choices=['hsv', 'hsv_tuner', 'sam2'],
        ),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz2 for visualization',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
