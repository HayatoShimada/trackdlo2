import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('trackdlo_bringup')
    eval_params_file = os.path.join(bringup_dir, 'config', 'evaluation_params.yaml')

    bag_dir = LaunchConfiguration('bag_dir')
    bag_rate = LaunchConfiguration('bag_rate')

    return LaunchDescription([
        DeclareLaunchArgument('bag_dir', default_value='',
                              description='Path to ROS2 bag directory'),
        DeclareLaunchArgument('bag_rate', default_value='0.5',
                              description='Bag playback rate'),

        # Evaluation C++ node
        Node(
            package='trackdlo_perception',
            executable='run_evaluation',
            name='evaluation',
            output='screen',
            parameters=[eval_params_file],
        ),

        # Simulate occlusion for evaluation
        Node(
            package='trackdlo_utils',
            executable='simulate_occlusion_eval',
            name='simulate_occlusion_eval',
            output='screen',
        ),

        # Bag playback
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '-r', bag_rate, bag_dir],
            output='screen',
        ),
    ])
