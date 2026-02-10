import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'planning_group', default_value='ur_manipulator',
            description='MoveIt planning group name'),
        DeclareLaunchArgument(
            'results_topic', default_value='/trackdlo/results_pc',
            description='TrackDLO results point cloud topic'),

        # DLO manipulation node
        Node(
            package='trackdlo_moveit',
            executable='dlo_manipulation_node',
            name='dlo_manipulation',
            output='screen',
            parameters=[{
                'planning_group': LaunchConfiguration('planning_group'),
                'results_topic': LaunchConfiguration('results_topic'),
                'approach_distance': 0.1,
                'grasp_offset_z': 0.05,
            }],
        ),
    ])
