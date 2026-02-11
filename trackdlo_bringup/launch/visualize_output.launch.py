import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as file:
        return yaml.safe_load(file)


def generate_launch_description():
    bringup_dir = get_package_share_directory('trackdlo_bringup')

    use_bag = LaunchConfiguration('bag')
    use_eval = LaunchConfiguration('eval')

    # Robot description (same as Gazebo) for MoveIt MotionPlanning display
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('trackdlo_description'),
            'urdf', 'ur5_workspace.urdf.xacro']),
    ])
    robot_description = {
        'robot_description': ParameterValue(
            robot_description_content, value_type=str)
    }

    # Semantic description (SRDF) for MoveIt MotionPlanning display
    robot_description_semantic_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('ur_moveit_config'), 'srdf', 'ur.srdf.xacro']),
        ' name:=ur',
        ' prefix:=""',
    ])
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(
            robot_description_semantic_content, value_type=str)
    }

    # Kinematics for MoveIt MotionPlanning display
    kinematics_yaml = load_yaml(
        'ur_moveit_config', os.path.join('config', 'kinematics.yaml'))
    robot_description_kinematics = {
        'robot_description_kinematics': kinematics_yaml
    }

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

        # RViz2 with MoveIt parameters for MotionPlanning display
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d', os.path.join(bringup_dir, 'rviz', 'tracking.rviz')
            ],
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                {'use_sim_time': True},
            ],
        ),
    ])
