import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = 'ur5'

    # Generate robot_description matching ur_moveit_config's move_group
    joint_limit_params = PathJoinSubstitution([
        FindPackageShare('ur_description'), 'config', ur_type, 'joint_limits.yaml'])
    kinematics_params = PathJoinSubstitution([
        FindPackageShare('ur_description'), 'config', ur_type, 'default_kinematics.yaml'])
    physical_params = PathJoinSubstitution([
        FindPackageShare('ur_description'), 'config', ur_type, 'physical_parameters.yaml'])
    visual_params = PathJoinSubstitution([
        FindPackageShare('ur_description'), 'config', ur_type, 'visual_parameters.yaml'])

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('ur_description'), 'urdf', 'ur.urdf.xacro']),
        ' robot_ip:=xxx.yyy.zzz.www',
        ' ur_type:=', ur_type,
        ' name:=ur',
        ' safety_limits:=true',
        ' safety_pos_margin:=0.15',
        ' safety_k_position:=20',
        ' joint_limit_params:=', joint_limit_params,
        ' kinematics_params:=', kinematics_params,
        ' physical_params:=', physical_params,
        ' visual_params:=', visual_params,
    ])
    robot_description = {
        'robot_description': ParameterValue(
            robot_description_content, value_type=str)
    }

    # Generate robot_description_semantic (SRDF)
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

    # Kinematics config
    kinematics_yaml = os.path.join(
        get_package_share_directory('ur_moveit_config'),
        'config', 'kinematics.yaml')

    # Include UR5 MoveIt2 configuration
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ur_moveit_config'),
                'launch', 'ur_moveit.launch.py')
        ]),
        launch_arguments={
            'ur_type': ur_type,
            'use_sim_time': 'true',
            'launch_rviz': 'false',
            'launch_servo': 'false',
        }.items(),
    )

    # DLO manipulation node (cable following)
    dlo_manipulation_node = Node(
        package='trackdlo_moveit',
        executable='dlo_manipulation_node',
        name='dlo_manipulation',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {
                'use_sim_time': True,
                'planning_group': 'ur_manipulator',
                'results_topic': '/trackdlo/results_pc',
                'approach_distance': 0.1,
                'grasp_offset_z': 0.05,
                'tracking_rate': 2.0,
                'position_tolerance': 0.02,
            },
        ],
    )

    return LaunchDescription([
        ur_moveit_launch,
        dlo_manipulation_node,
    ])
