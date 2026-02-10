import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_dir = get_package_share_directory('trackdlo_description')

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        os.path.join(description_dir, 'urdf', 'ur5_workspace.urdf.xacro'),
    ])
    robot_description = {
        'robot_description': ParameterValue(
            robot_description_content, value_type=str)
    }

    # Include UR5 MoveIt2 configuration
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ur_moveit_config'),
                'launch', 'ur_moveit.launch.py')
        ]),
        launch_arguments={
            'ur_type': 'ur5',
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
