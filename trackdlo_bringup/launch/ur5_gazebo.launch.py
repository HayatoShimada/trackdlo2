import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, ExecuteProcess, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory('trackdlo_bringup')
    description_dir = get_package_share_directory('trackdlo_description')
    world_file = os.path.join(bringup_dir, 'worlds', 'dlo_workspace.sdf')

    # Process URDF via xacro
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        os.path.join(description_dir, 'urdf', 'ur5_workspace.urdf.xacro'),
    ])
    robot_description = {
        'robot_description': ParameterValue(
            robot_description_content, value_type=str)
    }

    return LaunchDescription([
        # 1. Launch Gazebo Fortress with DLO workspace world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch', 'gz_sim.launch.py')
            ]),
            launch_arguments={'gz_args': f'-r {world_file}'}.items(),
        ),

        # 2. Robot State Publisher (publishes TFs from URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': True}],
        ),

        # 3. Spawn UR5 into Gazebo from /robot_description
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'ur5',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '0',
            ],
            output='screen',
        ),

        # 4. Spawn joint_state_broadcaster (delayed for Gazebo startup)
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller',
                         '--set-state', 'active',
                         'joint_state_broadcaster'],
                    output='screen',
                ),
            ],
        ),

        # 5. Spawn joint_trajectory_controller
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller',
                         '--set-state', 'active',
                         'joint_trajectory_controller'],
                    output='screen',
                ),
            ],
        ),

        # 6. Camera bridge: Gazebo → ROS2 topics
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_bridge',
            arguments=[
                '/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
                '/camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo'
                '@ignition.msgs.CameraInfo',
                '/camera/points@sensor_msgs/msg/PointCloud2'
                '@ignition.msgs.PointCloudPacked',
            ],
            remappings=[
                ('/camera/image', '/camera/color/image_raw'),
                ('/camera/depth_image', '/gz/camera/depth_raw'),
                ('/camera/camera_info', '/camera/color/camera_info'),
                ('/camera/points', '/camera/depth/color/points'),
            ],
            output='screen',
        ),

        # 7. Depth format converter: float32 meters → uint16 millimeters
        Node(
            package='trackdlo_utils',
            executable='depth_format_converter',
            name='depth_format_converter',
            parameters=[{
                'input_topic': '/gz/camera/depth_raw',
                'output_topic': '/camera/aligned_depth_to_color/image_raw',
            }],
            output='screen',
        ),

        # 8. Clock bridge: Gazebo sim time → ROS2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            ],
            output='screen',
        ),
    ])
