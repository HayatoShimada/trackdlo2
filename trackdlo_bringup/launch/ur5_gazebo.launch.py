import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('trackdlo_bringup')
    world_file = os.path.join(bringup_dir, 'worlds', 'dlo_workspace.sdf')

    return LaunchDescription([
        # Launch Gazebo Fortress with the DLO workspace world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('ros_gz_sim'),
                    'launch', 'gz_sim.launch.py')
            ]),
            launch_arguments={
                'gz_args': f'-r {world_file}',
            }.items(),
        ),

        # Spawn UR5 via ur_simulation_gazebo (if available)
        # Alternatively, use ros2_control + gz plugin
        # This is a placeholder - actual UR5 spawning depends on ur_simulation_gazebo package

        # Bridge: Gazebo camera topics → ROS2 topics
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_bridge',
            arguments=[
                '/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
                '/camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
                '/camera/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
            ],
            remappings=[
                ('/camera/image', '/camera/color/image_raw'),
                ('/camera/depth_image', '/camera/aligned_depth_to_color/image_raw'),
                ('/camera/camera_info', '/camera/color/camera_info'),
                ('/camera/points', '/camera/depth/color/points'),
            ],
            output='screen',
        ),

        # Static TF: world to base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base_link',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'world',
                '--child-frame-id', 'base_link',
            ],
        ),

        # Static TF: base_link to camera
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_camera',
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
    ])
