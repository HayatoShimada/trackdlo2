import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_dir = get_package_share_directory('trackdlo_bringup')

    return LaunchDescription([
        # Camera (or Gazebo sim)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'camera.launch.py')
            ),
        ),

        # TrackDLO perception pipeline
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'trackdlo.launch.py')
            ),
        ),

        # Visualization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'visualize_output.launch.py')
            ),
        ),
    ])
