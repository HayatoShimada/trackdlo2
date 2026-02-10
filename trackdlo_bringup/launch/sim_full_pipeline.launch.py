import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_dir = get_package_share_directory('trackdlo_bringup')
    moveit_dir = get_package_share_directory('trackdlo_moveit')

    return LaunchDescription([
        # Phase 1: Gazebo + UR5 + Camera bridge (immediate)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'ur5_gazebo.launch.py')
            ),
        ),

        # Phase 2: TrackDLO perception (10s delay for Gazebo startup)
        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(bringup_dir, 'launch',
                                     'trackdlo.launch.py')
                    ),
                ),
            ],
        ),

        # Phase 3: MoveIt2 + DLO manipulation (15s delay for perception)
        TimerAction(
            period=15.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(moveit_dir, 'launch',
                                     'moveit_planning.launch.py')
                    ),
                ),
            ],
        ),
    ])
