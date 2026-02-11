import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as file:
        return yaml.safe_load(file)


def generate_launch_description():
    # --- Robot Description (URDF) --- same xacro as Gazebo (world link + camera)
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

    # --- Semantic Description (SRDF) ---
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

    # --- Kinematics ---
    kinematics_yaml = load_yaml(
        'ur_moveit_config', os.path.join('config', 'kinematics.yaml'))
    # Unwrap ROS2 parameter format: /**:ros__parameters:robot_description_kinematics:...
    if '/**' in kinematics_yaml:
        kinematics_yaml = kinematics_yaml['/**']['ros__parameters']
    robot_description_kinematics = kinematics_yaml

    # --- Joint Limits for Planning ---
    joint_limits_yaml = {
        'robot_description_planning': load_yaml(
            'ur_moveit_config', os.path.join('config', 'joint_limits.yaml'))
    }

    # --- OMPL Planning Pipeline ---
    ompl_planning_yaml = load_yaml(
        'ur_moveit_config', os.path.join('config', 'ompl_planning.yaml'))
    ompl_planning_yaml['planning_plugin'] = 'ompl_interface/OMPLPlanner'
    ompl_planning_yaml['request_adapters'] = (
        'default_planner_request_adapters/AddTimeOptimalParameterization '
        'default_planner_request_adapters/ResolveConstraintFrames '
        'default_planner_request_adapters/FixWorkspaceBounds '
        'default_planner_request_adapters/FixStartStateBounds '
        'default_planner_request_adapters/FixStartStateCollision'
    )
    ompl_planning_yaml['start_state_max_bounds_error'] = 0.1
    planning_pipeline_config = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_planning_yaml,
    }

    # --- MoveIt Controller Manager ---
    moveit_controllers_yaml = load_yaml(
        'trackdlo_moveit', os.path.join('config', 'moveit_controllers.yaml'))
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_controllers_yaml,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    # --- Trajectory Execution ---
    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    # --- move_group node (direct launch, no robot_state_publisher) ---
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits_yaml,
            planning_pipeline_config,
            moveit_controllers,
            trajectory_execution,
            {'use_sim_time': True},
        ],
    )

    # --- DLO manipulation node ---
    dlo_manipulation_node = Node(
        package='trackdlo_moveit',
        executable='dlo_manipulation_node',
        name='dlo_manipulation',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {
                'use_sim_time': True,
                'planning_group': 'ur_manipulator',
                'results_topic': '/trackdlo/results_pc',
                'approach_distance': 0.3,
                'grasp_offset_z': 0.05,
                'tracking_rate': 2.0,
                'position_tolerance': 0.02,
            },
        ],
    )

    return LaunchDescription([
        move_group_node,
        dlo_manipulation_node,
    ])
