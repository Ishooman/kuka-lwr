import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def _move_group(context):
    builder = MoveItConfigsBuilder('single_lwr_robot', package_name='single_lwr_moveit')
    builder.robot_description(mappings={'hardware_interface': LaunchConfiguration('hardware_interface').perform(context)})
    # Load updated joint limits (override information from single_lwr_moveit) to respect T1 mode limits
    if LaunchConfiguration('t1_limits').perform(context).lower() == 'true':
        builder.joint_limits(file_path=os.path.join(
            get_package_share_directory('single_lwr_robot'), 'config', 't1_joint_limits.yaml'))
    moveit_config = builder.to_moveit_configs()

    should_publish = LaunchConfiguration('publish_monitored_planning_scene')
    move_group_configuration = {
        'publish_robot_description_semantic': True,
        'allow_trajectory_execution': LaunchConfiguration('allow_trajectory_execution'),
        'max_safe_path_cost': 1.0,
        'jiggle_fraction': 0.05,
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        'publish_planning_scene': should_publish,
        'publish_geometry_updates': should_publish,
        'publish_state_updates': should_publish,
        'publish_transforms_updates': should_publish,
        'monitor_dynamics': False,
        'use_sim_time': LaunchConfiguration('use_sim_time'),
    }

    # only move_group itself, rcl debug output floods the console
    args = ['--ros-args', '--log-level', 'move_group:=debug', '--log-level', 'moveit_ros:=debug'] if LaunchConfiguration('info').perform(context).lower() == 'true' else []
    return [Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict(), move_group_configuration],
        arguments=args,
        # Set the display variable, in case OpenGL code is used internally
        additional_env={'DISPLAY': os.environ.get('DISPLAY', '')},
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('allow_trajectory_execution', default_value='true'),
        DeclareLaunchArgument('publish_monitored_planning_scene', default_value='true'),
        DeclareLaunchArgument('info', default_value='false', description='Verbose mode'),
        DeclareLaunchArgument('t1_limits', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('hardware_interface', default_value='mock',
                              description='Only used to build the robot description (gazebo, fri, fril or mock)'),
        OpaqueFunction(function=_move_group),
    ])
