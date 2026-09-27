from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder('single_lwr_robot', package_name='single_lwr_moveit').to_moveit_configs()

    return LaunchDescription([
        DeclareLaunchArgument('rviz_config', default_value=str(moveit_config.package_path / 'config/moveit.rviz')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='rviz2',
            executable='rviz2',
            output='log',
            respawn=False,
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
