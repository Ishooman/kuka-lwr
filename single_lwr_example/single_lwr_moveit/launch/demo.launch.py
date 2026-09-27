# Self contained MoveIt demo, with mock hardware (no robot and no simulator):
#   robot_state_publisher, ros2_control_node (mock hardware) + controllers, move_group, rviz and optionally the warehouse db

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # the robot description uses mock hardware (see the xacro_args in .setup_assistant)
    moveit_config = MoveItConfigsBuilder('single_lwr_robot', package_name='single_lwr_moveit').to_moveit_configs()
    launch_dir = PathJoinSubstitution([FindPackageShare('single_lwr_moveit'), 'launch'])
    controllers_file = PathJoinSubstitution([FindPackageShare('single_lwr_robot'), 'config', 'controllers.yaml'])

    return LaunchDescription([
        # By default, we do not start a database (it can be large)
        DeclareLaunchArgument('db', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),

        # Given the published joint states, publish tf for the robot links
        Node(package='robot_state_publisher', executable='robot_state_publisher', respawn=True, output='screen',
             parameters=[moveit_config.robot_description]),
        # the stiffness dummy joints are not controlled, fill them with zeros
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'source_list': ['/lwr/joint_states']}]),

        # Fake joint driver, in the /lwr namespace as the real and simulated robots
        Node(package='controller_manager', executable='ros2_control_node', namespace='lwr', output='screen',
             parameters=[controllers_file],
             remappings=[('/lwr/controller_manager/robot_description', '/robot_description')]),
        Node(package='controller_manager', executable='spawner', namespace='lwr',
             arguments=['joint_state_broadcaster', 'joint_trajectory_controller', '-c', '/lwr/controller_manager']),

        # Run the main MoveIt executable
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([launch_dir, 'move_group.launch.py'])),
                                 launch_arguments={'allow_trajectory_execution': 'true', 'info': 'false'}.items()),

        # Run Rviz and load the default config to see the state of the move_group node
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([launch_dir, 'moveit_rviz.launch.py'])),
                                 condition=IfCondition(LaunchConfiguration('use_rviz'))),

        # If database loading was enabled, start mongodb as well
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([launch_dir, 'warehouse_db.launch.py'])),
                                 condition=IfCondition(LaunchConfiguration('db'))),
    ])
