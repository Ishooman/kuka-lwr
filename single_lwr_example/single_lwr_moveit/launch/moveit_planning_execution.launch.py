# The planning and execution components of MoveIt configured to run with the simulated (Gazebo) or the real robot.
#   ros2 launch single_lwr_moveit moveit_planning_execution.launch.py sim:=false ip:=192.168.0.20

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # the "sim" argument controls whether we connect to a Simulated or Real robot
        #  - if sim=false, a robot ip argument is required
        DeclareLaunchArgument('sim', default_value='true', description='If true, the robot will be simulated in Gazebo'),
        DeclareLaunchArgument('ip', default_value='192.168.0.20', description='The IP address of the robot'),
        DeclareLaunchArgument('port', default_value='49939', description='The listening port of the FRI interface'),
        DeclareLaunchArgument('t1_limits', default_value='false'),
        DeclareLaunchArgument('fril_init_file', default_value=PathJoinSubstitution(
            [FindPackageShare('single_lwr_robot'), 'config', '980241-FRI-Driver.init'])),
        DeclareLaunchArgument('controllers', default_value='joint_trajectory_controller'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('single_lwr_launch'), 'launch', 'single_lwr.launch.py'])),
            launch_arguments={
                'use_lwr_sim': LaunchConfiguration('sim'),
                'lwr_powered': PythonExpression(["'false' if '", LaunchConfiguration('sim'), "'.lower() == 'true' else 'true'"]),
                'ip': LaunchConfiguration('ip'),
                'port': LaunchConfiguration('port'),
                't1_limits': LaunchConfiguration('t1_limits'),
                'file': LaunchConfiguration('fril_init_file'),
                'load_moveit': 'true',
                'controllers': LaunchConfiguration('controllers'),
            }.items(),
        ),
    ])
