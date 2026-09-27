# Single KUKA LWR 4+ example: simulated in Gazebo (default), real robot (lwr_powered:=true) or mock hardware (both false).
#
#   ros2 launch single_lwr_launch single_lwr.launch.py
#   ros2 launch single_lwr_launch single_lwr.launch.py use_lwr_sim:=false lwr_powered:=true ip:=192.168.0.10
#   ros2 launch single_lwr_launch single_lwr.launch.py use_lwr_sim:=false use_rviz:=true   # mock hardware

import os
import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

import xacro


def _launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def is_true(name):
        return arg(name).lower() in ('true', '1', 'yes')

    robot_share = get_package_share_directory('single_lwr_robot')
    use_lwr_sim = is_true('use_lwr_sim')
    lwr_powered = is_true('lwr_powered')

    if use_lwr_sim:
        hardware_interface = 'gazebo'
    elif lwr_powered:
        hardware_interface = arg('real_interface')  # fri or fril
    else:
        hardware_interface = 'mock'

    controllers_file = os.path.join(robot_share, 'config', 'controllers.yaml')

    # the urdf parameter
    robot_description = xacro.process_file(
        os.path.join(robot_share, 'robot', arg('robot_name') + '.urdf.xacro'),
        mappings={
            'hardware_interface': hardware_interface,
            'controllers_file': controllers_file,
            'port': arg('port'),
            'ip': arg('ip'),
            'init_file': arg('file'),
        }).toxml()
    # gazebo_ros2_control passes the description as a command line parameter parsed as YAML, where
    # the comments can break it (e.g. a colon), so remove them
    robot_description = re.sub(r'<!--.*?-->', '', robot_description, flags=re.DOTALL)
    sim_time = {'use_sim_time': use_lwr_sim}

    actions = [
        # joint and robot state publishers of the full robot description
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'source_list': ['/lwr/joint_states']}, sim_time]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)}, sim_time]),
    ]

    if is_true('use_rviz'):
        actions.append(Node(
            package='rviz2', executable='rviz2', name='lwr_rviz', output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('single_lwr_launch'), 'launch', 'rviz_config.rviz')],
            parameters=[sim_time]))

    spawn_after = None
    if use_lwr_sim:
        # all the models of the world are installed locally: don't let the gazebo client block on the online model database
        actions.append(SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''))
        # enable/disable gui at will, the rviz listens to the simulation
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
            launch_arguments={
                'world': os.path.join(robot_share, 'worlds', 'simple_environment.world'),
                'pause': 'false',
                'gui': arg('gui'),
                'verbose': 'false',
            }.items()))
        # spawn the robot in gazebo, the gazebo_ros2_control plugin runs the controller manager (/lwr/controller_manager)
        spawn_after = Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
                           arguments=['-topic', 'robot_description', '-entity', arg('robot_name')])
        actions.append(spawn_after)
    else:
        # real robot or mock hardware, the controller manager loads the hardware interface from the robot description
        actions.append(Node(
            package='controller_manager', executable='ros2_control_node', namespace='lwr', output='screen',
            parameters=[controllers_file],
            remappings=[('/lwr/controller_manager/robot_description', '/robot_description')]))

    # spawn only desired controllers in current namespace
    spawners = [
        Node(package='controller_manager', executable='spawner', name='controller_spawner', namespace='lwr', output='screen',
             arguments=['joint_state_broadcaster', 'arm_state_controller'] + arg('controllers').split()
             + ['--controller-manager', '/lwr/controller_manager', '--controller-manager-timeout', '60']),
    ]
    if arg('stopped_controllers').split():
        spawners.append(Node(
            package='controller_manager', executable='spawner', name='controller_stopper', namespace='lwr', output='screen',
            arguments=arg('stopped_controllers').split()
            + ['--inactive', '--controller-manager', '/lwr/controller_manager', '--controller-manager-timeout', '60']))

    if spawn_after is not None:
        actions.append(RegisterEventHandler(OnProcessExit(target_action=spawn_after, on_exit=spawners)))
    else:
        actions += spawners

    # load moveit configuration
    if is_true('load_moveit'):
        moveit_launch = get_package_share_directory('single_lwr_moveit')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(moveit_launch, 'launch', 'move_group.launch.py')),
            launch_arguments={
                'allow_trajectory_execution': 'true',
                'info': 'false',
                # Load updated joint limits (override information from single_lwr_moveit) to respect T1 mode limits
                't1_limits': arg('t1_limits'),
                'use_sim_time': str(use_lwr_sim).lower(),
                'hardware_interface': hardware_interface,
            }.items()))
        # run Rviz and load the default config to see the state of the move_group node
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(moveit_launch, 'launch', 'moveit_rviz.launch.py')),
            launch_arguments={'use_sim_time': str(use_lwr_sim).lower()}.items()))

    return actions


def generate_launch_description():
    return LaunchDescription([
        # LAUNCH INTERFACE

        # in case you have different robot configurations
        DeclareLaunchArgument('robot_name', default_value='single_lwr_robot'),

        # the default is the simulator
        DeclareLaunchArgument('use_lwr_sim', default_value='true'),

        # set the parameters for the real interface
        DeclareLaunchArgument('lwr_powered', default_value='false'),
        DeclareLaunchArgument('real_interface', default_value='fri', description='fri or fril'),
        DeclareLaunchArgument('port', default_value='49939'),
        DeclareLaunchArgument('ip', default_value='192.168.0.10'),
        DeclareLaunchArgument('file', default_value=os.path.join(
            get_package_share_directory('single_lwr_robot'), 'config', '980241-FRI-Driver.init')),

        DeclareLaunchArgument('t1_limits', default_value='false'),
        DeclareLaunchArgument('controllers', default_value='joint_trajectory_controller'),
        DeclareLaunchArgument('stopped_controllers', default_value='gravity_compensation_controller one_task_inverse_kinematics'),

        # in case you want to load moveit from here, it might be hard with the real HW though
        DeclareLaunchArgument('load_moveit', default_value='false'),

        # set some ros tools
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('gui', default_value='false'),

        # LAUNCH IMPLEMENTATION
        OpaqueFunction(function=_launch_setup),
    ])
