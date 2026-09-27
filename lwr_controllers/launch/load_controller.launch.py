# Spawn (load, configure and activate) lwr controllers in a running controller manager.
#
#   ros2 launch lwr_controllers load_controller.launch.py controller:=one_task_inverse_kinematics
#   ros2 launch lwr_controllers load_controller.launch.py controller:="joint_trajectory_controller" \
#       stopped_controllers:="computed_torque_controller"
#
# The controllers must be declared in the controller manager parameters (see single_lwr_robot/config/controllers.yaml),
# or in the file given with param_file.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawners(context):
    controller_manager = LaunchConfiguration('controller_manager').perform(context)
    param_file = LaunchConfiguration('param_file').perform(context)
    controllers = LaunchConfiguration('controller').perform(context).split()
    stopped_controllers = LaunchConfiguration('stopped_controllers').perform(context).split()

    common_args = ['--controller-manager', controller_manager]
    if param_file:
        common_args += ['--param-file', param_file]

    actions = [Node(
        package='controller_manager', executable='spawner', name='controller_spawner', output='screen',
        arguments=['joint_state_broadcaster'] + controllers + common_args)]
    if stopped_controllers:
        actions.append(Node(
            package='controller_manager', executable='spawner', name='controller_spawner_stopped', output='screen',
            arguments=stopped_controllers + common_args + ['--inactive']))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='joint_trajectory_controller',
                              description='Name(s) of the controller(s) to be loaded and started, separated by spaces'),
        DeclareLaunchArgument('stopped_controllers', default_value='one_task_inverse_kinematics',
                              description='Additional controllers to be loaded, but not started (can be started later with ros2 control)'),
        DeclareLaunchArgument('controller_manager', default_value='/lwr/controller_manager',
                              description='The controller manager to load the controllers in'),
        DeclareLaunchArgument('param_file', default_value='',
                              description='Optional controller parameters file'),
        OpaqueFunction(function=_spawners),
    ])
