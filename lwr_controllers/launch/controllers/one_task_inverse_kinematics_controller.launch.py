from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('lwr_controllers'), 'launch', 'load_controller.launch.py'])),
            launch_arguments={'controller': 'one_task_inverse_kinematics', 'stopped_controllers': ''}.items(),
        ),
    ])
