from ament_index_python import get_package_share_directory
import launch
import os
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    amr_share_directory = get_package_share_directory('amr_bringup')
    control_share_directory = get_package_share_directory('control')

    record_path = Node(
        package='control',
        executable='record_path',
        output='screen',
        emulate_tty=True,
        remappings=[('/odom', '/pf/pose/odom')]
    )

    joystick_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('amr_teleop'),
                'launch',
                'teleop_joy_launch.py'
            ])
        )
    )

    return LaunchDescription([
        record_path,
        joystick_launch
    ])