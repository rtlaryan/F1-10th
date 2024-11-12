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

    state_estimation_directory = get_package_share_directory('state_estimation')

    particle_filter = Node(
        package='state_estimation',
        executable='particle_filter',
        output='screen',
        emulate_tty=True,
        parameters=[os.path.join(state_estimation_directory, 'config', 'particle_filter.yaml')],
    )

    map_path = os.path.join(state_estimation_directory, 'maps', 'AMR.yaml')
    map_server_node = Node(
          package='nav2_map_server',
          executable='map_server',
          parameters=[{'yaml_filename': map_path},
                      {'topic': 'map'},
                      {'frame_id': 'map'},
                      {'output': 'screen'},
                      {'use_sim_time': True}]
      )
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )
 

    return LaunchDescription([
        particle_filter,
        map_server_node,
        nav_lifecycle_node
    ])
