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

    # You can change this to match where you put your map.
    # If you rename the map files don't forget to edit the first line of the .yaml file to match the new name
    map_path = "/root/amr_ws/map.yaml"

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

    amr_share_directory = get_package_share_directory('amr_bringup')
    control_share_directory = get_package_share_directory('control')

    yahboomcar_description_share = get_package_share_directory('yahboomcar_description')
    urdf_path = os.path.join(yahboomcar_description_share, 'urdf', 'yahboomcar_R2.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    robot_description_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        parameters=[os.path.join(amr_share_directory, 'config', 'ekf.yaml')],
        remappings=[('/odometry/filtered', '/odom')]
    )

    lidar_param_path = os.path.join(control_share_directory, 'config', 'lidar_localization.yaml')
    lidar = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('ydlidar_ros2_driver'),
                'launch',
                'ydlidar_launch.py'
            ])
        ),
        launch_arguments = {'params_file': lidar_param_path}.items()
    )

    amr_hardware_driver = Node(
        package='amr_driver',
        executable='amr_driver',
        output='screen'
    )

    yahboom_odom = Node(
        package='yahboomcar_base_node',
        executable='base_node_R2'
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

    control_directory = get_package_share_directory('control')
    controller = Node(
        package='control',
        executable='control_node',
        output='screen',
        emulate_tty=True,
        parameters=[os.path.join(control_directory, 'config', 'controller.yaml')],
        remappings=[('/odom', '/pf/pose/odom')]
    )


    state_estimation_directory = get_package_share_directory('state_estimation')

    particle_filter = Node(
        package='state_estimation',
        executable='particle_filter',
        parameters=[os.path.join(state_estimation_directory, 'config', 'particle_filter.yaml')]
    )

    return LaunchDescription([
        robot_description_publisher,
        robot_localization,
        lidar,
        amr_hardware_driver,
        yahboom_odom,
        controller,
        map_server_node,
        nav_lifecycle_node,
        particle_filter
    ])
