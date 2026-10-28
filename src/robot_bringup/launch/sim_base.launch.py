import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_robot_description = get_package_share_directory('robot_description')
    pkg_robot_bringup = get_package_share_directory('robot_bringup')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_create3_description = get_package_share_directory('irobot_create_description')

    world_path = os.path.join(pkg_robot_bringup, 'worlds', 'maze.world')
    xacro_file = os.path.join(pkg_robot_description, 'urdf', 'create3_base.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # Let Gazebo resolve package://irobot_create_description/... mesh URIs
    model_path = str(Path(pkg_create3_description).parent.resolve())
    existing = os.environ.get('GAZEBO_MODEL_PATH', '')
    gazebo_model_path = model_path + (':' + existing if existing else '')
    set_model_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path)
    # Stop Gazebo hanging on the online model download (WSL)
    no_online_models = SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', '')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': True}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'source_list': ['/joint_states']
        }]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'extra_gazebo_args': '--ros-args -r /diffdrive_controller/cmd_vel_unstamped:=/cmd_vel'
        }.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'create3_base',
                   '-z', '0.05', '-timeout', '120'],
        output='screen'
    )

    diffdrive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diffdrive_controller', '--controller-manager-timeout', '120'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '120'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    dock_xacro = os.path.join(pkg_create3_description, 'urdf', 'dock', 'standard_dock.urdf.xacro')
    dock_description = xacro.process_file(dock_xacro, mappings={'gazebo': 'classic'}).toxml()

    dock_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='dock_state_publisher', output='screen',
        parameters=[{'robot_description': dock_description, 'use_sim_time': True}],
        remappings=[('robot_description', 'standard_dock_description')]
    )

    spawn_dock = Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        arguments=['-topic', 'standard_dock_description', '-entity', 'standard_dock',
                '-x', '0.157', '-y', '0.0', '-z', '0.0', '-Y', '3.1416', '-timeout', '120']
    )

    return LaunchDescription([
        set_model_path,
        no_online_models,
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo,
        spawn_entity,
        diffdrive_spawner,
        joint_state_broadcaster_spawner,
        dock_state_publisher,
        spawn_dock
    ])