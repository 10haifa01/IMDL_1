import os
import re
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro

def generate_launch_description():
    pkg_robot_description = get_package_share_directory('robot_description')
    pkg_robot_bringup = get_package_share_directory('robot_bringup')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_create3_description = get_package_share_directory('irobot_create_description')

    world_path = os.path.join(pkg_robot_bringup, 'worlds', 'maze.world')
    xacro_file = os.path.join(pkg_robot_description, 'urdf', 'create3_sensors.urdf.xacro')
    
    # 1. Robot Description (Custom Sensors + Create 3)
    robot_description_config = xacro.process_file(xacro_file, mappings={'sim': 'true'}).toxml()
    robot_description_param = ParameterValue(robot_description_config, value_type=str)

    # 2. Dock Description (Strip <collision> tags to make it non-solid)
    dock_xacro = os.path.join(pkg_create3_description, 'urdf', 'dock', 'standard_dock.urdf.xacro')
    dock_raw_xml = xacro.process_file(dock_xacro, mappings={'gazebo': 'classic'}).toxml()
    
    # Remove all collision tags from the dock XML
    dock_no_collision_xml = re.sub(r'<collision>.*?</collision>', '', dock_raw_xml, flags=re.DOTALL)
    dock_description_param = ParameterValue(dock_no_collision_xml, value_type=str)

    # Gazebo Model Paths
    desc_model_path = str(Path(pkg_robot_description).parent.resolve())
    create3_model_path = str(Path(pkg_create3_description).parent.resolve())
    existing = os.environ.get('GAZEBO_MODEL_PATH', '')
    gazebo_model_path = f"{desc_model_path}:{create3_model_path}" + (':' + existing if existing else '')
    set_model_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path)
    no_online_models = SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', '')

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_param,
            'use_sim_time': True
        }]
    )

    # Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'source_list': ['/joint_states']
        }]
    )

    # Gazebo World
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'extra_gazebo_args': '--ros-args -r /diffdrive_controller/cmd_vel_unstamped:=/cmd_vel'
        }.items()
    )

    # Spawn Robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'create3_base',
                   '-z', '0.05', '-timeout', '120'],
        output='screen'
    )

    # Spawners for Controllers
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

    # Dock State Publisher
    dock_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='dock_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': dock_description_param,
            'use_sim_time': True
        }],
        remappings=[('robot_description', 'standard_dock_description')]
    )

    # Spawn Non-Colliding Dock Entity
    spawn_dock = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
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