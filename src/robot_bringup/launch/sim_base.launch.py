import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_robot_description = get_package_share_directory('robot_description')
    pkg_robot_bringup = get_package_share_directory('robot_bringup')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world_path = os.path.join(pkg_robot_bringup, 'worlds', 'maze.world')
    xacro_file = os.path.join(pkg_robot_description, 'urdf', 'create3_base.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': True}]
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

    # Launch Gazebo and pass ROS 2 remapping directly to gzserver
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'extra_gazebo_args': '--ros-args -r /diffdrive_controller/cmd_vel_unstamped:=/cmd_vel'
        }.items()
    )

    # Spawn Entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'create3_base', '-z', '0.05'],
        output='screen'
    )

    # Controllers
    diffdrive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diffdrive_controller'],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo,
        spawn_entity,
        diffdrive_spawner,
        joint_state_broadcaster_spawner
    ])