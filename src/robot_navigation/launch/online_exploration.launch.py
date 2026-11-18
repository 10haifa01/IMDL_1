import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_robot_navigation = get_package_share_directory('robot_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    slam_params_file = os.path.join(pkg_robot_navigation, 'config', 'slam_params_mapping.yaml')
    nav2_params_file = os.path.join(pkg_robot_navigation, 'config', 'nav2_params.yaml')
    explore_params_file = os.path.join(pkg_robot_navigation, 'config', 'explore_params.yaml')

    # 1. Online SLAM Toolbox (Mapping Mode - builds /map and map->odom live)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': True}
        ]
    )

    # 2. Nav2 Navigation Stack (without AMCL or static map_server)
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file,
            'autostart': 'true'
        }.items()
    )

    # 3. Relay /cmd_vel -> Create 3 diffdrive controller topic
    cmd_vel_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        arguments=['/cmd_vel', '/diffdrive_controller/cmd_vel_unstamped'],
        output='screen'
    )

    # 4. Autonomous Frontier Exploration Node
    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[
            explore_params_file,
            {'use_sim_time': True}
        ]
    )

    # 5. RViz2 Visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    post_exploration_node = Node(
        package='robot_brain',
        executable='post_exploration_sweeper',
        name='post_exploration_sweeper',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        slam_toolbox_node,
        nav2_navigation_launch,
        cmd_vel_relay_node,
        explore_node,
        rviz_node,
        post_exploration_node
    ])