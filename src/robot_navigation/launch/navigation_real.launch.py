import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap

def generate_launch_description():
    # Paths
    pkg_robot_navigation = get_package_share_directory('robot_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    nav2_params_file = os.path.join(pkg_robot_navigation, 'config', 'nav2_params_real.yaml')
    map_yaml_file = os.path.join(pkg_robot_navigation, 'maps', 'lab_map.yaml')

    # Include slam_toolbox real localization launch
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_navigation, 'launch', 'localization_real.launch.py')
        )
    )

    # Wrap Nav2 stack
    nav2_group = GroupAction(
        actions=[
            SetRemap(src='/tf', dst='/robot1_ws/tf'),
            SetRemap(src='/tf_static', dst='/robot1_ws/tf_static'),
            SetRemap(src='/odom', dst='/robot1_ws/odom'),
            SetRemap(src='/cmd_vel', dst='/robot1_ws/cmd_vel'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file': nav2_params_file,
                    'map': map_yaml_file,
                    'autostart': 'true'
                }.items()
            )
        ]
    )

    return LaunchDescription([
        localization_launch,
        nav2_group
    ])