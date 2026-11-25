import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_robot_navigation = get_package_share_directory('robot_navigation')

    real_slam_params = os.path.join(
        pkg_robot_navigation, 'config', 'slam_params_mapping_real.yaml'
    )
    real_nav2_params = os.path.join(
        pkg_robot_navigation, 'config', 'nav2_params_real.yaml'
    )

    use_rviz = LaunchConfiguration('use_rviz')
    cmd_vel_target = LaunchConfiguration('cmd_vel_target')

    declare_use_rviz = DeclareLaunchArgument('use_rviz', default_value='true')
    declare_cmd_vel_target = DeclareLaunchArgument(
        'cmd_vel_target', default_value='/robot1_ws/cmd_vel'
    )

    task1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_navigation, 'launch', 'task1_locomotion.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'use_rviz': use_rviz,
            'cmd_vel_target': cmd_vel_target,
            'slam_params_file': real_slam_params,
            'nav2_params_file': real_nav2_params,
        }.items(),
    )

    return LaunchDescription([
        declare_use_rviz,
        declare_cmd_vel_target,
        task1_launch,
    ])