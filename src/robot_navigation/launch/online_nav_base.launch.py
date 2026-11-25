import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_robot_navigation = get_package_share_directory('robot_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    default_slam_params = os.path.join(
        pkg_robot_navigation, 'config', 'slam_params_mapping.yaml'
    )
    default_nav2_params = os.path.join(
        pkg_robot_navigation, 'config', 'nav2_params.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    cmd_vel_target = LaunchConfiguration('cmd_vel_target')
    slam_params_file = LaunchConfiguration('slam_params_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true'
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true'
    )
    declare_cmd_vel_target = DeclareLaunchArgument(
        'cmd_vel_target', default_value='/diffdrive_controller/cmd_vel_unstamped'
    )
    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file', default_value=default_slam_params
    )
    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params_file', default_value=default_nav2_params
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': 'true',
        }.items(),
    )

    cmd_vel_relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        arguments=['/cmd_vel', cmd_vel_target],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(use_rviz),
        arguments=[
            '-d',
            os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz'),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rviz,
        declare_cmd_vel_target,
        declare_slam_params,
        declare_nav2_params,
        slam_toolbox_node,
        nav2_navigation_launch,
        cmd_vel_relay_node,
        rviz_node,
    ])