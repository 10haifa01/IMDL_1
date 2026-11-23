import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_robot_navigation = get_package_share_directory('robot_navigation')
    pkg_robot_brain = get_package_share_directory('robot_brain')

    default_task_params = os.path.join(
        pkg_robot_brain, 'config', 'task_params.yaml'
    )
    task_params_file = LaunchConfiguration('task_params_file')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_task_params = DeclareLaunchArgument(
        'task_params_file', default_value=default_task_params
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true'
    )

    base_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_navigation, 'launch', 'online_nav_base.launch.py')
        ),
        launch_arguments={'use_rviz': use_rviz}.items(),
    )

    task2_node = Node(
        package='robot_brain',
        executable='task2_node',
        name='task2_node',
        output='screen',
        parameters=[task_params_file],
    )

    return LaunchDescription([
        declare_task_params,
        declare_use_rviz,
        base_nav_launch,
        task2_node,
    ])