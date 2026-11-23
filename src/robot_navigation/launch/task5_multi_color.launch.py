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
    pkg_robot_brain = get_package_share_directory('robot_brain')

    default_task_params = os.path.join(
        pkg_robot_brain, 'config', 'task_params.yaml'
    )
    explore_params_file = os.path.join(
        pkg_robot_navigation, 'config', 'explore_params.yaml'
    )

    task_params_file = LaunchConfiguration('task_params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_virtual_camera = LaunchConfiguration('use_virtual_camera')

    declare_task_params = DeclareLaunchArgument(
        'task_params_file', default_value=default_task_params
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='true'
    )
    declare_virtual_cam = DeclareLaunchArgument(
        'use_virtual_camera', default_value='true'
    )

    base_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_navigation, 'launch', 'online_nav_base.launch.py')
        ),
        launch_arguments={'use_rviz': use_rviz}.items(),
    )

    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[explore_params_file, {'use_sim_time': True}],
    )

    virtual_detector_node = Node(
        package='robot_brain',
        executable='virtual_cube_detector',
        name='virtual_cube_detector',
        condition=IfCondition(use_virtual_camera),
        output='screen',
        parameters=[task_params_file, {'task_mode': 'task5'}],
    )

    task5_node = Node(
        package='robot_brain',
        executable='task5_node',
        name='task5_node',
        output='screen',
        parameters=[task_params_file],
    )

    return LaunchDescription([
        declare_task_params,
        declare_use_rviz,
        declare_virtual_cam,
        base_nav_launch,
        explore_node,
        virtual_detector_node,
        task5_node,
    ])