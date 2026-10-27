import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_robot_bringup = get_package_share_directory('robot_bringup')
    rviz_config_path = os.path.join(pkg_robot_bringup, 'config', 'view_robot.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
    ])
