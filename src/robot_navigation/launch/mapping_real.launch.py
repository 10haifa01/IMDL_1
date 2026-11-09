import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_robot_bringup = get_package_share_directory('robot_bringup')
    pkg_robot_navigation = get_package_share_directory('robot_navigation')

    slam_params_file = os.path.join(pkg_robot_navigation, 'config', 'slam_params_mapping_real.yaml')

    # 1. Bring up real hardware (LIDAR + URDF state publisher)
    hardware_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_bringup, 'launch', 'real_robot.launch.py')
        )
    )

    # 2. Async SLAM Toolbox in Mapping Mode
    slam_toolbox_mapping_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file],
        remappings=[
            ('/tf', '/robot1_ws/tf'),
            ('/tf_static', '/robot1_ws/tf_static'),
            ('/odom', '/robot1_ws/odom')
        ]
    )

    return LaunchDescription([
        hardware_bringup,
        slam_toolbox_mapping_node
    ])