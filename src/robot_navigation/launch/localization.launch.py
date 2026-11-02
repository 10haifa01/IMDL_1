import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_robot_navigation = get_package_share_directory('robot_navigation')
    
    # Path to YAML parameters
    slam_params_file = os.path.join(pkg_robot_navigation, 'config', 'slam_params_localization.yaml')
    
    # Dynamic path to map graph file without extension
    map_file_path = os.path.join(pkg_robot_navigation, 'maps', 'lab_map_graph')

    slam_toolbox_localization_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {
                'use_sim_time': True,
                'map_file_name': map_file_path  # Overrides hardcoded YAML path
            }
        ]
    )

    return LaunchDescription([
        slam_toolbox_localization_node
    ])