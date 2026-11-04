import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_robot_bringup = get_package_share_directory('robot_bringup')
    pkg_robot_navigation = get_package_share_directory('robot_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    slam_params_file = os.path.join(pkg_robot_navigation, 'config', 'slam_params_mapping.yaml')

    # 1. Bring up Gazebo simulation base (World, Robot, Controllers, Dock)
    sim_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_bringup, 'launch', 'sim_base.launch.py')
        )
    )

    # 2. Async SLAM Toolbox in Mapping Mode
    slam_toolbox_mapping_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': True}
        ]
    )

    # 3. Launch RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        sim_base_launch,
        slam_toolbox_mapping_node,
        rviz_node
    ])