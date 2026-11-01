import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    slam_params = os.path.join(get_package_share_directory('robot_bringup'),
                               'config', 'slam_params_real.yaml')

    lidar_driver = Node(
        package='sllidar_ros2', executable='sllidar_node', name='sllidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 460800,        # verify in the package's C1 launch file
            'frame_id': 'rplidar_c1_link',
            'angle_compensate': True,
        }]
    )

    # Measured mount pose of the lidar relative to base_link (replace with real values)
    lidar_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.14', '--yaw', '0.0',
                   '--frame-id', 'base_link', '--child-frame-id', 'rplidar_c1_link']
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={'slam_params_file': slam_params, 'use_sim_time': 'false'}.items()
    )

    return LaunchDescription([lidar_driver, lidar_tf, slam])