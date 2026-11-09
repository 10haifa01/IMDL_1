import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro

def generate_launch_description():
    # Paths
    pkg_robot_description = get_package_share_directory('robot_description')
    xacro_file = os.path.join(pkg_robot_description, 'urdf', 'create3_sensors.urdf.xacro')

    # Process URDF for physical hardware execution (sim:=false)
    robot_description_config = xacro.process_file(xacro_file, mappings={'sim': 'false'}).toxml()
    robot_description_param = ParameterValue(robot_description_config, value_type=str)

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_param,
            'use_sim_time': False
        }],
        remappings=[
            ('/tf', '/robot1_ws/tf'),
            ('/tf_static', '/robot1_ws/tf_static')
        ]
    )

    # Lidar driver
    lidar_driver_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 460800,
            'frame_id': 'lidar_link',
            'angle_compensate': True,
        }],
        remappings=[('scan', '/scan')]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        lidar_driver_node
    ])