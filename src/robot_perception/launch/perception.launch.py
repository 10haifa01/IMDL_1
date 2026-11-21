import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_camera = LaunchConfiguration('use_camera')

    declare_use_camera_cmd = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Launch the RealSense D455 camera driver alongside the detector'
    )

    # Include the official RealSense ROS 2 launch file with depth-to-color alignment enabled
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
            'align_depth.enable': 'true',
        }.items(),
        condition=IfCondition(use_camera)
    )

    # Launch your HSV color detection node
    color_detector_node = Node(
        package='robot_perception',
        executable='color_detector_node',
        name='color_detector_node',
        output='screen'
    )

    return LaunchDescription([
        declare_use_camera_cmd,
        realsense_launch,
        color_detector_node,
    ])