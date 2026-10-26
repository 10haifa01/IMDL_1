import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    xacro_path = os.path.expanduser('~/IMDL_1/src/robot_description/urdf/lab_robot.urdf.xacro')
    robot_description_content = ParameterValue(Command(['xacro ', xacro_path]), value_type=str)

    # Let Gazebo resolve package://irobot_create_description/... mesh URIs
    create3_share = get_package_share_directory('irobot_create_description')
    model_path = str(Path(create3_share).parent.resolve())
    existing = os.environ.get('GAZEBO_MODEL_PATH', '')
    gazebo_model_path = model_path + (':' + existing if existing else '')

    set_model_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path)
    # Don't try to download models from the internet at startup
    no_online_models = SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', '')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        )
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content,
                     'use_sim_time': True}]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'create3',
                   '-z', '0.1', '-timeout', '120'],
        output='screen'
    )

    return LaunchDescription([
        set_model_path,
        no_online_models,
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
    ])