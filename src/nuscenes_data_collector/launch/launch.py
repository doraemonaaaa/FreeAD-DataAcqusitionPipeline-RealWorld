import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    param_file = os.path.join(
        get_package_share_directory('nuscenes_data_collector'),
        'config',
        'nuscenes_data_collector_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'data_collector_type',
            default_value='issac',
            description='Choose the type of NuScenesDataCollector node (issac or default)'
        ),
        
        Node(
            package='nuscenes_data_collector',
            executable=LaunchConfiguration('data_collector_type'),
            name='nuscenes_data_collector',
            output='screen',
            parameters=[param_file]
        )
    ])
