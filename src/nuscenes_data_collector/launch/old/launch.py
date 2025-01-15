import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare the launch arguments for topic names
    return LaunchDescription([
        # Declare arguments for IMU, LIDAR, and ODOM topics
        DeclareLaunchArgument(
            'imu_topic', default_value='/issac/imu', description='IMU topic name'
        ),
        DeclareLaunchArgument(
            'lidar_topic', default_value='/issac/point_cloud', description='LIDAR topic name'
        ),
        DeclareLaunchArgument(
            'odom_topic', default_value='/issac/odom', description='Odom topic name'
        ),

        # Launch the IMU node
        Node(
            package='nuscenes_data_collector',
            executable='imu_to_nuscenes_node',
            name='imu_to_nuscenes',
            output='screen',
            parameters=[{'imu_topic': LaunchConfiguration('imu_topic')}],
        ),

        # Launch the LIDAR node
        Node(
            package='nuscenes_data_collector',
            executable='lidar_to_nuscenes_node',
            name='lidar_to_nuscenes',
            output='screen',
            parameters=[{'lidar_topic': LaunchConfiguration('lidar_topic')}],
        ),

        # Launch the Odom node
        Node(
            package='nuscenes_data_collector',
            executable='odom_to_nuscenes_node',
            name='odom_to_nuscenes',
            output='screen',
            parameters=[{'odom_topic': LaunchConfiguration('odom_topic')}],
        )
    ])
