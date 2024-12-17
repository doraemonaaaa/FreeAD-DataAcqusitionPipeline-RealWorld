from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare Launch Arguments
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),
        DeclareLaunchArgument(
            name='point_cloud_topic', default_value='/scanner/cloud',
            description='Topic to subscribe to for PointCloud2 messages'
        ),
        DeclareLaunchArgument(
            name='scan_topic', default_value='/scan',
            description='Topic to publish LaserScan messages'
        ),
        DeclareLaunchArgument(
            name='target_frame', default_value='lidar_link',
            description='The frame to which the point cloud should be transformed'
        ),
        DeclareLaunchArgument(
            name='queue_size', default_value='50',
            description='The size to save point queue'
        ),

        # pointcloud_to_laserscan Node
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[
                ('cloud_in', LaunchConfiguration('point_cloud_topic')),  # Remap to custom PointCloud2 topic
                ('scan', LaunchConfiguration('scan_topic'))  # Remap to custom LaserScan topic
            ],
            parameters=[{
                'target_frame': LaunchConfiguration('target_frame'),  # Use dynamic frame name
                'transform_tolerance': 0.1,
                'min_height': 0.0,
                'max_height': 2.0,
                'angle_min': -3.141592,  # -M_PI/2
                'angle_max': 3.141592,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.1,
                'range_min': 0.1,
                'range_max': 10.0,
                'use_inf': False,
                'inf_epsilon': 1.0,
                'queue_size': LaunchConfiguration('queue_size'),
            }],
            name='pointcloud_to_laserscan'
        ),
        
        # If you need to add a static transform, you can uncomment the following
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher',
        #     arguments=[
        #         '--x', '0', '--y', '0', '--z', '0',
        #         '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
        #         '--frame-id', 'map', '--child-frame-id', 'cloud'
        #     ]
        # ),
    ])
