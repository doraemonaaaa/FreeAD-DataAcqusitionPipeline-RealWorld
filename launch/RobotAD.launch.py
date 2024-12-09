from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # 声明 use_sim_time 参数
    use_sim_time = LaunchConfiguration('use_sim_time')

    # LiDAR 启动文件路径
    rslidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rslidar_sdk'), 'launch', 'start.py')
        ),
        condition=UnlessCondition(use_sim_time)
    )

    # IMU 启动文件路径
    imu_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('miiboo_imu'), 'launch', 'imu.launch.py')
        ),
        condition=UnlessCondition(use_sim_time)
    )

    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Enable use_sim_time for simulation'
        ),

        # Hardware Manager 节点
        Node(
            package='robot_hardware',
            executable='hardware_manager',
            name='hardware_manager',
            output='screen'
        ),

        # 串口通信节点
        Node(
            package='serial_comm',
            executable='serial_node',
            name='serial_comm_node',
            output='screen'
        ),

        # LiDAR 驱动启动文件
        rslidar_driver,

        # IMU 驱动启动文件
        imu_driver
    ])
