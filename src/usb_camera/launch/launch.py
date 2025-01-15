import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    param_file = os.path.join(
        get_package_share_directory('usb_camera'),
        'config',
        'camera_params.yaml'  # 这里是 YAML 文件的路径
    )

    return LaunchDescription([
        Node(
            package='usb_camera',  # 这里是你的 ROS2 包名
            executable='usb_camera_node',  # 这里是可执行文件的名称
            name='usb_camera_node',  # 你想给节点设置的名称
            output='screen',  # 输出到终端
            parameters=[param_file]  # 使用外部的 YAML 文件作为参数配置
        )
    ])
