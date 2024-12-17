import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 设置串口路径参数
    serial_port_param = '/dev/motor'  # 替换为实际的串口设备路径
    baud_rate_param = 115200  # 波特率

    # 创建 SerialNode
    serial_node = Node(
        package='serial_comm',  # 替换为你的包名
        executable='serial_node',  # 可执行文件名称
        name='serial_node',        # 节点名称
        output='screen',           # 输出到终端
        parameters=[               # 参数列表
            {'serial_port': serial_port_param},
            {'baud_rate': baud_rate_param},
        ],
    )

    # 返回 LaunchDescription
    return LaunchDescription([
        serial_node
    ])
