import rclpy
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        LogInfo(
            condition=None,
            msg="Starting keyboard control node..."
        ),

        # 启动 keyboard_control_node 节点
        Node(
            package='keyboard_control',  # 你的包名
            executable='keyboard_control_node',  # 可执行文件名，通常与节点的名称相同
            name='keyboard_control_node',  # 节点名称
            output='screen',  # 输出到屏幕
            emulate_tty=True,  # 使得节点的输入支持键盘输入
        ),
    ])
