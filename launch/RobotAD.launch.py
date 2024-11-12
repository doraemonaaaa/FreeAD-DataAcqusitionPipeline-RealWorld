from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_hardware',
            executable='hardware_manager',
            name='hardware_manager',
            output='screen'
        ),
        # Node(
        #     package='keyboard_control',
        #     executable='keyboard_control_node',
        #     name='keyboard_control_node',
        #     output='screen'
        # ),
        Node(
            package='serial_comm',
            executable='serial_node',
            name='serial_comm_node',
            output='screen'
        )
    ])
