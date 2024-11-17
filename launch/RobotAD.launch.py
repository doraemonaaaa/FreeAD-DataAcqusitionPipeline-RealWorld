from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='robot_hardware',
        #     executable='hardware_manager',
        #     name='hardware_manager',
        #     output='screen'
        # ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[
                {"robot_description": "src/robot_hardware/robot.rdf"},
                "src/robot_hardware/diff_drive_controller.yaml"
            ]
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_drive_controller"],
            output="screen",
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
