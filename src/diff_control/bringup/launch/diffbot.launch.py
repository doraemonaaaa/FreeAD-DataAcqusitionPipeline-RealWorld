# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 声明启动参数
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",  # RViz是否自动启动
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",  # 是否启用仿真时间
            default_value="false",
            description="Enable use_sim_time=>true to use gazebo simulation, disable to use real hardware.",
        )
    )

    # 初始化参数
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # 通过xacro获取URDF描述
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),  # 执行xacro命令
            " ",
            PathJoinSubstitution(
                [FindPackageShare("diff_control"), "urdf", "diffbot.urdf.xacro"]  # 指定URDF文件路径
            ),
            " ",
            "use_sim_time:=",
            use_sim_time,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # 启动机器人状态发布器节点
    robot_state_pub_node = Node(
        package="robot_state_publisher",  # 发布机器人状态
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],  # 传递机器人描述
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),  # 重映射命令话题
        ],
    )

    # joint_state_publisher_node = Node(  # 注释掉的部分，可以启用发布关节状态的节点
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[{"use_sim_time": use_sim_time}]
    # )

    # 获取机器人控制器配置文件路径
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("diff_control"),
            "config",
            "diffbot_controllers.yaml",  # 控制器配置文件
        ]
    )

    # 启动控制节点
    control_node = Node(
        package="controller_manager",  # 控制器管理器包
        executable="ros2_control_node",  # ROS 2 控制节点
        parameters=[robot_description, robot_controllers],  # 传递机器人描述和控制器配置
        output="both",
    )

    # 启动关节状态广播器
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],  # 启动关节状态广播器
    )

    # 启动机器人控制器
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],  # 启动差分驱动控制器
    )

    # 配置RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("diff_control"), "config", "urdf.rviz"]  # 指定RViz配置文件路径
    )

    rviz_node = Node(
        package="rviz2",  # 启动RViz节点
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],  # 传递RViz配置文件
        condition=IfCondition(gui),  # 如果gui为true，启动RViz
    )

    # 延迟启动RViz，确保关节状态广播器已经启动
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],  # 在关节状态广播器退出后启动RViz
        )
    )

    # 延迟启动机器人控制器，确保关节状态广播器已启动
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],  # 在关节状态广播器退出后启动机器人控制器
        )
    )

    # 定义节点列表
    nodes = [
        control_node,
        robot_state_pub_node,
        # joint_state_publisher_node,  # 注释掉的节点
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    # 返回启动描述，包含启动参数和节点
    return LaunchDescription(declared_arguments + nodes)

    # 以下部分是注释掉的代码，暂时不启用
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "use_ros2_control",
    #         default_value="True",
    #         description="Get odom=>base_link with ros2_control.",
    #     )
    # )

    # use_ros2_control = LaunchConfiguration("use_ros2_control")

    # "use_ros2_control:=",
    # use_ros2_control,   # 注释在代码里面
# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 声明启动参数
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",  # RViz是否自动启动
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",  # 是否启用仿真时间
            default_value="false",
            description="Enable use_sim_time=>true to use gazebo simulation, disable to use real hardware.",
        )
    )

    # 初始化参数
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # 通过xacro获取URDF描述
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),  # 执行xacro命令
            " ",
            PathJoinSubstitution(
                [FindPackageShare("diff_control"), "urdf", "diffbot.urdf.xacro"]  # 指定URDF文件路径
            ),
            " ",
            "use_sim_time:=",
            use_sim_time,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # 启动机器人状态发布器节点
    robot_state_pub_node = Node(
        package="robot_state_publisher",  # 发布机器人状态
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],  # 传递机器人描述
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),  # 重映射命令话题
        ],
    )

    # joint_state_publisher_node = Node(  # 注释掉的部分，可以启用发布关节状态的节点
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[{"use_sim_time": use_sim_time}]
    # )

    # 获取机器人控制器配置文件路径
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("diff_control"),
            "config",
            "diffbot_controllers.yaml",  # 控制器配置文件
        ]
    )

    # 启动控制节点
    control_node = Node(
        package="controller_manager",  # 控制器管理器包
        executable="ros2_control_node",  # ROS 2 控制节点
        parameters=[robot_description, robot_controllers],  # 传递机器人描述和控制器配置
        output="both",
    )

    # 启动关节状态广播器
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],  # 启动关节状态广播器
    )

    # 启动机器人控制器
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],  # 启动差分驱动控制器
    )

    # 配置RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("diff_control"), "config", "urdf.rviz"]  # 指定RViz配置文件路径
    )

    rviz_node = Node(
        package="rviz2",  # 启动RViz节点
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],  # 传递RViz配置文件
        condition=IfCondition(gui),  # 如果gui为true，启动RViz
    )

    # 延迟启动RViz，确保关节状态广播器已经启动
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],  # 在关节状态广播器退出后启动RViz
        )
    )

    # 延迟启动机器人控制器，确保关节状态广播器已启动
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],  # 在关节状态广播器退出后启动机器人控制器
        )
    )

    # 定义节点列表
    nodes = [
        control_node,
        robot_state_pub_node,
        # joint_state_publisher_node,  # 注释掉的节点
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    # 返回启动描述，包含启动参数和节点
    return LaunchDescription(declared_arguments + nodes)

    # 以下部分是注释掉的代码，暂时不启用
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "use_ros2_control",
    #         default_value="True",
    #         description="Get odom=>base_link with ros2_control.",
    #     )
    # )

    # use_ros2_control = LaunchConfiguration("use_ros2_control")

    # "use_ros2_control:=",
    # use_ros2_control,   # 注释在代码里面
# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 声明启动参数
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",  # RViz是否自动启动
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",  # 是否启用仿真时间
            default_value="false",
            description="Enable use_sim_time=>true to use gazebo simulation, disable to use real hardware.",
        )
    )

    # 初始化参数
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # 通过xacro获取URDF描述
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),  # 执行xacro命令
            " ",
            PathJoinSubstitution(
                [FindPackageShare("diff_control"), "urdf", "diffbot.urdf.xacro"]  # 指定URDF文件路径
            ),
            " ",
            "use_sim_time:=",
            use_sim_time,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # 启动机器人状态发布器节点
    robot_state_pub_node = Node(
        package="robot_state_publisher",  # 发布机器人状态
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],  # 传递机器人描述
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),  # 重映射命令话题
        ],
    )

    # joint_state_publisher_node = Node(  # 注释掉的部分，可以启用发布关节状态的节点
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[{"use_sim_time": use_sim_time}]
    # )

    # 获取机器人控制器配置文件路径
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("diff_control"),
            "config",
            "diffbot_controllers.yaml",  # 控制器配置文件
        ]
    )

    # 启动控制节点
    control_node = Node(
        package="controller_manager",  # 控制器管理器包
        executable="ros2_control_node",  # ROS 2 控制节点
        parameters=[robot_description, robot_controllers],  # 传递机器人描述和控制器配置
        output="both",
    )

    # 启动关节状态广播器
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],  # 启动关节状态广播器
    )

    # 启动机器人控制器
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],  # 启动差分驱动控制器
    )

    # 配置RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("diff_control"), "config", "urdf.rviz"]  # 指定RViz配置文件路径
    )

    rviz_node = Node(
        package="rviz2",  # 启动RViz节点
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],  # 传递RViz配置文件
        condition=IfCondition(gui),  # 如果gui为true，启动RViz
    )

    # 延迟启动RViz，确保关节状态广播器已经启动
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],  # 在关节状态广播器退出后启动RViz
        )
    )

    # 延迟启动机器人控制器，确保关节状态广播器已启动
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],  # 在关节状态广播器退出后启动机器人控制器
        )
    )

    # 定义节点列表
    nodes = [
        control_node,
        robot_state_pub_node,
        # joint_state_publisher_node,  # 注释掉的节点
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    # 返回启动描述，包含启动参数和节点
    return LaunchDescription(declared_arguments + nodes)

    # 以下部分是注释掉的代码，暂时不启用
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "use_ros2_control",
    #         default_value="True",
    #         description="Get odom=>base_link with ros2_control.",
    #     )
    # )

    # use_ros2_control = LaunchConfiguration("use_ros2_control")

    # "use_ros2_control:=",
    # use_ros2_control,   # 注释在代码里面
# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 声明启动参数
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",  # RViz是否自动启动
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",  # 是否启用仿真时间
            default_value="false",
            description="Enable use_sim_time=>true to use gazebo simulation, disable to use real hardware.",
        )
    )

    # 初始化参数
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # 通过xacro获取URDF描述
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),  # 执行xacro命令
            " ",
            PathJoinSubstitution(
                [FindPackageShare("diff_control"), "urdf", "diffbot.urdf.xacro"]  # 指定URDF文件路径
            ),
            " ",
            "use_sim_time:=",
            use_sim_time,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # 启动机器人状态发布器节点
    robot_state_pub_node = Node(
        package="robot_state_publisher",  # 发布机器人状态
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],  # 传递机器人描述
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),  # 重映射命令话题
        ],
    )

    # joint_state_publisher_node = Node(  # 注释掉的部分，可以启用发布关节状态的节点
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[{"use_sim_time": use_sim_time}]
    # )

    # 获取机器人控制器配置文件路径
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("diff_control"),
            "config",
            "diffbot_controllers.yaml",  # 控制器配置文件
        ]
    )

    # 启动控制节点
    control_node = Node(
        package="controller_manager",  # 控制器管理器包
        executable="ros2_control_node",  # ROS 2 控制节点
        parameters=[robot_description, robot_controllers],  # 传递机器人描述和控制器配置
        output="both",
    )

    # 启动关节状态广播器
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],  # 启动关节状态广播器
    )

    # 启动机器人控制器
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],  # 启动差分驱动控制器
    )

    # 配置RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("diff_control"), "config", "urdf.rviz"]  # 指定RViz配置文件路径
    )

    rviz_node = Node(
        package="rviz2",  # 启动RViz节点
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],  # 传递RViz配置文件
        condition=IfCondition(gui),  # 如果gui为true，启动RViz
    )

    # 延迟启动RViz，确保关节状态广播器已经启动
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],  # 在关节状态广播器退出后启动RViz
        )
    )

    # 延迟启动机器人控制器，确保关节状态广播器已启动
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],  # 在关节状态广播器退出后启动机器人控制器
        )
    )

    # 定义节点列表
    nodes = [
        control_node,
        robot_state_pub_node,
        # joint_state_publisher_node,  # 注释掉的节点
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    # 返回启动描述，包含启动参数和节点
    return LaunchDescription(declared_arguments + nodes)

    # 以下部分是注释掉的代码，暂时不启用
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "use_ros2_control",
    #         default_value="True",
    #         description="Get odom=>base_link with ros2_control.",
    #     )
    # )

    # use_ros2_control = LaunchConfiguration("use_ros2_control")

    # "use_ros2_control:=",
    # use_ros2_control,   # 注释在代码里面
