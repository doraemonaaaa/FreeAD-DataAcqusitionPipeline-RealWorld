cmake_minimum_required(VERSION 3.5)
project(serial_comm)

# 依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)
find_package(robot_hardware REQUIRED)

# 添加可执行文件
add_executable(serial_node src/serial_node.cpp)

# 链接所需的库
ament_target_dependencies(serial_node
  rclcpp
  serial
  robot_hardware
)

# 安装可执行文件
install(TARGETS
  serial_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 Launch 文件和配置文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
