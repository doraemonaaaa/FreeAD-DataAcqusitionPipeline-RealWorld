cmake_minimum_required(VERSION 3.5)

# 项目名称必须与 package.xml 中的包名一致
project(usb_camera)

# 找到所需的包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 添加可执行文件
add_executable(${PROJECT_NAME}_node src/usb_camera_node.cpp)

# 链接依赖项
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

# 添加 include 目录
include_directories(
  include  # 添加自己的 include 目录
)

# 将 YAML 配置文件放到 share 目录
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装目标
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

# ament 宏
ament_package()