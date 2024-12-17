cmake_minimum_required(VERSION 3.8)
project(robot_hardware)

# 添加编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)  # 支持图像消息
find_package(cv_bridge REQUIRED)    # 支持 OpenCV 与 ROS 图像转换
find_package(OpenCV REQUIRED)       # 支持 OpenCV 功能
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 添加消息文件
set(msg_files
  "msg/HardwareData.msg"
  "msg/MotorData.msg"
)

# 生成消息
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

# 导出依赖，确保其他包可以使用生成的接口
ament_export_dependencies(rosidl_default_runtime)

# 添加可执行文件
add_executable(hardware_manager 
  src/hardware_manager.cpp
)

# 指定依赖项
ament_target_dependencies(hardware_manager
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
)

# 添加对生成的接口的依赖
rosidl_get_typesupport_target(
  hardware_manager
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
target_link_libraries(hardware_manager
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# 指定包含目录
target_include_directories(hardware_manager PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  # 项目的 include 路径
  $<INSTALL_INTERFACE:include>  # 安装时的路径
)


# 安装可执行文件
install(TARGETS hardware_manager
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装消息文件
install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}/msg
)

# 包设置
ament_package()
