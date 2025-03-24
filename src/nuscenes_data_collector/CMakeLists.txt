cmake_minimum_required(VERSION 3.8)
project(nuscenes_data_collector)

# 设置 CMake 策略
cmake_policy(SET CMP0074 NEW)
set(PCL_ROOT "/home/pyh/Documents/DataSet")

# 如果是 GCC 或 Clang 编译器，添加一些编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(message_filters REQUIRED)  # 添加 message_filters 依赖
find_package(tf2_msgs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)


# 定义公共头文件目录
set(INCLUDE_DIRS
  ${PROJECT_SOURCE_DIR}/include  # 主要的 include 路径
  ${PROJECT_SOURCE_DIR}/include/nuscenes_data_collector  # 头文件所在的子目录
)

# 编译两个可执行文件
add_executable(nuscenes_data_collector_node src/nuscenes_data_collector.cpp)
add_executable(nuscenes_data_collector_issac_node src/nuscenes_data_collector_issac.cpp)

# 设置公共的头文件目录
target_include_directories(nuscenes_data_collector_node PRIVATE ${INCLUDE_DIRS})
target_include_directories(nuscenes_data_collector_issac_node PRIVATE ${INCLUDE_DIRS})

include_directories(
  ${tf2_msgs_INCLUDE_DIRS}
)

# 连接所需的库
ament_target_dependencies(nuscenes_data_collector_node
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
  nlohmann_json
  nav_msgs
  vision_msgs
  geometry_msgs
  std_msgs
  OpenCV
  cv_bridge
  OpenSSL
  message_filters
  tf2_msgs
  rosidl_typesupport_cpp  
)

ament_target_dependencies(nuscenes_data_collector_issac_node
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
  nlohmann_json
  nav_msgs
  vision_msgs
  geometry_msgs
  std_msgs
  OpenCV
  cv_bridge
  OpenSSL
  message_filters
  tf2_msgs
  rosidl_typesupport_cpp  
)

# 确保链接 OpenSSL 的加密库（libcrypto）
target_link_libraries(nuscenes_data_collector_node OpenSSL::Crypto)
target_link_libraries(nuscenes_data_collector_issac_node OpenSSL::Crypto)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装目标
install(TARGETS
  nuscenes_data_collector_node
  nuscenes_data_collector_issac_node
  DESTINATION lib/${PROJECT_NAME}
)

# 包的安装规则
ament_package()
