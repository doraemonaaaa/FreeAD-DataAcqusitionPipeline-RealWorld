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

# 为节点分别定义源文件
set(IMU_SOURCE_FILES
  src/imu_to_nuscenes.cpp  # IMU 节点的源文件
)
set(LIDAR_SOURCE_FILES
  src/lidar_to_nuscenes.cpp  # Lidar 节点的源文件
)
set(ODOM_SOURCE_FILES
  src/odom_to_nuscenes.cpp  # Odom 节点的源文件
)

# 定义公共头文件目录
set(INCLUDE_DIRS
  ${PROJECT_SOURCE_DIR}/include  # 主要的 include 路径
  ${PROJECT_SOURCE_DIR}/include/nuscenes_data_collector  # 头文件所在的子目录
)

# 添加可执行文件
add_executable(imu_to_nuscenes_node ${IMU_SOURCE_FILES})
add_executable(lidar_to_nuscenes_node ${LIDAR_SOURCE_FILES})
add_executable(odom_to_nuscenes_node ${ODOM_SOURCE_FILES})

# 设置公共的头文件目录
target_include_directories(imu_to_nuscenes_node PRIVATE ${INCLUDE_DIRS})
target_include_directories(lidar_to_nuscenes_node PRIVATE ${INCLUDE_DIRS})
target_include_directories(odom_to_nuscenes_node PRIVATE ${INCLUDE_DIRS})

# 连接 IMU 节点需要的库
ament_target_dependencies(imu_to_nuscenes_node
  rclcpp
  sensor_msgs
  nlohmann_json
)

# 连接 Lidar 节点需要的库
ament_target_dependencies(lidar_to_nuscenes_node
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
  nlohmann_json
)

# 连接 Odom 节点需要的库
ament_target_dependencies(odom_to_nuscenes_node
  rclcpp
  sensor_msgs
  nlohmann_json
  nav_msgs  
)

# 安装目标
install(TARGETS
  imu_to_nuscenes_node
  lidar_to_nuscenes_node
  odom_to_nuscenes_node
  DESTINATION lib/${PROJECT_NAME}
)

# 包的安装规则
ament_package()
