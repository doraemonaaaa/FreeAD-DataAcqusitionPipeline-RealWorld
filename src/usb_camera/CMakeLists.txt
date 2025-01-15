cmake_minimum_required(VERSION 3.5)

# 项目名称必须与 package.xml 中的包名一致
project(usb_camera)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CURL REQUIRED)

# Explicitly find GDAL
find_package(GDAL REQUIRED)
find_library(GDAL_LIBRARY NAMES gdal HINTS /usr/lib /usr/lib/x86_64-linux-gnu)
if(NOT GDAL_LIBRARY)
  message(FATAL_ERROR "GDAL library not found")
endif()

# Find TIFF explicitly
find_package(TIFF REQUIRED)

# Ensure CURL is found
find_package(CURL REQUIRED)
find_library(CURL_LIBRARY NAMES curl)
if(NOT CURL_LIBRARY)
  message(FATAL_ERROR "CURL library not found")
endif()

# Add executable
add_executable(${PROJECT_NAME}_node src/usb_camera_node.cpp)

# Link libraries
target_link_libraries(${PROJECT_NAME}_node
  ${CURL_LIBRARY}
  ${GDAL_LIBRARY}
  TIFF::TIFF
)

# Link ROS 2 dependencies
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

# Install target
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

# ament macros
ament_package()
