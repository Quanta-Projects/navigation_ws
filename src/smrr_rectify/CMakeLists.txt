cmake_minimum_required(VERSION 3.8)
project(smrr_rectify)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(rectify_node src/rectify_node.cpp)
ament_target_dependencies(rectify_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_geometry
  message_filters
  OpenCV
)

install(TARGETS rectify_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
