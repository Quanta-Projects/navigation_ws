cmake_minimum_required(VERSION 3.8)
project(smrr_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(smrr_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(
  include
)

# Build BT nodes plugin library
add_library(smrr_bt_nodes SHARED
  src/bt_nodes/is_same_floor_condition.cpp
  src/bt_nodes/is_different_floor_condition.cpp
  src/bt_nodes/get_named_pose_action.cpp
  src/bt_nodes/bt_node_registration.cpp
)

ament_target_dependencies(smrr_bt_nodes
  rclcpp
  behaviortree_cpp_v3
  geometry_msgs
  tf2
  tf2_geometry_msgs
  ament_index_cpp
)

target_link_libraries(smrr_bt_nodes
  yaml-cpp
)

# Build mission executor executable
add_executable(smrr_bt_mission_executor
  src/smrr_bt_mission_executor.cpp
)

ament_target_dependencies(smrr_bt_mission_executor
  rclcpp
  nav2_behavior_tree
  nav2_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  smrr_interfaces
  ament_index_cpp
)

# Install C++ targets
install(TARGETS
  smrr_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  smrr_bt_mission_executor
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  smrr_navigation/startup_localizer.py
  smrr_navigation/named_goal_server.py
  smrr_navigation/named_goal_client.py
  smrr_navigation/location_subscriber.py
  smrr_navigation/smrr_multifloor_bt_navigator.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, config, maps, BT XMLs
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

# Install resource marker for package discovery
install(DIRECTORY
  resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
