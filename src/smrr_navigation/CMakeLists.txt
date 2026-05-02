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
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(smrr_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)

# ONNX Runtime - try find_package, fallback to manual linking
# Option 1: sudo apt install libonnxruntime-dev (if available)
# Option 2: Download from https://github.com/microsoft/onnxruntime/releases
set(ONNXRUNTIME_ROOT "$ENV{HOME}/libs/onnxruntime-linux-x64-1.18.1" CACHE PATH "ONNX Runtime SDK root")

find_package(onnxruntime QUIET)
if(NOT onnxruntime_FOUND)
  message(STATUS "onnxruntime not found via find_package, using manual configuration")
  
  # Search paths including downloaded SDK
  find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
    PATHS 
      ${ONNXRUNTIME_ROOT}/include
      /usr/include
      /usr/local/include
    NO_DEFAULT_PATH
  )
  find_library(ONNXRUNTIME_LIBRARY onnxruntime
    PATHS 
      ${ONNXRUNTIME_ROOT}/lib
      /usr/lib
      /usr/local/lib
      /usr/lib/x86_64-linux-gnu
    NO_DEFAULT_PATH
  )
  
  if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIBRARY)
    message(STATUS "Found ONNX Runtime library: ${ONNXRUNTIME_LIBRARY}")
    message(STATUS "ONNX Runtime includes: ${ONNXRUNTIME_INCLUDE_DIR}")
  else()
    message(FATAL_ERROR "ONNX Runtime not found. Download SDK from: https://github.com/microsoft/onnxruntime/releases\nExtract to: ${ONNXRUNTIME_ROOT}")
  endif()
else()
  message(STATUS "Found ONNX Runtime via find_package")
endif()

# Include directories
include_directories(
  include
)

# Build BT nodes plugin library
add_library(smrr_bt_nodes SHARED
  src/bt_nodes/is_same_floor_condition.cpp
  src/bt_nodes/is_different_floor_condition.cpp
  src/bt_nodes/get_named_pose_action.cpp
  src/bt_nodes/get_named_map_action.cpp
  src/bt_nodes/switch_map_action.cpp
  src/bt_nodes/publish_initial_pose_action.cpp
  src/bt_nodes/call_elevator_action.cpp
  src/bt_nodes/wait_for_door_open_action.cpp
  src/bt_nodes/wait_for_door_open_depth_action.cpp
  src/bt_nodes/wait_for_door_open_model_action.cpp
  src/bt_nodes/update_pose_timestamp_action.cpp
  src/bt_nodes/stop_robot_action.cpp
  src/bt_nodes/set_controller_params_action.cpp
  src/bt_nodes/set_amcl_params_action.cpp
  src/bt_nodes/set_goal_checker_params_action.cpp
  src/bt_nodes/set_costmap_inflation_action.cpp
  src/bt_nodes/check_floor_arrival_action.cpp
  src/bt_nodes/toggle_apriltag_action.cpp
  src/bt_nodes/check_elevator_direction_action.cpp
  src/bt_nodes/build_pose_vector_action.cpp
  src/bt_nodes/detect_call_button_action.cpp
  src/bt_nodes/press_floor_button_action.cpp
  src/bt_nodes/publish_current_floor_action.cpp
  src/bt_nodes/publish_bool_topic_action.cpp
  src/bt_nodes/bt_node_registration.cpp
)

ament_target_dependencies(smrr_bt_nodes
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  nav2_behavior_tree
  smrr_interfaces
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
  nav2_msgs
  rcl_interfaces
  std_srvs
)

# Add ONNX Runtime include directories
if(ONNXRUNTIME_INCLUDE_DIR)
  target_include_directories(smrr_bt_nodes PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
endif()

# Link libraries
if(onnxruntime_FOUND)
  target_link_libraries(smrr_bt_nodes yaml-cpp onnxruntime::onnxruntime)
else()
  target_link_libraries(smrr_bt_nodes yaml-cpp ${ONNXRUNTIME_LIBRARY})
endif()

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
  smrr_navigation/door_classifier_node.py
  smrr_navigation/test_floor_vision.py
  smrr_navigation/floor_arrival_server.py
  smrr_navigation/apriltag_manager_server.py
  smrr_navigation/elevator_direction_server.py
  smrr_navigation/elevator_call_button_server.py
  smrr_navigation/elevator_floor_button_server.py
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
  behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

install(DIRECTORY
  maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

# Install ONNX models for door classifier
install(DIRECTORY
  models/
  DESTINATION share/${PROJECT_NAME}/models
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
