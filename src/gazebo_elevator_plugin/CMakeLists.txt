# cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

# find_package(gazebo REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rcl_interfaces REQUIRED)

# include_directories(${GAZEBO_INCLUDE_DIRS})

# install(
#   DIRECTORY launch 
#   DESTINATION share/${PROJECT_NAME}
# )

# add_library(joint_mover_plugin SHARED model/moving_joint_model/plugin/joint_mover_plugin.cpp)
# target_link_libraries(joint_mover_plugin ${GAZEBO_LIBRARIES})


cmake_minimum_required(VERSION 3.5)
project(gazebo_plugins)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})

# Install Python scripts
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install other files such as models, if any
install(
  DIRECTORY model
  DESTINATION share/${PROJECT_NAME}/
)

add_library(multi_floor_door_control SHARED model/moving_joint_model/plugin/multi_floor_door_control.cpp)
target_link_libraries(multi_floor_door_control ${GAZEBO_LIBRARIES})

ament_package()
