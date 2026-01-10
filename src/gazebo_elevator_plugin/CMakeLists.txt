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
project(gazebo_elevator_plugin)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo REQUIRED)

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

# Build custom elevator plugin
add_library(CustomElevatorPlugin SHARED 
  model/moving_joint_model/plugin/CustomElevatorPlugin.cc
)
target_link_libraries(CustomElevatorPlugin ${GAZEBO_LIBRARIES})
target_include_directories(CustomElevatorPlugin PRIVATE 
  model/moving_joint_model/plugin
)

# Install the plugin library
install(TARGETS CustomElevatorPlugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
