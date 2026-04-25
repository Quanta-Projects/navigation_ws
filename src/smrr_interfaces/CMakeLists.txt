cmake_minimum_required(VERSION 3.5)
project(smrr_interfaces)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate services and actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToNamedPose.srv"
  "srv/StartMission.srv"
  "action/NavigateToNamedLocation.action"
  "action/CheckFloorArrival.action"
  "action/CheckElevatorDirection.action"
  "action/DetectCallButton.action"
  "action/PressFloorButton.action"
  DEPENDENCIES builtin_interfaces std_msgs action_msgs geometry_msgs
)

ament_package()
