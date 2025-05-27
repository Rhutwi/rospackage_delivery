cmake_minimum_required(VERSION 3.8)
project(status_tracker_node)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Task.msg"
  "msg/DeliveryStatus.msg"
  DEPENDENCIES std_msgs
)

# Install Python node
ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
  status_tracker_node/status_tracker_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
