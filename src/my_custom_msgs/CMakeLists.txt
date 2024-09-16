cmake_minimum_required(VERSION 3.5)
project(my_custom_msgs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Declare the message files to be generated
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ConeArray.msg"  # Adjust this to your actual message file name
  DEPENDENCIES std_msgs
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_package()

