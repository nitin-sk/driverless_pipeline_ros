cmake_minimum_required(VERSION 3.5)
project(my_detection_package)

# Find ament and other dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pyrealsense2 REQUIRED)
find_package(ultralytics REQUIRED)
find_package(perception_msgs REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/camera_node.py
  scripts/display_node.py
  scripts/slam_node.py
  scripts/yolo_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export the package
ament_package()

