cmake_minimum_required(VERSION 3.5)
project(object_collection_py)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ros2_control REQUIRED)
find_package(gazebo_ros REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

# Add your plugin
add_library(CameraPlugin SHARED src/CameraControllerPlugin.cc)
ament_target_dependencies(CameraPlugin
  rclpy
  sensor_msgs
  gazebo_ros
)

target_link_libraries(CameraPlugin
  ${OpenCV_LIBRARIES}
  ${cv_bridge_LIBRARIES}
)


# Install the plugin library
install(TARGETS
  CameraPlugin
  DESTINATION lib/${PROJECT_NAME}
)

# Install the plugin XML file
install(FILES
  gazebo_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install Python nodes
install(PROGRAMS
  src/object_collection_py/object_collection_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/config
)

# Install include files
install(DIRECTORY
  include/object_collection_py
  DESTINATION include/${PROJECT_NAME}
)

# Install package.xml file
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
