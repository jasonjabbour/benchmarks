cmake_minimum_required(VERSION 3.5)
project(e1_autonomous_quadruped)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(champ REQUIRED)
find_package(champ_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(message_filters REQUIRED)
find_package(tracetools_benchmark REQUIRED)
find_package(pointcloud_to_laserscan REQUIRED)

# Define dependencies list
set(dependencies
  rclcpp
  rclpy
  geometry_msgs
  champ
  champ_msgs
  sensor_msgs
  trajectory_msgs
  nav_msgs
  tf2_ros
  urdf
  message_filters
  tracetools_benchmark
  pointcloud_to_laserscan
)


include_directories(
  include
  ${champ_INCLUDE_DIRS}
  ${champ_INCLUDE_DIRS}/champ/
  ${champ_INCLUDE_DIRS}/champ/champ/
)

# LTTng flame chart and graphs
set(CMAKE_CXX_FLAGS "-g -O2 -finstrument-functions")

# Add quadruped controller library
add_library(robotperf_quadruped_controller SHARED
  src/robotperf_quadruped_controller.cpp
)

ament_target_dependencies(robotperf_quadruped_controller ${dependencies})
target_include_directories(robotperf_quadruped_controller PUBLIC include)

# Add quadruped controller node
add_executable(robotperf_quadruped_controller_node
  src/robotperf_quadruped_controller_node.cpp
)

ament_target_dependencies(robotperf_quadruped_controller_node ${dependencies})
target_link_libraries(robotperf_quadruped_controller_node robotperf_quadruped_controller)

# Install targets
install(
  TARGETS 
    robotperf_quadruped_controller
    robotperf_quadruped_controller_node
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}  # FIXED INSTALL PATH
)

# Install launch files properly
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Export include directories
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

# Package
ament_package()
