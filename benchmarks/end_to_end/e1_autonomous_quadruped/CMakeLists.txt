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
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(tracetools_benchmark REQUIRED)
find_package(champ REQUIRED)
find_package(champ_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(message_filters REQUIRED)
find_package(pointcloud_to_laserscan REQUIRED)

# This is critical to ensure both the top-level include and the "champ" subdir
# are on the include path so that #include <geometry/geometry.h> works:
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${champ_INCLUDE_DIRS}
  ${champ_INCLUDE_DIRS}/champ
)


########################################
# twist_input_component library
ament_auto_add_library(twist_input_component SHARED src/twist_input_component.cpp)
ament_target_dependencies(twist_input_component tracetools_benchmark rclcpp geometry_msgs)
target_compile_definitions(twist_input_component PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(twist_input_component "robotperf::control::TwistInputComponent")

########################################
# joint_trajectory_output_component library
ament_auto_add_library(joint_trajectory_output_component SHARED src/joint_trajectory_output_component.cpp)
ament_target_dependencies(joint_trajectory_output_component tracetools_benchmark rclcpp trajectory_msgs)
target_compile_definitions(joint_trajectory_output_component PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(joint_trajectory_output_component "robotperf::control::JointTrajectoryOutputComponent")

########################################
# joint_trajectory_input_component library
ament_auto_add_library(joint_trajectory_input_component SHARED src/joint_trajectory_input_component.cpp)
ament_target_dependencies(joint_trajectory_input_component
  tracetools_benchmark
  rclcpp
  trajectory_msgs
)
target_compile_definitions(joint_trajectory_input_component PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(joint_trajectory_input_component
  "robotperf::control::JointTrajectoryInputComponent"
)

########################################
# pointcloud_input_component library
ament_auto_add_library(pointcloud_input_component SHARED src/pointcloud_input_component.cpp)
ament_target_dependencies(pointcloud_input_component tracetools_benchmark rclcpp sensor_msgs)
target_compile_definitions(pointcloud_input_component PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(pointcloud_input_component "robotperf::perception::PointCloudInputComponent")

# PointCloud Input Component Executable
ament_auto_add_executable(pointcloud_input_component_node src/pointcloud_input_component_node.cpp)
ament_target_dependencies(pointcloud_input_component_node rclcpp sensor_msgs)
target_link_libraries(pointcloud_input_component_node pointcloud_input_component)


########################################
# laserscan_input_component library
ament_auto_add_library(laserscan_input_component SHARED src/laserscan_input_component.cpp)
ament_target_dependencies(laserscan_input_component tracetools_benchmark rclcpp sensor_msgs)
target_compile_definitions(laserscan_input_component PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(laserscan_input_component "robotperf::perception::LaserscanInputComponent")

# Laserscan Input Component Executable
ament_auto_add_executable(laserscan_input_component_node src/laserscan_input_component_node.cpp)
ament_target_dependencies(laserscan_input_component_node rclcpp sensor_msgs)
target_link_libraries(laserscan_input_component_node laserscan_input_component)

########################################
# laserscan_output_component library
ament_auto_add_library(laserscan_output_component SHARED src/laserscan_output_component.cpp)
ament_target_dependencies(laserscan_output_component tracetools_benchmark rclcpp sensor_msgs)
target_compile_definitions(laserscan_output_component PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(laserscan_output_component "robotperf::perception::LaserscanOutputComponent")

# Laserscan Output Component Executable
ament_auto_add_executable(laserscan_output_component_node src/laserscan_output_component_node.cpp)
ament_target_dependencies(laserscan_output_component_node rclcpp sensor_msgs)
target_link_libraries(laserscan_output_component_node laserscan_output_component)

########################################
# robotperf_quadruped_controller library
ament_auto_add_library(robotperf_quadruped_controller SHARED src/robotperf_quadruped_controller.cpp)
ament_target_dependencies(robotperf_quadruped_controller
  rclcpp
  tracetools_benchmark
  champ
  champ_msgs
  geometry_msgs
  urdf
  tf2
)
target_compile_definitions(robotperf_quadruped_controller PRIVATE "COMPOSITION_BUILDING_DLL")

########################################
# robotperf_quadruped_controller_node executable
ament_auto_add_executable(robotperf_quadruped_controller_node src/robotperf_quadruped_controller_node.cpp)
ament_target_dependencies(robotperf_quadruped_controller_node rclcpp)
target_link_libraries(robotperf_quadruped_controller_node robotperf_quadruped_controller)

########################################
# pointcloud_to_laserscan_component library
ament_auto_add_library(pointcloud_to_laserscan_component SHARED src/pointcloud_to_laserscan.cpp)
ament_target_dependencies(pointcloud_to_laserscan_component 
  tracetools_benchmark
  rclcpp
  sensor_msgs
  tf2_ros
  tf2
  message_filters
  tf2_sensor_msgs
)
target_compile_definitions(pointcloud_to_laserscan_component PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(pointcloud_to_laserscan_component
  "pointcloud_to_laserscan::PointCloudToLaserScanNode"
)
target_include_directories(pointcloud_to_laserscan_component PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# PointCloud to LaserScan Node Executable
ament_auto_add_executable(pointcloud_to_laserscan_node src/pointcloud_to_laserscan_node.cpp)
ament_target_dependencies(pointcloud_to_laserscan_node 
  rclcpp 
  sensor_msgs 
  tf2_ros 
  tf2 
  message_filters 
  tf2_sensor_msgs
)
target_link_libraries(pointcloud_to_laserscan_node pointcloud_to_laserscan_component)



########################################
# MessagePublisher Component
ament_auto_add_library(message_publisher_component SHARED src/message_publisher.cpp)  
ament_target_dependencies(message_publisher_component rclcpp tracetools_benchmark sensor_msgs nav_msgs geometry_msgs)  
target_compile_definitions(message_publisher_component PRIVATE "COMPOSITION_BUILDING_DLL") 
rclcpp_components_register_nodes(message_publisher_component "robotperf::benchmark::MessagePublisher")

# MessagePublisher Executable
ament_auto_add_executable(message_publisher_node src/message_publisher_node.cpp)  
ament_target_dependencies(message_publisher_node rclcpp)  
target_link_libraries(message_publisher_node message_publisher_component)  

########################################
# MessageSubscriber Component
ament_auto_add_library(message_subscriber_component SHARED src/message_subscriber.cpp)  
ament_target_dependencies(message_subscriber_component rclcpp tracetools_benchmark sensor_msgs nav_msgs geometry_msgs)  
target_compile_definitions(message_subscriber_component PRIVATE "COMPOSITION_BUILDING_DLL")  
rclcpp_components_register_nodes(message_subscriber_component "robotperf::benchmark::MessageSubscriber")  

# MessageSubscriber Executable
ament_auto_add_executable(message_subscriber_node src/message_subscriber_node.cpp) 
ament_target_dependencies(message_subscriber_node rclcpp)
target_link_libraries(message_subscriber_node message_subscriber_component)


########################################
# Install targets
install(
  TARGETS
    twist_input_component
    joint_trajectory_output_component
    joint_trajectory_input_component
    pointcloud_input_component
    laserscan_input_component
    laserscan_output_component
    robotperf_quadruped_controller
    robotperf_quadruped_controller_node
    pointcloud_to_laserscan_component
    pointcloud_input_component_node
    laserscan_input_component_node
    laserscan_output_component_node
    pointcloud_to_laserscan_node
    message_publisher_component 
    message_publisher_node
    message_subscriber_component
    message_subscriber_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)

ament_auto_package(INSTALL_TO_SHARE launch)