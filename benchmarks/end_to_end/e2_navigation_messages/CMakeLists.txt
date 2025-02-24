cmake_minimum_required(VERSION 3.5)
project(e2_navigation_messages)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

# Find remaining dependencies
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
find_package(e3_custom_messages REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${champ_INCLUDE_DIRS}
  ${champ_INCLUDE_DIRS}/champ
)

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