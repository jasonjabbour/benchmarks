cmake_minimum_required(VERSION 3.5)
project(e4_latentros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(tracetools_benchmark REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Helper macro to reduce boilerplate
macro(add_component LIB_NAME SRC_FILE PLUGIN_NAME DEPS)
  ament_auto_add_library(${LIB_NAME} SHARED src/${SRC_FILE})
  ament_target_dependencies(${LIB_NAME} tracetools_benchmark rclcpp ${DEPS})
  target_compile_definitions(${LIB_NAME} PRIVATE "COMPOSITION_BUILDING_DLL")
  rclcpp_components_register_nodes(${LIB_NAME} "${PLUGIN_NAME}")
endmacro()

add_component(odometry_input_component     odometry_input_component.cpp
  "robotperf::localization::OdometryInputComponent"     "nav_msgs")
add_component(odometry_output_component    odometry_output_component.cpp
  "robotperf::localization::OdometryOutputComponent"    "nav_msgs")
add_component(pose_cov_input_component     pose_with_covariance_input_component.cpp
  "robotperf::localization::PoseWithCovarianceInputComponent"  "geometry_msgs")
add_component(pose_cov_output_component    pose_with_covariance_output_component.cpp
  "robotperf::localization::PoseWithCovarianceOutputComponent" "geometry_msgs")
add_component(occupancy_grid_output_component occupancy_grid_output_component.cpp
  "robotperf::mapping::OccupancyGridOutputComponent"    "nav_msgs")
add_component(joint_state_input_component  joint_state_input_component.cpp
  "robotperf::localization::JointStateInputComponent"   "sensor_msgs")
add_component(joint_state_output_component joint_state_output_component.cpp
  "robotperf::control::JointStateOutputComponent"       "sensor_msgs")
add_component(path_input_component         path_input_component.cpp
  "robotperf::planning::PathInputComponent"             "nav_msgs")
add_component(path_output_component        path_output_component.cpp
  "robotperf::planning::PathOutputComponent"            "nav_msgs")
add_component(twist_output_component       twist_output_component.cpp
  "robotperf::planning::TwistOutputComponent"           "geometry_msgs")
add_component(pose_stamped_input_component pose_stamped_input_component.cpp
  "robotperf::planning::PoseStampedInputComponent"      "geometry_msgs")
add_component(pointcloud_input_padded_component pointcloud_input_padded_component.cpp
  "robotperf::perception::PointCloudInputPaddedComponent" "sensor_msgs")
add_component(e4_image_input_component image_input_component.cpp
  "robotperf::perception::E4ImageInputComponent"           "sensor_msgs")
add_component(e4_image_output_component image_output_component.cpp
  "robotperf::perception::E4ImageOutputComponent"          "sensor_msgs")
add_component(e4_image_input_nokey_component image_input_nokey_component.cpp
  "robotperf::perception::E4ImageInputNoKeyComponent"      "sensor_msgs")
add_component(e4_laserscan_output_component laserscan_output_component.cpp
  "robotperf::perception::E4LaserscanOutputComponent"      "sensor_msgs")
add_component(pointcloud_output_component pointcloud_output_component.cpp
  "robotperf::perception::PointCloudOutputComponent"      "sensor_msgs")

find_package(stereo_msgs REQUIRED)
add_component(disparity_output_component disparity_output_component.cpp
  "robotperf::perception::DisparityOutputComponent"       "stereo_msgs")

# NavFn benchmark node — standalone A* planner (no lifecycle/action)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_navfn_planner REQUIRED)
add_executable(navfn_benchmark_node src/navfn_benchmark_node.cpp)
ament_target_dependencies(navfn_benchmark_node
  rclcpp nav_msgs geometry_msgs
  nav2_costmap_2d nav2_navfn_planner
  tracetools_benchmark)
install(TARGETS navfn_benchmark_node DESTINATION lib/${PROJECT_NAME})

# RRT-Connect benchmark node — OMPL sampling-based planner
find_package(ompl REQUIRED)
add_executable(rrt_benchmark_node src/rrt_benchmark_node.cpp)
target_include_directories(rrt_benchmark_node PRIVATE ${OMPL_INCLUDE_DIRS})
target_link_libraries(rrt_benchmark_node ${OMPL_LIBRARIES})
ament_target_dependencies(rrt_benchmark_node
  rclcpp nav_msgs geometry_msgs
  tracetools_benchmark)
install(TARGETS rrt_benchmark_node DESTINATION lib/${PROJECT_NAME})

# Theta* benchmark node — standalone Theta* planner
find_package(nav2_theta_star_planner REQUIRED)
add_executable(theta_star_benchmark_node src/theta_star_benchmark_node.cpp)
ament_target_dependencies(theta_star_benchmark_node
  rclcpp nav_msgs geometry_msgs
  nav2_costmap_2d nav2_theta_star_planner
  tracetools_benchmark)
install(TARGETS theta_star_benchmark_node DESTINATION lib/${PROJECT_NAME})

# Savitzky-Golay path smoother benchmark node (standalone, no Nav2 dependency)
add_executable(smoother_benchmark_node src/smoother_benchmark_node.cpp)
ament_target_dependencies(smoother_benchmark_node
  rclcpp nav_msgs
  tracetools_benchmark)
install(TARGETS smoother_benchmark_node DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS
    odometry_input_component
    odometry_output_component
    pose_cov_input_component
    pose_cov_output_component
    occupancy_grid_output_component
    joint_state_input_component
    joint_state_output_component
    path_input_component
    path_output_component
    twist_output_component
    pose_stamped_input_component
    pointcloud_input_padded_component
    pointcloud_output_component
    e4_image_input_component
    e4_image_output_component
    e4_image_input_nokey_component
    e4_laserscan_output_component
    disparity_output_component
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.hpp")

# Install Python benchmark scripts
install(
  PROGRAMS
    scripts/planner_benchmark_client.py
    scripts/controller_benchmark_client.py
    scripts/goal_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE launch config)
