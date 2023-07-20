# Minimum CMake version to build this project
cmake_minimum_required(VERSION 3.5)
# The name of our package
project(a6_imu_transformer)

# If the C++ standard hasn't been set, default it to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# If we're using GCC or Clang, add compile flags for warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

# Find the required packages for this project. These will be used in the build process.
find_package(ament_cmake_auto REQUIRED)
# Tells CMake to automatically find all the build dependencies specified in the package.xml 
# file of the project. This means you don't have to manually call find_package() 
# for each dependency your project 
ament_auto_find_build_dependencies()


# find_package(tracetools_benchmark REQUIRED)

# find_package(imu_transformer REQUIRED)

# Include the headers from the 'include' directory in the project
include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

# LTTng flame chart and graphs
# see https://archive.eclipse.org/tracecompass/doc/stable/org.eclipse.tracecompass.doc.user/LTTng-UST-Analyses.html#Flame_Chart_View
set(CMAKE_CXX_FLAGS "-g -O2 -finstrument-functions")

# Specify the dependencies for the "humble_node" target. The specified packages
# need to be located by find_package() before this and linked to the target
# ament_target_dependencies(imu_transformer)

# Install the project's target binary into the 'lib/${PROJECT_NAME}' directory
install(
  TARGETS ${install_targets}
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE launch)
