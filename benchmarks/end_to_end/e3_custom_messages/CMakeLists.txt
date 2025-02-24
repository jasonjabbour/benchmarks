cmake_minimum_required(VERSION 3.5)
project(e3_custom_messages)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
    "msg/CustomImu.msg"
    "msg/CustomLaserScan.msg"
    "msg/CustomPoint.msg"
    "msg/CustomPointCloud2.msg"
    "msg/CustomPointField.msg"
    "msg/CustomPose.msg"
    "msg/CustomPoseWithCovariance.msg"
    "msg/CustomPoseWithCovarianceStamped.msg"
    "msg/CustomQuaternion.msg"
    "msg/CustomVector3.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
