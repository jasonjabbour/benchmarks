/*
   @@@@@@@@@@@@@@@@@@@@
   @@@@@@@@@&@@@&&@@@@@
   @@@@@ @@  @@    @@@@
   @@@@@ @@  @@    @@@@
   @@@@@ @@  @@    @@@@ Copyright (c) 2023, Acceleration Robotics®
   @@@@@ @@  @@    @@@@ Author: Víctor Mayoral Vilches <victor@accelerationrobotics.com>
   @@@@@ @@  @@    @@@@ Author: Alejandra Martínez Fariña <alex@accelerationrobotics.com>
   @@@@@@@@@&@@@@@@@@@@
   @@@@@@@@@@@@@@@@@@@@

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License. 
*/

/** \mainpage tracetools_benchmark: tracing tools and instrumentation for RobotPerf benchmarks
 *
 * `tracetools_benchmark` provides utilities to instrument ROS 2 selected packages
 *  that demonstrate hardware acceleration.
 *
 * It provides two main headers:
 *
 * - tracetools/tracetools.h
 *   - instrumentation functions
 * - tracetools/utils.hpp
 *   - utility functions
 */

#ifndef TRACETOOLS_BENCHMARK__TRACETOOLS_H_
#define TRACETOOLS_BENCHMARK__TRACETOOLS_H_

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "tracetools_benchmark/config.h"
#include "tracetools_benchmark/visibility_control.hpp"

#ifndef TRACETOOLS_DISABLED
/// Call a tracepoint.
/**
 * This is the preferred method over calling the actual function directly.
 */
#  define TRACEPOINT(event_name, ...) \
  (ros_trace_ ## event_name)(__VA_ARGS__)
#  define DECLARE_TRACEPOINT(event_name, ...) \
  TRACETOOLS_PUBLIC void ros_trace_ ## event_name(__VA_ARGS__);
#else
#  define TRACEPOINT(event_name, ...) ((void) (0))
#  define DECLARE_TRACEPOINT(event_name, ...)
#endif

#ifdef __cplusplus
extern "C"
{
#endif

/// Get tracing compilation status.
/**
 * \return `true` if tracing is enabled, `false` otherwise
 */
TRACETOOLS_PUBLIC bool ros_trace_compile_status();

/// `robotperf_image_input_cb_init`
/**
 * Tracepoint while initiating the callback of robotperf::perception::ImageInputComponent component
 *
 * Notes the `tracetools_benchmark` version automatically.
 *
 * \param[in] image_input_node rclcpp::node::Node subject to the callback
 * \param[in] image_input_image_msg image ROS message stored as sensor_msgs::msg::Image::ConstSharedPtr
 * \param[in] image_input_info_msg info ROS message as sensor_msgs::msg::CameraInfo::ConstSharedPtr
 * \param[in] image_input_header_nsec_arg nanosec field of the header (std_msgs/Header) of sensor_msgs::msg::Image's ROS message 
 * \param[in] image_input_header_sec_arg sec field of the header (std_msgs/Header) of sensor_msgs::msg::Image's ROS message
 * \param[in] image_input_image_msg_size size of image ROS message stored as bytes
 * \param[in] image_input_info_msg_size size of info ROS message as bytes
 */
DECLARE_TRACEPOINT(
  robotperf_image_input_cb_init,
  const void * image_input_node,
  const void * image_input_image_msg,
  const void * image_input_info_msg,
  uint32_t image_input_header_nsec_arg,
  uint32_t image_input_header_sec_arg,
  size_t image_input_image_msg_size,
  size_t image_input_info_msg_size)

/// `robotperf_image_input_cb_fini`
/**
 * Tracepoint while finishing the callback of robotperf::perception::ImageOutputComponent component
 *
 * Notes the `tracetools_benchmark` version automatically.
 *
 * \param[in] image_input_node rclcpp::node::Node subject to the callback
 * \param[in] image_input_image_msg image ROS message stored as sensor_msgs::msg::Image::ConstSharedPtr
 * \param[in] image_input_info_msg info ROS message as sensor_msgs::msg::CameraInfo::ConstSharedPtr
 * \param[in] image_input_header_nsec_arg nanosec field of the header (std_msgs/Header) of sensor_msgs::msg::Image's ROS message 
 * \param[in] image_input_header_sec_arg sec field of the header (std_msgs/Header) of sensor_msgs::msg::Image's ROS message 
 * \param[in] image_input_image_msg_size size of image ROS message stored as bytes
 * \param[in] image_input_info_msg_size size of info ROS message as bytes
 */
DECLARE_TRACEPOINT(
  robotperf_image_input_cb_fini,
  const void * image_input_node,
  const void * image_input_image_msg,
  const void * image_input_info_msg,
  uint32_t image_input_header_nsec_arg,
  uint32_t image_input_header_sec_arg,
  size_t image_input_image_msg_size,
  size_t image_input_info_msg_size)

/// `robotperf_image_output_cb_init`
/**
 * Tracepoint while initiating the callback of robotperf::perception::ImageInputComponent component
 *
 * Notes the `tracetools_benchmark` version automatically.
 *
 * \param[in] image_output_node rclcpp::node::Node subject to the callback
 * \param[in] image_output_image_msg image ROS message stored as sensor_msgs::msg::Image::ConstSharedPtr
 * \param[in] image_output_info_msg info ROS message as sensor_msgs::msg::CameraInfo::ConstSharedPtr
 * \param[in] image_input_header_nsec_arg nanosec field of the header (std_msgs/Header) of sensor_msgs::msg::Image's ROS message 
 * \param[in] image_input_header_sec_arg sec field of the header (std_msgs/Header) of sensor_msgs::msg::Image's ROS message 
 * \param[in] image_output_image_msg_size size of image ROS message stored as bytes
 * \param[in] image_output_info_msg_size size of info ROS message as bytes
 */
DECLARE_TRACEPOINT(
  robotperf_image_output_cb_init,
  const void * image_output_node,
  const void * image_output_image_msg,
  const void * image_output_info_msg,
  uint32_t image_input_header_nsec_arg,
  uint32_t image_input_header_sec_arg,
  size_t image_output_image_msg_size,
  size_t image_output_info_msg_size)


/// `robotperf_image_output_cb_fini`
/**
 * Tracepoint while finishing the callback of robotperf::perception::ImageOutputComponent component
 *
 * Notes the `tracetools_benchmark` version automatically.
 *
 * \param[in] image_output_node rclcpp::node::Node subject to the callback
 * \param[in] image_output_image_msg image ROS message stored as sensor_msgs::msg::Image::ConstSharedPtr
 * \param[in] image_output_info_msg info ROS message as sensor_msgs::msg::CameraInfo::ConstSharedPtr
 * \param[in] image_input_header_nsec_arg nanosec field of the header (std_msgs/Header) of sensor_msgs::msg::Image's ROS message 
 * \param[in] image_input_header_sec_arg sec field of the header (std_msgs/Header) of sensor_msgs::msg::Image's ROS message 
 * \param[in] image_output_image_msg_size size of image ROS message stored as bytes
 * \param[in] image_output_info_msg_size size of info ROS message as bytes
 */
DECLARE_TRACEPOINT(
  robotperf_image_output_cb_fini,
  const void * image_output_node,
  const void * image_output_image_msg,
  const void * image_output_info_msg,
  uint32_t image_input_header_nsec_arg,
  uint32_t image_input_header_sec_arg,
  size_t image_output_image_msg_size,
  size_t image_output_info_msg_size)


/// `robotperf_pointcloud_input_cb_init`
/**
 * Trace point while initiating the callback of robotperf::perception::PointCloudInputComponent component
 * 
 * Notes the `tracetools_benchmark` version automatically.
 * 
 * \param[in] pointcloud_input_node rclcpp::node::Node subject to the callback
 * \param[in] pointcloud_input_pointcloud_msg pointcloud ROS message stored as sensor_msgs::msg::PointCloud2::ConstSharedPtr
 * \param[in] pointcloud_input_header_nsec_arg nanosec field of the header (std_msgs/Header) of sensor_msgs::msg::PointCloud2's ROS message
 * \param[in] pointcloud_input_header_sec_arg sec field of the header (std_msgs/Header) of sensor_msgs::msg::PointCloud2's ROS message
 * \param[in] pointcloud_input_msg_size size of the pointcloud ROS message stored as bytes
 * \param[in] key Unique identifier for tracking the message across components.
 */
DECLARE_TRACEPOINT(
  robotperf_pointcloud_input_cb_init, 
  const void * pointcloud_input_node, 
  const void * pointcloud_input_pointcloud_msg, 
  uint32_t pointcloud_input_header_nsec_arg, 
  uint32_t pointcloud_input_header_sec_arg, 
  size_t pointcloud_input_msg_size,
  uint32_t key)

/// `robotperf_pointcloud_input_cb_fini`
/**
 * Tracepoint while finishing the callback of robotperf::perception::PointCloudInputComponent component
 * 
 * Notes the `tracetools_benchmark` version automatically. 
 * 
 * \param[in] pointcloud_input_node rclcpp::node::Node subject to the callback
 * \param[in] pointcloud_input_pointcloud_msg pointcloud ROS message stored as sensor_msgs::msg::PointCloud2::ConstSharedPtr
 * \param[in] pointcloud_input_header_nsec_arg nanosec field of the header (std_msgs/Header) of sensor_msgs::msg::PointCloud2's ROS message
 * \param[in] pointcloud_input_header_sec_arg sec field of the header (std_msgs/Header) of sensor_msgs::msg::PointCloud2's ROS message
 * \param[in] pointcloud_input_msg_size size of the pointcloud ROS message stored as bytes
 * \param[in] key Unique identifier for tracking the message across components.
 */
DECLARE_TRACEPOINT(
  robotperf_pointcloud_input_cb_fini, 
  const void * pointcloud_input_node, 
  const void * pointcloud_input_pointcloud_msg, 
  uint32_t pointcloud_input_header_nsec_arg, 
  uint32_t pointcloud_input_header_sec_arg, 
  size_t pointcloud_input_msg_size,
  uint32_t key)

/// `robotperf_pointcloud_output_cb_init`
/**
 * Tracepoint while initiating the callback of robotperf::perception::PointCloudOutputComponent component
 *
 * Notes the `tracetools_benchmark` version automatically.
 *
 * \param[in] pointcloud_output_node rclcpp::node::Node subject to the callback
 * \param[in] pointcloud_output_pointcloud_msg pointcloud ROS message stored as sensor_msgs::msg::PointCloud2::ConstSharedPtr
 * \param[in] image_input_header_nsec_arg nanosec field of the header (std_msgs/Header) of sensor_msgs::msg::PointCloud2's ROS message 
 * \param[in] image_input_header_sec_arg sec field of the header (std_msgs/Header) of sensor_msgs::msg::PointCloud2's ROS message 
 * \param[in] pointcloud_output_msg_size size of pointcloud ROS message stored as bytes
 * \param[in] key Unique identifier for tracking the message across components.
 */
DECLARE_TRACEPOINT(
  robotperf_pointcloud_output_cb_init,
  const void * pointcloud_output_node,
  const void * pointcloud_output_pointcloud_msg,
  uint32_t image_input_header_nsec_arg,
  uint32_t image_input_header_sec_arg,
  size_t pointcloud_output_msg_size,
  uint32_t key)

/// `robotperf_pointcloud_output_cb_fini`
/**
 * Tracepoint while finishing the callback of robotperf::perception::PointCloudOutputComponent component
 *
 * Notes the `tracetools_benchmark` version automatically.
 *
 * \param[in] pointcloud_output_node rclcpp::node::Node subject to the callback
 * \param[in] pointcloud_output_pointcloud_msg pointcloud ROS message stored as sensor_msgs::msg::PointCloud2::ConstSharedPtr
 * \param[in] image_input_header_nsec_arg nanosec field of the header (std_msgs/Header) of sensor_msgs::msg::PointCloud2's ROS message 
 * \param[in] image_input_header_sec_arg sec field of the header (std_msgs/Header) of sensor_msgs::msg::PointCloud2's ROS message 
 * \param[in] pointcloud_output_msg_size size of pointcloud ROS message stored as bytes
 * \param[in] key Unique identifier for tracking the message across components.
 */
DECLARE_TRACEPOINT(
  robotperf_pointcloud_output_cb_fini,
  const void * pointcloud_output_node,
  const void * pointcloud_output_pointcloud_msg,
  uint32_t image_input_header_nsec_arg,
  uint32_t image_input_header_sec_arg,
  size_t pointcloud_output_msg_size,
  uint32_t key)

/// `robotperf_laserscan_input_cb_init`
/**
 * Tracepoint while initiating the callback of robotperf::perception::LaserscanInputComponent component
 *
 * Notes the `tracetools_benchmark` version automatically.
 *
 * \param[in] laserscan_input_node rclcpp::node::Node subject to the callback
 * \param[in] laserscan_input_scan_msg laserscan ROS message stored as sensor_msgs::msg::LaserScan::SharedPtr
 * \param[in] laserscan_input_header_nsec_arg nanosec field of the header (std_msgs/Header) of sensor_msgs::msg::LaserScan's ROS message 
 * \param[in] laserscan_input_header_sec_arg sec field of the header (std_msgs/Header) of sensor_msgs::msg::LaserScan's ROS message 
 * \param[in] laserscan_input_msg_size size of laserscan ROS message stored as bytes
 * \param[in] key Unique identifier for tracking the message across components.
 */
DECLARE_TRACEPOINT(
  robotperf_laserscan_input_cb_init,
  const void * laserscan_input_node,
  const void * laserscan_input_scan_msg,
  uint32_t laserscan_input_header_nsec_arg,
  uint32_t laserscan_input_header_sec_arg,
  size_t laserscan_input_msg_size,
  uint32_t key)

/// `robotperf_laserscan_input_cb_fini`
/**
 * Tracepoint while finishing the callback of robotperf::perception::LaserscanInputComponent component
 *
 * Notes the `tracetools_benchmark` version automatically.
 *
 * \param[in] laserscan_input_node rclcpp::node::Node subject to the callback
 * \param[in] laserscan_input_scan_msg laserscan ROS message stored as sensor_msgs::msg::LaserScan::SharedPtr
 * \param[in] laserscan_input_header_nsec_arg nanosec field of the header (std_msgs/Header) of sensor_msgs::msg::LaserScan's ROS message 
 * \param[in] laserscan_input_header_sec_arg sec field of the header (std_msgs/Header) of sensor_msgs::msg::LaserScan's ROS message 
 * \param[in] laserscan_input_msg_size size of laserscan ROS message stored as bytes
 * \param[in] key Unique identifier for tracking the message across components.
 */
DECLARE_TRACEPOINT(
  robotperf_laserscan_input_cb_fini,
  const void * laserscan_input_node,
  const void * laserscan_input_scan_msg,
  uint32_t laserscan_input_header_nsec_arg,
  uint32_t laserscan_input_header_sec_arg,
  size_t laserscan_input_msg_size,
  uint32_t key)

/// `robotperf_laserscan_output_cb_init`
/**
 * Tracepoint while initiating the callback of robotperf::perception::LaserscanOutputComponent component
 *
 * Notes the `tracetools_benchmark` version automatically.
 *
 * \param[in] laserscan_output_node rclcpp::node::Node subject to the callback
 * \param[in] laserscan_output_scan_msg laserscan ROS message stored as sensor_msgs::msg::LaserScan::SharedPtr
 * \param[in] laserscan_output_header_nsec_arg nanosec field of the header (std_msgs/Header) of sensor_msgs::msg::LaserScan's ROS message 
 * \param[in] laserscan_output_header_sec_arg sec field of the header (std_msgs/Header) of sensor_msgs::msg::LaserScan's ROS message 
 * \param[in] laserscan_output_msg_size size of laserscan ROS message stored as bytes
 * \param[in] key Unique identifier for tracking the message across components.
 */
DECLARE_TRACEPOINT(
  robotperf_laserscan_output_cb_init,
  const void * laserscan_output_node,
  const void * laserscan_output_scan_msg,
  uint32_t laserscan_output_header_nsec_arg,
  uint32_t laserscan_output_header_sec_arg,
  size_t laserscan_output_msg_size,
  uint32_t key)

/// `robotperf_laserscan_output_cb_fini`
/**
 * Tracepoint while finishing the callback of robotperf::perception::LaserscanOutputComponent component
 *
 * Notes the `tracetools_benchmark` version automatically.
 *
 * \param[in] laserscan_output_node rclcpp::node::Node subject to the callback
 * \param[in] laserscan_output_scan_msg laserscan ROS message stored as sensor_msgs::msg::LaserScan::SharedPtr
 * \param[in] laserscan_output_header_nsec_arg nanosec field of the header (std_msgs/Header) of sensor_msgs::msg::LaserScan's ROS message 
 * \param[in] laserscan_output_header_sec_arg sec field of the header (std_msgs/Header) of sensor_msgs::msg::LaserScan's ROS message 
 * \param[in] laserscan_output_msg_size size of laserscan ROS message stored as bytes
 * \param[in] key Unique identifier for tracking the message across components.
 */
DECLARE_TRACEPOINT(
  robotperf_laserscan_output_cb_fini,
  const void * laserscan_output_node,
  const void * laserscan_output_scan_msg,
  uint32_t laserscan_output_header_nsec_arg,
  uint32_t laserscan_output_header_sec_arg,
  size_t laserscan_output_msg_size,
  uint32_t key)


/// `robotperf_joint_trajectory_output_cb_init`
/**
 * Tracepoint while initiating the callback of robotperf::control::JointTrajectoryOutputComponent component
 *
 * Notes the `tracetools_benchmark` version automatically.
 *
 * \param[in] joint_trajectory_output_node rclcpp::node::Node subject to the callback
 * \param[in] joint_trajectory_output_msg joint trajectory ROS message stored as trajectory_msgs::msg::JointTrajectory::SharedPtr
 * \param[in] joint_trajectory_output_header_nsec_arg nanosec field of the header (std_msgs/Header) of trajectory_msgs::msg::JointTrajectory's ROS message 
 * \param[in] joint_trajectory_output_header_sec_arg sec field of the header (std_msgs/Header) of trajectory_msgs::msg::JointTrajectory's ROS message 
 * \param[in] joint_trajectory_output_msg_size size of joint trajectory ROS message stored as bytes
 */
DECLARE_TRACEPOINT(
  robotperf_joint_trajectory_output_cb_init,
  const void * joint_trajectory_output_node,
  const void * joint_trajectory_output_msg,
  uint32_t joint_trajectory_output_header_nsec_arg,
  uint32_t joint_trajectory_output_header_sec_arg,
  size_t joint_trajectory_output_msg_size)


/// `robotperf_joint_trajectory_output_cb_fini`
/**
 * Tracepoint while finishing the callback of robotperf::control::JointTrajectoryOutputComponent component
 *
 * Notes the `tracetools_benchmark` version automatically.
 *
 * \param[in] joint_trajectory_output_node rclcpp::node::Node subject to the callback
 * \param[in] joint_trajectory_output_msg joint trajectory ROS message stored as trajectory_msgs::msg::JointTrajectory::SharedPtr
 * \param[in] joint_trajectory_output_header_nsec_arg nanosec field of the header (std_msgs/Header) of trajectory_msgs::msg::JointTrajectory's ROS message 
 * \param[in] joint_trajectory_output_header_sec_arg sec field of the header (std_msgs/Header) of trajectory_msgs::msg::JointTrajectory's ROS message 
 * \param[in] joint_trajectory_output_msg_size size of joint trajectory ROS message stored as bytes
 */
DECLARE_TRACEPOINT(
  robotperf_joint_trajectory_output_cb_fini,
  const void * joint_trajectory_output_node,
  const void * joint_trajectory_output_msg,
  uint32_t joint_trajectory_output_header_nsec_arg,
  uint32_t joint_trajectory_output_header_sec_arg,
  size_t joint_trajectory_output_msg_size)

/// `robotperf_joint_trajectory_input_cb_init`
/**
 * Tracepoint while initiating the callback of
 * robotperf::control::JointTrajectoryInputComponent component.
 *
 * \param[in] joint_trajectory_input_node pointer to node
 * \param[in] joint_trajectory_input_msg pointer to the input JointTrajectory message
 * \param[in] joint_trajectory_input_header_nsec_arg header.stamp.nanosec
 * \param[in] joint_trajectory_input_header_sec_arg header.stamp.sec
 * \param[in] joint_trajectory_input_msg_size size in bytes
 */
DECLARE_TRACEPOINT(
  robotperf_joint_trajectory_input_cb_init,
  const void * joint_trajectory_input_node,
  const void * joint_trajectory_input_msg,
  uint32_t joint_trajectory_input_header_nsec_arg,
  uint32_t joint_trajectory_input_header_sec_arg,
  size_t joint_trajectory_input_msg_size
);

/// `robotperf_joint_trajectory_input_cb_fini`
/**
 * Tracepoint while finishing the callback of
 * robotperf::control::JointTrajectoryInputComponent component.
 */
DECLARE_TRACEPOINT(
  robotperf_joint_trajectory_input_cb_fini,
  const void * joint_trajectory_input_node,
  const void * joint_trajectory_input_msg,
  uint32_t joint_trajectory_input_header_nsec_arg,
  uint32_t joint_trajectory_input_header_sec_arg,
  size_t joint_trajectory_input_msg_size
);

/// `robotperf_twist_input_cb_init`
/**
 * Tracepoint while initiating the callback of robotperf::control::TwistInputComponent component.
 *
 * Notes the `tracetools_benchmark` version automatically.
 *
 * \param[in] twist_input_node rclcpp::node::Node subject to the callback.
 * \param[in] twist_input_msg twist ROS message stored as geometry_msgs::msg::TwistStamped::SharedPtr.
 * \param[in] twist_input_msg_size size of twist ROS message stored as bytes.
 */
DECLARE_TRACEPOINT(
  robotperf_twist_input_cb_init,
  const void * twist_input_node,
  const void * twist_input_msg,
  size_t twist_input_msg_size)

/// `robotperf_twist_input_cb_fini`
/**
 * Tracepoint while finishing the callback of robotperf::control::TwistInputComponent component.
 *
 * Notes the `tracetools_benchmark` version automatically.
 *
 * \param[in] twist_input_node rclcpp::node::Node subject to the callback.
 * \param[in] twist_input_msg twist ROS message stored as geometry_msgs::msg::TwistStamped::SharedPtr.
 * \param[in] twist_input_msg_size size of twist ROS message stored as bytes.
 */
DECLARE_TRACEPOINT(
  robotperf_twist_input_cb_fini,
  const void * twist_input_node,
  const void * twist_input_msg,
  size_t twist_input_msg_size)


/// `robotperf_msg_published`
/**
 * Tracepoint when a ROS2 message is published.
 *
 * \param[in] node Pointer to the ROS2 node publishing the message.
 * \param[in] msg Pointer to the ROS2 message being published.
 * \param[in] key Unique identifier for tracking the message across components.
 */
DECLARE_TRACEPOINT(
    robotperf_msg_published_1,
    const void * node,
    const void * msg,
    uint32_t key);

DECLARE_TRACEPOINT(
    robotperf_msg_published_2,
    const void * node,
    const void * msg,
    uint32_t key);

/// `robotperf_msg_received`
/**
 * Tracepoint when a ROS2 message is received.
 *
 * \param[in] node Pointer to the ROS2 node receiving the message.
 * \param[in] msg Pointer to the received ROS2 message.
 * \param[in] key Unique identifier for tracking the message across components.
 */
DECLARE_TRACEPOINT(
    robotperf_msg_received_1,
    const void * node,
    const void * msg,
    uint32_t key);

DECLARE_TRACEPOINT(
    robotperf_msg_received_2,
    const void * node,
    const void * msg,
    uint32_t key);


DECLARE_TRACEPOINT(
    robotperf_msg_published_size_1,
    const void * node,
    const void * msg,
    uint32_t key, 
    size_t size);

DECLARE_TRACEPOINT(
    robotperf_msg_received_size_1,
    const void * node,
    const void * msg,
    uint32_t key, 
    size_t size);



#ifdef __cplusplus
}
#endif

#endif  // TRACETOOLS_BENCHMARK__TRACETOOLS_H_
