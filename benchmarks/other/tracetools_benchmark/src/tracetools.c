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

#include "tracetools_benchmark/tracetools.h"

#ifndef TRACETOOLS_DISABLED

#ifdef TRACETOOLS_LTTNG_ENABLED
# include "tracetools_benchmark/tp_call.h"
# define CONDITIONAL_TP(...) \
  tracepoint(TRACEPOINT_PROVIDER, __VA_ARGS__)
#else
# define CONDITIONAL_TP(...)
#endif

bool ros_trace_compile_status()
{
#ifdef TRACETOOLS_LTTNG_ENABLED
  return true;
#else
  return false;
#endif
}

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
#else
# pragma warning(push)
# pragma warning(disable: 4100)
#endif

// image_input
void TRACEPOINT(
  robotperf_image_input_cb_init,
  const void * image_input_node_arg,
  const void * image_input_image_msg_arg,
  const void * image_input_info_msg_arg,
  uint32_t image_input_header_nsec_arg,
  uint32_t image_input_header_sec_arg,
  size_t image_input_image_msg_size_arg,
  size_t image_input_info_msg_size_arg)
{
  CONDITIONAL_TP(
    robotperf_image_input_cb_init,
    image_input_node_arg,
    image_input_image_msg_arg,
    image_input_info_msg_arg,
    image_input_header_nsec_arg,
    image_input_header_sec_arg,
    image_input_image_msg_size_arg,
    image_input_info_msg_size_arg);
}
void TRACEPOINT(
  robotperf_image_input_cb_fini,
  const void * image_input_node_arg,
  const void * image_input_image_msg_arg,
  const void * image_input_info_msg_arg,
  uint32_t image_input_header_nsec_arg,
  uint32_t image_input_header_sec_arg,
  size_t image_input_image_msg_size_arg,
  size_t image_input_info_msg_size_arg)
{
  CONDITIONAL_TP(
    robotperf_image_input_cb_fini,
    image_input_node_arg,
    image_input_image_msg_arg,
    image_input_info_msg_arg,
    image_input_header_nsec_arg,
    image_input_header_sec_arg,
    image_input_image_msg_size_arg,
    image_input_info_msg_size_arg);
}

// image_output
void TRACEPOINT(
  robotperf_image_output_cb_init,
  const void * image_output_node_arg,
  const void * image_output_image_msg_arg,
  const void * image_output_info_msg_arg,
  uint32_t image_input_header_nsec_arg,
  uint32_t image_input_header_sec_arg,
  size_t image_output_image_msg_size_arg,
  size_t image_output_info_msg_size_arg)
{
  CONDITIONAL_TP(
    robotperf_image_output_cb_init,
    image_output_node_arg,
    image_output_image_msg_arg,
    image_output_info_msg_arg,
    image_input_header_nsec_arg,
    image_input_header_sec_arg,
    image_output_image_msg_size_arg,
    image_output_info_msg_size_arg);
}
void TRACEPOINT(
  robotperf_image_output_cb_fini,
  const void * image_output_node_arg,
  const void * image_output_image_msg_arg,
  const void * image_output_info_msg_arg,
  uint32_t image_input_header_nsec_arg,
  uint32_t image_input_header_sec_arg,
  size_t image_output_image_msg_size_arg,
  size_t image_output_info_msg_size_arg)
{
  CONDITIONAL_TP(
    robotperf_image_output_cb_fini,
    image_output_node_arg,
    image_output_image_msg_arg,
    image_output_info_msg_arg,
    image_input_header_nsec_arg,
    image_input_header_sec_arg,
    image_output_image_msg_size_arg,
    image_output_info_msg_size_arg);
}

// pointcloud_input
void TRACEPOINT(
  robotperf_pointcloud_input_cb_init, 
  const void * pointcloud_input_node_arg, 
  const void * pointcloud_input_pointcloud_msg_arg, 
  uint32_t pointcloud_input_header_nsec_arg, 
  uint32_t pointcloud_input_header_sec_arg, 
  size_t pointcloud_input_msg_size_arg,
  uint32_t key_arg)
{
  CONDITIONAL_TP(
    robotperf_pointcloud_input_cb_init, 
    pointcloud_input_node_arg, 
    pointcloud_input_pointcloud_msg_arg,
    pointcloud_input_header_nsec_arg, 
    pointcloud_input_header_sec_arg, 
    pointcloud_input_msg_size_arg,
    key_arg);
}

void TRACEPOINT(
  robotperf_pointcloud_input_cb_fini, 
  const void * pointcloud_input_node_arg, 
  const void * pointcloud_input_pointcloud_msg_arg, 
  uint32_t pointcloud_input_header_nsec_arg, 
  uint32_t pointcloud_input_header_sec_arg, 
  size_t pointcloud_input_msg_size_arg,
  uint32_t key_arg)
{
  CONDITIONAL_TP(
    robotperf_pointcloud_input_cb_fini, 
    pointcloud_input_node_arg, 
    pointcloud_input_pointcloud_msg_arg,
    pointcloud_input_header_nsec_arg, 
    pointcloud_input_header_sec_arg, 
    pointcloud_input_msg_size_arg,
    key_arg);
}

// pointcloud_output
void TRACEPOINT(
  robotperf_pointcloud_output_cb_init,
  const void * pointcloud_output_node,
  const void * pointcloud_output_pointcloud_msg,
  uint32_t image_input_header_nsec_arg,
  uint32_t image_input_header_sec_arg,
  size_t pointcloud_output_msg_size_arg,
  uint32_t key_arg)
{
  CONDITIONAL_TP(
    robotperf_pointcloud_output_cb_init,
    pointcloud_output_node,
    pointcloud_output_pointcloud_msg,
    image_input_header_nsec_arg,
    image_input_header_sec_arg,
    pointcloud_output_msg_size_arg,
    key_arg);
}

void TRACEPOINT(
  robotperf_pointcloud_output_cb_fini,
  const void * pointcloud_output_node,
  const void * pointcloud_output_pointcloud_msg,
  uint32_t image_input_header_nsec_arg,
  uint32_t image_input_header_sec_arg,
  size_t pointcloud_output_msg_size_arg,
  uint32_t key_arg)
{
  CONDITIONAL_TP(
    robotperf_pointcloud_output_cb_fini,
    pointcloud_output_node,
    pointcloud_output_pointcloud_msg,
    image_input_header_nsec_arg,
    image_input_header_sec_arg,
    pointcloud_output_msg_size_arg,
    key_arg);
}

// laserscan_input
void TRACEPOINT(
  robotperf_laserscan_input_cb_init,
  const void * laserscan_input_node_arg,
  const void * laserscan_input_scan_msg_arg,
  uint32_t laserscan_input_header_nsec_arg,
  uint32_t laserscan_input_header_sec_arg,
  size_t laserscan_input_msg_size_arg,
  uint32_t key_arg)
{
  CONDITIONAL_TP(
    robotperf_laserscan_input_cb_init,
    laserscan_input_node_arg,
    laserscan_input_scan_msg_arg,
    laserscan_input_header_nsec_arg,
    laserscan_input_header_sec_arg,
    laserscan_input_msg_size_arg,
    key_arg);
}

void TRACEPOINT(
  robotperf_laserscan_input_cb_fini,
  const void * laserscan_input_node_arg,
  const void * laserscan_input_scan_msg_arg,
  uint32_t laserscan_input_header_nsec_arg,
  uint32_t laserscan_input_header_sec_arg,
  size_t laserscan_input_msg_size_arg,
  uint32_t key_arg)
{
  CONDITIONAL_TP(
    robotperf_laserscan_input_cb_fini,
    laserscan_input_node_arg,
    laserscan_input_scan_msg_arg,
    laserscan_input_header_nsec_arg,
    laserscan_input_header_sec_arg,
    laserscan_input_msg_size_arg,
    key_arg);
}

// laserscan_output
void TRACEPOINT(
  robotperf_laserscan_output_cb_init,
  const void * laserscan_output_node_arg,
  const void * laserscan_output_scan_msg_arg,
  uint32_t laserscan_output_header_nsec_arg,
  uint32_t laserscan_output_header_sec_arg,
  size_t laserscan_output_msg_size_arg,
  uint32_t key_arg)
{
  CONDITIONAL_TP(
    robotperf_laserscan_output_cb_init,
    laserscan_output_node_arg,
    laserscan_output_scan_msg_arg,
    laserscan_output_header_nsec_arg,
    laserscan_output_header_sec_arg,
    laserscan_output_msg_size_arg,
    key_arg);
}

void TRACEPOINT(
  robotperf_laserscan_output_cb_fini,
  const void * laserscan_output_node_arg,
  const void * laserscan_output_scan_msg_arg,
  uint32_t laserscan_output_header_nsec_arg,
  uint32_t laserscan_output_header_sec_arg,
  size_t laserscan_output_msg_size_arg,
  uint32_t key_arg)
{
  CONDITIONAL_TP(
    robotperf_laserscan_output_cb_fini,
    laserscan_output_node_arg,
    laserscan_output_scan_msg_arg,
    laserscan_output_header_nsec_arg,
    laserscan_output_header_sec_arg,
    laserscan_output_msg_size_arg,
    key_arg);
}

// joint_trajectory_output
void TRACEPOINT(
  robotperf_joint_trajectory_output_cb_init,
  const void * joint_trajectory_output_node_arg,
  const void * joint_trajectory_output_scan_msg_arg,
  uint32_t joint_trajectory_output_header_nsec_arg,
  uint32_t joint_trajectory_output_header_sec_arg,
  size_t joint_trajectory_output_msg_size_arg)
{
  CONDITIONAL_TP(
    robotperf_joint_trajectory_output_cb_init,
    joint_trajectory_output_node_arg,
    joint_trajectory_output_scan_msg_arg,
    joint_trajectory_output_header_nsec_arg,
    joint_trajectory_output_header_sec_arg,
    joint_trajectory_output_msg_size_arg);
}

void TRACEPOINT(
  robotperf_joint_trajectory_output_cb_fini,
  const void * joint_trajectory_output_node_arg,
  const void * joint_trajectory_output_msg_arg,
  uint32_t joint_trajectory_output_header_nsec_arg,
  uint32_t joint_trajectory_output_header_sec_arg,
  size_t joint_trajectory_output_msg_size_arg)
{
  CONDITIONAL_TP(
    robotperf_joint_trajectory_output_cb_fini,
    joint_trajectory_output_node_arg,
    joint_trajectory_output_msg_arg,
    joint_trajectory_output_header_nsec_arg,
    joint_trajectory_output_header_sec_arg,
    joint_trajectory_output_msg_size_arg);
}

// joint_trajectory_input
void TRACEPOINT(
  robotperf_joint_trajectory_input_cb_init,
  const void * joint_trajectory_input_node_arg,
  const void * joint_trajectory_input_msg_arg,
  uint32_t joint_trajectory_input_header_nsec_arg,
  uint32_t joint_trajectory_input_header_sec_arg,
  size_t joint_trajectory_input_msg_size_arg)
{
  CONDITIONAL_TP(
    robotperf_joint_trajectory_input_cb_init,
    joint_trajectory_input_node_arg,
    joint_trajectory_input_msg_arg,
    joint_trajectory_input_header_nsec_arg,
    joint_trajectory_input_header_sec_arg,
    joint_trajectory_input_msg_size_arg
  );
}

void TRACEPOINT(
  robotperf_joint_trajectory_input_cb_fini,
  const void * joint_trajectory_input_node_arg,
  const void * joint_trajectory_input_msg_arg,
  uint32_t joint_trajectory_input_header_nsec_arg,
  uint32_t joint_trajectory_input_header_sec_arg,
  size_t joint_trajectory_input_msg_size_arg)
{
  CONDITIONAL_TP(
    robotperf_joint_trajectory_input_cb_fini,
    joint_trajectory_input_node_arg,
    joint_trajectory_input_msg_arg,
    joint_trajectory_input_header_nsec_arg,
    joint_trajectory_input_header_sec_arg,
    joint_trajectory_input_msg_size_arg
  );
}


// twist_input
void TRACEPOINT(
  robotperf_twist_input_cb_init,
  const void * twist_input_node_arg,
  const void * twist_input_msg_arg,
  size_t twist_input_msg_size_arg)
{
  CONDITIONAL_TP(
    robotperf_twist_input_cb_init,
    twist_input_node_arg,
    twist_input_msg_arg,
    twist_input_msg_size_arg);
}

void TRACEPOINT(
  robotperf_twist_input_cb_fini,
  const void * twist_input_node_arg,
  const void * twist_input_msg_arg,
  size_t twist_input_msg_size_arg)
{
  CONDITIONAL_TP(
    robotperf_twist_input_cb_fini,
    twist_input_node_arg,
    twist_input_msg_arg,
    twist_input_msg_size_arg);
}

// Generic Published Tracepoints
void TRACEPOINT(
    robotperf_msg_published_1,
    const void * node_arg,
    const void * msg_arg,
    uint32_t key_arg)
{
    CONDITIONAL_TP(
        robotperf_msg_published_1,
        node_arg,
        msg_arg,
        key_arg);
}

void TRACEPOINT(
    robotperf_msg_published_2,
    const void * node_arg,
    const void * msg_arg,
    uint32_t key_arg)
{
    CONDITIONAL_TP(
        robotperf_msg_published_2,
        node_arg,
        msg_arg,
        key_arg);
}

// Generic Received Tracepoints
void TRACEPOINT(
    robotperf_msg_received_1,
    const void * node_arg,
    const void * msg_arg,
    uint32_t key_arg)
{
    CONDITIONAL_TP(
        robotperf_msg_received_1,
        node_arg,
        msg_arg,
        key_arg);
}

void TRACEPOINT(
    robotperf_msg_received_2,
    const void * node_arg,
    const void * msg_arg,
    uint32_t key_arg)
{
    CONDITIONAL_TP(
        robotperf_msg_received_2,
        node_arg,
        msg_arg,
        key_arg);
}


// Generic Tracepoints with Message Size
void TRACEPOINT(
    robotperf_msg_published_size_1,
    const void * node_arg,
    const void * msg_arg,
    uint32_t key_arg, 
    size_t size_arg)
{
    CONDITIONAL_TP(
        robotperf_msg_published_size_1,
        node_arg,
        msg_arg,
        key_arg, 
        size_arg);
}


void TRACEPOINT(
    robotperf_msg_received_size_1,
    const void * node_arg,
    const void * msg_arg,
    uint32_t key_arg, 
    size_t size_arg)
{
    CONDITIONAL_TP(
        robotperf_msg_received_size_1,
        node_arg,
        msg_arg,
        key_arg, 
        size_arg);
}

#ifndef _WIN32
# pragma GCC diagnostic pop
#else
# pragma warning(pop)
#endif

#endif  // TRACETOOLS_DISABLED
