/*
   @@@@@@@@@@@@@@@@@@@@
   @@@@@@@@@&@@@&&@@@@@
   @@@@@ @@  @@    @@@@
   @@@@@ @@  @@    @@@@
   @@@@@ @@  @@    @@@@ Copyright (c) 2023, Acceleration Robotics®
   @@@@@ @@  @@    @@@@ Author: Víctor Mayoral Vilches <victor@accelerationrobotics.com>
   @@@@@ @@  @@    @@@@ AUthor: Alejandra Martínez Fariña <alex@accelerationrobotics.com>
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

// Provide fake header guard for cpplint
#undef TRACETOOLS_BENCHMARK__TP_CALL_H_
#ifndef TRACETOOLS_BENCHMARK__TP_CALL_H_
#define TRACETOOLS_BENCHMARK__TP_CALL_H_

#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER robotperf_benchmarks

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "tracetools_benchmark/tp_call.h"

#if !defined(_TRACETOOLS_BENCHMARK__TP_CALL_H_) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define _TRACETOOLS_BENCHMARK__TP_CALL_H_

#include <lttng/tracepoint.h>
#include <stdint.h>
#include <stdbool.h>

// robotperf image_input init callback
TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,              // tracepoint provider name
  robotperf_image_input_cb_init,    // tracepoint name
  TP_ARGS(
    // input arguments, see https://lttng.org/docs/v2.12/#doc-tpp-def-input-args
    const void *, image_input_node_arg,
    const void *, image_input_image_msg_arg,
    const void *, image_input_info_msg_arg,
    uint32_t, image_input_header_nsec_arg,
    uint32_t, image_input_header_sec_arg,
    size_t, image_input_image_msg_size_arg,
    size_t, image_input_info_msg_size_arg),
  TP_FIELDS(
    // output event fields, see https://lttng.org/man/3/lttng-ust/v2.12/#doc-ctf-macros
    ctf_integer_hex(const void *, image_input_node, image_input_node_arg)
    ctf_integer_hex(const void *, image_input_image_msg, image_input_image_msg_arg)
    ctf_integer_hex(const void *, image_input_info_msg, image_input_info_msg_arg)
    ctf_integer(uint32_t, image_input_header_nsec, image_input_header_nsec_arg)
    ctf_integer(uint32_t, image_input_header_sec, image_input_header_sec_arg)
    ctf_integer(size_t, image_input_image_msg_size, image_input_image_msg_size_arg)
    ctf_integer(size_t, image_input_info_msg_size, image_input_info_msg_size_arg)
    ctf_string(version, tracetools_benchmark_VERSION)
  )
)
// robotperf image_input end of callback (after publication)
TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  robotperf_image_input_cb_fini,
  TP_ARGS(
    const void *, image_input_node_arg,
    const void *, image_input_image_msg_arg,
    const void *, image_input_info_msg_arg,
    uint32_t, image_input_header_nsec_arg,
    uint32_t, image_input_header_sec_arg,
    size_t, image_input_image_msg_size_arg,
    size_t, image_input_info_msg_size_arg),
  TP_FIELDS(
    ctf_integer_hex(const void *, image_input_node, image_input_node_arg)
    ctf_integer_hex(const void *, image_input_image_msg, image_input_image_msg_arg)
    ctf_integer_hex(const void *, image_input_info_msg, image_input_info_msg_arg)
    ctf_integer(uint32_t, image_input_header_nsec, image_input_header_nsec_arg)
    ctf_integer(uint32_t, image_input_header_sec, image_input_header_sec_arg)
    ctf_integer(size_t, image_input_image_msg_size, image_input_image_msg_size_arg)
    ctf_integer(size_t, image_input_info_msg_size, image_input_info_msg_size_arg)
    ctf_string(version, tracetools_benchmark_VERSION)
  )
)

// robotperf image_output init callback
TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,              // tracepoint provider name
  robotperf_image_output_cb_init,    // tracepoint name
  TP_ARGS(
    // input arguments, see https://lttng.org/docs/v2.12/#doc-tpp-def-input-args
    const void *, image_output_node_arg,
    const void *, image_output_image_msg_arg,
    const void *, image_output_info_msg_arg,
    uint32_t, image_input_header_nsec_arg,
    uint32_t, image_input_header_sec_arg,
    size_t, image_output_image_msg_size_arg,
    size_t, image_output_info_msg_size_arg),
  TP_FIELDS(
    // output event fields, see https://lttng.org/man/3/lttng-ust/v2.12/#doc-ctf-macros
    ctf_integer_hex(const void *, image_output_node, image_output_node_arg)
    ctf_integer_hex(const void *, image_output_image_msg, image_output_image_msg_arg)
    ctf_integer_hex(const void *, image_output_info_msg, image_output_info_msg_arg)
    ctf_integer(uint32_t, image_input_header_nsec, image_input_header_nsec_arg)
    ctf_integer(uint32_t, image_input_header_sec, image_input_header_sec_arg)
    ctf_integer(size_t, image_output_image_msg_size, image_output_image_msg_size_arg)
    ctf_integer(size_t, image_output_info_msg_size, image_output_info_msg_size_arg)
    ctf_string(version, tracetools_benchmark_VERSION)
  )
)
// robotperf image_output end of callback (after publication)
TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  robotperf_image_output_cb_fini,
  TP_ARGS(
    const void *, image_output_node_arg,
    const void *, image_output_image_msg_arg,
    const void *, image_output_info_msg_arg,
    uint32_t, image_input_header_nsec_arg,
    uint32_t, image_input_header_sec_arg,
    size_t, image_output_image_msg_size_arg,
    size_t, image_output_info_msg_size_arg),
  TP_FIELDS(
    ctf_integer_hex(const void *, image_output_node, image_output_node_arg)
    ctf_integer_hex(const void *, image_output_image_msg, image_output_image_msg_arg)
    ctf_integer_hex(const void *, image_output_info_msg, image_output_info_msg_arg)
    ctf_integer(uint32_t, image_input_header_nsec, image_input_header_nsec_arg)
    ctf_integer(uint32_t, image_input_header_sec, image_input_header_sec_arg)
    ctf_integer(size_t, image_output_image_msg_size, image_output_image_msg_size_arg)
    ctf_integer(size_t, image_output_info_msg_size, image_output_info_msg_size_arg)
    ctf_string(version, tracetools_benchmark_VERSION)
  )
)

// robotperf pointcloud_input init callback
TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  robotperf_pointcloud_input_cb_init,
  TP_ARGS(
    const void *, pointcloud_input_node_arg,
    const void *, pointcloud_input_pointcloud_msg_arg,
    uint32_t, pointcloud_input_header_nsec_arg,
    uint32_t, pointcloud_input_header_sec_arg,
    size_t, pointcloud_input_msg_size_arg,
    uint32_t, key_arg),
  TP_FIELDS(
    ctf_integer_hex(const void *, pointcloud_input_node, pointcloud_input_node_arg)
    ctf_integer_hex(const void *, pointcloud_input_pointcloud_msg, pointcloud_input_pointcloud_msg_arg)
    ctf_integer(uint32_t, pointcloud_input_header_nsec, pointcloud_input_header_nsec_arg)
    ctf_integer(uint32_t, pointcloud_input_header_sec, pointcloud_input_header_sec_arg)
    ctf_integer(size_t, pointcloud_input_msg_size, pointcloud_input_msg_size_arg)
    ctf_integer(uint32_t, key, key_arg)
    ctf_string(version, tracetools_benchmark_VERSION)
  )
)

// robotperf pointcloud_input end of callback (after publication)
TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  robotperf_pointcloud_input_cb_fini,
  TP_ARGS(
    const void *, pointcloud_input_node_arg,
    const void *, pointcloud_input_pointcloud_msg_arg,
    uint32_t, pointcloud_input_header_nsec_arg,
    uint32_t, pointcloud_input_header_sec_arg,
    size_t, pointcloud_input_msg_size_arg,
    uint32_t, key_arg),
  TP_FIELDS(
    ctf_integer_hex(const void *, pointcloud_input_node, pointcloud_input_node_arg)
    ctf_integer_hex(const void *, pointcloud_input_pointcloud_msg, pointcloud_input_pointcloud_msg_arg)
    ctf_integer(uint32_t, pointcloud_input_header_nsec, pointcloud_input_header_nsec_arg)
    ctf_integer(uint32_t, pointcloud_input_header_sec, pointcloud_input_header_sec_arg)
    ctf_integer(size_t, pointcloud_input_msg_size, pointcloud_input_msg_size_arg)
    ctf_integer(uint32_t, key, key_arg)
    ctf_string(version, tracetools_benchmark_VERSION)
  )
)

// robotperf pointcloud_output init callback
TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  robotperf_pointcloud_output_cb_init,
  TP_ARGS(
    const void *, pointcloud_output_node_arg,
    const void *, pointcloud_output_pointcloud_msg_arg,
    uint32_t, image_input_header_nsec_arg,
    uint32_t, image_input_header_sec_arg,
    size_t, pointcloud_output_msg_size_arg,
    uint32_t, key_arg),
  TP_FIELDS(
    ctf_integer_hex(const void *, pointcloud_output_node, pointcloud_output_node_arg)
    ctf_integer_hex(const void *, pointcloud_output_pointcloud_msg, pointcloud_output_pointcloud_msg_arg)
    ctf_integer(uint32_t, image_input_header_nsec, image_input_header_nsec_arg)
    ctf_integer(uint32_t, image_input_header_sec, image_input_header_sec_arg)
    ctf_integer(size_t, pointcloud_output_msg_size, pointcloud_output_msg_size_arg)
    ctf_integer(uint32_t, key, key_arg)
    ctf_string(version, tracetools_benchmark_VERSION)
  )
)

// robotperf pointcloud_output end of callback (after publication)
TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  robotperf_pointcloud_output_cb_fini,
  TP_ARGS(
    const void *, pointcloud_output_node_arg,
    const void *, pointcloud_output_pointcloud_msg_arg,
    uint32_t, image_input_header_nsec_arg,
    uint32_t, image_input_header_sec_arg,
    size_t, pointcloud_output_msg_size_arg,
    uint32_t, key_arg),
  TP_FIELDS(
    ctf_integer_hex(const void *, pointcloud_output_node, pointcloud_output_node_arg)
    ctf_integer_hex(const void *, pointcloud_output_pointcloud_msg, pointcloud_output_pointcloud_msg_arg)
    ctf_integer(uint32_t, image_input_header_nsec, image_input_header_nsec_arg)
    ctf_integer(uint32_t, image_input_header_sec, image_input_header_sec_arg)
    ctf_integer(size_t, pointcloud_output_msg_size, pointcloud_output_msg_size_arg)
    ctf_integer(uint32_t, key, key_arg)
    ctf_string(version, tracetools_benchmark_VERSION)
  )
)

// robotperf laserscan_input init callback
TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,                 // tracepoint provider name
  robotperf_laserscan_input_cb_init,   // tracepoint name
  TP_ARGS(
    // input arguments
    const void *, laserscan_input_node_arg,
    const void *, laserscan_input_scan_msg_arg,
    uint32_t, laserscan_input_header_nsec_arg,
    uint32_t, laserscan_input_header_sec_arg,
    size_t, laserscan_input_msg_size_arg,
    uint32_t, key_arg),
  TP_FIELDS(
    // output event fields
    ctf_integer_hex(const void *, laserscan_input_node, laserscan_input_node_arg)
    ctf_integer_hex(const void *, laserscan_input_scan_msg, laserscan_input_scan_msg_arg)
    ctf_integer(uint32_t, laserscan_input_header_nsec, laserscan_input_header_nsec_arg)
    ctf_integer(uint32_t, laserscan_input_header_sec, laserscan_input_header_sec_arg)
    ctf_integer(size_t, laserscan_input_msg_size, laserscan_input_msg_size_arg)
    ctf_integer(uint32_t, key, key_arg)
    ctf_string(version, tracetools_benchmark_VERSION)
  )
)

// robotperf laserscan_input end of callback (after publication)
TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  robotperf_laserscan_input_cb_fini,
  TP_ARGS(
    const void *, laserscan_input_node_arg,
    const void *, laserscan_input_scan_msg_arg,
    uint32_t, laserscan_input_header_nsec_arg,
    uint32_t, laserscan_input_header_sec_arg,
    size_t, laserscan_input_msg_size_arg,
    uint32_t, key_arg),
  TP_FIELDS(
    ctf_integer_hex(const void *, laserscan_input_node, laserscan_input_node_arg)
    ctf_integer_hex(const void *, laserscan_input_scan_msg, laserscan_input_scan_msg_arg)
    ctf_integer(uint32_t, laserscan_input_header_nsec, laserscan_input_header_nsec_arg)
    ctf_integer(uint32_t, laserscan_input_header_sec, laserscan_input_header_sec_arg)
    ctf_integer(size_t, laserscan_input_msg_size, laserscan_input_msg_size_arg)
    ctf_integer(uint32_t, key, key_arg)
    ctf_string(version, tracetools_benchmark_VERSION)
  )
)

// robotperf laserscan_output init callback
TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,                 // tracepoint provider name
  robotperf_laserscan_output_cb_init,  // tracepoint name
  TP_ARGS(
    // input arguments
    const void *, laserscan_output_node_arg,
    const void *, laserscan_output_scan_msg_arg,
    uint32_t, laserscan_output_header_nsec_arg,
    uint32_t, laserscan_output_header_sec_arg,
    size_t, laserscan_output_msg_size_arg,
    uint32_t, key_arg),
  TP_FIELDS(
    // output event fields
    ctf_integer_hex(const void *, laserscan_output_node, laserscan_output_node_arg)
    ctf_integer_hex(const void *, laserscan_output_scan_msg, laserscan_output_scan_msg_arg)
    ctf_integer(uint32_t, laserscan_output_header_nsec, laserscan_output_header_nsec_arg)
    ctf_integer(uint32_t, laserscan_output_header_sec, laserscan_output_header_sec_arg)
    ctf_integer(size_t, laserscan_output_msg_size, laserscan_output_msg_size_arg)
    ctf_integer(uint32_t, key, key_arg)
    ctf_string(version, tracetools_benchmark_VERSION)
  )
)

// robotperf laserscan_output end of callback (after publication)
TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  robotperf_laserscan_output_cb_fini,
  TP_ARGS(
    const void *, laserscan_output_node_arg,
    const void *, laserscan_output_scan_msg_arg,
    uint32_t, laserscan_output_header_nsec_arg,
    uint32_t, laserscan_output_header_sec_arg,
    size_t, laserscan_output_msg_size_arg,
    uint32_t, key_arg),
  TP_FIELDS(
    ctf_integer_hex(const void *, laserscan_output_node, laserscan_output_node_arg)
    ctf_integer_hex(const void *, laserscan_output_scan_msg, laserscan_output_scan_msg_arg)
    ctf_integer(uint32_t, laserscan_output_header_nsec, laserscan_output_header_nsec_arg)
    ctf_integer(uint32_t, laserscan_output_header_sec, laserscan_output_header_sec_arg)
    ctf_integer(size_t, laserscan_output_msg_size, laserscan_output_msg_size_arg)
    ctf_integer(uint32_t, key, key_arg)
    ctf_string(version, tracetools_benchmark_VERSION)
  )
)

// robotperf joint_trajectory_output init callback
TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  robotperf_joint_trajectory_output_cb_init,
  TP_ARGS(
    const void *, joint_trajectory_output_node_arg,
    const void *, joint_trajectory_output_msg_arg,
    uint32_t, joint_trajectory_output_header_nsec_arg,
    uint32_t, joint_trajectory_output_header_sec_arg,
    size_t, joint_trajectory_output_msg_size_arg),
  TP_FIELDS(
    ctf_integer_hex(const void *, joint_trajectory_output_node, joint_trajectory_output_node_arg)
    ctf_integer_hex(const void *, joint_trajectory_output_msg, joint_trajectory_output_msg_arg)
    ctf_integer(uint32_t, joint_trajectory_output_header_nsec, joint_trajectory_output_header_nsec_arg)
    ctf_integer(uint32_t, joint_trajectory_output_header_sec, joint_trajectory_output_header_sec_arg)
    ctf_integer(size_t, joint_trajectory_output_msg_size, joint_trajectory_output_msg_size_arg)
    ctf_string(version, tracetools_benchmark_VERSION)
  )
)

// robotperf joint_trajectory_output end of callback (after publication)
TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  robotperf_joint_trajectory_output_cb_fini,
  TP_ARGS(
    const void *, joint_trajectory_output_node_arg,
    const void *, joint_trajectory_output_msg_arg,
    uint32_t, joint_trajectory_output_header_nsec_arg,
    uint32_t, joint_trajectory_output_header_sec_arg,
    size_t, joint_trajectory_output_msg_size_arg),
  TP_FIELDS(
    ctf_integer_hex(const void *, joint_trajectory_output_node, joint_trajectory_output_node_arg)
    ctf_integer_hex(const void *, joint_trajectory_output_msg, joint_trajectory_output_msg_arg)
    ctf_integer(uint32_t, joint_trajectory_output_header_nsec, joint_trajectory_output_header_nsec_arg)
    ctf_integer(uint32_t, joint_trajectory_output_header_sec, joint_trajectory_output_header_sec_arg)
    ctf_integer(size_t, joint_trajectory_output_msg_size, joint_trajectory_output_msg_size_arg)
    ctf_string(version, tracetools_benchmark_VERSION)
  )
)

// robotperf joint_trajectory_input init callback
TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  robotperf_joint_trajectory_input_cb_init,
  TP_ARGS(
    const void *, joint_trajectory_input_node_arg,
    const void *, joint_trajectory_input_msg_arg,
    uint32_t, joint_trajectory_input_header_nsec_arg,
    uint32_t, joint_trajectory_input_header_sec_arg,
    size_t, joint_trajectory_input_msg_size_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, joint_trajectory_input_node, joint_trajectory_input_node_arg)
    ctf_integer_hex(const void *, joint_trajectory_input_msg, joint_trajectory_input_msg_arg)
    ctf_integer(uint32_t, joint_trajectory_input_header_nsec, joint_trajectory_input_header_nsec_arg)
    ctf_integer(uint32_t, joint_trajectory_input_header_sec,  joint_trajectory_input_header_sec_arg)
    ctf_integer(size_t, joint_trajectory_input_msg_size, joint_trajectory_input_msg_size_arg)
    ctf_string(version, tracetools_benchmark_VERSION)
  )
)

// robotperf joint_trajectory_input end of callback
TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  robotperf_joint_trajectory_input_cb_fini,
  TP_ARGS(
    const void *, joint_trajectory_input_node_arg,
    const void *, joint_trajectory_input_msg_arg,
    uint32_t, joint_trajectory_input_header_nsec_arg,
    uint32_t, joint_trajectory_input_header_sec_arg,
    size_t, joint_trajectory_input_msg_size_arg
  ),
  TP_FIELDS(
    ctf_integer_hex(const void *, joint_trajectory_input_node, joint_trajectory_input_node_arg)
    ctf_integer_hex(const void *, joint_trajectory_input_msg, joint_trajectory_input_msg_arg)
    ctf_integer(uint32_t, joint_trajectory_input_header_nsec, joint_trajectory_input_header_nsec_arg)
    ctf_integer(uint32_t, joint_trajectory_input_header_sec,  joint_trajectory_input_header_sec_arg)
    ctf_integer(size_t, joint_trajectory_input_msg_size, joint_trajectory_input_msg_size_arg)
    ctf_string(version, tracetools_benchmark_VERSION)
  )
)

// robotperf twist_input init callback
TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  robotperf_twist_input_cb_init,
  TP_ARGS(
    const void *, twist_input_node_arg,
    const void *, twist_input_msg_arg,
    size_t, twist_input_msg_size_arg),
  TP_FIELDS(
    ctf_integer_hex(const void *, twist_input_node, twist_input_node_arg)
    ctf_integer_hex(const void *, twist_input_msg, twist_input_msg_arg)
    ctf_integer(size_t, twist_input_msg_size, twist_input_msg_size_arg)
    ctf_string(version, tracetools_benchmark_VERSION)
  )
)

// robotperf twist_input end of callback (after publication)
TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  robotperf_twist_input_cb_fini,
  TP_ARGS(
    const void *, twist_input_node_arg,
    const void *, twist_input_msg_arg,
    size_t, twist_input_msg_size_arg),
  TP_FIELDS(
    ctf_integer_hex(const void *, twist_input_node, twist_input_node_arg)
    ctf_integer_hex(const void *, twist_input_msg, twist_input_msg_arg)
    ctf_integer(size_t, twist_input_msg_size, twist_input_msg_size_arg)
    ctf_string(version, tracetools_benchmark_VERSION)
  )
)

// Tracepoint for message publication 1
TRACEPOINT_EVENT(
    TRACEPOINT_PROVIDER,
    robotperf_msg_published_1,
    TP_ARGS(
        const void *, node_arg,
        const void *, msg_arg,
        uint32_t, key_arg),
    TP_FIELDS(
        ctf_integer_hex(const void *, node, node_arg)
        ctf_integer_hex(const void *, msg, msg_arg)
        ctf_integer(uint32_t, key, key_arg)
        ctf_string(version, tracetools_benchmark_VERSION)
    )
)

// Tracepoint for message publication 2
TRACEPOINT_EVENT(
    TRACEPOINT_PROVIDER,
    robotperf_msg_published_2,
    TP_ARGS(
        const void *, node_arg,
        const void *, msg_arg,
        uint32_t, key_arg),
    TP_FIELDS(
        ctf_integer_hex(const void *, node, node_arg)
        ctf_integer_hex(const void *, msg, msg_arg)
        ctf_integer(uint32_t, key, key_arg)
        ctf_string(version, tracetools_benchmark_VERSION)
    )
)

// Tracepoint for message reception 1
TRACEPOINT_EVENT(
    TRACEPOINT_PROVIDER,
    robotperf_msg_received_1,
    TP_ARGS(
        const void *, node_arg,
        const void *, msg_arg,
        uint32_t, key_arg),
    TP_FIELDS(
        ctf_integer_hex(const void *, node, node_arg)
        ctf_integer_hex(const void *, msg, msg_arg)
        ctf_integer(uint32_t, key, key_arg)
        ctf_string(version, tracetools_benchmark_VERSION)
    )
)

// Tracepoint for message reception 2
TRACEPOINT_EVENT(
    TRACEPOINT_PROVIDER,
    robotperf_msg_received_2,
    TP_ARGS(
        const void *, node_arg,
        const void *, msg_arg,
        uint32_t, key_arg),
    TP_FIELDS(
        ctf_integer_hex(const void *, node, node_arg)
        ctf_integer_hex(const void *, msg, msg_arg)
        ctf_integer(uint32_t, key, key_arg)
        ctf_string(version, tracetools_benchmark_VERSION)
    )
)

// Tracepoint for message publication WITH SIZE 1
TRACEPOINT_EVENT(
    TRACEPOINT_PROVIDER,
    robotperf_msg_published_size_1,
    TP_ARGS(
        const void *, node_arg,
        const void *, msg_arg,
        uint32_t, key_arg,
        size_t, msg_size_arg),
    TP_FIELDS(
        ctf_integer_hex(const void *, node, node_arg)
        ctf_integer_hex(const void *, msg, msg_arg)
        ctf_integer(uint32_t, key, key_arg)
        ctf_integer(size_t, msg_size, msg_size_arg)
        ctf_string(version, tracetools_benchmark_VERSION)
    )
)

// Tracepoint for message reception WITH SIZE 1
TRACEPOINT_EVENT(
    TRACEPOINT_PROVIDER,
    robotperf_msg_received_size_1,
    TP_ARGS(
        const void *, node_arg,
        const void *, msg_arg,
        uint32_t, key_arg, 
        size_t, msg_size_arg),
    TP_FIELDS(
        ctf_integer_hex(const void *, node, node_arg)
        ctf_integer_hex(const void *, msg, msg_arg)
        ctf_integer(uint32_t, key, key_arg)
        ctf_integer(size_t, msg_size, msg_size_arg)
        ctf_string(version, tracetools_benchmark_VERSION)
    )
)

#endif  // _TRACETOOLS_BENCHMARK__TP_CALL_H_

#include <lttng/tracepoint-event.h>

#endif  // TRACETOOLS_BENCHMARK__TP_CALL_H_
